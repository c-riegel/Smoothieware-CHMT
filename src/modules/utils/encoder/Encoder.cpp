#include "Encoder.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "StepTicker.h"
#include "Conveyor.h"

#include "stm32f4xx_hal.h"
#include "mbed.h" // for us_ticker_read()
#include <math.h>

#define encoder_enable_checksum          CHECKSUM("encoder_enable")
#define encoder_x_counts_per_mm_checksum CHECKSUM("encoder_x_counts_per_mm")
#define encoder_y_counts_per_mm_checksum CHECKSUM("encoder_y_counts_per_mm")
#define alpha_max_travel_checksum        CHECKSUM("alpha_max_travel")
#define beta_max_travel_checksum         CHECKSUM("beta_max_travel")

// Timeout safety: 3x expected move time + 500ms floor.
// Expected time = distance / (feed_rate / 60).
// Catches stalled motors without being so long the user power-cycles first.
#define TIMEOUT_SAFETY_MULT  3
#define TIMEOUT_FLOOR_US     500000

static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim5;

// Stepper motor pointers for ISR access
static StepperMotor *x_stepper = nullptr;
static StepperMotor *y_stepper = nullptr;

// Encoder instance pointer for ISR callbacks
static Encoder *encoder_instance = nullptr;

// Output Compare ISR for X axis (TIM2 CC3)
extern "C" void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_CC3IF) {
        TIM2->SR = ~TIM_SR_CC3IF;
        TIM2->DIER &= ~TIM_DIER_CC3IE;
        if (encoder_instance) encoder_instance->on_x_target_reached();
    }
}

// Output Compare ISR for Y axis (TIM5 CC3)
extern "C" void TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_CC3IF) {
        TIM5->SR = ~TIM_SR_CC3IF;
        TIM5->DIER &= ~TIM_DIER_CC3IE;
        if (encoder_instance) encoder_instance->on_y_target_reached();
    }
}

void Encoder::on_x_target_reached()
{
    dbg_x_oc_count++;
    if (segment_mode) {
        // Segment chaining: mark X done, try to advance
        x_segment_done = true;
        try_advance_segment();
    } else {
        // Single move: stop motor
        if (x_stepper) {
            x_stepper->stop_moving();
            x_stepper->set_encoder_controlled(false);
        }
    }
}

void Encoder::on_y_target_reached()
{
    dbg_y_oc_count++;
    if (segment_mode) {
        y_segment_done = true;
        try_advance_segment();
    } else {
        if (y_stepper) {
            y_stepper->stop_moving();
            y_stepper->set_encoder_controlled(false);
        }
    }
}

void Encoder::try_advance_segment()
{
    // Called from OC ISR. Both TIM2 and TIM5 ISRs are priority 2 —
    // same-priority interrupts don't preempt on Cortex-M4, so no race.
    if (!x_segment_done || !y_segment_done) return;

    // Both axes reached their targets for this segment.
    segments[current_segment].completed_at = us_ticker_read();

    // Stop motors between segments.
    if (x_stepper) x_stepper->stop_moving();
    if (y_stepper) y_stepper->stop_moving();

    int next = current_segment + 1;
    if (next < segment_count) {
        current_segment = next;
        arm_segment(next);
    } else {
        // All segments complete
        if (x_stepper) {
            x_stepper->set_encoder_controlled(false);
            x_stepper->set_encoder_segment_mode(false);
        }
        if (y_stepper) {
            y_stepper->set_encoder_controlled(false);
            y_stepper->set_encoder_segment_mode(false);
        }
        segment_mode = false;
        x_move_armed = false;
        y_move_armed = false;
        segments_done_at = us_ticker_read();
        segments_complete = true;
    }
}

void Encoder::precompute_segment_timeouts()
{
    // Called once after all segments are received, before any movement.
    // For each segment, compute the timeout based on remaining distance to the
    // final target and the segment's feed rate.
    int32_t final_x = segments[segment_count - 1].x_target;
    int32_t final_y = segments[segment_count - 1].y_target;
    int32_t start_x = get_x_count();
    int32_t start_y = get_y_count();

    for (int i = 0; i < segment_count; i++) {
        // Remaining distance from this segment's starting point to the final target.
        // For segment 0, use current encoder position. For later segments, use
        // the previous segment's target as the starting point.
        int32_t seg_start_x = (i == 0) ? start_x : segments[i - 1].x_target;
        int32_t seg_start_y = (i == 0) ? start_y : segments[i - 1].y_target;

        float remain_x = fabsf((float)(final_x - seg_start_x) / x_counts_per_mm);
        float remain_y = fabsf((float)(final_y - seg_start_y) / y_counts_per_mm);
        float dist = remain_x > remain_y ? remain_x : remain_y;

        float feed = segments[i].feed_rate;
        if (feed < 60.0f) feed = 60.0f; // floor at 1mm/s
        float speed_mm_s = feed / 60.0f;

        uint32_t expected_us = (uint32_t)(dist / speed_mm_s * 1000000.0f);
        segments[i].timeout_us = expected_us * TIMEOUT_SAFETY_MULT + TIMEOUT_FLOOR_US;
    }
}

void Encoder::arm_segment(int index)
{
    // Record when this segment was armed and encoder state (for diagnostics)
    segments[index].armed_at = us_ticker_read();
    segments[index].x_enc_at_arm = get_x_count();
    segments[index].y_enc_at_arm = get_y_count();
    segments[index].x_skipped = false;
    segments[index].y_skipped = false;

    // Pre-set done flags for axes that don't move in this segment
    x_segment_done = !segments[index].has_x;
    y_segment_done = !segments[index].has_y;

    // Clear polling fallback flags so stale hits don't trigger false advancement
    x_stepper->encoder_target_hit = false;
    y_stepper->encoder_target_hit = false;

    float tick_freq = THEKERNEL->step_ticker->get_frequency();

    if (segments[index].has_x) {
        int32_t current_x = get_x_count();
        bool gte = segments[index].x_target >= current_x;
        x_stepper->encoder_target = segments[index].x_target;
        x_stepper->set_encoder_check_gte(gte);

        float x_steps_per_sec = (segments[index].feed_rate / 60.0f) * x_stepper->get_steps_per_mm();
        x_stepper->encoder_steps_per_tick = (int64_t)round(((double)x_steps_per_sec / (double)tick_freq) * (double)STEPTICKER_FPSCALE);
        x_stepper->encoder_step_counter = 0;

        // Set stepper direction from encoder direction.
        // gte=true means encoder count increasing toward target.
        // For positive counts_per_mm: encoder increasing = positive robot movement = direction false
        // For negative counts_per_mm: encoder increasing = negative robot movement = direction true
        x_stepper->set_direction(x_counts_per_mm > 0 ? !gte : gte);

        TIM2->CCR3 = (uint32_t)segments[index].x_target;
        TIM2->SR = ~TIM_SR_CC3IF;
        TIM2->DIER |= TIM_DIER_CC3IE;
        x_stepper->set_encoder_segment_mode(true);

        // Overshoot check: if encoder already at or past target, complete immediately.
        // Handles sub-resolution moves (target == current) and momentum overshoot.
        int32_t recheck_x = get_x_count();
        if (gte ? (recheck_x >= segments[index].x_target) : (recheck_x <= segments[index].x_target)) {
            TIM2->DIER &= ~TIM_DIER_CC3IE;
            x_segment_done = true;
        } else {
            x_stepper->start_moving();
        }
    }

    if (segments[index].has_y) {
        int32_t current_y = get_y_count();
        bool gte = segments[index].y_target >= current_y;
        y_stepper->encoder_target = segments[index].y_target;
        y_stepper->set_encoder_check_gte(gte);

        float y_steps_per_sec = (segments[index].feed_rate / 60.0f) * y_stepper->get_steps_per_mm();
        y_stepper->encoder_steps_per_tick = (int64_t)round(((double)y_steps_per_sec / (double)tick_freq) * (double)STEPTICKER_FPSCALE);
        y_stepper->encoder_step_counter = 0;

        // Same direction logic as X (see above)
        y_stepper->set_direction(y_counts_per_mm > 0 ? !gte : gte);

        TIM5->CCR3 = (uint32_t)segments[index].y_target;
        TIM5->SR = ~TIM_SR_CC3IF;
        TIM5->DIER |= TIM_DIER_CC3IE;
        y_stepper->set_encoder_segment_mode(true);

        // Overshoot check (same as X above)
        int32_t recheck_y = get_y_count();
        if (gte ? (recheck_y >= segments[index].y_target) : (recheck_y <= segments[index].y_target)) {
            TIM5->DIER &= ~TIM_DIER_CC3IE;
            y_segment_done = true;
        } else {
            y_stepper->start_moving();
        }
    }

    // Use precomputed timeout — write arm_time and timeout together to avoid
    // race with on_idle reading them at different times across segment transitions.
    uint32_t now = us_ticker_read();
    x_timeout_us = segments[index].timeout_us;
    y_timeout_us = segments[index].timeout_us;
    x_arm_time_us = now;
    y_arm_time_us = now;
    x_move_armed = true;
    y_move_armed = true;

    // If both axes already done (sub-resolution or no movement), advance immediately
    if (x_segment_done && y_segment_done) {
        try_advance_segment();
    }
}

Encoder::Encoder()
{
    encoder_enabled = false;
    x_counts_per_mm = 0;
    y_counts_per_mm = 0;
    x_encoder_offset = 0;
    y_encoder_offset = 0;
    x_move_armed = false;
    y_move_armed = false;
    x_arm_time_us = 0;
    x_timeout_us = 0;
    y_arm_time_us = 0;
    y_timeout_us = 0;
    x_move_distance_mm = 0;
    y_move_distance_mm = 0;
    x_feed_rate_mmpm = 0;
    y_feed_rate_mmpm = 0;
    segment_mode = false;
    buffering = false;
    segment_count = 0;
    segments_received = 0;
    current_segment = 0;
    x_segment_done = false;
    y_segment_done = false;
    segments_complete = false;
    segments_done_at = 0;
    last_reported_segment = -1;
    dbg_x_oc_count = 0;
    dbg_y_oc_count = 0;
    dbg_x_poll_count = 0;
    dbg_y_poll_count = 0;
    dbg_x_enc_at_done = 0;
    dbg_y_enc_at_done = 0;
    dbg_x_target_at_done = 0;
    dbg_y_target_at_done = 0;
    dbg_x_done_pending = false;
    dbg_y_done_pending = false;
}

void Encoder::on_module_loaded()
{
    encoder_enabled = THEKERNEL->config->value(encoder_enable_checksum)->by_default(false)->as_bool();
    if (!encoder_enabled) {
        delete this;
        return;
    }

    x_counts_per_mm = THEKERNEL->config->value(encoder_x_counts_per_mm_checksum)->by_default(0)->as_number();
    y_counts_per_mm = THEKERNEL->config->value(encoder_y_counts_per_mm_checksum)->by_default(0)->as_number();

    // Store pointers for ISR access
    x_stepper = THEROBOT->actuators[X_AXIS];
    y_stepper = THEROBOT->actuators[Y_AXIS];
    encoder_instance = this;

    // Set encoder counter register pointers so step ticker can poll position
    x_stepper->encoder_cnt_reg = &TIM2->CNT;
    y_stepper->encoder_cnt_reg = &TIM5->CNT;

    init_encoders();
    init_output_compare();

    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_HALT);
}

void Encoder::init_encoders()
{
    // Configure GPIO pins for TIM2 encoder (X axis): PA15 = CH1, PB3 = CH2
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;

    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOB, &gpio);

    // Configure GPIO pins for TIM5 encoder (Y axis): PA0 = CH1, PA1 = CH2
    gpio.Pin = GPIO_PIN_0;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_1;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    // Configure TIM2 in encoder mode (X axis)
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF; // 32-bit timer, full range
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    TIM_Encoder_InitTypeDef encoder_config = {0};
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12; // count on both edges
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 0x0F; // max filtering for noise rejection
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Filter = 0x0F;

    HAL_TIM_Encoder_Init(&htim2, &encoder_config);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // Configure TIM5 in encoder mode (Y axis)
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF; // 32-bit timer, full range
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Encoder_Init(&htim5, &encoder_config);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

void Encoder::init_output_compare()
{
    // Configure CC3 for output compare on TIM2 (X axis)
    // CC3 is free — CC1/CC2 are used by the encoder interface
    TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;  // CC3 as output
    TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;  // frozen mode (no pin output, just interrupt)
    TIM2->SR = ~TIM_SR_CC3IF;          // clear any pending CC3 flag
    // CC3 interrupt is NOT enabled here — armed per-move by arm_x_target()

    NVIC_SetVector(TIM2_IRQn, (uint32_t)TIM2_IRQHandler);
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);

    // Configure CC3 for output compare on TIM5 (Y axis)
    TIM5->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM5->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM5->SR = ~TIM_SR_CC3IF;

    NVIC_SetVector(TIM5_IRQn, (uint32_t)TIM5_IRQHandler);
    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);
}

uint32_t Encoder::compute_single_timeout_x()
{
    float speed_mm_s = x_feed_rate_mmpm / 60.0f;
    if (speed_mm_s < 1.0f) speed_mm_s = 1.0f;
    uint32_t expected_us = (uint32_t)(x_move_distance_mm / speed_mm_s * 1000000.0f);
    return expected_us * TIMEOUT_SAFETY_MULT + TIMEOUT_FLOOR_US;
}

uint32_t Encoder::compute_single_timeout_y()
{
    float speed_mm_s = y_feed_rate_mmpm / 60.0f;
    if (speed_mm_s < 1.0f) speed_mm_s = 1.0f;
    uint32_t expected_us = (uint32_t)(y_move_distance_mm / speed_mm_s * 1000000.0f);
    return expected_us * TIMEOUT_SAFETY_MULT + TIMEOUT_FLOOR_US;
}

void Encoder::arm_x_target(int32_t target)
{
    int32_t current = get_x_count();
    x_move_distance_mm = fabsf((float)(target - current) / x_counts_per_mm);
    x_feed_rate_mmpm = THEROBOT->get_feed_rate(); // mm/min
    x_arm_time_us = us_ticker_read();

    // Compute constant stepping rate from feed rate: F(mm/min) -> steps_per_tick (2.62 fixed-point)
    float steps_per_sec = (x_feed_rate_mmpm / 60.0f) * x_stepper->get_steps_per_mm();
    float tick_freq = THEKERNEL->step_ticker->get_frequency();
    x_stepper->encoder_steps_per_tick = (int64_t)round(((double)steps_per_sec / (double)tick_freq) * (double)STEPTICKER_FPSCALE);

    // Set stepper encoder fields for step ticker polling
    x_stepper->encoder_target = target;
    x_stepper->set_encoder_check_gte(target >= current);

    // Also arm OC hardware interrupt for fast response at exact count
    TIM2->CCR3 = (uint32_t)target;
    TIM2->SR = ~TIM_SR_CC3IF;
    TIM2->DIER |= TIM_DIER_CC3IE;
    x_stepper->set_encoder_controlled(true);
    x_move_armed = true;
}

void Encoder::arm_y_target(int32_t target)
{
    int32_t current = get_y_count();
    y_move_distance_mm = fabsf((float)(target - current) / y_counts_per_mm);
    y_feed_rate_mmpm = THEROBOT->get_feed_rate(); // mm/min
    y_arm_time_us = us_ticker_read();

    // Compute constant stepping rate from feed rate: F(mm/min) -> steps_per_tick (2.62 fixed-point)
    float steps_per_sec = (y_feed_rate_mmpm / 60.0f) * y_stepper->get_steps_per_mm();
    float tick_freq = THEKERNEL->step_ticker->get_frequency();
    y_stepper->encoder_steps_per_tick = (int64_t)round(((double)steps_per_sec / (double)tick_freq) * (double)STEPTICKER_FPSCALE);

    // Set stepper encoder fields for step ticker polling
    y_stepper->encoder_target = target;
    y_stepper->set_encoder_check_gte(target >= current);

    // Also arm OC hardware interrupt for fast response at exact count
    TIM5->CCR3 = (uint32_t)target;
    TIM5->SR = ~TIM_SR_CC3IF;
    TIM5->DIER |= TIM_DIER_CC3IE;
    y_stepper->set_encoder_controlled(true);
    y_move_armed = true;
}

void Encoder::disarm_x()
{
    TIM2->DIER &= ~TIM_DIER_CC3IE;
    if (x_stepper) {
        x_stepper->stop_moving();
        x_stepper->set_encoder_controlled(false);
        x_stepper->set_encoder_segment_mode(false);
    }
    x_move_armed = false;
}

void Encoder::disarm_y()
{
    TIM5->DIER &= ~TIM_DIER_CC3IE;
    if (y_stepper) {
        y_stepper->stop_moving();
        y_stepper->set_encoder_controlled(false);
        y_stepper->set_encoder_segment_mode(false);
    }
    y_move_armed = false;
}

int32_t Encoder::get_x_count()
{
    return (int32_t)TIM2->CNT;
}

int32_t Encoder::get_y_count()
{
    return (int32_t)TIM5->CNT;
}

void Encoder::set_x_count(int32_t count)
{
    TIM2->CNT = (uint32_t)count;
}

void Encoder::set_y_count(int32_t count)
{
    TIM5->CNT = (uint32_t)count;
}

void Encoder::on_idle(void *argument)
{
    // Step ticker polling fallback: if the step ticker detected that an encoder
    // target was reached in segment mode, it stopped the motor and set
    // encoder_target_hit. Pick that up here and advance the segment state machine.
    if (segment_mode) {
        bool advanced = false;
        if (x_stepper->encoder_target_hit) {
            x_stepper->encoder_target_hit = false;
            if (!x_segment_done) {
                x_segment_done = true;
                advanced = true;
            }
        }
        if (y_stepper->encoder_target_hit) {
            y_stepper->encoder_target_hit = false;
            if (!y_segment_done) {
                y_segment_done = true;
                advanced = true;
            }
        }
        if (advanced && x_segment_done && y_segment_done) {
            try_advance_segment();
        }
    }

    // Report segment transitions as they happen
    if (segment_mode || segments_complete) {
        int cur = current_segment;
        while (last_reported_segment < cur - 1) {
            // A completed segment: last_reported_segment+1 has completed_at set
            int i = last_reported_segment + 1;
            uint32_t dur = segments[i].completed_at - segments[i].armed_at;
            THEKERNEL->streams->printf("s%d: dur=%lu xe=%ld/%ld ye=%ld/%ld %c%c\n",
                i, dur,
                segments[i].x_enc_at_arm, segments[i].x_target,
                segments[i].y_enc_at_arm, segments[i].y_target,
                segments[i].x_skipped ? 'X' : '.',
                segments[i].y_skipped ? 'Y' : '.');
            last_reported_segment = i;
        }
    }

    if (segments_complete) {
        // Report the final segment
        int i = segment_count - 1;
        uint32_t dur = segments[i].completed_at - segments[i].armed_at;
        THEKERNEL->streams->printf("s%d: dur=%lu xe=%ld/%ld ye=%ld/%ld %c%c\n",
            i, dur,
            segments[i].x_enc_at_arm, segments[i].x_target,
            segments[i].y_enc_at_arm, segments[i].y_target,
            segments[i].x_skipped ? 'X' : '.',
            segments[i].y_skipped ? 'Y' : '.');
        THEKERNEL->streams->printf("seg complete: total=%lu us\n",
            segments[i].completed_at - segments[0].armed_at);
        segments_complete = false;

        // Discard planner blocks that were queued during buffering (we bypassed
        // them entirely — encoder segments drove the motors directly).
        // Release the held queue so the conveyor resumes normal operation.
        THECONVEYOR->discard_queue();
        THECONVEYOR->release_queue();
    }

    // Periodic status: during segment execution or armed single moves (not during buffering — serial conflict)
    if (segment_mode || (!buffering && (x_move_armed || y_move_armed))) {
        static uint32_t last_status_time = 0;
        uint32_t now = us_ticker_read();
        if (now - last_status_time > 200000) { // every 200ms
            last_status_time = now;
            int32_t ex = get_x_count();
            int32_t ey = get_y_count();
            float sx = THEROBOT->actuators[0]->get_current_position();
            float sy = THEROBOT->actuators[1]->get_current_position();
            if (segment_mode) {
                int cs = current_segment;
                THEKERNEL->streams->printf("st: enc %ld,%ld seg=%d/%d xt=%ld yt=%ld t=%lu\n",
                    ex, ey, cs, segment_count,
                    segments[cs].x_target, segments[cs].y_target, now);
            } else if (buffering) {
                THEKERNEL->streams->printf("st: enc %ld,%ld step %.2f,%.2f BUF %d/%d t=%lu\n",
                    ex, ey, sx, sy, segments_received, segment_count, now);
            } else {
                THEKERNEL->streams->printf("st: enc %ld,%ld step %.2f,%.2f feed=%.0f t=%lu\n",
                    ex, ey, sx, sy, THEROBOT->get_feed_rate(), now);
            }
        }

        // Safety timeout only during active segment execution
        if (segment_mode && x_move_armed) {
            uint32_t elapsed = now - segments[0].armed_at;
            if (elapsed > 10000000) { // 10 seconds
                THEKERNEL->streams->printf("error: segment safety timeout 10s, seg=%d/%d enc x=%ld y=%ld\n",
                    current_segment, segment_count, get_x_count(), get_y_count());
                disarm_x();
                disarm_y();
                segment_mode = false;
                buffering = false;
                THECONVEYOR->release_queue();
                THEKERNEL->call_event(ON_HALT, nullptr);
            }
        }
    }

    // Timeout checks for non-segment single moves only
    if (x_move_armed && !segment_mode) {
        if (!x_stepper->is_moving() && !x_stepper->is_encoder_controlled()) {
            x_move_armed = false;
        } else if (x_stepper->is_moving()) {
            uint32_t elapsed_us = us_ticker_read() - x_arm_time_us;
            uint32_t timeout = compute_single_timeout_x();
            if (elapsed_us > timeout) {
                THEKERNEL->streams->printf("error: encoder X move timeout (single)\n");
                disarm_x();
                THEKERNEL->call_event(ON_HALT, nullptr);
            }
        }
    }

    if (y_move_armed && !segment_mode) {
        if (!y_stepper->is_moving() && !y_stepper->is_encoder_controlled()) {
            y_move_armed = false;
        } else if (y_stepper->is_moving()) {
            uint32_t elapsed_us = us_ticker_read() - y_arm_time_us;
            uint32_t timeout = compute_single_timeout_y();
            if (elapsed_us > timeout) {
                THEKERNEL->streams->printf("error: encoder Y move timeout (single)\n");
                disarm_y();
                THEKERNEL->call_event(ON_HALT, nullptr);
            }
        }
    }
}

void Encoder::on_halt(void *argument)
{
    if (argument == nullptr) {
        // Entering halt: clean up all encoder-controlled state
        if (x_move_armed) disarm_x();
        if (y_move_armed) disarm_y();
        if (segment_mode || buffering) {
            segment_mode = false;
            buffering = false;
            THECONVEYOR->release_queue();
        }
    }
}

void Encoder::report_encoder_position(Gcode *gcode)
{
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "EX:%ld EY:%ld", get_x_count(), get_y_count());
    gcode->txt_after_ok.append(buf, n);
}

void Encoder::report_stepper_position(Gcode *gcode)
{
    int32_t sx = THEKERNEL->robot->actuators[0]->get_current_step();
    int32_t sy = THEKERNEL->robot->actuators[1]->get_current_step();
    char buf[40];
    int n = snprintf(buf, sizeof(buf), "SX:%ld SY:%ld", sx, sy);
    gcode->txt_after_ok.append(buf, n);
}

void Encoder::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (gcode->has_g && (gcode->g == 0 || gcode->g == 1)) {
        // Encoder-driven position control: only when calibrated
        if (x_counts_per_mm == 0 || y_counts_per_mm == 0) return;

        int32_t x_target = (int32_t)((THEROBOT->get_axis_position(X_AXIS) - x_encoder_offset) * x_counts_per_mm);
        int32_t y_target = (int32_t)((THEROBOT->get_axis_position(Y_AXIS) - y_encoder_offset) * y_counts_per_mm);
        bool has_x = gcode->has_letter('X');
        bool has_y = gcode->has_letter('Y');

        if (buffering) {
            // M920 segment buffering: store target instead of arming
            if (segments_received < segment_count) {
                segments[segments_received].x_target = x_target;
                segments[segments_received].y_target = y_target;
                segments[segments_received].feed_rate = THEROBOT->get_feed_rate(); // mm/min
                segments[segments_received].has_x = has_x;
                segments[segments_received].has_y = has_y;
                segments[segments_received].timeout_us = 0; // computed below once all segments arrive
                segments_received++;

                if (segments_received == segment_count) {
                    // All segments received — precompute timeouts before any movement starts
                    precompute_segment_timeouts();

                    // Summary: first and last segment targets (full dump via M925)
                    int last = segment_count - 1;
                    THEKERNEL->streams->printf("seg recv: %d enc x=%ld y=%ld s0:xt=%ld,yt=%ld s%d:xt=%ld,yt=%ld t=%lu\n",
                        segment_count, get_x_count(), get_y_count(),
                        segments[0].x_target, segments[0].y_target,
                        last, segments[last].x_target, segments[last].y_target,
                        us_ticker_read());

                    // Start execution — queue stays held so planner blocks
                    // don't interfere. We step independently via encoder_segment_mode
                    // in the step ticker. Queue is flushed when segments complete.
                    buffering = false;
                    segment_mode = true;
                    current_segment = 0;
                    THEKERNEL->streams->printf("arm seg 0 t=%lu\n", us_ticker_read());
                    arm_segment(0);
                }
            }
        } else if (!segment_mode) {
            // Normal single-move mode
            if (has_x) arm_x_target(x_target);
            if (has_y) arm_y_target(y_target);
        }
        return;
    }

    if (gcode->has_m) {
        switch (gcode->m) {
            case 918: // report encoder positions
                report_encoder_position(gcode);
                break;

            case 919: { // set encoder counters
                if (gcode->has_letter('X')) {
                    int32_t val = (int32_t)gcode->get_value('X');
                    set_x_count(val);
                    if (x_counts_per_mm != 0)
                        x_encoder_offset = THEROBOT->get_axis_position(X_AXIS) - (float)val / x_counts_per_mm;
                    else
                        x_encoder_offset = THEROBOT->get_axis_position(X_AXIS);
                }
                if (gcode->has_letter('Y')) {
                    int32_t val = (int32_t)gcode->get_value('Y');
                    set_y_count(val);
                    if (y_counts_per_mm != 0)
                        y_encoder_offset = THEROBOT->get_axis_position(Y_AXIS) - (float)val / y_counts_per_mm;
                    else
                        y_encoder_offset = THEROBOT->get_axis_position(Y_AXIS);
                }
                THEKERNEL->streams->printf("M919: enc x=%ld y=%ld step x=%.2f y=%.2f t=%lu\n",
                    get_x_count(), get_y_count(),
                    THEROBOT->actuators[0]->get_current_position(),
                    THEROBOT->actuators[1]->get_current_position(),
                    us_ticker_read());
                break;
            }

            case 920: { // buffer N segments before executing
                if (buffering || segment_mode) {
                    gcode->stream->printf("error: segment mode already active\n");
                    break;
                }
                int count = 0;
                if (gcode->has_letter('S')) count = (int)gcode->get_value('S');
                if (count < 1 || count > MAX_ENCODER_SEGMENTS) {
                    gcode->stream->printf("error: S must be 1-%d\n", MAX_ENCODER_SEGMENTS);
                    break;
                }
                if (x_counts_per_mm == 0 || y_counts_per_mm == 0) {
                    gcode->stream->printf("error: encoder not calibrated\n");
                    break;
                }
                segment_count = count;
                segments_received = 0;
                last_reported_segment = -1;
                buffering = true;
                THECONVEYOR->hold_queue();
                THEKERNEL->streams->printf("M920: buf %d enc x=%ld y=%ld t=%lu\n",
                    count, get_x_count(), get_y_count(), us_ticker_read());
                break;
            }

            case 921: // report stepper step counts
                report_stepper_position(gcode);
                break;

            case 922: // set stepper step counters (only when idle)
                if (!THEKERNEL->conveyor->is_idle()) {
                    gcode->stream->printf("error: machine is moving\n");
                } else {
                    if (gcode->has_letter('X')) {
                        int32_t steps = (int32_t)gcode->get_value('X');
                        float mm = (float)steps / THEROBOT->actuators[0]->get_steps_per_mm();
                        THEROBOT->actuators[0]->set_last_milestones(mm, steps);
                    }
                    if (gcode->has_letter('Y')) {
                        int32_t steps = (int32_t)gcode->get_value('Y');
                        float mm = (float)steps / THEROBOT->actuators[1]->get_steps_per_mm();
                        THEROBOT->actuators[1]->set_last_milestones(mm, steps);
                    }
                }
                break;

            case 923: { // set encoder counts per mm
                if (gcode->has_letter('X')) x_counts_per_mm = gcode->get_value('X');
                if (gcode->has_letter('Y')) y_counts_per_mm = gcode->get_value('Y');
                // Synchronize: zero encoders and set offsets to current position
                set_x_count(0);
                set_y_count(0);
                x_encoder_offset = THEROBOT->get_axis_position(X_AXIS);
                y_encoder_offset = THEROBOT->get_axis_position(Y_AXIS);
                char buf[40];
                int n = snprintf(buf, sizeof(buf), "X:%.4f Y:%.4f", x_counts_per_mm, y_counts_per_mm);
                gcode->txt_after_ok.append(buf, n);
                break;
            }

            case 925: { // report encoder debug state
                uint32_t now = us_ticker_read();
                gcode->stream->printf("X: oc=%lu poll=%lu armed=%d enc_ctrl=%d gte=%d target=%ld count=%ld\n",
                    dbg_x_oc_count, x_stepper->encoder_poll_hits,
                    (int)x_move_armed, (int)x_stepper->is_encoder_controlled(),
                    (int)x_stepper->get_encoder_check_gte(),
                    (int32_t)x_stepper->encoder_target, get_x_count());
                if (x_move_armed) {
                    uint32_t tmo = segment_mode ? x_timeout_us : compute_single_timeout_x();
                    gcode->stream->printf("X: arm=%lu now=%lu elapsed=%lu timeout=%lu seg_mode=%d\n",
                        x_arm_time_us, now, now - x_arm_time_us, tmo, (int)segment_mode);
                }
                gcode->stream->printf("Y: oc=%lu poll=%lu armed=%d enc_ctrl=%d gte=%d target=%ld count=%ld\n",
                    dbg_y_oc_count, y_stepper->encoder_poll_hits,
                    (int)y_move_armed, (int)y_stepper->is_encoder_controlled(),
                    (int)y_stepper->get_encoder_check_gte(),
                    (int32_t)y_stepper->encoder_target, get_y_count());
                if (y_move_armed) {
                    uint32_t tmo = segment_mode ? y_timeout_us : compute_single_timeout_y();
                    gcode->stream->printf("Y: arm=%lu now=%lu elapsed=%lu timeout=%lu seg_mode=%d\n",
                        y_arm_time_us, now, now - y_arm_time_us, tmo, (int)segment_mode);
                }
                break;
            }

            case 924: { // auto-calibrate encoder counts per mm
                if (!THEKERNEL->conveyor->is_idle()) {
                    gcode->stream->printf("error: machine is moving\n");
                    break;
                }
                float cal_distance = 0;
                if (gcode->has_letter('D')) {
                    cal_distance = gcode->get_value('D');
                } else {
                    float x_travel = THEKERNEL->config->value(alpha_max_travel_checksum)->by_default(500)->as_number();
                    float y_travel = THEKERNEL->config->value(beta_max_travel_checksum)->by_default(500)->as_number();
                    cal_distance = (x_travel < y_travel ? x_travel : y_travel) / 2.0f;
                }
                auto_calibrate(gcode, cal_distance);
                break;
            }
        }
    }
}

void Encoder::auto_calibrate(Gcode *gcode, float distance)
{
    // Zero encoder counts at current (home) position
    set_x_count(0);
    set_y_count(0);

    float cal_speed = 10.0f; // mm/s — slow for accuracy, minimizes lost steps

    THEROBOT->push_state();

    // Move away from home (positive direction from home_to_min)
    float delta[3] = {distance, distance, 0};
    THEROBOT->delta_move(delta, cal_speed, 3);
    THECONVEYOR->wait_for_idle();

    // Capture encoder counts after calibration move
    int32_t ex = get_x_count();
    int32_t ey = get_y_count();

    // Move back to starting position
    float delta_back[3] = {-distance, -distance, 0};
    THEROBOT->delta_move(delta_back, cal_speed, 3);
    THECONVEYOR->wait_for_idle();

    THEROBOT->pop_state();

    // counts_per_mm = encoder_counts / position_change
    // Signed result captures encoder polarity
    x_counts_per_mm = (float)ex / distance;
    y_counts_per_mm = (float)ey / distance;

    // Set encoder offset to current position (we're back at home)
    x_encoder_offset = THEROBOT->get_axis_position(X_AXIS);
    y_encoder_offset = THEROBOT->get_axis_position(Y_AXIS);

    // Re-zero encoders at home position
    set_x_count(0);
    set_y_count(0);

    char buf[60];
    int n = snprintf(buf, sizeof(buf), "EX:%ld EY:%ld X:%.4f Y:%.4f", ex, ey, x_counts_per_mm, y_counts_per_mm);
    gcode->txt_after_ok.append(buf, n);
}
