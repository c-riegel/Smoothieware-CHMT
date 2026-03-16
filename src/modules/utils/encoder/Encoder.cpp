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

// Strip a letter parameter and its numeric value from a G-code command string in-place.
// E.g. strip_gcode_letter(cmd, 'X') turns "G1X265.174Y73.707F600" into "G1Y73.707F600".
static void strip_gcode_letter(char* cmd, char letter)
{
    char* read = cmd;
    char* write = cmd;
    while (*read) {
        if (*read == letter) {
            read++; // skip the letter
            // skip optional minus sign
            if (*read == '-') read++;
            // skip digits
            while (*read >= '0' && *read <= '9') read++;
            // skip optional decimal point + digits
            if (*read == '.') {
                read++;
                while (*read >= '0' && *read <= '9') read++;
            }
        } else {
            *write++ = *read++;
        }
    }
    *write = '\0';
}

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
    if (next < encoder_segment_count) {
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
    int32_t final_x = segments[encoder_segment_count - 1].x_target;
    int32_t final_y = segments[encoder_segment_count - 1].y_target;
    int32_t start_x = get_x_count();
    int32_t start_y = get_y_count();

    for (int i = 0; i < encoder_segment_count; i++) {
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

    // Pre-set done flags for axes that don't move in this segment
    x_segment_done = !segments[index].has_x;
    y_segment_done = !segments[index].has_y;

    // Clear polling fallback flags so stale hits don't trigger false advancement
    x_stepper->encoder_target_hit = false;
    y_stepper->encoder_target_hit = false;

    if (segments[index].has_x) {
        int32_t current_x = get_x_count();
        bool gte = segments[index].x_target >= current_x;
        x_stepper->encoder_target = segments[index].x_target;
        x_stepper->set_encoder_check_gte(gte);

        // Use precomputed per-axis stepping rate (vector-decomposed at buffer time)
        x_stepper->encoder_steps_per_tick = segments[index].x_steps_per_tick;
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

        // Use precomputed per-axis stepping rate (vector-decomposed at buffer time)
        y_stepper->encoder_steps_per_tick = segments[index].y_steps_per_tick;
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
    encoder_segment_count = 0;
    encoder_segments_received = 0;
    current_segment = 0;
    x_segment_done = false;
    y_segment_done = false;
    segments_complete = false;
    segments_done_at = 0;
    last_reported_segment = -1;
    pending_feed_rate = 0;
    pending_acceleration = 0;
    dbg_x_oc_count = 0;
    dbg_y_oc_count = 0;
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

    // Encoder must process ON_GCODE_RECEIVED BEFORE Robot so it can strip
    // X/Y from G1 commands during segment buffering. Robot registered first
    // (in Kernel init), so we unregister it, register Encoder, then re-register
    // Robot — putting Encoder ahead in the dispatch order.
    THEKERNEL->unregister_for_event(ON_GCODE_RECEIVED, THEKERNEL->robot);
    this->register_for_event(ON_GCODE_RECEIVED);
    THEKERNEL->register_for_event(ON_GCODE_RECEIVED, THEKERNEL->robot);

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

    if (segments_complete) {
        int last = encoder_segment_count - 1;
        THEKERNEL->streams->printf("seg complete: %d segs, total=%lu us, enc x=%ld y=%ld\n",
            encoder_segment_count,
            segments[last].completed_at - segments[0].armed_at,
            get_x_count(), get_y_count());
        segments_complete = false;

        // Position sync handled by continuous sync in on_idle (below).
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
                // No printf during segment execution — real-time motor control
                // must not be delayed. As-executed data is cached in the segment
                // struct (armed_at, completed_at, x_enc_at_arm, y_enc_at_arm)
                // and dumped after segments complete.
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
                    current_segment, encoder_segment_count, get_x_count(), get_y_count());
                disarm_x();
                disarm_y();
                segment_mode = false;
                buffering = false;
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
                THEKERNEL->streams->printf("error: encoder X move timeout (single) enc=%ld target=%ld delta=%.4fmm elapsed=%luus\n",
                    get_x_count(), x_stepper->encoder_target, (float)(x_stepper->encoder_target - get_x_count()) / x_counts_per_mm, elapsed_us);
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
                THEKERNEL->streams->printf("error: encoder Y move timeout (single) enc=%ld target=%ld delta=%.4fmm elapsed=%luus\n",
                    get_y_count(), y_stepper->encoder_target, (float)(y_stepper->encoder_target - get_y_count()) / y_counts_per_mm, elapsed_us);
                disarm_y();
                THEKERNEL->call_event(ON_HALT, nullptr);
            }
        }
    }

    // Continuous encoder sync: whenever the conveyor is idle and no encoder
    // segments are running, keep Robot's position matched to encoder reality.
    // This ensures correct position after ANY move (planner, degraded, legacy).
    if (!segment_mode && !buffering && x_counts_per_mm != 0 && THECONVEYOR->is_idle()) {
        float actual_x = (float)get_x_count() / x_counts_per_mm + x_encoder_offset;
        float actual_y = (float)get_y_count() / y_counts_per_mm + y_encoder_offset;
        float current_z = THEROBOT->get_axis_position(Z_AXIS);
        THEROBOT->reset_axis_position(actual_x, actual_y, current_z);
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

        bool has_x = gcode->has_letter('X');
        bool has_y = gcode->has_letter('Y');

        if (buffering) {
            // M920 segment buffering mode.
            // If this G1 has X or Y, buffer encoder data and strip X/Y from
            // the gcode so the planner only creates blocks for Z/A/B/C/D.
            // If no X or Y, skip buffering — let it pass through entirely.

            if ((has_x || has_y) && encoder_segments_received < MAX_ENCODER_SEGMENTS) {
                // Compute encoder targets from G-code commanded position (not Robot's
                // current position, since Encoder runs before Robot in dispatch order).
                int32_t x_target = has_x ? (int32_t)((gcode->get_value('X') - x_encoder_offset) * x_counts_per_mm) :
                    (encoder_segments_received > 0 ? segments[encoder_segments_received - 1].x_target : get_x_count());
                int32_t y_target = has_y ? (int32_t)((gcode->get_value('Y') - y_encoder_offset) * y_counts_per_mm) :
                    (encoder_segments_received > 0 ? segments[encoder_segments_received - 1].y_target : get_y_count());

                // Debug printf removed — blocking serial TX during segment buffering
                // causes incoming character drops. Re-enable only with DMA TX.

                // Per-axis minimum delta check: if the move on an axis is smaller
                // than MIN_ENCODER_DELTA counts, don't encoder-control that axis.
                int32_t prev_x = (encoder_segments_received > 0) ? segments[encoder_segments_received - 1].x_target : get_x_count();
                int32_t prev_y = (encoder_segments_received > 0) ? segments[encoder_segments_received - 1].y_target : get_y_count();
                bool enc_x = has_x && (abs(x_target - prev_x) >= MIN_ENCODER_DELTA);
                bool enc_y = has_y && (abs(y_target - prev_y) >= MIN_ENCODER_DELTA);

                if (enc_x || enc_y) {
                    // Capture F directly from G-code line (not Robot's modal state)
                    if (gcode->has_letter('F'))
                        pending_feed_rate = gcode->get_value('F');

                    segments[encoder_segments_received].x_target = x_target;
                    segments[encoder_segments_received].y_target = y_target;
                    segments[encoder_segments_received].feed_rate = pending_feed_rate;
                    segments[encoder_segments_received].acceleration = pending_acceleration;
                    segments[encoder_segments_received].has_x = enc_x;
                    segments[encoder_segments_received].has_y = enc_y;
                    segments[encoder_segments_received].timeout_us = 0;

                    // Precompute per-axis stepping rates from vector-decomposed feed rate.
                    float tick_freq = THEKERNEL->step_ticker->get_frequency();
                    float dx_mm = enc_x ? (float)(x_target - prev_x) / x_counts_per_mm : 0;
                    float dy_mm = enc_y ? (float)(y_target - prev_y) / y_counts_per_mm : 0;
                    float dist = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);
                    float feed_mmps = pending_feed_rate / 60.0f;

                    if (dist > 0.001f) {
                        float x_speed = feed_mmps * fabsf(dx_mm) / dist;
                        float y_speed = feed_mmps * fabsf(dy_mm) / dist;
                        // Safety clamp to axis max rate
                        float x_max = x_stepper->get_max_rate();
                        float y_max = y_stepper->get_max_rate();
                        if (x_speed > x_max) x_speed = x_max;
                        if (y_speed > y_max) y_speed = y_max;
                        // Safety clamp velocity change between consecutive segments
                        if (encoder_segments_received > 0) {
                            // Recover previous speed from stored steps_per_tick
                            float prev_xr = (float)((double)segments[encoder_segments_received - 1].x_steps_per_tick / (double)STEPTICKER_FPSCALE * (double)tick_freq / (double)x_stepper->get_steps_per_mm());
                            float prev_yr = (float)((double)segments[encoder_segments_received - 1].y_steps_per_tick / (double)STEPTICKER_FPSCALE * (double)tick_freq / (double)y_stepper->get_steps_per_mm());
                            bool clamped = false;
                            if (x_speed > prev_xr + MAX_STEP_VELOCITY_CHANGE) { x_speed = prev_xr + MAX_STEP_VELOCITY_CHANGE; clamped = true; }
                            if (y_speed > prev_yr + MAX_STEP_VELOCITY_CHANGE) { y_speed = prev_yr + MAX_STEP_VELOCITY_CHANGE; clamped = true; }
                            if (clamped) {
                                // dv clamp fired (silent — printf during buffering causes crashes)
                            }
                        }
                        segments[encoder_segments_received].x_steps_per_tick = (int64_t)round(((double)(x_speed * x_stepper->get_steps_per_mm()) / (double)tick_freq) * (double)STEPTICKER_FPSCALE);
                        segments[encoder_segments_received].y_steps_per_tick = (int64_t)round(((double)(y_speed * y_stepper->get_steps_per_mm()) / (double)tick_freq) * (double)STEPTICKER_FPSCALE);
                    } else {
                        segments[encoder_segments_received].x_steps_per_tick = 0;
                        segments[encoder_segments_received].y_steps_per_tick = 0;
                    }

                    encoder_segments_received++;
                }

                // ALWAYS strip X/Y during M920 buffering — the planner must never
                // create X/Y steps for encoder-backed axes, regardless of whether
                // the segment was above or below the encoder threshold.
                if (has_x) strip_gcode_letter(const_cast<char*>(gcode->get_command()), 'X');
                if (has_y) strip_gcode_letter(const_cast<char*>(gcode->get_command()), 'Y');
                // else: both axes below threshold — let planner handle entirely
            }

            segments_received++;

            if (segments_received == segment_count) {
                buffering = false;

                if (encoder_segments_received > 0) {
                    // We have encoder segments — precompute timeouts and start
                    encoder_segment_count = encoder_segments_received;
                    precompute_segment_timeouts();

                    int last = encoder_segment_count - 1;
                    // seg recv silent — printf during buffering causes crashes

                    segment_mode = true;
                    current_segment = 0;
                    // arm seg 0 silent — printf during buffering causes crashes
                    arm_segment(0);
                }
                // else: all segments were Z/A/B/C/D only — no encoder work needed.
                // Planner handles everything via the stripped G1 commands.
            }

            // DON'T return — let the (stripped) G1 pass through to Robot/Planner
            // so Z/A/B/C/D axes are handled by the normal planner.

        }
        // Single-move encoder arming disabled — all encoder control goes through
        // M920 segment batches. Non-M920 G1 commands (including degraded moves
        // from interpolation failure) use the planner only.
        // DON'T return — let G1 pass through to Robot/Planner for stepping.
        // In buffering mode, X/Y are stripped so planner handles Z/A/B/C/D only.
        // In single-move mode, planner handles all axes alongside encoder detection.
    }

    if (gcode->has_m) {
        switch (gcode->m) {
            case 400: // wait for moves — also wait for encoder segments
                if (segment_mode) {
                    while (segment_mode && !THEKERNEL->is_halted()) {
                        THEKERNEL->call_event(ON_IDLE, nullptr);
                    }
                }
                break; // let M400 pass through to Conveyor handler too

            case 204: // capture acceleration during segment buffering
                if (buffering && gcode->has_letter('S')) {
                    pending_acceleration = gcode->get_value('S');
                }
                break;

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
                    gcode->stream->printf("error: encoder not calibrated (x_cpm=%.4f y_cpm=%.4f)\n",
                        x_counts_per_mm, y_counts_per_mm);
                    break;
                }

                // Position sync: ensure Robot's position matches encoder reality
                // Position sync handled by continuous sync in on_idle.

                segment_count = count;
                segments_received = 0;
                encoder_segments_received = 0;
                encoder_segment_count = 0;
                last_reported_segment = -1;
                pending_feed_rate = THEROBOT->get_feed_rate();
                pending_acceleration = 0;
                buffering = true;
                // DO NOT hold queue — Z/A/B/C/D planner blocks must flow through
                // M920 buf silent — printf during buffering causes crashes
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
                THEKERNEL->streams->printf("M923: m=%u has_x=%d has_y=%d\n",
                    gcode->m, (int)gcode->has_letter('X'), (int)gcode->has_letter('Y'));
                if (gcode->has_letter('X')) x_counts_per_mm = gcode->get_value('X');
                if (gcode->has_letter('Y')) y_counts_per_mm = gcode->get_value('Y');
                // Synchronize: zero encoders and set offsets to current position
                set_x_count(0);
                set_y_count(0);
                x_encoder_offset = THEROBOT->get_axis_position(X_AXIS);
                y_encoder_offset = THEROBOT->get_axis_position(Y_AXIS);
                char buf[40];
                int n = snprintf(buf, sizeof(buf), "CPM:%.4f,%.4f", x_counts_per_mm, y_counts_per_mm);
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
