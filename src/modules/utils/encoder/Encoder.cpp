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
#include "Conveyor.h"

#include "stm32f4xx_hal.h"
#include "mbed.h" // for us_ticker_read()
#include <math.h>

#define encoder_enable_checksum          CHECKSUM("encoder_enable")
#define encoder_x_counts_per_mm_checksum CHECKSUM("encoder_x_counts_per_mm")
#define encoder_y_counts_per_mm_checksum CHECKSUM("encoder_y_counts_per_mm")
#define alpha_max_travel_checksum        CHECKSUM("alpha_max_travel")
#define beta_max_travel_checksum         CHECKSUM("beta_max_travel")

// Timeout: 2 seconds per mm of travel, plus a 2-second floor for short moves.
// This corresponds to a minimum expected speed of 0.5 mm/s — anything slower
// than that is almost certainly stalled.
#define TIMEOUT_US_PER_MM  2000000
#define TIMEOUT_FLOOR_US   2000000

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
    // Stop motors to trigger block transition in StepTicker.
    if (x_stepper) x_stepper->stop_moving();
    if (y_stepper) y_stepper->stop_moving();

    int next = current_segment + 1;
    if (next < segment_count) {
        current_segment = next;
        arm_segment(next);
    } else {
        // All segments complete
        if (x_stepper) x_stepper->set_encoder_controlled(false);
        if (y_stepper) y_stepper->set_encoder_controlled(false);
        segment_mode = false;
        x_move_armed = false;
        y_move_armed = false;
    }
}

void Encoder::arm_segment(int index)
{
    // Pre-set done flags for axes that don't move in this segment
    x_segment_done = !segments[index].has_x;
    y_segment_done = !segments[index].has_y;

    if (segments[index].has_x) {
        TIM2->CCR3 = (uint32_t)segments[index].x_target;
        TIM2->SR = ~TIM_SR_CC3IF;
        TIM2->DIER |= TIM_DIER_CC3IE;
        x_stepper->set_encoder_controlled(true);
    }

    if (segments[index].has_y) {
        TIM5->CCR3 = (uint32_t)segments[index].y_target;
        TIM5->SR = ~TIM_SR_CC3IF;
        TIM5->DIER |= TIM_DIER_CC3IE;
        y_stepper->set_encoder_controlled(true);
    }

    // Reset timeout for this segment
    float dist_x = segments[index].has_x ? fabsf((float)(segments[index].x_target - get_x_count()) / x_counts_per_mm) : 0;
    float dist_y = segments[index].has_y ? fabsf((float)(segments[index].y_target - get_y_count()) / y_counts_per_mm) : 0;
    float dist = dist_x > dist_y ? dist_x : dist_y;
    x_move_distance_mm = dist;
    y_move_distance_mm = dist;
    x_arm_time_us = us_ticker_read();
    y_arm_time_us = x_arm_time_us;
    x_move_armed = true;
    y_move_armed = true;

    // Edge case: neither axis moves (shouldn't happen in practice)
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
    y_arm_time_us = 0;
    x_move_distance_mm = 0;
    y_move_distance_mm = 0;
    segment_mode = false;
    buffering = false;
    segment_count = 0;
    segments_received = 0;
    current_segment = 0;
    x_segment_done = false;
    y_segment_done = false;
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

void Encoder::arm_x_target(int32_t target)
{
    x_move_distance_mm = fabsf((float)(target - get_x_count()) / x_counts_per_mm);
    x_arm_time_us = us_ticker_read();

    TIM2->CCR3 = (uint32_t)target;
    TIM2->SR = ~TIM_SR_CC3IF;
    TIM2->DIER |= TIM_DIER_CC3IE;
    x_stepper->set_encoder_controlled(true);
    x_move_armed = true;
}

void Encoder::arm_y_target(int32_t target)
{
    y_move_distance_mm = fabsf((float)(target - get_y_count()) / y_counts_per_mm);
    y_arm_time_us = us_ticker_read();

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
    }
    x_move_armed = false;
}

void Encoder::disarm_y()
{
    TIM5->DIER &= ~TIM_DIER_CC3IE;
    if (y_stepper) {
        y_stepper->stop_moving();
        y_stepper->set_encoder_controlled(false);
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
    // Check for move timeouts on armed encoder-controlled axes
    if (x_move_armed) {
        if (!x_stepper->is_moving() && !segment_mode) {
            // OC fired normally, just clear armed state
            x_move_armed = false;
        } else if (x_stepper->is_moving()) {
            uint32_t elapsed_us = us_ticker_read() - x_arm_time_us;
            uint32_t timeout_us = TIMEOUT_FLOOR_US + (uint32_t)(x_move_distance_mm * TIMEOUT_US_PER_MM);
            if (elapsed_us > timeout_us) {
                disarm_x();
                if (segment_mode) {
                    disarm_y();
                    segment_mode = false;
                    buffering = false;
                    THECONVEYOR->release_queue();
                }
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->streams->printf("error: encoder X move timeout\n");
            }
        }
    }

    if (y_move_armed) {
        if (!y_stepper->is_moving() && !segment_mode) {
            y_move_armed = false;
        } else if (y_stepper->is_moving()) {
            uint32_t elapsed_us = us_ticker_read() - y_arm_time_us;
            uint32_t timeout_us = TIMEOUT_FLOOR_US + (uint32_t)(y_move_distance_mm * TIMEOUT_US_PER_MM);
            if (elapsed_us > timeout_us) {
                disarm_y();
                if (segment_mode) {
                    disarm_x();
                    segment_mode = false;
                    buffering = false;
                    THECONVEYOR->release_queue();
                }
                THEKERNEL->call_event(ON_HALT, nullptr);
                THEKERNEL->streams->printf("error: encoder Y move timeout\n");
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
                segments[segments_received].has_x = has_x;
                segments[segments_received].has_y = has_y;
                segments_received++;

                if (segments_received == segment_count) {
                    // All segments received — start execution
                    buffering = false;
                    segment_mode = true;
                    current_segment = 0;
                    arm_segment(0);
                    THECONVEYOR->release_queue();
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
                buffering = true;
                THECONVEYOR->hold_queue();
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
