/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "Module.h"
#include "Pin.h"

class StepperMotor  : public Module {
    public:
        StepperMotor(Pin& step, Pin& dir, Pin& en);
        ~StepperMotor();

        void set_motor_id(uint8_t id) { motor_id= id; }
        uint8_t get_motor_id() const { return motor_id; }

        // called from step ticker ISR
        inline bool step() { step_pin.set(1); current_position_steps += (direction?-1:1); return moving; }
        // called from unstep ISR
        inline void unstep() { step_pin.set(0); }
        // called from step ticker ISR
        inline void set_direction(bool f) { dir_pin.set(f); direction= f; }

        void enable(bool state) { en_pin.set(!state); };
        bool is_enabled() const { return !en_pin.get(); };
        bool is_moving() const { return moving; };
        void start_moving() { moving= true; }
        void stop_moving() { moving= false; }

        void manual_step(bool dir);

        bool which_direction() const { return direction; }

        float get_steps_per_second()  const { return steps_per_second; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        void set_last_milestones(float, int32_t);
        void update_last_milestones(float mm, int32_t steps);
        float get_last_milestone(void) const { return last_milestone_mm; }
        int32_t get_last_milestone_steps(void) const { return last_milestone_steps; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }
        uint32_t get_current_step(void) const { return current_position_steps; }
        float get_max_rate(void) const { return max_rate; }
        void set_max_rate(float mr) { max_rate= mr; }
        void set_acceleration(float a) { acceleration= a; }
        float get_acceleration() const { return acceleration; }
        bool is_selected() const { return selected; }
        void set_selected(bool b) { selected= b; }
        bool is_extruder() const { return extruder; }
        void set_extruder(bool b) { extruder= b; }
        bool is_encoder_controlled() const { return encoder_controlled; }
        void set_encoder_controlled(bool b) { encoder_controlled= b; }
        void set_encoder_check_gte(bool b) { encoder_check_gte= b; }
        bool get_encoder_check_gte() const { return encoder_check_gte; }
        bool is_encoder_segment_mode() const { return encoder_segment_mode; }
        void set_encoder_segment_mode(bool b) { encoder_segment_mode= b; }

        // Encoder position control (set by Encoder module, polled by step ticker ISR)
        inline bool encoder_target_reached() const {
            if(encoder_cnt_reg == nullptr) return false;
            int32_t count = (int32_t)*encoder_cnt_reg;
            return encoder_check_gte ? (count >= encoder_target) : (count <= encoder_target);
        }

        volatile uint32_t *encoder_cnt_reg;     // pointer to TIMx->CNT, nullptr if no encoder
        volatile int32_t encoder_target;         // target encoder count for current move
        volatile int64_t encoder_steps_per_tick; // constant stepping rate for encoder mode (2.62 fixed-point)
        volatile uint32_t encoder_poll_hits;     // debug: times step ticker polling caught target
        volatile bool encoder_target_hit;        // set by step ticker polling when target reached in segment mode
        volatile int64_t encoder_step_counter;   // 2.62 fixed point accumulator for block-free stepping

        int32_t steps_to_target(float);


    private:
        void on_halt(void *argument);
        void on_enable(void *argument);

        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        float steps_per_second;
        float steps_per_mm;
        float max_rate; // this is not really rate it is in mm/sec, misnamed used in Robot and Extruder
        float acceleration;

        volatile int32_t current_position_steps;
        int32_t last_milestone_steps;
        float   last_milestone_mm;

        volatile struct {
            uint8_t motor_id:8;
            volatile bool direction:1;
            volatile bool moving:1;
            bool selected:1;
            bool extruder:1;
            bool encoder_controlled:1;
            bool encoder_check_gte:1;    // true: count >= target, false: count <= target
            bool encoder_segment_mode:1; // true: OC ISR handles completion, step ticker hands off
        };
};

