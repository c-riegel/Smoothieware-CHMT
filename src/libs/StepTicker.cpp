/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#include "StepTicker.h"

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "StepperMotor.h"
#include "StreamOutputPool.h"
#include "Block.h"
#include "Conveyor.h"

#include "stm32f407xx.h" // mbed.h lib
#include <math.h>
#include <mri.h>

#ifdef STEPTICKER_DEBUG_PIN
// debug pins, only used if defined in src/makefile
#include "gpio.h"
GPIO stepticker_debug_pin(STEPTICKER_DEBUG_PIN);
#define SET_STEPTICKER_DEBUG_PIN(n) {if(n) stepticker_debug_pin.set(); else stepticker_debug_pin.clear(); }
#else
#define SET_STEPTICKER_DEBUG_PIN(n)
#endif

#define TIM7_PRESCALER      15
#define TIM14_PRESCALER     1

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);
extern "C" void PendSV_Handler(void);

StepTicker *StepTicker::instance;

StepTicker::StepTicker()
{
    instance = this; // setup the Singleton instance of the stepticker

    // Configure the timer
    __TIM7_CLK_ENABLE();
    TIM7->CR1 = TIM_CR1_URS;    // int on overflow
    NVIC_SetVector(TIM7_IRQn, (uint32_t)TIM7_IRQHandler);

    __TIM14_CLK_ENABLE();
    TIM14->CR1 = TIM_CR1_URS | TIM_CR1_OPM;  // int on overflow, one-shot mode
    NVIC_SetVector(TIM8_TRG_COM_TIM14_IRQn, (uint32_t)TIM8_TRG_COM_TIM14_IRQHandler);

    NVIC_SetVector(PendSV_IRQn, (uint32_t)PendSV_Handler);

    // Default start values
    this->set_frequency(100000);
    this->set_unstep_time(5);

    this->unstep.reset();
    this->num_motors = 0;

    this->running = false;
    this->current_block = nullptr;

    #ifdef STEPTICKER_DEBUG_PIN
    // setup debug pin if defined
    stepticker_debug_pin.output();
    stepticker_debug_pin= 0;
    #endif
}

StepTicker::~StepTicker()
{
}

//called when everything is setup and interrupts can start
void StepTicker::start()
{
    TIM7->DIER = TIM_DIER_UIE;     // update interrupt en
    TIM14->DIER = TIM_DIER_UIE;     // update interrupt en
    NVIC_EnableIRQ(TIM7_IRQn);     // enable interrupt handler
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);     // enable interrupt handler

    NVIC_EnableIRQ(PendSV_IRQn);     // enable interrupt handler

    current_tick= 0;
    TIM7->CR1 |= TIM_CR1_CEN;      // start step timer
}

// Set the base stepping frequency
void StepTicker::set_frequency( float frequency )
{
    this->frequency = frequency;
    this->period = floorf((SystemCoreClock >> 1) / TIM7_PRESCALER / frequency); // SystemCoreClock/2 = Timer increments in a second

    TIM7->PSC = TIM7_PRESCALER-1;
    TIM7->ARR = this->period;
}

// Set the reset delay, must be called after set_frequency
void StepTicker::set_unstep_time( float microseconds )
{
    uint32_t delay = floorf(((SystemCoreClock >> 1) / TIM14_PRESCALER) * (microseconds / 1000000.0F)); // SystemCoreClock/2 = Timer increments in a second
    TIM14->ARR = delay;

    // TODO check that the unstep time is less than the step period, if not slow down step ticker
}

// Reset step pins on any motor that was stepped
void StepTicker::unstep_tick()
{
    for (int i = 0; i < num_motors; i++) {
        if(this->unstep[i]) {
            this->motor[i]->unstep();
        }
    }
    this->unstep.reset();
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler (void)
{
    TIM14->SR = ~TIM_SR_UIF;
    StepTicker::getInstance()->unstep_tick();
}

// The actual interrupt handler where we do all the work
extern "C" void TIM7_IRQHandler (void)
{
    // Reset interrupt register
    TIM7->SR = ~TIM_SR_UIF;
    StepTicker::getInstance()->step_tick();
}

extern "C" void PendSV_Handler(void)
{
    StepTicker::getInstance()->handle_finish();
}

// slightly lower priority than TIMER0, the whole end of block/start of block is done here allowing the timer to continue ticking
void StepTicker::handle_finish (void)
{
    // all moves finished signal block is finished
    if(finished_fnc) finished_fnc();
}

// step clock
void StepTicker::step_tick (void)
{
    //SET_STEPTICKER_DEBUG_PIN(running ? 1 : 0);

    if(THEKERNEL->is_halted()) {
        running= false;
        current_tick = 0;
        current_block= nullptr;
        return;
    }

    // Encoder segment mode: step motors independently of planner blocks.
    // OpenPnP is the motion planner — we just follow its commanded rates.
    // This runs even when no block is active (queue held during segments).
    {
        bool any_segment_motor = false;
        for (uint8_t m = 0; m < num_motors; m++) {
            if(!motor[m]->is_encoder_segment_mode()) continue;
            any_segment_motor = true;
            if(!motor[m]->is_moving()) continue;

            motor[m]->encoder_step_counter += motor[m]->encoder_steps_per_tick;
            if(motor[m]->encoder_step_counter >= STEPTICKER_FPSCALE) {
                motor[m]->encoder_step_counter -= STEPTICKER_FPSCALE;
                motor[m]->step();
                unstep.set(m);

                // Polling fallback: catch target if OC missed
                if(motor[m]->encoder_target_reached()) {
                    motor[m]->encoder_poll_hits++;
                    motor[m]->stop_moving();
                    motor[m]->encoder_target_hit = true;
                }
            }
        }
        // If any motor is in segment mode, skip normal block processing
        if(any_segment_motor) {
            if(unstep.any()) {
                TIM14->CR1 |= TIM_CR1_CEN;
            }
            return;
        }
    }

    // Normal block-based stepping below (only when no encoder segments active)
    if(!running){
        // check if anything new available
        if(THECONVEYOR->get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue
            if(!running) return;
        }else{
            return;
        }
    }

    bool still_moving= false;
    // foreach motor, if it is active see if time to issue a step to that motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue; // not active

        current_block->tick_info[m].steps_per_tick += current_block->tick_info[m].acceleration_change;

        if(current_tick == current_block->tick_info[m].next_accel_event) {
            if(current_tick == current_block->accelerate_until) { // We are done accelerating, deceleration becomes 0 : plateau
                current_block->tick_info[m].acceleration_change = 0;
                if(current_block->decelerate_after < current_block->total_move_ticks) {
                    current_block->tick_info[m].next_accel_event = current_block->decelerate_after;
                    if(current_tick != current_block->decelerate_after) { // We are plateauing
                        // steps/sec / tick frequency to get steps per tick
                        current_block->tick_info[m].steps_per_tick = current_block->tick_info[m].plateau_rate;
                    }
                }
            }

            if(current_tick == current_block->decelerate_after) { // We start decelerating
                current_block->tick_info[m].acceleration_change = current_block->tick_info[m].deceleration_change;
            }
        }

        // protect against rounding errors and such
        if(current_block->tick_info[m].steps_per_tick <= 0) {
            if(motor[m]->is_encoder_controlled()) {
                // Encoder mode: sustain plateau speed — encoder decides when done
                current_block->tick_info[m].steps_per_tick = current_block->tick_info[m].plateau_rate;
                current_block->tick_info[m].acceleration_change = 0;
            } else {
                current_block->tick_info[m].counter = STEPTICKER_FPSCALE; // we force completion this step by setting to 1.0
                current_block->tick_info[m].steps_per_tick = 0;
            }
        }

        current_block->tick_info[m].counter += current_block->tick_info[m].steps_per_tick;

        if(current_block->tick_info[m].counter >= STEPTICKER_FPSCALE) { // >= 1.0 step time
            current_block->tick_info[m].counter -= STEPTICKER_FPSCALE; // -= 1.0F;
            ++current_block->tick_info[m].step_count;

            // step the motor
            bool ismoving= motor[m]->step(); // returns false if the moving flag was set to false externally (probes, endstops etc)
            // we stepped so schedule an unstep
            unstep.set(m);

            if(motor[m]->is_encoder_controlled()) {
                if(!ismoving || motor[m]->encoder_target_reached()) {
                    if(ismoving) motor[m]->encoder_poll_hits++;
                    current_block->tick_info[m].steps_to_move = 0;
                    motor[m]->stop_moving();
                    motor[m]->set_encoder_controlled(false);
                }
            } else {
                if(!ismoving || current_block->tick_info[m].step_count == current_block->tick_info[m].steps_to_move) {
                    current_block->tick_info[m].steps_to_move = 0;
                    motor[m]->stop_moving(); // let motor know it is no longer moving
                }
            }
        }

        // see if any motors are still moving after this tick
        if(motor[m]->is_moving()) still_moving= true;
    }

    // do this after so we start at tick 0
    current_tick++; // count number of ticks

    // We may have set a pin on in this tick, now we reset the timer to set it off
    // Note there could be a race here if we run another tick before the unsteps have happened,
    // right now it takes about 3-4us but if the unstep were near 10uS or greater it would be an issue
    // also it takes at least 2us to get here so even when set to 1us pulse width it will still be about 3us
    if( unstep.any()) {
        // CEN should have cleared by one-shot mode
        TIM14->CR1 |= TIM_CR1_CEN;
    }


    // see if any motors are still moving
    if(!still_moving) {
        //SET_STEPTICKER_DEBUG_PIN(0);

        // all moves finished
        current_tick = 0;

        // get next block
        // do it here so there is no delay in ticks
        THECONVEYOR->block_finished();

        if(THECONVEYOR->get_next_block(&current_block)) { // returns false if no new block is available
            running= start_next_block(); // returns true if there is at least one motor with steps to issue

        }else{
            current_block= nullptr;
            running= false;
        }

        // all moves finished
        // we delegate the slow stuff to the pendsv handler which will run as soon as this interrupt exits
        //NVIC_SetPendingIRQ(PendSV_IRQn); //this doesn't work
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    }
}

// only called from the step tick ISR (single consumer)
bool StepTicker::start_next_block()
{
    if(current_block == nullptr) return false;

    bool ok= false;
    // need to prepare each active motor
    for (uint8_t m = 0; m < num_motors; m++) {
        if(current_block->tick_info[m].steps_to_move == 0) continue;

        ok= true; // mark at least one motor is moving
        // set direction bit here
        // NOTE this would be at least 10us before first step pulse.
        // TODO does this need to be done sooner, if so how without delaying next tick
        motor[m]->set_direction(current_block->direction_bits[m]);
        motor[m]->start_moving(); // also let motor know it is moving now
    }

    current_tick= 0;

    if(ok) {
        //SET_STEPTICKER_DEBUG_PIN(1);
        return true;

    }else{
        // this is an edge condition that should never happen, but we need to discard this block if it ever does
        // basically it is a block that has zero steps for all motors
        THECONVEYOR->block_finished();
    }

    return false;
}


// returns index of the stepper motor in the array and bitset
int StepTicker::register_motor(StepperMotor* m)
{
    motor[num_motors++] = m;
    return num_motors - 1;
}
