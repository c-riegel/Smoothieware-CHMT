/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/nuts_bolts.h"
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "SlowTicker.h"
#include "StepTicker.h"
#include "libs/Hook.h"
#include "modules/robot/Conveyor.h"
#include "Gcode.h"

#include "stm32f407xx.h" // mbed.h lib

#include <mri.h>

// This module uses a Timer to periodically call hooks
// Modules register with a function ( callback ) and a frequency, and we then call that function at the given frequency.


extern "C" void TIM6_DAC_IRQHandler(void);

SlowTicker* global_slow_ticker;

SlowTicker::SlowTicker(){
    global_slow_ticker = this;

    // ISP button FIXME: WHy is this here?
    //ispbtn.from_string("2.10")->as_input()->pull_up();

    __TIM6_CLK_ENABLE();
    TIM6->CR1 = TIM_CR1_URS;    // int on overflow
    NVIC_SetVector(TIM6_DAC_IRQn, (uint32_t)TIM6_DAC_IRQHandler);

    max_frequency = 5;  // initial max frequency is set to 5Hz
    set_frequency(max_frequency);
    flag_1s_flag = 0;
}

void SlowTicker::start()
{
    TIM6->DIER = TIM_DIER_UIE;      // update interrupt en
    NVIC_EnableIRQ(TIM6_DAC_IRQn);    // Enable interrupt handler
    TIM6->CR1 |= TIM_CR1_CEN;  // start
}

void SlowTicker::on_module_loaded(){
    register_for_event(ON_IDLE);
}

// Set the base frequency we use for all sub-frequencies
void SlowTicker::set_frequency( int frequency ){
    this->interval = ((SystemCoreClock >> 1) / SLOWTICKER_PRESCALER) / frequency;   // SystemCoreClock/4 = Timer increments in a second

    TIM6->PSC = SLOWTICKER_PRESCALER - 1;
    TIM6->ARR = this->interval;

    flag_1s_count= (SystemCoreClock >> 1) / SLOWTICKER_PRESCALER;
}

// The actual interrupt being called by the timer, this is where work is done
void SlowTicker::tick(){

    // Call all hooks that need to be called
    for (Hook* hook : this->hooks){
        hook->countdown -= this->interval;
        if (hook->countdown < 0)
        {
            hook->countdown += hook->interval;
            hook->call();
        }
    }

    // deduct tick time from second counter
    flag_1s_count -= this->interval;
    // if a whole second has elapsed,
    if (flag_1s_count < 0)
    {
        // add a second to our counter
        flag_1s_count += (SystemCoreClock >> 1) / SLOWTICKER_PRESCALER;
        // and set a flag for idle event to pick up
        flag_1s_flag++;
    }

    // Enter MRI mode if the ISP button is pressed
    // TODO: This should have it's own module
    //if (ispbtn.get() == 0)
    //    __debugbreak();

}

bool SlowTicker::flag_1s(){
    // atomic flag check routine
    // first disable interrupts
    __disable_irq();
    // then check for a flag
    if (flag_1s_flag)
    {
        // if we have a flag, decrement the counter
        flag_1s_flag--;
        // re-enable interrupts
        __enable_irq();
        // and tell caller that we consumed a flag
        return true;
    }
    // if no flag, re-enable interrupts and return false
    __enable_irq();
    return false;
}

#include "gpio.h"
extern GPIO leds[];
void SlowTicker::on_idle(void*)
{
    static uint16_t ledcnt= 0;
    if(THEKERNEL->is_using_leds()) {
        // flash led 3 to show we are alive
        leds[1]= (ledcnt++ & 0x1000) ? 1 : 0;
    }

    // if interrupt has set the 1 second flag
    if (flag_1s())
        // fire the on_second_tick event
        THEKERNEL->call_event(ON_SECOND_TICK);
}

extern "C" void TIM6_DAC_IRQHandler (void){
    TIM6->SR = ~TIM_SR_UIF;

    global_slow_ticker->tick();
}

