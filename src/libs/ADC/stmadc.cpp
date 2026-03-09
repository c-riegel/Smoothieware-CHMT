/* mbed Library - ADC
 * Copyright (c) 2010, sblandford
 * released under MIT license http://mbed.org/licence/mit
 */

#include "stm32f407xx.h"
#undef ADC

#include "mbed.h"
#include "stmadc.h"
#include "mri.h"

#include "pinmap.h"
#include "PeripheralPins.h"

#include <vector>
#include "SlowTicker.h"
#include "Kernel.h"

#define STM_ADC ADC1

extern "C" uint32_t Set_GPIO_Clock(uint32_t port);

using namespace mbed;

ADC *ADC::instance;

ADC::ADC(int sample_rate, int cclk_div)
{
    scan_count_active = scan_count_next = 0;
    scan_index = 0;
    attached = 0;

    memset(scan_chan_lut, 0xFF, sizeof(scan_chan_lut));

    __HAL_RCC_ADC1_CLK_ENABLE();

    // adcclk /8 prescaler
    ADC123_COMMON->CCR |= ADC_CCR_ADCPRE;

    // use long sampling time to reduce isr call freq, to reduce chance of overflow
    // 168 Mhz / 2 (APB CLK) / 8 (ADCCLK) / (480+15) = ~47 us conversion
    // for max 16 scan channels, thats max sampling rate of ~1.3 kHz
    STM_ADC->SMPR1 = ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13 | 
                     ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15 | ADC_SMPR1_SMP16 | ADC_SMPR1_SMP17 | 
                     ADC_SMPR1_SMP18;

    STM_ADC->SMPR2 = ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | 
                     ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7 | 
                     ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9;

    // overrun ie, scan mode, end of conv. ie
    STM_ADC->CR1 = ADC_CR1_OVRIE | ADC_CR1_SCAN | ADC_CR1_EOCIE;

    // interrupt after every conversion
    // turn on adc
    STM_ADC->CR2 = ADC_CR2_EOCS | ADC_CR2_ADON;

    NVIC_SetVector(ADC_IRQn, (uint32_t)&_adcisr);
    NVIC_EnableIRQ(ADC_IRQn);

    _adc_g_isr = NULL;
    instance = this;
};

void ADC::_adcisr(void)
{
    instance->adcisr();
}


void ADC::adcisr(void)
{
    // dr read clears eoc bit
    // must read data before checking overflow bit
    uint16_t data = STM_ADC->DR; // to be sure we are valid

    if (STM_ADC->SR & ADC_SR_OVR) {
        // conversion was clobbered by overflow, clear its flag too, clear strt so we restart
        STM_ADC->SR &= ~(ADC_SR_OVR | ADC_SR_EOC | ADC_SR_STRT);

        // overrun will abort scan sequence, next start will resume from beginning
        scan_index = 0;
        __debugbreak();

    } else {
        // don't clear EOC flag, it could have popped after we read dr, and dr may have new valid data
        if (_adc_g_isr != NULL && (interrupt_mask & (1 << scan_index)))
            _adc_g_isr(scan_index, data);
        
        if (++scan_index >= scan_count_active) {
            STM_ADC->SR &= ~(ADC_SR_STRT); // clear strt so next tick starts scan
            scan_index = 0;
        }
    }
}

uint8_t ADC::_pin_to_channel(PinName pin) {
    uint32_t function = pinmap_find_function(pin, PinMap_ADC);
    uint8_t chan = 0xFF;

    if (function != (uint32_t)NC)
        chan = scan_chan_lut[STM_PIN_CHANNEL(function)];

    return chan;
}

// enable or disable an ADC pin
uint8_t ADC::setup(PinName pin, int state) {
    uint32_t function = pinmap_find_function(pin, PinMap_ADC);
    uint8_t stm_chan = 0xFF;
    uint8_t chan = 0xFF;

    // we don't support dealloc for now, exit early if all channels full or pin doesn't support adc
    if (!state || scan_count_next >= ADC_CHANNEL_COUNT || function == (uint32_t)NC) 
        return chan;

    // set analog mode for gpio (b11)
    GPIO_TypeDef *gpio = (GPIO_TypeDef *) Set_GPIO_Clock(STM_PORT(pin));
    gpio->MODER |= (0x3 << (2*STM_PIN(pin)));
    
    stm_chan = STM_PIN_CHANNEL(function);
    chan = scan_count_next++;

    scan_chan_lut[stm_chan] = chan;

    // configure adc scan channel
    if (chan < 6) {
        STM_ADC->SQR3 |= (stm_chan << (ADC_SQR3_SQ2_Pos*chan));
    } else if (chan < 12) {
        STM_ADC->SQR2 |= (stm_chan << (ADC_SQR2_SQ8_Pos*(chan - 6)));
    } else if (chan < 16) {
        STM_ADC->SQR1 |= (stm_chan << (ADC_SQR1_SQ14_Pos*(chan - 12)));
    }

    return chan;
}

// enable or disable burst mode
void ADC::burst(int state) {
    if (state && !attached) {
        THEKERNEL->slow_ticker->attach(1000, this, &ADC::on_tick);
        attached = 1;
    }
}

// set interrupt enable/disable for pin to state
void ADC::interrupt_state(PinName pin, int state) {
    int chan = _pin_to_channel(pin);

    if (chan < ADC_CHANNEL_COUNT) {
        if (state)
            interrupt_mask |= (1 << chan);
        else
            interrupt_mask &= ~(1 << chan);
    }
}

// append global interrupt handler to function isr
void ADC::append(void(*fptr)(int chan, uint32_t value)) {
    _adc_g_isr = fptr;
}

//Callback for attaching to slowticker as scan start timer 
uint32_t ADC::on_tick(uint32_t dummy) {
    // previous conversion still running
    if (STM_ADC->SR & ADC_SR_STRT)
        return dummy;

    // synchronize scan_count_active used by isr and scan_count_next while adding channels
    if (scan_count_active != scan_count_next) {
        scan_count_active = scan_count_next;

        // increase scan count
        STM_ADC->SQR1 = (STM_ADC->SQR1 & (~ADC_SQR1_L)) | ((scan_count_active-1) << ADC_SQR1_L_Pos);
    }
    STM_ADC->CR2 |= ADC_CR2_SWSTART;

    return dummy;
}


