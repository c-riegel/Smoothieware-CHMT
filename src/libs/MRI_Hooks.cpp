#include "MRI_Hooks.h"

#include <stm32f407xx.h>
#include <mri.h>

// This is used by MRI to turn pins on and off when entering and leaving MRI. Useful for not burning everything down
// See http://smoothieware.org/mri-debugging 

extern "C" {
    static GPIO_TypeDef* const gpios[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

    #define GPIO_PORT_COUNT     ((int)(sizeof(gpios)/sizeof(gpios[0])))
    #define GPIO_PIN_MAX        16

    static uint32_t _set_high_on_debug[GPIO_PORT_COUNT] = {
//         (1 << 4) | (1 << 10) | (1 << 19) | (1 << 21), // smoothieboard stepper EN pins
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };
    static uint32_t _set_low_on_debug[GPIO_PORT_COUNT]  = {
        0,
        0,
//         (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7), // smoothieboard heater outputs
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    };

    static uint32_t _previous_state[GPIO_PORT_COUNT];

    static GPIO_TypeDef* io;
    static int i;

    void __mriPlatform_EnteringDebuggerHook()
    {
        for (i = 0; i < GPIO_PORT_COUNT; i++)
        {
            io          = gpios[i];

            _previous_state[i] = io->IDR;

            io->BSRR    = _set_high_on_debug[i];
            io->BSRR    = (_set_low_on_debug[i] << GPIO_BSRR_BR0_Pos);
        }
    }

    void __mriPlatform_LeavingDebuggerHook()
    {
        for (i = 0; i < GPIO_PORT_COUNT; i++)
        {
            io           = gpios[i];

            io->BSRR   =   _previous_state[i]  & (_set_high_on_debug[i] | _set_low_on_debug[i]);
            io->BSRR   = ((~_previous_state[i]) & (_set_high_on_debug[i] | _set_low_on_debug[i])) << GPIO_BSRR_BR0_Pos;
        }
    }

    void set_high_on_debug(int port, int pin)
    {
        if ((port >= GPIO_PORT_COUNT) || (port < 0))
            return;
        if ((pin >= GPIO_PIN_MAX) || (pin < 0))
            return;
        _set_high_on_debug[port] |= (1<<pin);
    }

    void set_low_on_debug(int port, int pin)
    {
        if ((port >= GPIO_PORT_COUNT) || (port < 0))
            return;
        if ((pin >= GPIO_PIN_MAX) || (pin < 0))
            return;
        _set_low_on_debug[port] |= (1<<pin);
    }
}
