/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "gpio_api.h"
#include "wait_api.h"
#include "toolchain.h"
#include "mbed_interface.h"

WEAK void mbed_die(void) {
#if !defined (NRF51_H) && !defined(TARGET_EFM32)
	__disable_irq();	// dont allow interrupts to disturb the flash pattern
#endif

    while (1) {
        wait_ms(150);
    }
}
