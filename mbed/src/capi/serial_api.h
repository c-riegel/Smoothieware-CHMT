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
#ifndef MBED_SERIAL_API_H
#define MBED_SERIAL_API_H

#include "device.h"

#if DEVICE_SERIAL

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ParityNone = 0,
    ParityOdd = 1,
    ParityEven = 2,
    ParityForced1 = 3,
    ParityForced0 = 4
} SerialParity;

typedef enum {
    RxIrq,
    TxIrq,
    RxIdleIrq,
    DmaHFIrq,
    DmaTCIrq
} SerialIrq;

typedef void (*uart_irq_handler)(uint32_t id, SerialIrq event);

typedef struct serial_s serial_t;

void serial_init       (serial_t *obj, PinName tx, PinName rx, PinName rts, PinName cts);
void serial_free       (serial_t *obj);
void serial_baud       (serial_t *obj, int baudrate);
void serial_format     (serial_t *obj, int data_bits, SerialParity parity, int stop_bits);

void serial_activate_rxdma(serial_t *obj, unsigned char *rx_buff, int len );
void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id);
void serial_irq_set    (serial_t *obj, SerialIrq irq, uint32_t enable);

int serial_get_dma_buffer_index(serial_t *obj);
int  serial_getc       (serial_t *obj);
void serial_putc       (serial_t *obj, int c);
void serial_send_string(serial_t *obj, const char *str );
int  serial_readable   (serial_t *obj);
int  serial_writable   (serial_t *obj);
void serial_clear      (serial_t *obj);

void serial_pinout_tx(PinName tx);

#ifdef __cplusplus
}
#endif

#endif

#endif
