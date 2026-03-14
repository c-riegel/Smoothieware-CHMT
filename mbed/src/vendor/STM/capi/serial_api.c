/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2015, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "mbed_assert.h"
#include "serial_api.h"

#if DEVICE_SERIAL

#include "cmsis.h"
#include "pinmap.h"
#include <string.h>
#include "PeripheralPins.h"
#include "mbed_error.h"

#define UART_NUM (8)

static uint32_t serial_irq_ids[UART_NUM] = {0, 0, 0, 0, 0, 0, 0, 0};

static uart_irq_handler irq_handler;

UART_HandleTypeDef UartHandle;

int stdio_uart_inited = 0;
serial_t stdio_uart;

static void init_uart(serial_t *obj)
{
    UartHandle.Instance = (USART_TypeDef *)(obj->uart);

    UartHandle.Init.BaudRate   = obj->baudrate;
    UartHandle.Init.WordLength = obj->databits;
    UartHandle.Init.StopBits   = obj->stopbits;
    UartHandle.Init.Parity     = obj->parity;
    UartHandle.Init.HwFlowCtl  = obj->hw_flowcontrol;

    if (obj->pin_rx == NC) {
        UartHandle.Init.Mode = UART_MODE_TX;
    } else if (obj->pin_tx == NC) {
        UartHandle.Init.Mode = UART_MODE_RX;
    } else {
        UartHandle.Init.Mode = UART_MODE_TX_RX;
    }

    if (HAL_UART_Init(&UartHandle) != HAL_OK) {
        error("Cannot initialize UART");
    }
}

void serial_init(serial_t *obj, PinName tx, PinName rx, PinName rts, PinName cts)
{
    // Determine the UART to use (UART_1, UART_2, ...)
    UARTName uart_tx = (UARTName)pinmap_peripheral(tx, PinMap_UART_TX);
    UARTName uart_rx = (UARTName)pinmap_peripheral(rx, PinMap_UART_RX);

    // Get the peripheral name (UART_1, UART_2, ...) from the pin and assign it to the object
    obj->uart = (UARTName)pinmap_merge(uart_tx, uart_rx);
    MBED_ASSERT(obj->uart != (UARTName)NC);

    // Enable USART clock
    switch (obj->uart) {
        case UART_1:
            __HAL_RCC_USART1_CLK_ENABLE();
            __DMA2_CLK_ENABLE(); 
            obj->index = 0;
            break;
        case UART_2:
            __HAL_RCC_USART2_CLK_ENABLE();
            __DMA1_CLK_ENABLE(); 
            obj->index = 1;
            break;
#if defined(USART3_BASE)
        case UART_3:
            __HAL_RCC_USART3_CLK_ENABLE();
            obj->index = 2;
            break;
#endif
#if defined(UART4_BASE)
        case UART_4:
            __HAL_RCC_UART4_CLK_ENABLE();
            obj->index = 3;
            break;
#endif
#if defined(UART5_BASE)
        case UART_5:
            __HAL_RCC_UART5_CLK_ENABLE();
            obj->index = 4;
            break;
#endif
#if defined(USART6_BASE)
        case UART_6:
            __HAL_RCC_USART6_CLK_ENABLE();
            obj->index = 5;
            break;
#endif
#if defined(UART7_BASE)
        case UART_7:
            __HAL_RCC_UART7_CLK_ENABLE();
            obj->index = 6;
            break;
#endif
#if defined(UART8_BASE)
        case UART_8:
            __HAL_RCC_UART8_CLK_ENABLE();
            obj->index = 7;
            break;
#endif
    }

    // Configure the UART pins
    pinmap_pinout(tx, PinMap_UART_TX);
    pinmap_pinout(rx, PinMap_UART_RX);
    if (tx != NC) {
        pin_mode(tx, PullUp);
    }
    if (rx != NC) {
        pin_mode(rx, PullUp);
    }

    if (rts != NC) {
        pin_function( rts, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));   // In DMA mode, we set RTS output as discrete GPIO, and handle by software to deal with crappy usb-serial devices inability to do correct (prompt) rts hs.
    }
    if (cts != NC) {
        pinmap_pinout(cts, PinMap_UART_CTS);
		pin_mode(cts, PullUp );
		obj->hw_flowcontrol = UART_HWCONTROL_CTS;
	} else {
		obj->hw_flowcontrol = UART_HWCONTROL_NONE;
	}
		
    // Configure UART
    obj->baudrate = 9600;
    obj->databits = UART_WORDLENGTH_8B;
    obj->stopbits = UART_STOPBITS_1;
    obj->parity   = UART_PARITY_NONE;

    obj->pin_tx = tx;
    obj->pin_rx = rx;

    init_uart(obj);

    // For stdio management
    if (obj->uart == STDIO_UART) {
        stdio_uart_inited = 1;
        memcpy(&stdio_uart, obj, sizeof(serial_t));
    }
}

void serial_free(serial_t *obj)
{
    // Reset UART and disable clock
    switch (obj->uart) {
        case UART_1:
            __USART1_FORCE_RESET();
            __USART1_RELEASE_RESET();
            __USART1_CLK_DISABLE();
            break;
        case UART_2:
            __USART2_FORCE_RESET();
            __USART2_RELEASE_RESET();
            __USART2_CLK_DISABLE();
            break;
#if defined(USART3_BASE)
        case UART_3:
            __USART3_FORCE_RESET();
            __USART3_RELEASE_RESET();
            __USART3_CLK_DISABLE();
            break;
#endif
#if defined(UART4_BASE)
        case UART_4:
            __UART4_FORCE_RESET();
            __UART4_RELEASE_RESET();
            __UART4_CLK_DISABLE();
            break;
#endif
#if defined(UART5_BASE)
        case UART_5:
            __UART5_FORCE_RESET();
            __UART5_RELEASE_RESET();
            __UART5_CLK_DISABLE();
            break;
#endif
#if defined(USART6_BASE)
        case UART_6:
            __USART6_FORCE_RESET();
            __USART6_RELEASE_RESET();
            __USART6_CLK_DISABLE();
            break;
#endif
#if defined(UART7_BASE)
        case UART_7:
            __UART7_FORCE_RESET();
            __UART7_RELEASE_RESET();
            __UART7_CLK_DISABLE();
            break;
#endif
#if defined(UART8_BASE)
        case UART_8:
            __UART8_FORCE_RESET();
            __UART8_RELEASE_RESET();
            __UART8_CLK_DISABLE();
            break;
#endif
    }
    // Configure GPIOs
    pin_function(obj->pin_tx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
    pin_function(obj->pin_rx, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));

    serial_irq_ids[obj->index] = 0;
}

void serial_baud(serial_t *obj, int baudrate)
{
    obj->baudrate = baudrate;
    init_uart(obj);
}

void serial_format(serial_t *obj, int data_bits, SerialParity parity, int stop_bits)
{
    if (data_bits == 9) {
        obj->databits = UART_WORDLENGTH_9B;
    } else {
        obj->databits = UART_WORDLENGTH_8B;
    }

    switch (parity) {
        case ParityOdd:
        case ParityForced0:
            obj->parity = UART_PARITY_ODD;
            break;
        case ParityEven:
        case ParityForced1:
            obj->parity = UART_PARITY_EVEN;
            break;
        default: // ParityNone
            obj->parity = UART_PARITY_NONE;
            break;
    }

    if (stop_bits == 2) {
        obj->stopbits = UART_STOPBITS_2;
    } else {
        obj->stopbits = UART_STOPBITS_1;
    }

    init_uart(obj);
}

/******************************************************************************
 * INTERRUPTS HANDLING
 ******************************************************************************/
static void dma1_irq( void )
{
    if ( DMA1->HISR & DMA_HISR_TCIF5 )
    {
        irq_handler(serial_irq_ids[1], DmaTCIrq);
    }
    if ( DMA1->HISR & DMA_HISR_HTIF5 )
    {
        irq_handler(serial_irq_ids[1], DmaHFIrq);
    }
    
    DMA1->HIFCR |= DMA_HISR_TCIF5 | DMA_HISR_HTIF5 | DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5;    
}

static void dma2_irq( void )
{
    if ( DMA2->LISR & DMA_LISR_TCIF2 )
    {
        irq_handler(serial_irq_ids[0], DmaTCIrq);
    }
    if ( DMA2->LISR & DMA_LISR_HTIF2 )
    {
        irq_handler(serial_irq_ids[0], DmaHFIrq);
    }
    
    DMA2->LIFCR |= DMA_LISR_TCIF2 | DMA_LISR_HTIF2 | DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2;    
}

static void uart_irq(UARTName name, int id)
{
    UartHandle.Instance = (USART_TypeDef *)name;
    if (serial_irq_ids[id] != 0) {
        if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TC) != RESET) {
            irq_handler(serial_irq_ids[id], TxIrq);
            __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TC);
        }
        if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) {
            irq_handler(serial_irq_ids[id], RxIrq);
            __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
        }
        if (__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE) != RESET) {
            __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
            irq_handler(serial_irq_ids[id], RxIdleIrq);
        }
    }
}

static void uart1_irq(void)
{
    uart_irq(UART_1, 0);
}

static void uart2_irq(void)
{
    uart_irq(UART_2, 1);
}

#if defined(USART3_BASE)
static void uart3_irq(void)
{
    uart_irq(UART_3, 2);
}
#endif

#if defined(UART4_BASE)
static void uart4_irq(void)
{
    uart_irq(UART_4, 3);
}
#endif

#if defined(UART5_BASE)
static void uart5_irq(void)
{
    uart_irq(UART_5, 4);
}
#endif

#if defined(USART6_BASE)
static void uart6_irq(void)
{
    uart_irq(UART_6, 5);
}
#endif

#if defined(UART7_BASE)
static void uart7_irq(void)
{
    uart_irq(UART_7, 6);
}
#endif

#if defined(UART8_BASE)
static void uart8_irq(void)
{
    uart_irq(UART_8, 7);
}
#endif

void serial_activate_rxdma(serial_t *obj, unsigned char* rx_buffer, int len )
{
	switch (obj->uart) {
		case UART_1:
		    // Clear /half/complete/ flags, in the event they are set..
            DMA2->LIFCR |= DMA_LISR_TCIF2 | DMA_LISR_HTIF2 | DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2;

            DMA2_Stream2->NDTR = len;
            DMA2_Stream2->PAR = (uint32_t)&(USART1->DR);
            DMA2_Stream2->M0AR = (uint32_t)rx_buffer;
		
            DMA2_Stream2->CR = (0x4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE; // DMA FIFO is not enabled, to make things easier.
		
            USART1->SR &= ~USART_SR_IDLE;

            DMA2_Stream2->CR |= DMA_SxCR_EN;
            USART1->CR3 |= USART_CR3_DMAR;
            break;
		    
        case UART_2:
		    // Clear /half/complete/ flags, in the event they are set..
            DMA1->HIFCR |= DMA_HISR_TCIF5 | DMA_HISR_HTIF5 | DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5;

            DMA1_Stream5->NDTR = len;
            DMA1_Stream5->PAR = (uint32_t)&(USART2->DR);
            DMA1_Stream5->M0AR = (uint32_t)rx_buffer;
		
            DMA1_Stream5->CR = (0x4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_PL_1 | DMA_SxCR_PL_0 | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE; // DMA FIFO is not enabled, to make things easier.
		
            USART2->SR &= ~USART_SR_IDLE;

            DMA1_Stream5->CR |= DMA_SxCR_EN;
            USART2->CR3 |= USART_CR3_DMAR;
            break;

		default:
		    break;
    }
}

void serial_irq_handler(serial_t *obj, uart_irq_handler handler, uint32_t id)
{
    irq_handler = handler;
    serial_irq_ids[obj->index] = id;
}

void serial_irq_set(serial_t *obj, SerialIrq irq, uint32_t enable)
{
    IRQn_Type irq_n = (IRQn_Type)0, dma_irq_n = (IRQn_Type)0;
    uint32_t vector = 0, dma_vector = 0;

    UartHandle.Instance = (USART_TypeDef *)(obj->uart);

    switch (obj->uart) {
        case UART_1:
            irq_n = USART1_IRQn;
            vector = (uint32_t)&uart1_irq;
            dma_vector= ((uint32_t)&dma2_irq);
            dma_irq_n = DMA2_Stream2_IRQn;
            break;

        case UART_2:
            irq_n = USART2_IRQn;
            vector = (uint32_t)&uart2_irq;
            dma_vector= ((uint32_t)&dma1_irq);
            dma_irq_n = DMA1_Stream5_IRQn;
            break;
#if defined(USART3_BASE)
        case UART_3:
            irq_n = USART3_IRQn;
            vector = (uint32_t)&uart3_irq;
            break;
#endif
#if defined(UART4_BASE)
        case UART_4:
            irq_n = UART4_IRQn;
            vector = (uint32_t)&uart4_irq;
            break;
#endif
#if defined(UART5_BASE)
        case UART_5:
            irq_n = UART5_IRQn;
            vector = (uint32_t)&uart5_irq;
            break;
#endif
#if defined(USART6_BASE)
        case UART_6:
            irq_n = USART6_IRQn;
            vector = (uint32_t)&uart6_irq;
            break;
#endif
#if defined(UART7_BASE)
        case UART_7:
            irq_n = UART7_IRQn;
            vector = (uint32_t)&uart7_irq;
            break;
#endif
#if defined(UART8_BASE)
        case UART_8:
            irq_n = UART8_IRQn;
            vector = (uint32_t)&uart8_irq;
            break;
#endif
    }

    if (enable) {

        switch (irq) {
            case RxIrq:
                __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);
                break;
                
            case TxIrq:
                __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_TXE);
                break;
                
            case RxIdleIrq:
                __HAL_UART_ENABLE_IT(&UartHandle, UART_IT_IDLE);
                break;

            case DmaTCIrq:
            case DmaHFIrq:
                break;
        }
        if (dma_vector) {
            NVIC_SetVector(dma_irq_n, dma_vector);
            NVIC_EnableIRQ(dma_irq_n);
        }
        
        NVIC_SetVector(irq_n, vector);
        NVIC_EnableIRQ(irq_n);

    } else { // disable

        switch (irq)
        {
            case RxIrq:
                __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_RXNE);
                break;
                
            case TxIrq:
                __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_TXE);
                break;
                
            case RxIdleIrq:
                __HAL_UART_DISABLE_IT(&UartHandle, UART_IT_IDLE);
                break;
            case DmaTCIrq:
            case DmaHFIrq:
                break;
        }

        
        if ( !(UartHandle.Instance->CR1 & (USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_IDLEIE)) )  { // All disabled?
            NVIC_DisableIRQ(irq_n);
        }

    }
}

/******************************************************************************
 * READ/WRITE
 ******************************************************************************/

int serial_getc(serial_t *obj)
{
    USART_TypeDef *uart = (USART_TypeDef *)(obj->uart);
    while (!serial_readable(obj));
    return (int)(uart->DR & 0x1FF);
}

void serial_putc(serial_t *obj, int c)
{
    USART_TypeDef *uart = (USART_TypeDef *)(obj->uart);
    while (!serial_writable(obj));
    uart->DR = (uint32_t)(c & 0x1FF);
}

void serial_send_string( serial_t *obj, const char *str )
{
    USART_TypeDef *uart = (USART_TypeDef *)(obj->uart);
    int len = strlen(str);
    unsigned int strInFlash = ((unsigned)str >= FLASH_BASE && (unsigned)str < FLASH_END);	// true if str points into flash memory
    unsigned int nSend;	// number of bytes to send
    const char * pSend;	// pointer to data to send, either str or obj->dmxtxbuffer, depending on strInFlash state
    
    switch (obj->uart) {
    	case UART_1:
    		do {
                // wait for end of active dma operation
                if (DMA2_Stream7->CR & DMA_SxCR_EN) {
                    while (! (DMA2->HISR & DMA_HISR_TCIF7))
                        ;
                    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
                    uart->CR3 &= ~USART_CR3_DMAT;
                }

                // Clear /half/complete/ flags from last tx..
                DMA2->HIFCR |= DMA_HISR_TCIF7 | DMA_HISR_HTIF7;

                if(DMA2->HISR & (DMA_HISR_TEIF7 | DMA_HISR_DMEIF7 | DMA_HISR_FEIF7 )) { // error on DMA
                    DMA2->HIFCR |= (DMA_HISR_TEIF7 | DMA_HISR_DMEIF7 | DMA_HISR_FEIF7 ); // should never happen.
                }

				if (strInFlash) {
				    // send all data directly
				    pSend = str;
				    nSend = len;
				} else {
                    // copy as much data as possible into dmatxbuffer
			        nSend = len;
			        if (nSend > sizeof(obj->dmatxbuffer))
			            nSend = sizeof(obj->dmatxbuffer);
			        memcpy(obj->dmatxbuffer, str, nSend);
			        pSend = obj->dmatxbuffer;
			    }
		        str += nSend;
		        len -= nSend;

                DMA2_Stream7->NDTR = nSend;
                DMA2_Stream7->PAR = (uint32_t)&(uart->DR);
                DMA2_Stream7->M0AR = (uint32_t)pSend;
                DMA2_Stream7->FCR |= DMA_SxFCR_DMDIS;

                DMA2_Stream7->CR = (0x4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1;

                uart->SR &= ~USART_SR_TC;

                DMA2_Stream7->CR |= DMA_SxCR_EN;
                uart->CR3 |= USART_CR3_DMAT;
            } while (len > 0);
		    break;

    	case UART_2:
    		do {
                // wait for end of active dma operation
                if (DMA1_Stream6->CR & DMA_SxCR_EN) {
                    while (!(DMA1->HISR & DMA_HISR_TCIF6))
                        ;
                    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
                    uart->CR3 &= ~USART_CR3_DMAT;
                }

                // Clear /half/complete/ flags from last tx..
                DMA1->HIFCR |= DMA_HISR_TCIF6 | DMA_HISR_HTIF6;

                if(DMA1->HISR & (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6 )) { // error on DMA
    	            DMA1->HIFCR |= (DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6 ); // should never happen.
                }

			    // copy as much data as possible into dmatxbuffer
			    unsigned int n = len;
			    if (n > sizeof(obj->dmatxbuffer))
			        n = sizeof(obj->dmatxbuffer);
			    memcpy(obj->dmatxbuffer, str, n);
			    str += n;
			    len -= n;

                DMA1_Stream6->NDTR = len;
                DMA1_Stream6->PAR = (uint32_t)&(uart->DR);
                DMA1_Stream6->M0AR = (uint32_t)str;
                DMA1_Stream6->FCR |= DMA_SxFCR_DMDIS;

                DMA1_Stream6->CR = (0x4 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_PL_1 | DMA_SxCR_MSIZE_1;

                uart->SR &= ~USART_SR_TC;

                DMA1_Stream6->CR |= DMA_SxCR_EN;
                uart->CR3 |= USART_CR3_DMAT;
            } while (len > 0);
		    break;
		    
		default:
		    break;
    }
}

int serial_get_dma_buffer_index(serial_t *obj)
{
    return DMA2_Stream2->NDTR;
}

int serial_readable(serial_t *obj)
{
    int status;
    UartHandle.Instance = (USART_TypeDef *)(obj->uart);
    // Check if data is received
    status = ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_RXNE) != RESET) ? 1 : 0);
    return status;
}

int serial_writable(serial_t *obj)
{
    int status;
    UartHandle.Instance = (USART_TypeDef *)(obj->uart);
    // Check if data is transmitted
    status = ((__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_TXE) != RESET) ? 1 : 0);
    return status;
}

void serial_clear(serial_t *obj)
{
    UartHandle.Instance = (USART_TypeDef *)(obj->uart);
    __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_TXE);
    __HAL_UART_CLEAR_FLAG(&UartHandle, UART_FLAG_RXNE);
}

void serial_pinout_tx(PinName tx)
{
    pinmap_pinout(tx, PinMap_UART_TX);
}

void serial_break_set(serial_t *obj)
{
    UartHandle.Instance = (USART_TypeDef *)(obj->uart);
    HAL_LIN_SendBreak(&UartHandle);
}

void serial_break_clear(serial_t *obj)
{
}

#endif
