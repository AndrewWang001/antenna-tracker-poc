/**
 * uart_drv.h - interrupt-driven UART driver (USART1/2/3, STM32F1).
 *
 * Pins, AFIO remap, baud rate and IRQ priority come from uart_cfg_t, so the
 * same driver runs any USART on any of its legal pin sets. RX bytes go from
 * the ISR into a FreeRTOS stream buffer; TX bytes are queued in a second
 * stream buffer and drained by the TXE interrupt, so writers never busy-wait.
 *
 * Call uart_init() from a task (after the scheduler has started).
 */
#ifndef UART_DRV_H
#define UART_DRV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "semphr.h"

typedef struct {
    USART_TypeDef *instance;    /* USART1, USART2 or USART3 */
    uint32_t       baud;
    GPIO_TypeDef  *tx_port;
    uint16_t       tx_pin;
    GPIO_TypeDef  *rx_port;
    uint16_t       rx_pin;
    bool           remap;       /* USART1->PB6/PB7, USART2->PD5/PD6, USART3->PC10/PC11 */
    uint32_t       irq_prio;
    size_t         rx_buf_size;
    size_t         tx_buf_size;
} uart_cfg_t;

typedef struct {
    uart_cfg_t           cfg;
    StreamBufferHandle_t rx;
    StreamBufferHandle_t tx;
    SemaphoreHandle_t    tx_lock;
    volatile uint32_t    rx_bytes, rx_dropped, rx_overrun, rx_framing;
} uart_t;

bool   uart_init(uart_t *u, const uart_cfg_t *cfg);
size_t uart_write(uart_t *u, const void *data, size_t len, uint32_t timeout_ms);
void   uart_puts(uart_t *u, const char *s);
int    uart_getc(uart_t *u, uint32_t timeout_ms);   /* -1 on timeout */

#endif /* UART_DRV_H */
