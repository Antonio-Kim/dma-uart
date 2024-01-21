#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "stm32l4xx.h"

void uart2_tx_init(void);
char uart2_read(void);
void uart2_rxtx_init(void);
void uart2_rx_interrupt_init(void);
void dma1_stream7_init(uint32_t src, uint32_t dst, uint32_t len);

#define ISR_RXNE			(1U<<5)

#endif
