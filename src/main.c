#include <stdio.h>
#include "uart.h"

#define GPIOAEN		(1U<<0)
#define GPIOA_5		(1U<<5)
#define LED_PIN		GPIOA_5

char key;
static void dma_callback(void);
void DMA1_Stream7_IRQHandler(void);

int main(void) {

	char message[31] = "Hello from STM32 DMA Transfer\n\r";

	RCC->AHB2ENR |= GPIOAEN;
	GPIOA->MODER |= (1U<<10);
	GPIOA->MODER &= ~(1U<<11);

	uart2_tx_init();
	dma1_stream7_init((uint32_t) message, (uint32_t) &USART2->TDR, 31);

	while (1) {

	}
}

static void dma_callback(void) {
	GPIOA->ODR |= LED_PIN;
}

void DMA1_Stream7_IRQHandler(void) {
	// Check for Transfer Complete Interrupt
	if (DMA1->ISR & (1U<<21)) {
		// Clear flag
		DMA1->IFCR |= (1U<<25);
		dma_callback();
	}
}
