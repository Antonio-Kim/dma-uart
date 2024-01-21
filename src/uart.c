#include "uart.h"

#define GPIOAEN				(1U<<0)
#define USART2EN			(1U<<17)
#define CR1_TE				(1U<<3)
#define CR1_UE				(1U<<0)
#define CR1_RE				(1U<<2)
#define ISR_TXE				(1U<<7)
#define SYS_FREQ			16000000
#define APB1_CLK			SYS_FREQ
#define UART_BAUDRATE		115200
#define CR1_RXNEIE			(1U<<5)
#define AHB1_DMA1EN			(1U<<0)
#define DMA1_CCR2EN			(1U<<0)
#define DMA1_MINCEN			(1U<<7)
#define DMA1_READ_MEM		(1U<<4)
#define DMA1_CCR_TCIE		(1U<<1)
#define DMATEN				(1U<<7)


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);


void uart2_write(int ch);
int __io_putchar(int ch) {
	uart2_write(ch);
	return ch;
}

void dma1_stream7_init(uint32_t src, uint32_t dst, uint32_t len) {
	// enable clock access to DMA
	RCC->AHB1ENR |= AHB1_DMA1EN;
	// Disable DMA1 Channel 7
	DMA1_Channel7->CCR &= ~DMA1_CCR2EN;
	// Wait until DMA1 Channel 7 is disabled
	while (MA1_Channel7->CCR & DMA1_CCR2EN) {}
	// Clear all interrupt flags of Channel 7
	DMA1->IFCR &= ~(1U<<24);
	DMA1->IFCR &= ~(1U<<25);
	DMA1->IFCR &= ~(1U<<26);
	DMA1->IFCR &= ~(1U<<27);
	// set destination buffer
	DMA1_Channel7->CPAR = dst;
	// Set source buffer
	DMA1_Channel7->CMAR = src;
	// Set the length
	DMA1_Channel7->CNDTR = len;
	// Select Channel 7 to be UART_TX
	DMA1_CSELR->CSELR &= ~(1U<<24);
	DMA1_CSELR->CSELR |= (1U<<25);
	DMA1_CSELR->CSELR &= ~(1U<<26);
	DMA1_CSELR->CSELR &= ~(1U<<27);
	// Enable memory increment
	DMA1_Channel7->CCR |= DMA1_MINCEN;
	// Configure transfer direction
	DMA1_Channel7->CCR |= DMA1_READ_MEM;

	// Enable DMA transfer complete interrupt
	DMA1_Channel7->CCR |= DMA1_CCR_TCIE;

	// Enable DMA1 Stream 7
	DMA1_Channel7->CCR = DMA1_CCR2EN;
	// Enable UART2 transmitter DMA
	USART2->CR3 |= DMATEN;
	// DMA Interrupt enable in NVIC
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void uart2_rx_interrupt_init(void) {
	// Configure UART GPIO pins
	// 1. Enable Clock Access to GPIOA
	RCC->AHB2ENR|= GPIOAEN;
	// 2. Set PA2 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	// 3. Set PA2 Alternate function type to UART_TX (AF7)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	// 4. Set PA3 Mode to alternate function mode
	GPIOA->MODER &= ~(1U<<6);
	GPIOA->MODER |= (1U<<7);
	// 5. Set PA3 alternate function type to UART_RX
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &= ~(1U<<15);


	// Configure UART Module
	// 1. Enable clock access to UART2
	RCC->APB1ENR1 |= USART2EN;
	// 2. Configure Baud Rate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	// 3. Configure the transfer direction ****NB This might cause some issues. Upper registers are not reserved
	USART2->CR1 = CR1_TE | CR1_RE;
	// 4. Enable RXNE interrupt
	USART2->CR1 |= CR1_RXNEIE;
	// 5. Enable UART2 Interrupt in NVIC
	NVIC_EnableIRQ(USART2_IRQn);
	// 6. Enable UART Module
	USART2->CR1 |= CR1_UE;

}

void uart2_rxtx_init(void) {
	// Configure UART GPIO pins
	// 1. Enable Clock Access to GPIOA
	RCC->AHB2ENR|= GPIOAEN;
	// 2. Set PA2 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	// 3. Set PA2 Alternate function type to UART_TX (AF7)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	// 4. Set PA3 Mode to alternate function mode
	GPIOA->MODER &= ~(1U<<6);
	GPIOA->MODER |= (1U<<7);
	// 5. Set PA3 alternate function type to UART_RX
	GPIOA->AFR[0] |= (1U<<12);
	GPIOA->AFR[0] |= (1U<<13);
	GPIOA->AFR[0] |= (1U<<14);
	GPIOA->AFR[0] &= ~(1U<<15);


	// Configure UART Module
	// 1. Enable clock access to UART2
	RCC->APB1ENR1 |= USART2EN;
	// 2. Configure Baud Rate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	// 3. Configure the transfer direction ****NB This might cause some issues. Upper registers are not reserved
	USART2->CR1 = CR1_TE | CR1_RE;
	// 4. Enable UART Module
	USART2->CR1 |= CR1_UE;

}

void uart2_tx_init(void) {
	// Configure UART GPIO pins
	// 1. Enable Clock Access to GPIOA
	RCC->AHB2ENR|= GPIOAEN;
	// 2. Set PA2 mode to alternate function mode
	GPIOA->MODER &= ~(1U<<4);
	GPIOA->MODER |= (1U<<5);
	// 3. Set PA2 Alternate function type to UART_TX (AF7)
	GPIOA->AFR[0] |= (1U<<8);
	GPIOA->AFR[0] |= (1U<<9);
	GPIOA->AFR[0] |= (1U<<10);
	GPIOA->AFR[0] &= ~(1U<<11);

	// Configure UART Module
	// 1. Enable clock access to UART2
	RCC->APB1ENR1 |= USART2EN;
	// 2. Configure Baud Rate
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);
	// 3. Configure the transfer direction ****NB This might cause some issues. Upper registers are not reserved
	USART2->CR1 = CR1_TE;
	// 4. Enable UART Module
	USART2->CR1 |= CR1_UE;

}

char uart2_read(void) {
	// 1. Make sure receive data register is empty
	while(!(USART2->ISR & ISR_RXNE)){}
	// 2. Read data
	return USART2->TDR;
}

void uart2_write(int ch) {
	// 1. Make sure transmit data register is empty
	while(!(USART2->ISR & ISR_TXE)){}
	// 2. Write to transmit data register
	USART2->TDR = ch & 0xFF;
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate) {
	USARTx->BRR = compute_uart_bd(PeriphClk, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate) {
	return (PeriphClk + (BaudRate / 2U)) / BaudRate;
}
