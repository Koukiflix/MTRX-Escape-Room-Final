#include "initialisation.h"
#include "main.h"

#define ALTFUNCTION 0xA00
#define RXTX 0x770000
#define HIGHSPEED 0xF00
#define BAUDRATE 0x46

// enable the clocks for desired peripherals (GPIOA, C and E)
void enable_clocks() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOEEN;

	// store a 1 in bit for the TIM3 enable flag
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
}


// initialise the discovery board I/O (just outputs: inputs are selected by default)
void initialise_board() {
	// get a pointer to the second half word of the MODER register (for outputs pe8-15)
	uint16_t *led_output_registers = ((uint16_t *)&(GPIOE->MODER)) + 1;
	*led_output_registers = 0x5555;
}



void enableUSART1()
{
    // Enable GPIO C and USART1's clocks
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN_Msk;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;

    // Set GPIO C to use UART as alternate function
    GPIOC->MODER = ALTFUNCTION;
    GPIOC->AFR[0] = RXTX;
    GPIOC->OSPEEDR = HIGHSPEED;

    // Set the baud rate and ready USART 1 for both receive and transmit
    USART1->BRR = BAUDRATE;
    USART1->CR1 |= USART_CR1_RE_Msk;
    USART1->CR1 |= USART_CR1_TE_Msk;
    USART1->CR1 |= USART_CR1_UE_Msk;
}
