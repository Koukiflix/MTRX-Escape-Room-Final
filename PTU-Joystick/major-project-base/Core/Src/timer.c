#include "timer.h"
#include "main.h"

void initialise_timer() {
	// enable the timer 2 through the RCC registers
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// set the prescaler so that 1 count is 1 microsecond
	//  8MHz = 0.000000125, 1 microsecond is 0.000001,
	//	prescaler 0.000001/0.000000125 = 8
	TIM3->PSC = 800;  // 1 microsecond / count
	TIM3->ARR = 65000; // 1 second before reset

	// make the timer2 trigger an interrupt when there is an overflow
	TIM3->DIER |= TIM_DIER_UIE;

	// make the timer2 trigger an interrupt when there is
	//  a successful output compare (on channel 1)
	TIM3->DIER |= TIM_DIER_CC1IE;

	// finally, enable the timer2
	TIM3->CR1 |= TIM_CR1_CEN;

	// Tell the NVIC module that timer2 interrupts should be handled
	NVIC_EnableIRQ(TIM3_IRQn);

	// Enable the output Capture/Compare for channel 1
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCR1 = 32500; // 50% duty cycle (500000/1000000)
}
