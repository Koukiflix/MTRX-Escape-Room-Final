/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Tuesday PM, M-O
 * @brief          : Main program body for Game Timer Module
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#include <stdint.h>
#include <math.h>

#include "stm32f303xc.h"

#include "timer_main.h"
#include "timer_countInterrupt.h"
#include "timer_buzzInterrupt.h"
#include "timer_display.h"
#include "timer_led.h"


// Declare functions
void initialise_clocks();
void initialise_board();

// Initialise global variables
int time_reached = 0; // End-of-timer flag
int start_condition = 0; // Start flag

// DRIVER FUNCTION //
int main()
{
	__disable_irq();

	initialise_clocks();

    __enable_irq();

    initialise_board();

    // Enable TIM2 interrupt
    NVIC_EnableIRQ(TIM2_IRQn);

    // Forever Loop
    for (;;)
    {
    	// Enable TIM3 interrupt
    	if ((time_reached == 1) && (RCC->APB1ENR | RCC_APB1ENR_TIM3EN == RCC_APB1ENR_TIM3EN)) {
    		NVIC_EnableIRQ(TIM3_IRQn);
    	}

    	// Loop LED pattern
    	twinkle();
    }
}


// FUNCTION TO INITIALISE CLOCKS - TIM2, TIM3 //
void initialise_clocks()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOEEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configure TIM2
    TIM2->PSC = 8000; // (8 MHz / 8000 = 1000 Hz)
    TIM2->ARR = 1000; // per 1 second
    TIM2->DIER |= TIM_DIER_UIE; // Trigger interrupt at overflow
    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2

	// Configure TIM3
	TIM3->PSC = 160;
	TIM3->ARR = 20;
	TIM3->DIER |= TIM_DIER_UIE; // Trigger interrupt at overflow
	TIM3->DIER |= TIM_DIER_CC1IE; // Trigger interrupt when successful output compare (on Channel 1)
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3
	TIM3->CCER |= TIM_CCER_CC1E; // Enable output Capture/Compare for Channel 1
	TIM3->CCR1 = 10;
	/*
	 * USER INSTRUCTIONS:
	 * Change TIM3->ARR to alter the frequency range of the buzzer,
	 * Change TIM3->CCR1 to alter the duty cycle.
	 */
}

// FUNCTION TO INITIALISE BOARD //
void initialise_board()
{
	// 7-Seg 1 (Seconds Units)
    GPIOA->MODER |= GPIO_MODER_MODER1_0; // A
    GPIOA->MODER |= GPIO_MODER_MODER2_0; // B
    GPIOA->MODER |= GPIO_MODER_MODER3_0; // C
    GPIOA->MODER |= GPIO_MODER_MODER4_0; // D
    GPIOA->MODER |= GPIO_MODER_MODER5_0; // E
    GPIOA->MODER |= GPIO_MODER_MODER6_0; // F
    GPIOA->MODER |= GPIO_MODER_MODER7_0; // G

    // 7-Seg 2 (Seconds Tens)
    GPIOC->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);

    // 7-Seg 3 (Minutes)
    GPIOD->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0);

    // Buzzer PWM
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT_1;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;

    // LED Loop
    uint16_t *led_output = ((uint16_t *)&(GPIOE->MODER)) + 1;
    *led_output = 0x5555;
}

