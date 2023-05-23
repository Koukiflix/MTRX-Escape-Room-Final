/**
 ******************************************************************************
 * @file           : buzzInterrupt.c
 * @author         : Tuesday PM, M-O
 * @brief          : TIM3 interrupt handler for Game Timer Module
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
#include "timer_buzzInterrupt.h"


// TIM2 INTERRUPT HANDLER //
void TIM3_IRQHandler()
{
	if ((TIM3->SR & TIM_SR_UIF) != 0){
		TIM3->SR &= ~TIM_SR_UIF;
		// Set HIGH output
		GPIOB->ODR |= GPIO_ODR_1;
	}

	// CC1IF successful output compare
	if ((TIM3->SR & TIM_SR_CC1IF) != 0){
		TIM3->SR &= ~TIM_SR_CC1IF;
		// Set LOW output
		GPIOB->ODR &= ~(GPIO_ODR_1);
	}
}

