/**
 ******************************************************************************
 * @file           : countInterrupt.c
 * @author         : Tuesday PM, M-O
 * @brief          : TIM2 interrupt handler for Game Timer Module
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
#include "timer_display.h"
#include "timer_countInterrupt.h"


// TIM2 INTERRUPT HANDLER //
void TIM2_IRQHandler()
{
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;

        // Compute time
        static int time_count = 300;
        /*
         * USER INSTRUCTIONS:
         * Set game run time by adjusting the variable "time_count". The variable is computed in seconds.
         */

        int minutes = time_count / 60;
        int seconds = time_count % 60;
        int seconds_units = seconds % 10;
        int seconds_tens = (seconds / 10) % 10;

        // Set start condition flag
        if ((GPIOA->IDR & GPIO_IDR_0) != 0) {
        	start_condition = 1;
        }

        // Update 7-seg display
        display_units(seconds_units);
        display_tens(seconds_tens);
        display_minutes(minutes);

        // Update time
        if (time_count > 0 && start_condition != 0) {
        	time_count--;
        }
        else {
        	// Set end condition
        	if (start_condition != 0) {
        		time_reached = 1;
			}
        	minutes = 0;
        	seconds_units = 0;
        	seconds_tens = 0;
        	start_condition = 0;
        }
    }
}

