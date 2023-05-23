/**
 ******************************************************************************
 * @file           : led.c
 * @author         : Tuesday PM, M-O
 * @brief          : LED code loop for Game Timer Module
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
#include <timer_countInterrupt.h>

#include "stm32f303xc.h"

#include "timer_main.h"
#include "timer_led.h"


// TWINKLE TWINKLE LITTLE STAR CODE //
void twinkle()
{
	// Configure LED
	uint8_t *led_register = ((uint8_t*)&(GPIOE->ODR)) + 1;

	// Initialise twinkle values
	int notes_to_led[15] = {1, 1, 5, 5, 6, 6, 5, 0, 4, 4, 3, 3, 2, 2, 1};
	/*
	 * USER INSTRUCTIONS:
	 * Set note/led arrangement by adjusting values in and length of array "notes_to_led",
	 * where LED1 = C, LED2 = D, LED3 = E, etc.
	 */

	// Toggle LED
	if ((start_condition != 0) && (time_reached != 1)) {
		for (uint8_t i = 0; i < 15; i++) {
			// Toggle current LED
			*led_register |= (1 << (notes_to_led[i] - 1));
			// Delay and reset per LED
			for (volatile uint32_t delay = 0; delay < 300000; delay++);
			*led_register = 0b00000000;
			for (volatile uint32_t delay = 0; delay < 300000; delay++);
		}
		// Delay loop
		for (volatile uint32_t delay = 0; delay < 1950000; delay++);
	}
}

