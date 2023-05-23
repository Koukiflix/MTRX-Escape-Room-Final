/**
 ******************************************************************************
 * @file           : display.c
 * @author         : Tuesday PM, M-O
 * @brief          : 7-seg display functions for Game Timer Module
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


// FUNCTION TO DISPLAY SECONDS UNITS DIGIT //
void display_units(int digit)
{
	// Define 7-segment digit configuration (0-9)
	char sec_units[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
	// Clear bit
	GPIOA->ODR &= ~(GPIO_ODR_1 | GPIO_ODR_2 | GPIO_ODR_3 | GPIO_ODR_4 | GPIO_ODR_5 | GPIO_ODR_6 | GPIO_ODR_7);
	// Send bit
	GPIOA->ODR |= ~(sec_units[digit] << 1);
}

// FUNCTION TO DISPLAY SECONDS TENS DIGIT //
void display_tens(int digit)
{
	// Define 7-segment digit configuration (0-9)
	char sec_tens[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
	// Clear bit
	GPIOC->ODR &= ~(GPIO_ODR_1 | GPIO_ODR_2 | GPIO_ODR_3 | GPIO_ODR_4 | GPIO_ODR_5 | GPIO_ODR_6 | GPIO_ODR_7);
	// Send bit
	GPIOC->ODR |= ~(sec_tens[digit] << 1);
}

// FUNCTION TO DISPLAY MINUTES DIGIT //
void display_minutes(int digit)
{
	// Define 7-segment digit configuration (0-9)
	char min[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
	// Clear bit
	GPIOD->ODR &= ~(GPIO_ODR_1 | GPIO_ODR_2 | GPIO_ODR_3 | GPIO_ODR_4 | GPIO_ODR_5 | GPIO_ODR_6 | GPIO_ODR_7);
	// Send bit
	GPIOD->ODR |= ~(min[digit] << 1 | GPIO_ODR_8); // DP held at HIGH
}


/*
 * USER INSTRUCTIONS:
 * User can adjust 7-segment display value by changing the number of available characters/digits in arrays,
 * and setting the corresponding hex values of the wanted characters in the below arrays.
 */

