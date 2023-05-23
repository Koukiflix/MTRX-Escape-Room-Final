#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


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
void TIM2_IRQHandler();
void TIM3_IRQHandler();
void display_units(int digit);
void display_tens(int digit);
void display_minutes(int digit);
void twinkle();

// Declare global variables
extern int time_reached;
extern int start_condition;


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

