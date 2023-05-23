#ifndef INC_TIMER_DISPLAY_H_
#define INC_TIMER_DISPLAY_H_


#include <stdint.h>
#include <math.h>

#include "stm32f303xc.h"

#include "timer_main.h"
#include "timer_display.h"


void display_units(int digit);
void display_tens(int digit);
void display_minutes(int digit);


#endif /* __DISPLAY_H */

