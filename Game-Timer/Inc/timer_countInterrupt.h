#ifndef INC_TIMER_COUNTINTERRUPT_H_
#define INC_TIMER_COUNTINTERRUPT_H_


#include <stdint.h>
#include <math.h>

#include "stm32f303xc.h"

#include "timer_main.h"
#include "timer_display.h"
#include "timer_countInterrupt.h"

void TIM2_IRQHandler();

extern int time_reached;
extern int start_condition;


#endif /* __COUNTINTERRUPT_H */

