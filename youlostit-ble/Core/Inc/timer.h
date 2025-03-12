/*
 * timer.h
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#ifndef TIMER_H_
#define TIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void timer_init(LPTIM_TypeDef* timer);
void timer_reset(LPTIM_TypeDef* timer);
void timer_set_seconds(LPTIM_TypeDef* timer, uint16_t period_s);

#endif /* TIMER_H_ */
