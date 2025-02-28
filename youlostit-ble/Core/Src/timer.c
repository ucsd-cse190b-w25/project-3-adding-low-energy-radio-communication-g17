/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

void timer_init(TIM_TypeDef* timer) {
	// Enables the TIM2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Disable the timer so we can configure it (Step 1.1)
	timer->CR1 &= ~TIM_CR1_CEN;

	// Resets timer state and sets default to 1 ms period (Step 1.2)
	timer->CNT = 0;
	timer->ARR = 4000 - 1;
	timer->PSC = 0;

	// Sets ARPE bit to enable auto reloading (Step 2)
	timer->CR1 |= TIM_CR1_ARPE;
	// Enabling NVIC and set priority to allow our timer to support interrupts (Step 3.1)
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);

	// Re-enables timer (Step 5)
	timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer) {
	timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms) {
	// Sets PreScaler to inverse to simply multiply the default parameters
	timer->CNT = 0;
	timer->PSC = period_ms - 1;
}
