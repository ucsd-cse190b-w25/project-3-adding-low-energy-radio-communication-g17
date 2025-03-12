/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

void timer_init(LPTIM_TypeDef* timer) {
    // Enable power and allow backup domain access
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP; // Unlock backup domain

    // Reset backup domain to clear previous LSE settings
    RCC->BDCR |= RCC_BDCR_BDRST;
    RCC->BDCR &= ~RCC_BDCR_BDRST;

    // Enable LSE
    RCC->BDCR |= RCC_BDCR_LSEON;

    // Wait for LSE to stabilize
    while (!(RCC->BDCR & RCC_BDCR_LSERDY));

    // Enable LPTIM1 peripheral clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    // Ensure LPTIM1 is disabled before configuration
    timer->CR &= ~LPTIM_CR_ENABLE;

    // Select LSE as the clock source for LPTIM1
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL;  // Clear bits
    RCC->CCIPR |= (RCC_CCIPR_LPTIM1SEL_1 | RCC_CCIPR_LPTIM1SEL_0); // Set LSE as clock source

    // Set ARR (Auto-Reload Register) for a 1-second interval (LSE = 32.768 kHz)
    timer->ARR = 32768 - 1;
    timer->CNT = 0;

    timer->CR |= LPTIM_CFGR_PRELOAD;

    // Enable interrupt for auto-reload match
    timer->IER |= LPTIM_IER_ARRMIE;

    // Enable NVIC interrupt for LPTIM1
    NVIC_EnableIRQ(LPTIM1_IRQn);
    NVIC_SetPriority(LPTIM1_IRQn, 2);

    // Enable LPTIM1 and start the timer
    timer->CR |= LPTIM_CR_ENABLE;
    timer->CR |= LPTIM_CR_CNTSTRT;
}

void timer_reset(LPTIM_TypeDef* timer) {
    // Disable the LPTIM1
    timer->CR &= ~LPTIM_CR_ENABLE;

    // Clear the CNT register (resets the counter)
    timer->CNT = 0;

    // Restart the timer
    timer->CR |= LPTIM_CR_ENABLE;
}

void timer_set_seconds(LPTIM_TypeDef* timer, uint16_t period_s) {

    // Disable the LPTIM1
    timer->CR &= ~LPTIM_CR_ENABLE;
	timer->CNT = 0;

	uint32_t CLOCK_SPEED = 32768;
	uint32_t TIMER_TICK = CLOCK_SPEED * period_s;
	uint32_t ARR_VAL = TIMER_TICK - 1;
	uint8_t PSC = 0;

	if (ARR_VAL > 0xFFFF) {

		PSC = ARR_VAL / 0xFFFF;

		if (PSC > 7) {
			PSC = 7;
		}

		ARR_VAL = (ARR_VAL / (PSC+1)) - 1;
	}

    // Set the auto-reload register to the calculated value
    timer->ARR = ARR_VAL;

    // Set the prescaler if needed
    timer->CFGR &= ~LPTIM_CFGR_PRESC;  // Clear prescaler bits
    timer->CFGR |= (PSC << LPTIM_CFGR_PRESC_Pos);  // Set the prescaler

    // Restart the timer
    timer->CR |= LPTIM_CR_ENABLE;
    timer->CR |= LPTIM_CR_CNTSTRT;
}
