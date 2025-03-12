/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
	// enable GPIO clocks for LED peripherals
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	/* Configure PA5 as an output by clearing all bits and setting the mode */
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	/* Configure the GPIO output as push pull (transistor for high and low) */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

	/* Disable the internal pull-up and pull-down resistors */
	GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

	/* Configure the GPIO to use low speed mode */
	GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

	/* Turn off the LED */
	GPIOA->ODR &= ~GPIO_ODR_OD5;

	// configure PA14 as output
	GPIOB->MODER &= ~GPIO_MODER_MODE14; // clear bits
	GPIOB->MODER |= GPIO_MODER_MODE14_0; // set mode

	// configure the GPIO output as push pull (transistor for high and low)
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

	// Disable the internal pull-up and pull-down resistors
	GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;

	// Configure the GPIO to use low speed mode
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

	// Turn off LED
	GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{
	// check if 1st bit is on for PA5 LED peripheral to turn on, otherwise switch it off
	if(led & 0x01) {
	  GPIOA->ODR |= GPIO_ODR_OD5;
	} else {
	  GPIOA->ODR &= ~GPIO_ODR_OD5;
	}
	// check if 2nd bit is on for PB14 LED peripheral to turn on, otherwise switch it off
	if(led & 0x02) {
	  GPIOB->ODR |= GPIO_ODR_OD14;
	} else {
	  GPIOB->ODR &= ~GPIO_ODR_OD14;
	}
}
