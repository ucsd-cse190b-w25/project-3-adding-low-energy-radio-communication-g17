/*
 * i2c.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Daniel Lov
 */

/* Include memory map of our MCU */
#include <stm32l475xx.h>

	void i2c_init() {

		// Enable I2C2 clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

		// Reset I2C2
		RCC->APB1RSTR1 |= RCC_APB1RSTR1_I2C2RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C2RST;

		// Disable I2C2 before configuring
		I2C2->CR1 &= ~I2C_CR1_PE;

		// Set PB10 and PB11 to Alternate Function mode
		GPIOB->MODER &= ~GPIO_MODER_MODE10;
		GPIOB->MODER &= ~GPIO_MODER_MODE11;

		GPIOB->MODER |= GPIO_MODER_MODE10_1;
		GPIOB->MODER |= GPIO_MODER_MODE11_1;

		// Set Open-Drain output type
		GPIOB->OTYPER |= GPIO_OTYPER_OT10;
		GPIOB->OTYPER |= GPIO_OTYPER_OT11;

		GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD10;
		GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD11;

		// Enable Pull-Up resistors
		GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_0;
		GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;

		// Set Alternate Function AF4 (I2C2) for PB10 and PB11
		GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos) | (4 << GPIO_AFRH_AFSEL11_Pos);

		// Configures to 40kHz Frequency
		I2C2->TIMINGR =
		  (0x0 << I2C_TIMINGR_PRESC_Pos) |
		  (0x31 << I2C_TIMINGR_SCLL_Pos) |
		  (0x2D << I2C_TIMINGR_SCLH_Pos) |
		  (0x2 << I2C_TIMINGR_SDADEL_Pos) |
		  (0x4 << I2C_TIMINGR_SCLDEL_Pos);

		// Enables Peripheral
		I2C2->CR1 |= I2C_CR1_PE;
	}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len)
	{

		// Wait for the bus to be free
		while (I2C2->ISR & I2C_ISR_BUSY);

		// Clears previously set flags
		I2C2->ICR |= I2C_ICR_NACKCF;
		I2C2->ICR |= I2C_ICR_STOPCF;

		// Configs and resets CR2 registers
	    I2C2->CR2 &= ~I2C_CR2_START; // clear start
	    I2C2->CR2 &= ~I2C_CR2_ADD10; // disable 10-bit addressing
	    I2C2->CR2 &= ~I2C_CR2_SADD; // clear address
	    I2C2->CR2 &= ~I2C_CR2_NBYTES; // clear length


	    I2C2->CR2 |= (uint32_t)(address << 1);  // Shift the address left if it's 7-bit
	    I2C2->CR2 |= (uint32_t) len << I2C_CR2_NBYTES_Pos; // set length
//	    I2C2->CR2 |= I2C_CR2_AUTOEND; // enable autoend

	    if (dir) // READ
	    {
	        I2C2->CR2 |= I2C_CR2_RD_WRN; // set read mode
	        I2C2->CR2 |= I2C_CR2_START; // generate start condition

	        // NACK check
	        if (I2C2->ISR & I2C_ISR_NACKF) {
	            I2C2->ICR |= I2C_ICR_NACKCF; // Clear NACK flag
	            I2C2->CR2 |= I2C_CR2_STOP;   // Send STOP condition
	            return 1; // Error
	        }

	        for (uint8_t i = 0; i < len; i++)
	        {
	            while (!(I2C2->ISR & I2C_ISR_RXNE));
	            data[i] = I2C2->RXDR;
	        }
	    }
	    else // WRITE
	    {
	        I2C2->CR2 &= ~I2C_CR2_RD_WRN; // set write mode
	        I2C2->CR2 |= I2C_CR2_START; // generate start condition

	        // NACK check
	        if (I2C2->ISR & I2C_ISR_NACKF) {
	            I2C2->ICR |= I2C_ICR_NACKCF; // Clear NACK flag
	            I2C2->CR2 |= I2C_CR2_STOP;   // Send STOP condition
	            return 1; // Error
	        }

	        for (uint8_t i = 0; i < len; i++) {
	        	// Waits for TXIS to be ready
	            while (!(I2C2->ISR & I2C_ISR_TXIS));
	            I2C2->TXDR = data[i];
	        }
	    }

	    // Waits for any transfers to be complete
	    while (!(I2C2->ISR & I2C_ISR_TC));

	    I2C2->CR2 |= I2C_CR2_STOP;


	    // Clear transaction flags
	    I2C2->ICR |= I2C_ICR_STOPCF; // Clear STOP flag
	    I2C2->ICR |= I2C_ICR_NACKCF; // Clear NACK flag
	    return 0;
	}
