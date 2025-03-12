/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"
#include "i2c.h"
#include "lsm6dsl.h"
#include "led.h"
#include "timer.h"

#include <stdlib.h>
#include <stdio.h>

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

volatile int led_cnt = 0; // led cycle counter initialized at 0 to track where in the pattern we are to output
volatile int set_leds = 0; // indicator to set leds
volatile int enable_leds = 0; // indicator to enable leds (when in lost mode)


#define	MOTION_THRESHOLD	5000 // threshold macro for which device registers motion (to filter out noise)
#define MINUTE_CYCLE	400 // minute cycle macro to track when 1 minute passed at 40Hz
#define SECOND_CYCLE 40
#define PRIVTAG_NAME "TEAM_G17"
volatile int minute_cnt = 0; // counter to track minute passed
volatile int second_cnt = 0;
volatile int in_motion = 0; // indicator for device to check for motion
volatile int stationary = 1;
volatile uint8_t minutes_lost = 0; // counter to track the number of minutes and display with leds
volatile uint8_t seconds_lost = 0; // counter to track the number of minutes and display with leds

//void EXTI15_10_IRQHandler(void)
//{
//    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)  // Check if EXTI11 triggered
//    {
//        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);  // Clear interrupt flag
//        stationary = 1;
//    }
//}

//void LPTIM1_IRQHandler() {
//    // Check if the auto-reload match interrupt occurred
//    if (LPTIM1->ISR & LPTIM_ISR_ARRM)
//    {
//        printf("we are not moving!!\n");
//
//        if (++second_cnt == SECOND_CYCLE) {
//            seconds_lost++;
//            second_cnt = 0;
//        }
//
//        // Check if a full minute has passed
//        if (++minute_cnt == MINUTE_CYCLE)
//        {
//            minutes_lost++;
//            minute_cnt = 0;
//        }
//
//        // Clear the interrupt flag by writing 1 to ICR
//        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
//    }
//}

void LPTIM1_IRQHandler() {
    // Print ISR register before clearing the flag
    printf("ISR before clear: %08lX\n", LPTIM1->ISR);

    // Check if the interrupt is triggered by the ARR match
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        // Clear the ARR match interrupt flag
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF;
        LPTIM1->ICR |= LPTIM_ICR_CMPMCF;
        // Print the timer's current count
        printf("LPTIM1 CNT: %lu\n", LPTIM1->CNT);

        // Print the interrupt trigger time
        printf("Interrupt Triggered! Time: %lu\n", second_cnt);

        // Increment second count
        second_cnt++;

        // Print ISR register after clearing the flag
        printf("ISR after clear: %08lX\n", LPTIM1->ISR);
    }
}


// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

void check_wakeup_source(void) {
    for (int i = 0; i < 8; i++) {
        uint32_t pending = NVIC->ISPR[i];
        if (pending) {
            for (int j = 0; j < 32; j++) {
                if (pending & (1 << j)) {
                    int irq_number = i * 32 + j;
                    printf("Interrupt %d triggered WFI\n", irq_number);
                }
            }
        }
    }
}

int main(void) {
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();


  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

  ble_init();
  setDiscoverability(0);

  timer_init(LPTIM1);
  if ((RCC->CCIPR & RCC_CCIPR_LPTIM1SEL) == (RCC_CCIPR_LPTIM1SEL_1 | RCC_CCIPR_LPTIM1SEL_0)) {
      printf("LPTIM1 is using LSE as the clock source.\n");
  } else {
      printf("LPTIM1 is NOT using LSE as the clock source.\n");
  }
//  timer_set_seconds(LPTIM1, 1);
  __enable_irq();

  // Initialize I2C
  i2c_init();

  // Initialize Accelerometer
  lsm6dsl_init();

  // Initialize LEDs
  leds_init();

  // initialize previous and current x,y,z accelerometer values
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;

  RCC->AHB1ENR |= RCC_APB1ENR1_PWREN;
  PWR->CR1 &= ~PWR_CR1_LPMS;
  PWR->CR1 |= PWR_CR1_LPMS_STOP2;
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;


  // Read Current Accelerometer Values and Set as Previous
//  lsm6dsl_read_xyz(&x, &y, &z);

  uint8_t nonDiscoverable = 0;
  while (1) {
	  leds_set(0x00);
//	  if (stationary) {
//		  leds_set(0x11);
//		  stationary = 0;
//	  } else {
//		  minutes_lost = 0;
//		  seconds_lost = 0;
//		  second_cnt = 0;
//		  minute_cnt = 0;
//
//		  in_motion = 1;
//
//
//		  disconnectBLE();
//		  standbyBLE();
//		  setDiscoverability(0);
//		  nonDiscoverable = 1;
//		  leds_set(0x00);
//		  stationary = 1;
//	  }
	  lsm6dsl_read_xyz(&x, &y, &z);
	  if (!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)) {
	    catchBLE();
	  } else {
		  if (minutes_lost) {
			  nonDiscoverable = 0;
			  setDiscoverability(1);

			  // Send a string to the NORDIC UART service, remember to not include the newline
			  unsigned char buffer_name[20];
			  const unsigned char name[] = PRIVTAG_NAME;
			  unsigned char buffer_time[20];

			  snprintf((char *)buffer_name, sizeof(buffer_name), "PrivTag %s", name);
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char *)buffer_name), buffer_name);

			  snprintf((char *)buffer_time, sizeof(buffer_time), "Lost for %d s", seconds_lost);
			  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen((char *)buffer_time), buffer_time);

			  HAL_Delay(9000);
		  }

		  HAL_Delay(1000);
	  }
	  check_wakeup_source();
	  __WFI();
	  check_wakeup_source();
	  leds_set(0x11);
	  HAL_Delay(1000);
  }
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Trigger on falling edge (adjust as needed)
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // No internal pull-up/down
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
