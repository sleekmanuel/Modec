/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define Data_BUFFER_SIZE 12  // Define buffer size as required
#define DATA_BUFFER_SIZE 12  // Define buffer size as required
#define DEBOUNCE_DELAY_MS 1
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


typedef struct {
    uint8_t rx_buffer[DATA_BUFFER_SIZE];
    uint8_t received_byte; 					// Buffer to store received data
    volatile uint8_t data_received_flag;	// Flag to indicate data reception
    volatile uint8_t overflow_flag;			// Flag to indicate UART_Rx overflow
    uint8_t myAddress[8];						// Store Source address Low
} XBeeModule;
extern XBeeModule XBeeData;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;
extern uint8_t TxData_Presence[11];
extern uint8_t TxData_NoPresence[11];
extern uint8_t LoadStatus;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void FlashLED();
void IndicateErrorAndReset(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PIR_Pin GPIO_PIN_0
#define PIR_GPIO_Port GPIOA
#define PIR_EXTI_IRQn EXTI0_IRQn
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOA
#define Error_Pin GPIO_PIN_0
#define Error_GPIO_Port GPIOB
#define XBEE_SLEEP_Pin GPIO_PIN_8
#define XBEE_SLEEP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
void SetLowPowerMode(uint8_t enable);
void ToggleLED(uint16_t delay_ms, uint8_t count, uint8_t PVD);
void FlashLED(void);
void IndicateErrorAndReset(void);
void GracefulShutdown(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
