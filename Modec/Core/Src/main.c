/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t lastDebounceTime = 0;

uint8_t TxData_Presence[6] = {0x42, 0x26, 0x7F, 0x1C, 0xC0, 0x0F};
uint8_t TxData_NoPresence[6] = {0x42, 0x26, 0x7F, 0x1C, 0xC0, 0x0A};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void EnterLowPowerMode(void);
void FlashLED(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      // If presence is not detected or timer is inactive, enter low-power mode
      if (__HAL_TIM_GET_COUNTER(&htim2) == 0)
      {
        //  EnterLowPowerMode();
      }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 /* If detection pin is active, delay for 1 ms ...
  *
 */
	     if(GPIO_Pin == PIR_Pin)
	     {
	         /* Get the current time (in milliseconds) */
	         uint32_t currentTime = HAL_GetTick(); // HAL_GetTick() returns the system time in ms
	         /* Check if enough time has passed since the last press to consider this a valid press */
	         if((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS)
	         {
	        	 if(TIM2->CNT > 0)
	        	 {
	        		 TIM2->CNT = 0; //Reset timer
	        		 TIM2->CR1 |= TIM_CR1_CEN; // Enable the timer
	        	 }else
	        	 {
	        		 HAL_UART_Transmit(&huart1, TxData_Presence, sizeof(TxData_Presence), HAL_MAX_DELAY);
	        		 HAL_TIM_Base_Start_IT(&htim2);     /* Start 30 secs timer */
	        		 HAL_GPIO_WritePin(GPIOA, LED_Pin, SET);
	        	 }
        		 lastDebounceTime = currentTime;   /* Update the last debounce time */
	         }
	         HAL_PWR_DisableSleepOnExit();   // Exit low-power mode when PIR detects presence
	     }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // Check if the interrupt is from TIM2
  {
	  HAL_TIM_Base_Stop_IT(&htim2);     /* Start 30 secs timer */
	  HAL_UART_Transmit(&huart1, TxData_NoPresence, sizeof(TxData_NoPresence), HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(GPIOA, LED_Pin, RESET);

      HAL_PWR_EnableSleepOnExit();   // Set to enter low-power mode after timer expires

  }
}

/**
  * @brief  Flash LED to indicate low voltage
  */
void FlashLED(void)
{
    for(int i = 0; i < 10; i++)
    {
        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOA, LED_Pin);
        HAL_Delay(100);
    }
}

/**
  * @brief  Enter low-power (STOP) mode.
  */
void EnterLowPowerMode(void)
{
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);  // Turn off LED before sleep
    HAL_SuspendTick();  // Suspend SysTick for power saving
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);  // Enter STOP mode
    HAL_ResumeTick();  // Resume SysTick after wake-up
    //SystemClock_Config();  // Reconfigure system clock on wake-up
}

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
