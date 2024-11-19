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
#define ERROR_FILE_ADDRESS 0x0803FFFA
#define ERROR_LINE_ADDRESS 0x0803FFFB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t lastDebounceTime = 0;
volatile uint8_t ResetCount = 0;

uint8_t TxData_Presence[6] = {0x42, 0x26, 0x7F, 0x1C, 0xC0, 0x0F};
uint8_t TxData_NoPresence[6] = {0x42, 0x26, 0x7F, 0x1C, 0xC0, 0x0A};
volatile uint32_t ErrorFile;
volatile uint32_t ErrorLine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SetLowPowerMode(uint8_t enable);
void ToggleLED(uint16_t delay_ms, uint8_t count, uint8_t PVD);
void FlashLED(void);
void IndicateErrorAndReset(void);
void GracefulShutdown(void);
void StoreErrorCode(uint32_t code, uint32_t code2);
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
  //SetLowPowerMode(1);        // Enable low power on startup
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

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
	        	 }
        		 lastDebounceTime = currentTime;   /* Update the last debounce time */
	         }
	     }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // Check if the interrupt is from TIM2
  {
	  HAL_TIM_Base_Stop_IT(&htim2);     /* Start 30 secs timer */
	  HAL_UART_Transmit(&huart1, TxData_NoPresence, sizeof(TxData_NoPresence), HAL_MAX_DELAY);
	  //SetLowPowerMode(1);  // Enable low power on no presence

  }
}
/**
  * @brief  Flash LED with customizable delay and count
  * @param  delay_ms: Delay in milliseconds
  * @param  count: Number of times to toggle LED
  * @param	PVD: LED to toggle 1 for PVD 0 for error
  */
void ToggleLED(uint16_t delay_ms, uint8_t count, uint8_t PVD)
{
   if(PVD)
   {
		for (uint8_t i = 0; i < count; i++)
        {
			HAL_GPIO_TogglePin(GPIOA, LED_Pin);
			HAL_Delay(delay_ms);
			HAL_GPIO_TogglePin(GPIOA, LED_Pin);
			HAL_Delay(delay_ms);
        }
   }else
   {

		 for(uint8_t j = 0; j < count; j++)
		 {
		 	HAL_GPIO_TogglePin(Error_GPIO_Port, Error_Pin);
		    HAL_Delay(delay_ms);
		 	HAL_GPIO_TogglePin(Error_GPIO_Port, Error_Pin);
		 	HAL_Delay(delay_ms);
		 }
   }
}

/**
  * @brief  Flash LED to indicate low voltage
  */
void FlashLED(void)
{
    ToggleLED(100,5,1); // toggle PVD LED
}


/**
  * @brief Enter or Exit Low Power (STOP) Mode
  * @param enable: 1 to enable, 0 to disable
  */
void SetLowPowerMode(uint8_t enable)
{
    if (enable)
    {
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
        HAL_SuspendTick();

       // HAL_IWDG_Stop(&hiwdg); // Stop watchdog

        HAL_PWR_EnableSleepOnExit();
        HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

        HAL_ResumeTick();
        SystemClock_Config();

        // Re-enable Watchdog after exit from low power mode
       // HAL_IWDG_Start(&hiwdg);
    }
    else
    {
        HAL_PWR_DisableSleepOnExit();
    }
}
/**
 * @brief store error code in flash memory
 * @param code: code to be stored
 */
void StoreErrorCode(uint32_t code1, uint32_t code2)
{
    HAL_FLASH_Unlock();  // Unlock Flash for writing
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, ERROR_FILE_ADDRESS, code1);  // Write code to Flash
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, ERROR_LINE_ADDRESS, code2);  // Write code to Flash
    HAL_FLASH_Lock();    // Lock Flash after writing
}


/**
  * @brief  Graceful Shutdown Procedure
  */
void GracefulShutdown(void)
{
    ResetCount +=1;
    // Safely shut down peripherals or save data here before reset
    HAL_UART_DeInit(&huart1); // Deinitialize UART
    HAL_TIM_Base_Stop_IT(&htim2); // Stop Timer 2
    HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); // Turn off LED

    //  store status or error codes in non-volatile memory
    if (ErrorFile != 0xFFFFFFFF)
    	StoreErrorCode(ErrorFile, ErrorLine);  // Store error code in Flash

    HAL_Delay(500); // Allow time for final tasks
}

/**
  * @brief  Error handler with graceful shutdown and reset
  */
void IndicateErrorAndReset(void)
{
    GracefulShutdown();
    NVIC_SystemReset(); // Trigger a system reset

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
  //Visual Display in Error mode. Blink LED continuously
  	 ToggleLED(100, 5, 0); //Toggle Error LED
	 IndicateErrorAndReset();
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
	ErrorFile = file;
	ErrorLine = line;
	IndicateErrorAndReset();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
