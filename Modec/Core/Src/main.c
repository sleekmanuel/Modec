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
#include <stdio.h>
#include <string.h>
#include "math.h"
#include <stdint.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS 1
#define ERROR_FILE_ADDRESS 0x0803FFFA
#define ERROR_LINE_ADDRESS 0x0803FFFB
#define Data_BUFFER_SIZE 12 // Transmission Buffer size
#define ADDRESS_HIGH 0x13A200  // High address on Xbee devices
#define MAX_STRING_LENGTH 2   // Each character string will be 2 bytes long (1 character + null terminator)
#define MAX_PAIR_LENGTH 3     // Each paired string will be 3 bytes long (2 characters + null terminator)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t lastDebounceTime = 0;
volatile uint8_t data_received_flag = 0;  // Flag to indicate data reception
volatile uint8_t overflow_flag = 0;		  // Flag to indicate UART_Rx overflow
volatile uint32_t ErrorFile;
volatile uint32_t ErrorLine;

uint8_t TxData_Presence[11] = {0x34, 0x32, 0x33, 0x36, 0x43, 0x31, 0x46, 0x37, 0xC0, 0x0F, 0x0D};
uint8_t TxData_NoPresence[11] = {0x34, 0x32, 0x33, 0x36, 0x43, 0x31, 0x46, 0x37, 0xC0, 0x0A, 0x0D};
char* str = "Woke up from interrupt";
uint8_t mySerialLow[8];       // Store Source address Low
uint8_t myDestLow[8];			// store destination address low
uint8_t Control;                //used to determine if message is a request or command
uint8_t Data;				   // Transmission data
uint8_t received_byte;		  // Process UART_Rx by byte
uint8_t rx_buffer[Data_BUFFER_SIZE];   // Buffer to store received data
uint8_t LoadStatus = 0;
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
void enterCommandMode(void);
void requestSerialNumberLow(void);
void requestDestNumberLow(void);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void TxPowerLevel(uint8_t Level);
void exitCommandMode(void);


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

  /* --------------------------Zigbee Configuration Begin------------------------------------------------*/

  // enter AT command mode
  enterCommandMode();
  //Request and store XBee Serial Number Low
  requestSerialNumberLow();
  /*..........Set Tx Power level.............
   * Power levels 4 (highest) - 0 (lowest)
   * TxPowerLevel(2);
   .........................................*/

  /*..........Set Destination Address..........
   * Use ADDRESS_HIGH for DH
   * setDestinationAddress(ADDRESS_HIGH, 0x4236C1F7);
   */

  // Exit command mode
  exitCommandMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  SetLowPowerMode(1);  // Enable low power on no presence
  while (1)
  {
	  if(data_received_flag)
	  {
		  //Check if the message is meant for me
		  if(memcmp(mySerialLow, rx_buffer, 8) == 0){
			  Control = rx_buffer[8];
			  // extract command information
			  Data = rx_buffer[9];
			  if(Control == 0xB3)
			  {
				  if(Data == 0x11)
				 {
				 	LoadStatus = 1;					// Feedback: Load is active
				 	HAL_TIM_Base_Start_IT(&htim2);     /* Start 30 secs timer */
				 	HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
				 }else if(Data == 0xAA)
				 	{
					 	 LoadStatus = 0;			// Feedback: Load is inactive
				 	}else
				 	{;}
			  	}
	     }
		 data_received_flag = 0;  // resets received status to expect new data
		 HAL_UART_Receive_IT(&huart1, &received_byte, 1);  // Continue receiving
	  }
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

void enterCommandMode(void)
{
    char command_mode[3] = "+++";
    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    // Receive the "OK" response from XBee
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 3);
}

// Function to request XBee Serial Number Low (ATSL)
void requestSerialNumberLow(void)
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char at_command[] = "ATSL\r";  // Command to request Serial Number Low
    // Send the ATSL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    // Receive the response (Serial Number Low)
    HAL_UART_Receive_IT(&huart1,  &received_byte, 1);
    while(!data_received_flag); //wait for Rx to complete
    memcpy(mySerialLow, rx_buffer, 8);  // Move the received data to the transmission buffer
    data_received_flag = 0; //reset receive flag
}
// Function to request XBee Serial Number Low (ATSL)
void requestDestNumberLow(void)
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char at_command[] = "ATDL\r";  // Command to request Serial Number Low
    // Send the ATSL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    // Receive the response (Serial Number Low)
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    while(!data_received_flag); //wait for Rx to complete
    memcpy(myDestLow, rx_buffer, 8);  // Move the received data to the transmission buffer
    data_received_flag = 0; //reset receive flag
}
/**
  * @brief  AT command function to Set Destination address
  * @param  DH: Destination Address High
  * @param  DL: Destination Address Low
  */
void setDestinationAddress(uint32_t DH, uint32_t DL)
{
    char at_high[20];
    char at_low[20];

    // Format the AT commands
    snprintf(at_high, sizeof(at_high), "ATDH %08X\r", (unsigned int)DH);
    snprintf(at_low, sizeof(at_low), "ATDL %08X\r", (unsigned int)DL);

    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    // Transmit ATDH command
    HAL_UART_Transmit(&huart1, (uint8_t *)at_high, strlen(at_high), HAL_MAX_DELAY);

    // Enable reception interrupt
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    // Wait for reception to complete
    while (!data_received_flag);

    // Check response for ATDH
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
        // Reset flag and buffer
        data_received_flag = 0;
        memset(rx_buffer, 0, Data_BUFFER_SIZE);

        // Transmit ATDL command
        HAL_UART_Transmit(&huart1, (uint8_t *)at_low, strlen(at_low), HAL_MAX_DELAY);

        // Enable reception interrupt
        HAL_UART_Receive_IT(&huart1, &received_byte, 1);

        // Wait for reception to complete
        while (!data_received_flag);

        // Check response for ATDL
        if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
            // Reset flag and buffer
            data_received_flag = 0;
            memset(rx_buffer, 0, Data_BUFFER_SIZE);

            // Save changes with ATWR command
            HAL_UART_Transmit(&huart1, (uint8_t *)"ATWR\r", 5, HAL_MAX_DELAY);
            HAL_UART_Receive_IT(&huart1, &received_byte, 1);

            // Wait for reception to complete
            while (!data_received_flag);

            // Check response for ATWR
            if (strncmp((char *)rx_buffer, "OK", 2) != 0) {
                // Handle memory write failure
                printf("Failed to write changes to memory!\n");
            }
        } else {
            // Handle ATDL failure
            printf("Failed to set destination low address!\n");
        }
    } else {
        // Handle ATDH failure
        printf("Failed to set destination high address!\n");
    }
}
/**
  * @brief  AT command function to Transmit power level
  * @param  Level: Transmit power level (4) highest (0) lowest
  */
void TxPowerLevel(uint8_t Level)
{
	char PL[10];
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
   // char at_command[] = "ATPL2";  // Command to request Serial Number Low
    // Format the AT commands
    snprintf(PL, sizeof(PL), "ATPL %01X\r", (unsigned int)Level);
    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)PL, strlen(PL), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    // Wait for reception to complete
    while (!data_received_flag);
    if (strncmp((char *)rx_buffer, "OK", 2) == 0) {
        data_received_flag = 0;
        memset(rx_buffer, 0, Data_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart1, &received_byte, 1);
        // Wait for reception to complete
        while (!data_received_flag);
        if (strncmp((char *)rx_buffer, "OK", 2) != 0) {
        	// Handle memory write failure
             printf("Failed to write changes to memory!\n");
       }
    }
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
}
// Function to exit XBee AT Command Mode
void exitCommandMode(void)
{
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Send ATCN command to exit command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 3);
    // Wait for reception to complete
    while (!data_received_flag);
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
}

/*
 * Receive interrupt callback function
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    static uint8_t index = 0;

    if (huart->Instance == USART1) {
    	SetLowPowerMode(0); //Exit Low Power Mode
        if (index < Data_BUFFER_SIZE - 1) {
            rx_buffer[index++] = received_byte;

            if (received_byte == '\r') {  // End of response
                data_received_flag = 1;
                rx_buffer[index] = '\0';  // Null-terminate
                index = 0;  // Reset for next reception
            }
        } else {
            overflow_flag = 1;  // Signal buffer overflow
            index = 0;  // Optionally reset the buffer
        }

        HAL_UART_Receive_IT(&huart1, &received_byte, 1);  // Continue receiving
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 /* If detection pin is active, delay for 1 ms ...
  *
 */
   		if(GPIO_Pin == PIR_Pin)
	     {
   			SetLowPowerMode(0); //Exit Low Power Mode
	         /* Get the current time (in milliseconds) */
	         uint32_t currentTime = HAL_GetTick(); // HAL_GetTick() returns the system time in ms
	         /* Check if enough time has passed since the last press to consider this a valid press */
	         if((currentTime - lastDebounceTime) >= DEBOUNCE_DELAY_MS)
	         {
	        	 if(!LoadStatus){
	        		 if(TIM2->CNT > 0)
	        		 {
	        			 TIM2->CNT = 0; //Reset timer
	        			 TIM2->CR1 |= TIM_CR1_CEN; // Enable the timer
	        			 HAL_UART_Transmit(&huart1, TxData_Presence, sizeof(TxData_Presence), HAL_MAX_DELAY);
	        		 }else
	        		 {
	        			 HAL_UART_Transmit(&huart1, TxData_Presence, sizeof(TxData_Presence), HAL_MAX_DELAY);
	        			 HAL_TIM_Base_Start_IT(&htim2);     /* Start 30 secs timer */
	        		 }
	        	 }else{
	        		 TIM2->CNT = 0; //Reset timer
	        		 TIM2->CR1 |= TIM_CR1_CEN; // Enable the timer
	        	 }
	        	 lastDebounceTime = currentTime;   /* Update the last debounce time */
	         }
	     }

}


// Turn off Switch after 30 secs and enter low power mode

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // Check if the interrupt is from TIM2
  {
	  HAL_TIM_Base_Stop_IT(&htim2);     /* Start 30 secs timer */
	  HAL_UART_Transmit(&huart1, TxData_NoPresence, sizeof(TxData_NoPresence), HAL_MAX_DELAY);
	  SetLowPowerMode(1);  // Enable low power on no presence

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
        }
   }else
   {

		 for(uint8_t j = 0; j < count; j++)
		 {
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
    ToggleLED(500,5,1); // toggle PVD LED
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
        // Set GPIOs to analog mode for unused pins
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // Enter STOP mode
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

        // Exit STOP mode
        HAL_ResumeTick();
        SystemClock_Config();

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
    // Safely shut down peripherals or save data here before reset
    HAL_UART_DeInit(&huart1); // Deinitialize UART
    HAL_TIM_Base_Stop_IT(&htim2); // Stop Timer 2
   // HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET); // Turn off LED

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
