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
  * Initialization:
	System clock, peripherals (GPIO, USART, Timer), and Zigbee module configuration.
  * Low Power Mode:
	Functions to manage low-power mode by enabling/disabling STOP mode and controlling
	the Zigbee module sleep state.
  * Zigbee Communication:
	Configures Zigbee parameters like sleep mode, power level, and destination address.
	Sends and receives data through UART, processes commands, and responds accordingly.
  * Interrupt Handlers:
	Handles UART reception and GPIO interrupts for detecting events such as motion
	(PIR sensor input).
  * Error Handling:
	Implements robust error handling with flashing LEDs and error logging into flash memory,
	followed by a system reset.
  * LED Indications:
	LEDs are used to signal errors, low voltage, and operational states.
  * Timers:
	Utilizes a timer to track activity periods (e.g., 15 seconds of device activation).
  ******************************************************************************
  */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <zigbee.h>
#include <stdlib.h>
#include <mcp9808.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t Control;	 //used to determine if message is a request or command
    uint8_t Data;
    uint8_t DestAddress[8];	// Transmission data
} ZigbeeMessage;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define XBEE_SERIAL_LOW_ADDRESS  0x0803FF70 // Flash memory address for XBee serial low
#define ADDRESS_HIGH 0x13A200  // High address on Xbee devices

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t txData_Presence[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0xC0, 0x0F, 0x0D};
uint8_t txData_NoPresence[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0xC0, 0x0A, 0x0D};
uint8_t loadStatus = 0;
uint8_t serialLowBuffer[8] = {0};
uint64_t serialLow = 0;
uint64_t flashData =0;
uint8_t deviceCount = 1;
volatile uint8_t DestIndex = 15; 	//initialized with arbitary number above MAX DEVICE
float ambientTemperature = 0.0f;			//Ambient temperature reading
uint8_t tempRead = 0;				//initialize temperature reading status
uint8_t tUpper;						//Upper temperature limit
uint8_t tLower;						//lower temperature limit
uint8_t tCritical;					//critical temperature limit

ZigbeeMessage receivedMessage = // Instance and Initialization of the ZigbeeMessage typedef structure
{
		.Control = 0,
		.Data = 0,
		.DestAddress = {0}
};
XBeeModule XBeeData =
{			// Initialize UART receive variables
		.rx_buffer = {0},
		.received_byte = 0,
		.data_received_flag = 0,
		.overflow_flag = 0,
		.myAddress = {0}
};

NodeDiscovery newNode[MAX_DEVICES];
NodeDiscovery router;
//create instance of XBeeModule typedef struct
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RequestAndStoreSerialLow(void);
void EraseFlashSector(uint32_t address);
void WriteFlash(uint32_t address, uint64_t data);
void uint64ToUint8Array(uint64_t value, uint8_t array[8]);
//void WriteFlash64(uint32_t address, uint64_t data);
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
  MX_I2C1_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  /* --------------------------Zigbee Configuration Begin-----------------------------------------*/
  /*
   * ONLY USE FUNCTIONS DURING INITIAL CONFIGURATION OR MAKING CHANGES TO CONFIGURATION
   * COMMENT OUT FUNCTIONS WHEN NOT IN USE
   * STORE FLASHDATA FOR SERIAL DATA LOW FOR OPERATIONAL USE
   */
  // Request and store XBee serial low
   //  RequestAndStoreSerialLow();


  /*..........Set Destination Address..........
   * Use ADDRESS_HIGH for DH
   * setDestinationAddress(ADDRESS_HIGH, 0x4236C1F7);
   */

   /*..........Check and Set Sleep Mode Levels.............
    * level 1 (pin hibernate)
    * level 0 (No Sleep mode)
    * SleepMode(1);
    * RQSleepMode();
    .........................................*/

  /*..........Check & Set Tx Power level.............
   * Power levels 4 (highest) - 0 (lowest)
   * TxPowerLevel(2); SET
   * RQPowerLevel();  CHECK
   .........................................*/

  enterCommandMode();
  XBee_NodeDiscovery();
  exitCommandMode();
//
  flashData= *(uint64_t *)XBEE_SERIAL_LOW_ADDRESS; //Store serial low number from flash memory
  uint64ToUint8Array(flashData,  XBeeData.myAddress); // Convert Data to Array

  //search discovered node for it's router
  for(int a = 0; a < deviceCount; a++)
  {
	  if(strncmp(newNode[a].NodeID, "Switch01", strlen("Switch01")) == 0)		//Check for a device counterpart
	  {
		  if(memcmp(newNode[a].dType, "01", 2) == 0)		//Check if device is a router
		  {
			  DestIndex = a;		//store router index
			  HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);
			  HAL_Delay(3000);
			  HAL_GPIO_WritePin(GPIOA, LED_Pin, 0);
			   HAL_Delay(3000);
			  break;
		  }else{
			  printf("Device %d is not a Router\n", a);
			 // HAL_GPIO_WritePin(GPIOA, LED_Pin, 1);
		  }

	  }else{
		  printf("Node %d does not match 'Switch01'\n", a);

	  }
  }


  // Handle case where no router is found
  if (DestIndex == 15)
  {
      printf("No router found. Exiting.\n");
      // Handle the error (e.g., retry discovery or halt)
      //HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, 1);
  }else{
      // Copy router details
      memcpy(&router, &newNode[DestIndex], sizeof(NodeDiscovery));
      //copy router address to presence and no presence data
      memcpy(txData_Presence, router.SerialLow, ADDRESS_SIZE);
      memcpy(txData_NoPresence, router.SerialLow, ADDRESS_SIZE);
 }
  /* --------------------------Zigbee Configuration End-------------------------------------------*/



  /* --------------------------MCP9808 Configuration Start-------------------------------------------*/

  if (MCP9808_IsConnected() == HAL_OK) {
      // Initialize the temperature sensor
      if (MCP9808_Init() == HAL_OK) {

          // Read ambient temperature
          while (!tempRead) {
              if (MCP9808_ReadTemperature(&ambientTemperature) == HAL_OK) {
                  tempRead = 1; // Successfully read temperature
              } else {
                  // Handle error (blink error LED)
                  HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_SET);
                  HAL_Delay(3000);
                  HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_RESET);
              }
          }

          // Set thresholds
          tUpper = ambientTemperature + 10;
          tLower = ambientTemperature - 10;
          tCritical = ambientTemperature + 30;

          if (MCP9808_ConfigureInterrupts(tUpper, tLower, tCritical) != HAL_OK) {
              // Handle configuration error
              HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_SET);
              HAL_Delay(3000);
              HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_RESET);
          }
      } else {
          HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_SET);
          HAL_Delay(3000);
          HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_RESET);
      }
  } else {
      HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_SET);
      HAL_Delay(3000);
      HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_RESET);
  }

 /* --------------------------MCP9808 Configuration End-------------------------------------------*/

  //    // Indicate Device is ready to run
      HAL_GPIO_WritePin(Active_LED_GPIO_Port, Active_LED_Pin, GPIO_PIN_SET);
  //    HAL_Delay(1000);
  //    HAL_GPIO_WritePin(Active_LED_GPIO_Port, Active_LED_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // SetLowPowerMode(1);  // Enable low power
  while (1)
  {
//	  /*Configure GPIO pin Output Level */
//	  HAL_GPIO_WritePin(Error_GPIO_Port, Error_Pin, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);

 	  if(XBeeData.data_received_flag)
	  {		//get zigbee message from xbee module
 		  memcpy(receivedMessage.DestAddress, XBeeData.rx_buffer, 8);	//copy destination address to zigbee message
		  receivedMessage.Control = XBeeData.rx_buffer[8];
		  receivedMessage.Data = XBeeData.rx_buffer[9];
		  //Check if the message is meant for me
		  if(memcmp(XBeeData.myAddress, receivedMessage.DestAddress, 8) == 0)
		  {

			  if(receivedMessage.Control == 0xB3)
			  {
				  if(receivedMessage.Data == 0x11)
				 {
				 	loadStatus = 1;					// Feedback: Load is active
				 	HAL_TIM_Base_Start_IT(&htim2);     /* Start 15 secs timer */

				 }else if(receivedMessage.Data == 0xAA)
				 	{
					 	 loadStatus = 0;			// Feedback: Load is inactive
					 	// SetLowPowerMode(1);  // Enable low power on no presence
				 	}else
				 	{;}
			  	}
	     }
		  XBeeData.data_received_flag = 0;  // resets received status to expect new data
		 HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);  // Continue receiving
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
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//Convert a uint64_t  into a uint8_t array in little-endian order
void uint64ToUint8Array(uint64_t value, uint8_t array[8]) {
    for (int i = 0; i < 8; i++) {
        array[i] = (value >> (i * 8)) & 0xFF; // Extract each byte
    }
}

/*
 * Receive interrupt callback function
 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t index = 0;

    if (huart->Instance == USART1)
    {
    	SetLowPowerMode(0); //Exit Low Power Mode
        if (index < DATA_BUFFER_SIZE - 1)
        {
        	XBeeData.rx_buffer[index++] = XBeeData.received_byte;

            if (XBeeData.received_byte == '\r')// End of response
            {
            	XBeeData.data_received_flag = 1;
            	XBeeData.rx_buffer[index] = '\0';  // Null-terminate
                index = 0;  // Reset for next reception
            }
        } else
        {
        	XBeeData.overflow_flag = 1;  // Signal buffer overflow
            XBeeData.rx_buffer[index] = '\0';	// Null-terminate
            index = 0; //reset the buffer
        }

        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);  // Continue receiving
    }
}

/**
  * @brief Enter or Exit Low Power (STOP) Mode. Set or Reset XBee from sleep mode
  * @param enable: 1 to enable, 0 to disable
  */
void SetLowPowerMode(uint8_t enable)
{
    if (enable)
    {
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, XBEE_SLEEP_Pin, GPIO_PIN_SET); //XBee to enter sleep mode

        // Enter STOP mode
        HAL_SuspendTick();
        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

        // Exit STOP mode
        HAL_ResumeTick();
        SystemClock_Config();
        HAL_GPIO_WritePin(GPIOA, XBEE_SLEEP_Pin, GPIO_PIN_RESET); //XBee to exit sleep mode

    }
    else
    {
        HAL_PWR_DisableSleepOnExit();
    }
}
/* --------------------------Flash Programming Start-------------------------------------------*/

/*
 *  @brief Store XBEE serial number to flash memory
 *  @serialLow 	 64bit serial numberto store in flash
 */
void StoreXBeeSerialLow(uint64_t serialLow)
{
    uint32_t address = XBEE_SERIAL_LOW_ADDRESS;

    // Step 1: Erase the Flash page
    EraseFlashSector(address);

    // Step 2: Write the 32-bit serialLow to Flash
    WriteFlash(address, serialLow);
}

/*
 *  @brief Request serial low number via XBee AT command and store in memory
 */
void RequestAndStoreSerialLow(void)
{
    // Request serial low number
    if(requestParameter("ATSL\r", serialLowBuffer, sizeof(serialLowBuffer)) == XBEE_SUCCESS)
    {
    	for (int i = 0; i < 8; i++)		// Convert received buffer to 32-bit integer (assume little-endian format)
    	{
    		serialLow |= ((uint64_t)serialLowBuffer[i] << (8 * i)); //little-endian
    		//serialLow |= ((uint64_t)serialLowBuffer[i] << (8 * (7 - i))); //big-endian
    	}
    	StoreXBeeSerialLow(serialLow);		// Store the serial number in Flash memory
    }else{
        // Indicate failure via LED blink for 2 sec
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
    }
}

/*
 *  @brief Erase flash sector to prep for memory programming
 *  @address Starting address to flash sector
 */
void EraseFlashSector(uint32_t address)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;

    // Convert the address to a page number
    uint64_t page = (address - FLASH_BASE) / FLASH_PAGE_SIZE;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = page;             // Page number for newer MCUs
    eraseInitStruct.NbPages = 1;             // Number of pages to erase

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return;
    }

    HAL_FLASH_Lock();
}

/*
 *  @brief Write to flash memory after erase
 *  @addressFlash memory address to be written into
 *  @data 64bit data to be written to memory
 */
void WriteFlash(uint32_t address, uint64_t data)
{
    HAL_FLASH_Unlock();

    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) != HAL_OK)
    {

        // Indicate failure via LED blink for 1 sec
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(GPIOA, LED_Pin, GPIO_PIN_RESET);
    }

    HAL_FLASH_Lock();
}
/* --------------------------Flash Programming End-------------------------------------------*/


/**
  * @brief  Graceful Shutdown Procedure
  */
void GracefulShutdown(void)
{
    // Safely shut down peripherals or save data here before reset
    HAL_UART_DeInit(&huart1); // Deinitialize UART
    HAL_TIM_Base_Stop_IT(&htim2); // Stop Timer 2
    HAL_I2C_MspDeInit(&hi2c1);	//Deinitialize i2c

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
  	 ToggleLED(100, 10, 0); //Toggle Error LED
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

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
