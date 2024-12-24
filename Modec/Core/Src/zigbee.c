/*
 * Zigbee.c
 *
 *  Created on: Nov 25, 2024
 *      Author: Chidi Onwuka
 *
 *  Description:
 *  This file provides a set of functions to interface with and configure an XBee Zigbee module via UART.
 *  It includes functionality for:
 *    - Entering and exiting AT command mode
 *    - Querying and setting device parameters (e.g., serial number, destination address, power level)
 *    - Managing sleep modes
 *    - Sending commands and interpreting responses
 *
 *  Key Features:
 *    - Uses HAL UART for communication with the XBee module.
 *    - Supports interrupt-driven UART reception for asynchronous communication.
 *    - Includes error handling for command responses.
 *
 *  Function Overview:
 *    1. Command Mode Management:
 *       - `enterCommandMode()` - Enters AT command mode.
 *       - `exitCommandMode()` - Exits AT command mode.
 *
 *    2. Query Functions:
 *       - `requestParameter()` - Retrieves the AT configuration parameters of the XBee module. No need to
 *       enter or exit at command mode to use
 *       - `RQPowerLevel()` - Queries the transmit power level.
 *       - `RQSleepMode()` - Queries the current sleep mode setting.
 *
 *    3. Configuration Functions:
 *       - `setDestinationAddress()` - Sets the destination address (high and low).
 *       - `TxPowerLevel()` - Sets the transmit power level.
 *       - `SleepMode()` - Configures the sleep mode setting.
 *
 *  Usage:
 *  Ensure that `huart1` and necessary external buffers/flags are initialized before using these functions.
 */


#include <zigbee.h>




/**
 * @brief  Enter XBee AT Command Mode by sending "+++".
 */
void enterCommandMode(void)
{
    char command_mode[3] = "+++";
    // Send "+++" to enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);  // Small delay for XBee to respond
    // Receive the "OK" response from XBee
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_buffer, 3);
}


/**
 * @brief  Enter XBee AT Command Mode and request config parameter and exit. returns int response
 * @param at_command: AT command to enter
 * @param output_buffer: register to hold parameter requested
 * @param length: length of parameter requested
 */


int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length) {
    // Clear buffer and reset flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;

    char command_mode[] = "+++";
    char exit_command[] = "ATCN\r";

    // Define timeout duration (in milliseconds)
    const uint32_t timeout_duration = 2000; // 2 seconds

    // Enter AT command mode
    HAL_UART_Transmit(&huart1, (uint8_t *)command_mode, strlen(command_mode), HAL_MAX_DELAY);
    HAL_Delay(1000);
    HAL_UART_Receive_IT(&huart1, &received_byte, 3);

    // Send the parameter request command
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);

    //implement timeout for xbee response
   uint32_t start_time = HAL_GetTick();
    while (!data_received_flag) {
        if ((HAL_GetTick() - start_time) >= timeout_duration) {
            return XBEE_TIMEOUT_ERROR;
        }
    }

    if (strlen((char *)rx_buffer) < length) return XBEE_ERROR_RESPONSE;
    memcpy(output_buffer, rx_buffer, length);

    // Exit AT command mode
    data_received_flag = 0;
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    //implement timeout for xbee response
    start_time = HAL_GetTick();
    while (!data_received_flag) {
        if ((HAL_GetTick() - start_time) >= timeout_duration) {
            return XBEE_TIMEOUT_ERROR;
        }
    }

    return strncmp((char *)rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
}



/**
 * @brief  Set XBee Destination Address using ATDH and ATDL commands.
 * @param  DH: Destination High Address
 * @param  DL: Destination Low Address
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
 * @brief  Change the XBee Transmit Power Level (ATPL command).
 * @param  Level: Power level (0 to 4, where 4 is the highest).
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


/**
 * @brief  Request current power level (ATPL command).
 */
void RQPowerLevel()
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
    char at_command[] = "ATPL\r";  // Command to request Serial Number Low
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    // Wait for reception to complete
    while (!data_received_flag);
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
}

/*
  * @brief  Change XBee Sleep Mode (ATSM command).
  * @param  Level: Sleep mode level (0 to 6).
  * level 6 (Micropython sleep)
  * level 5 (Cyclic Pin-wake)
  * level 4 (Cyclic Sleep)
  * level 3 (Reserved)
  * level 2 (Reserved)
  * level 1 (pin hibernate)
  * level 0 (No Sleep mode)
  */
void SleepMode(uint8_t Level)
{
	char SM[10];
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
   // char at_command[] = "ATPL2";  // Command to request Serial Number Low
    // Format the AT commands
    snprintf(SM, sizeof(SM), "ATSM %01X\r", (unsigned int)Level);
    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)SM, strlen(SM), HAL_MAX_DELAY);
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


/**
 * @brief  Request current sleep mode (ATSM command).
 */
void RQSleepMode()
{
    // Clear rx_buffer and reset the data_received_flag
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
    char at_command[] = "ATSM\r";  // Command to request Serial Number Low
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &received_byte, 1);
    // Wait for reception to complete
    while (!data_received_flag);
    memset(rx_buffer, 0, Data_BUFFER_SIZE);
    data_received_flag = 0;
}


/**
 * @brief  Exit XBee AT Command Mode (ATCN command).
 */
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
