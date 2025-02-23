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

uint32_t start_time;


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
    HAL_UART_Receive_IT(&huart1, (uint8_t*)XBeeData.rx_buffer, 3);
}


/**
 * @brief  Enter XBee AT Command Mode and request config parameter and exit. returns int response
 * @param at_command: AT command to enter
 * @param output_buffer: register to hold parameter requested
 * @param length: length of parameter requested
 */


int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length)
{
    char exit_command[] = "ATCN\r";

    // Send the parameter request command
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    //implement timeout for xbee response
    start_time = HAL_GetTick();
    while (!XBeeData.data_received_flag)
    {
        if ((HAL_GetTick() - start_time) >= XBEE_TIMEOUT_DURATION)
        {
            return XBEE_TIMEOUT_ERROR;
        }
    }

    if (strlen((char *)XBeeData.rx_buffer) < length) return XBEE_ERROR_RESPONSE;
    memcpy(output_buffer, XBeeData.rx_buffer, length);

    // Exit AT command mode
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    //implement timeout for xbee response
    start_time = HAL_GetTick();
    while (!XBeeData.data_received_flag)
    {
        if ((HAL_GetTick() - start_time) >= XBEE_TIMEOUT_DURATION) {
            return XBEE_TIMEOUT_ERROR;
        }
    }

    return strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
}

/**
 * @brief  Enter XBee AT Command Mode and set new config parameter and exit. returns int response
 * @param at_command: AT command to enter with parameter to set
 *
 */

int setParameter(const char *at_command)
{
    char command_mode[] = "+++";
    char write_command[] = "ATWR\r";
    char exit_command[] = "ATCN\r";

    // Send the parameter request command
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    //implement timeout for xbee response
    start_time = HAL_GetTick();
    while (!XBeeData.data_received_flag)
    {
        if ((HAL_GetTick() - start_time) >= XBEE_TIMEOUT_DURATION)
        {
            return XBEE_TIMEOUT_ERROR;
        }
    }

    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
    	 // Reset flag and buffer
    	 XBeeData.data_received_flag = 0;
    	 memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);

    	 // Save changes with ATWR command
    	 HAL_UART_Transmit(&huart1, (uint8_t *)write_command, strlen(write_command), HAL_MAX_DELAY);
    	 HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    	 // Wait for reception to complete
    	 while (!XBeeData.data_received_flag);
    	 if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0){
        	 // Reset flag and buffer
        	 XBeeData.data_received_flag = 0;
        	 memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        	 // Exit AT command mode
        	 HAL_UART_Transmit(&huart1, (uint8_t *)exit_command, strlen(exit_command), HAL_MAX_DELAY);
        	 HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
        	 // Wait for reception to complete
        	 while (!XBeeData.data_received_flag);
    	 }
    }

    return strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
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
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;

    // Transmit ATDH command
    HAL_UART_Transmit(&huart1, (uint8_t *)at_high, strlen(at_high), HAL_MAX_DELAY);

    // Enable reception interrupt
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

    // Wait for reception to complete
    while (!XBeeData.data_received_flag);

    // Check response for ATDH
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
        // Reset flag and buffer
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);

        // Transmit ATDL command
        HAL_UART_Transmit(&huart1, (uint8_t *)at_low, strlen(at_low), HAL_MAX_DELAY);

        // Enable reception interrupt
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

        // Wait for reception to complete
        while (!XBeeData.data_received_flag);

        // Check response for ATDL
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
        {
            // Reset flag and buffer
        	XBeeData.data_received_flag = 0;
            memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);

            // Save changes with ATWR command
            HAL_UART_Transmit(&huart1, (uint8_t *)"ATWR\r", 5, HAL_MAX_DELAY);
            HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

            // Wait for reception to complete
            while (!XBeeData.data_received_flag);

            // Check response for ATWR
            if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
            {
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
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
   // char at_command[] = "ATPL2";  // Command to request Serial Number Low
    // Format the AT commands
    snprintf(PL, sizeof(PL), "ATPL %01X\r", (unsigned int)Level);
    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)PL, strlen(PL), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
        // Wait for reception to complete
        while (!XBeeData.data_received_flag);
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
        {
        	// Handle memory write failure
             printf("Failed to write changes to memory!\n");
       }
    }
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}


/**
 * @brief  Request current power level (ATPL command).
 */
void RQPowerLevel()
{
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
    char at_command[] = "ATPL\r";  // Command to request Serial Number Low
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
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
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
   // char at_command[] = "ATPL2";  // Command to request Serial Number Low
    // Format the AT commands
    snprintf(SM, sizeof(SM), "ATSM %01X\r", (unsigned int)Level);
    char write[] = "ATWR\r";
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)SM, strlen(SM), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
    {
    	XBeeData.data_received_flag = 0;
        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
        // Wait for reception to complete
        while (!XBeeData.data_received_flag);
        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
        {
        	// Handle memory write failure
             printf("Failed to write changes to memory!\n");
       }
    }
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}


/**
 * @brief  Request current sleep mode (ATSM command).
 */
void RQSleepMode()
{
    // Clear rx_buffer and reset the data_received_flag
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
    char at_command[] = "ATSM\r";  // Command to request Serial Number Low
    //send ATPL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
    XBeeData.data_received_flag = 0;
}

/**
 * Node Discovery is used to discover devices within the XBee network.
 * When issued, the local XBee module scans the network and reports details about each discovered device,
 * including its addresses, Node Identifier, parent address, and other parameters.
 * it also increments ny1 the number of devices found
 */
int XBee_NodeDiscovery()
{
	// Clear buffer and reset flag
	  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	  XBeeData.data_received_flag = 0;

	  HAL_UART_Transmit(&huart1, (uint8_t *)"ATND\r", 5, 1000); // send command for ATND
	  HAL_Delay(2000);
	  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	  for(int i = 0; i < MAX_DEVICES; i++){
		  //implement timeout for xbee response
		  start_time = HAL_GetTick();
		  while (!XBeeData.data_received_flag)
		  {
		      if ((HAL_GetTick() - start_time) >= XBEE_TIMEOUT_DURATION)
		      {
		         return XBEE_TIMEOUT_ERROR;
		      }
		  }
		  if(memcmp(XBeeData.rx_buffer, "\r", 1) == 0) break; //end of discovery
		  memcpy(newNode[i].NetAddress, XBeeData.rx_buffer, sizeof(newNode[i].NetAddress)); // store Network Address
		  //clear buffer and reset flag
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].SerialHigh, XBeeData.rx_buffer, sizeof(newNode[i].SerialHigh)); //store Serial # High
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].SerialLow, XBeeData.rx_buffer, sizeof(newNode[i].SerialLow)); //store serial # Low
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].NodeID, XBeeData.rx_buffer, sizeof(newNode[i].NodeID));		//store Node Identifier
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].pAddress, XBeeData.rx_buffer, sizeof(newNode[i].pAddress));		//store parent Address
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].dType, XBeeData.rx_buffer, sizeof(newNode[i].dType));		//store device type
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].RSSI, XBeeData.rx_buffer, sizeof(newNode[i].RSSI));			//store RSSI
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].pID, XBeeData.rx_buffer, sizeof(newNode[i].pID));		//profile ID
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memcpy(newNode[i].manID, XBeeData.rx_buffer, sizeof(newNode[i].manID));	//manufacturer ID
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
		  while (!XBeeData.data_received_flag);
		  memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);			//receive and reset carriage return (signifies end of one device spec)
		  XBeeData.data_received_flag = 0;
		  HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);

		  deviceCount++; //increment # of devices discovered

		    // Check if the maximum device limit is reached
		    if (deviceCount >= MAX_DEVICES) break;
	  }
	  return XBEE_SUCCESS;
}
/*
 * Factory reset chip. Enter command mode and Exit with independent functions
 */
void factoryReset(){
	// Clear rx_buffer and reset the data_received_flag
	memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	XBeeData.data_received_flag = 0;
	char at_command[] = "ATRE\r";  // Command for factory reset
	char write[] = "ATWR\r";		// Command to write to NVMe
	//send ATPL command
	HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	while (!XBeeData.data_received_flag);
	if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0)
	    {
			XBeeData.data_received_flag = 0;
	        memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	        HAL_UART_Transmit(&huart1, (uint8_t*)write, strlen(write), HAL_MAX_DELAY);
	        HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
	        // Wait for reception to complete
	        while (!XBeeData.data_received_flag);
	        if (strncmp((char *)XBeeData.rx_buffer, "OK", 2) != 0)
	        {
	        	// Handle memory write failure
	             printf("Failed to write changes to memory!\n");
	       }
	    }
	    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
	    XBeeData.data_received_flag = 0;
}

int writeCommand(){
    char at_command[] = "ATWR\r";  // Command to write to XBEE EEPROM
             // Reset flag and buffer
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
        	 // Send the ATSL command
    HAL_UART_Transmit(&huart1, (uint8_t*)at_command, strlen(at_command), HAL_MAX_DELAY);
        	 // Receive the response (Serial Number Low)
    HAL_UART_Receive_IT(&huart1, &XBeeData.received_byte, 1);
    while(!XBeeData.data_received_flag); //wait for Rx to complete

    return strncmp((char *)XBeeData.rx_buffer, "OK", 2) == 0 ? XBEE_SUCCESS : XBEE_ERROR_RESPONSE;
}

/**
 * @brief  Exit XBee AT Command Mode (ATCN command).
 */
void exitCommandMode(void)
{
    char exit_command[] = "ATCN\r";  // Command to exit AT command mode

    // Send ATCN command to exit command mode
    HAL_UART_Transmit(&huart1, (uint8_t*)exit_command, strlen(exit_command), HAL_MAX_DELAY);
    HAL_UART_Receive_IT(&huart1, (uint8_t*)XBeeData.rx_buffer, 3);
    // Wait for reception to complete
    while (!XBeeData.data_received_flag);
    XBeeData.data_received_flag = 0;
    memset(XBeeData.rx_buffer, 0, DATA_BUFFER_SIZE);
}
