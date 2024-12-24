/*
 * Zigbee.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Chidi Onwuka
 */

#ifndef INC_ZIGBEE_H_
#define INC_ZIGBEE_H_

#include "main.h"
#include <stdio.h>
#include <string.h>

#define Data_BUFFER_SIZE 12
#define XBEE_SUCCESS        0
#define XBEE_ERROR_RESPONSE 1
#define XBEE_TIMEOUT_ERROR  2

// External declarations for UART handle and buffers
extern UART_HandleTypeDef huart1;          // UART handle
extern uint8_t rx_buffer[Data_BUFFER_SIZE]; // Buffer to store received data
extern uint8_t received_byte;              // Variable to store single received byte
extern uint8_t mySerialLow[8];             // Array to store Serial Number Low
extern uint8_t myDestLow[8];               // Array to store Destination Number Low
extern volatile uint8_t data_received_flag; // Flag to indicate data reception completion

void enterCommandMode(void);
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void TxPowerLevel(uint8_t Level);
void RQPowerLevel();
void SleepMode(uint8_t Level);
void RQSleepMode();
void exitCommandMode(void);



#endif /* INC_ZIGBEE_H_ */
