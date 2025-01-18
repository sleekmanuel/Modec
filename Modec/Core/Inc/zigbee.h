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

//#define Data_BUFFER_SIZE 12
#define XBEE_SUCCESS        0
#define XBEE_ERROR_RESPONSE 1
#define XBEE_TIMEOUT_ERROR  2
#define XBEE_TIMEOUT_DURATION 5000

// External declarations for UART handle and buffers
extern UART_HandleTypeDef huart1;          // UART handle

void enterCommandMode(void);
int requestParameter(const char *at_command, uint8_t *output_buffer, size_t length);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void TxPowerLevel(uint8_t Level);
void RQPowerLevel();
int setParameter(const char *at_command);
void SleepMode(uint8_t Level);
void RQSleepMode();
int XBee_NodeDiscovery(void);
void FactoryReset(void);
void exitCommandMode(void);



#endif /* INC_ZIGBEE_H_ */
