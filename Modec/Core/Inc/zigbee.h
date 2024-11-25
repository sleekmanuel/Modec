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

void enterCommandMode(void);
void requestSerialNumberLow(void);
void requestDestNumberLow(void);
void setDestinationAddress(uint32_t DH, uint32_t DL);
void TxPowerLevel(uint8_t Level);
void RQPowerLevel();
void SleepMode(uint8_t Level);
void RQSleepMode();
void exitCommandMode(void);

#endif /* INC_ZIGBEE_H_ */
