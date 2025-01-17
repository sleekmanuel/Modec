/*
 * mcp9808.h
 *
 *  Created on: Jan 15, 2025
 *      Author: root
 */

#ifndef INC_MCP9808_H_
#define INC_MCP9808_H_

#include "stm32l4xx_hal.h"
#include "main.h"
#include <stdint.h>
#include "i2c.h"

// MCP9808 I2C Address (default, 7-bit)
#define MCP9808_ADDR         0x18

// MCP9808 Registers
#define MCP9808_REG_TEMP     0x05
#define MCP9808_REG_CONFIG   0x01
#define MCP9808_REG_ALERT_UPPER 0x02
#define MCP9808_REG_ALERT_LOWER 0x03
#define MCP9808_REG_ALERT_CRIT  0x04
#define MCP9808_REG_ALERT_MASK  0x08

// MCP9808 Configuration Values
#define MCP9808_CONFIG_DEFAULT  0x0000
#define MCP9808_CONFIG_INT_ENABLE  0x0002
#define MCP9808_CONFIG_ALERT_ACTIVE_LOW  0x0004


HAL_StatusTypeDef MCP9808_Init(void);
HAL_StatusTypeDef MCP9808_ReadTemperature(float *temperature);
HAL_StatusTypeDef MCP9808_ConfigureInterrupts(float upper_threshold, float lower_threshold, float critical_threshold);
HAL_StatusTypeDef MCP9808_IsConnected(void);
#endif /* INC_MCP9808_H_ */
