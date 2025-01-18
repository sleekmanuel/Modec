/*
 * mcp9808.c
 *
 *  Created on: Jan 15, 2025
 *      Author: root
 */

/**
 * @brief Reads the temperature from the MCP9808 sensor and converts it to Celsius.
 * @param[out] temperature Pointer to store the temperature value in Celsius.
 * @return HAL status: HAL_OK if successful, HAL_ERROR otherwise.
 * @note The temperature range is -40°C to +125°C with a resolution of 0.0625°C.
 */

#include "mcp9808.h"

// I2C handle defined in your project (replace as needed)
extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Check if MCP9808 is connected.
 * @return HAL status
 */
HAL_StatusTypeDef MCP9808_IsConnected(void) {
    return HAL_I2C_IsDeviceReady(&hi2c1, MCP9808_ADDR << 1, 5, 2000);
}

/**
 * @brief Initialize the MCP9808 sensor.
 * @return HAL status
 */
HAL_StatusTypeDef MCP9808_Init(void) {
    uint8_t config_data[3] = {
    		MCP9808_REG_CONFIG,
    		MCP9808_CONFIG_DEFAULT >> 8,
    		MCP9808_CONFIG_DEFAULT & 0xFF
    };
    return HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, config_data, 3, HAL_MAX_DELAY);
}


/**
 * @brief Read the temperature from MCP9808.
 * @param[out] temperature Pointer to store the temperature in Celsius.
 * @return HAL status
 */
HAL_StatusTypeDef MCP9808_ReadTemperature(float *temperature) {
    uint8_t reg = MCP9808_REG_TEMP;
    uint8_t temp_data[2];

    // Request the temperature register
    if (HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read the 2-byte temperature data
    if (HAL_I2C_Master_Receive(&hi2c1, MCP9808_ADDR << 1, temp_data, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Process temperature data
    uint16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
    raw_temp &= 0x0FFF; // Mask off unused bits

    // Convert to Celsius
    *temperature = raw_temp * 0.0625;
    if (raw_temp & 0x1000) {
        *temperature -= 256.0; // Handle negative temperatures
    }

    return HAL_OK;
}


/**
 * @brief Configure temperature thresholds and enable interrupts.
 * @param upper_threshold Upper temperature threshold in Celsius.
 * @param lower_threshold Lower temperature threshold in Celsius.
 * @param critical_threshold Critical temperature threshold in Celsius.
 * @return HAL status
 */
HAL_StatusTypeDef MCP9808_ConfigureInterrupts(float upper_threshold, float lower_threshold, float critical_threshold) {
    uint16_t upper_raw = (uint16_t)(upper_threshold / 0.0625) & 0x0FFF;
    uint16_t lower_raw = (uint16_t)(lower_threshold / 0.0625) & 0x0FFF;
    uint16_t critical_raw = (uint16_t)(critical_threshold / 0.0625) & 0x0FFF;

    uint8_t threshold_data[3];

    // Set upper threshold
    threshold_data[0] = MCP9808_REG_ALERT_UPPER;
    threshold_data[1] = (upper_raw >> 8) & 0xFF;
    threshold_data[2] = upper_raw & 0xFF;
    if (HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, threshold_data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set lower threshold
    threshold_data[0] = MCP9808_REG_ALERT_LOWER;
    threshold_data[1] = (lower_raw >> 8) & 0xFF;
    threshold_data[2] = lower_raw & 0xFF;
    if (HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, threshold_data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Set critical threshold
    threshold_data[0] = MCP9808_REG_ALERT_CRIT;
    threshold_data[1] = (critical_raw >> 8) & 0xFF;
    threshold_data[2] = critical_raw & 0xFF;
    if (HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, threshold_data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    uint16_t config_value = MCP9808_CONFIG_INT_ENABLE | MCP9808_CONFIG_ALERT_ACTIVE_LOW;

    uint8_t config_data[3] = {
        MCP9808_REG_CONFIG,
        (config_value >> 8) & 0xFF, // MSB
        config_value & 0xFF         // LSB
    };

    if (HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR << 1, config_data, 3, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

