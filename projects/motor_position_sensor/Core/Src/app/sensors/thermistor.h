#pragma once

/************************************************************************************************
 * @file   thermistor.h
 *
 * @brief  Header file for the motor thermistor ADC reader
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "stm32f1xx_hal.h"

/* Intra-component Headers */

/**
 * @defgroup Sensors
 * @brief    Drive Position Sensor peripherals
 * @{
 */

/**
 * @brief   Configure the thermistor ADC channel and run calibration
 * @return  HAL_OK on success
 */
HAL_StatusTypeDef thermistor_init(void);

/**
 * @brief   Take a single blocking thermistor reading
 * @param   value Output raw ADC count, set to 0 on error
 * @return  HAL_OK on success
 */
HAL_StatusTypeDef thermistor_read(uint16_t *value);

/**
 * @brief   Get the running count of ADC errors
 * @return  Number of ADC errors since boot
 */
uint32_t thermistor_error_count(void);

/** @} */
