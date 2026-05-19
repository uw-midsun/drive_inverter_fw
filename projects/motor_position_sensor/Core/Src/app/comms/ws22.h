#pragma once

/************************************************************************************************
 * @file   ws22.h
 *
 * @brief  Header file for the WaveSculptor22 position packet protocol
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
 * @defgroup Comms
 * @brief    Drive Position Sensor host link
 * @{
 */

/**
 * @brief   Queue and send the WS22 device type announcement packet
 * @param   angle_raw Raw 16 bit angle word
 * @return  HAL_OK if the packet was accepted for transmission
 */
HAL_StatusTypeDef ws22_send_device_type(uint16_t angle_raw);

/**
 * @brief   Queue and send a WS22 thermistor and angle data packet
 * @param   thermistor_raw Raw thermistor ADC count
 * @param   angle_raw Raw 16 bit angle word
 * @return  HAL_OK if the packet was accepted for transmission
 */
HAL_StatusTypeDef ws22_send_data(uint16_t thermistor_raw, uint16_t angle_raw);

/**
 * @brief   Get the running count of UART errors
 * @return  Number of UART errors since boot
 */
uint32_t ws22_error_count(void);

/** @} */
