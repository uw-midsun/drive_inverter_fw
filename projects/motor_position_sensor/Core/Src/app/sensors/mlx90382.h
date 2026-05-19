#pragma once

/************************************************************************************************
 * @file   mlx90382.h
 *
 * @brief  Header file for the MLX90382 magnetic angle sensor driver
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
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
 * @brief   Initialize the MLX90382 over SPI2 for YZ sensing in 5 byte frames
 * @return  HAL_OK on success
 */
HAL_StatusTypeDef mlx90382_init(void);

/**
 * @brief   Start the next angle frame over SPI DMA when the sensor is idle
 * @return  HAL_OK if a transfer started, HAL_BUSY if one is already in flight
 */
HAL_StatusTypeDef mlx90382_kick(void);

/**
 * @brief   Take the latest completed angle reading
 * @param   angle_raw Output raw 16 bit angle word
 * @return  true if a fresh frame was consumed, false if none is ready
 */
bool mlx90382_take_reading(uint16_t *angle_raw);

/**
 * @brief   Get the running count of SPI errors
 * @return  Number of SPI errors since boot
 */
uint32_t mlx90382_error_count(void);

/** @} */
