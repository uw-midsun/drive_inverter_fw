#pragma once

/************************************************************************************************
 * @file   calibration.h
 *
 * @brief  Header file for encoder offset and linearization calibration
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

/**
 * @brief   Run the full mechanical calibration sweep and store the LUT
 * @param   Ud Direct axis voltage applied during the sweep (V)
 * @param   Vbus DC bus voltage (V)
 */
void calibrate_run(float Ud, float Vbus);

/**
 * @brief   Run the encoder offset only calibration
 * @param   Ud Direct axis voltage applied during the sweep (V)
 * @param   steps Number of electrical steps to sample
 * @param   delay Settling delay per step (ms)
 */
void calibrate_offset(float Ud, uint16_t steps, uint16_t delay);

/**
 * @brief   Clear calibration to identity and save to flash
 */
void calibrate_clear(void);

/** @} */
