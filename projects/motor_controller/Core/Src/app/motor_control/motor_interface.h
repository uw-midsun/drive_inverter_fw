#pragma once

/************************************************************************************************
 * @file   motor_interface.h
 *
 * @brief  Header file for the WaveSculptor encoder interface
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "main.h"
#include "usart.h"

/* Intra-component Headers */
#include "motor_config.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

#define ENCODER_BITS 14                     /**< Encoder resolution in bits */
#define ENCODER_COUNTS (1U << ENCODER_BITS) /**< Counts per mechanical revolution */
#define ENCODER_MASK                                            \
  (ENCODER_COUNTS - 1U)       /**< Wrap mask for encoder counts \
                               */
#define WS22_DMA_BUF_SIZE 128 /**< UART DMA ring size, power of 2 */

#define LUT_BITS 6                                    /**< Encoder linearization LUT size in bits */
#define LUT_SIZE (1 << LUT_BITS)                      /**< Encoder linearization LUT entries */
#define LUT_INDEX_MASK (LUT_SIZE - 1U)                /**< Wrap mask for the LUT bin index */
#define LUT_INTERP_SHIFT (ENCODER_BITS - LUT_BITS)    /**< Raw counts to LUT bin shift */
#define LUT_FRAC_MASK ((1U << LUT_INTERP_SHIFT) - 1U) /**< Sub bin interpolation fraction mask */

#define CAL_SAMPLES (LUT_SIZE * POLE_PAIRS) /**< Calibration sweep sample count */
#define CALIB_FLASH_ADDR 0x0807F800         /**< Last flash page on the G474RE */

/**
 * @brief   Decoded WaveSculptor sensor frame
 */
typedef struct {
  uint16_t position;    /**< Raw encoder position counts */
  uint16_t temperature; /**< Reported temperature code */
  uint8_t device_type;  /**< Reported device type */
  uint8_t valid;        /**< Set when a valid frame was parsed */
} motor_sensor_data_t;

/**
 * @brief   Initialize the motor interface and start UART DMA
 */
void motor_interface_init(void);

/**
 * @brief   Get the calibrated encoder position
 * @return  Encoder position in counts
 */
uint16_t motor_interface_get_position(void);

/**
 * @brief   Get the calibrated encoder position in radians
 * @return  Encoder position (rad)
 */
float motor_interface_get_position_rad(void);

/**
 * @brief   Get the raw encoder position with direction only applied
 * @return  Encoder position in counts
 */
uint16_t motor_interface_get_position_raw(void);

/**
 * @brief   Get the latest decoded sensor frame
 * @return  Decoded sensor data
 */
motor_sensor_data_t motor_interface_get_data(void);

/**
 * @brief   Set the encoder zero offset
 * @param   off Offset in counts
 */
void motor_interface_set_offset(uint16_t off);

/**
 * @brief   Set the encoder direction reversal flag
 * @param   rev Non zero to reverse direction
 */
void motor_interface_set_reverse(uint8_t rev);

/**
 * @brief   Set the encoder linearization LUT
 * @param   lut Pointer to LUT_SIZE entries
 */
void motor_interface_set_lut(const int16_t *lut);

/**
 * @brief   Load calibration from flash
 */
void motor_interface_load_calibration(void);

/**
 * @brief   Save calibration to flash
 */
void motor_interface_save_calibration(void);

/** @} */
