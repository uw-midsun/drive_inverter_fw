#pragma once

/************************************************************************************************
 * @file   motor_config.h
 *
 * @brief  Header file for motor electrical parameters
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

#define MOTOR_RS 0.1f            /**< Stator resistance (ohms) */
#define MOTOR_LS 0.0001f         /**< Stator inductance (henries) */
#define MOTOR_FLUX_LINKAGE 0.01f /**< Permanent magnet flux linkage (Wb) */
#define POLE_PAIRS 16            /**< Motor pole pairs */

/** @} */
