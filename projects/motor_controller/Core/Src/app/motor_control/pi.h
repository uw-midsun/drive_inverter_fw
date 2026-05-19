#pragma once

/************************************************************************************************
 * @file   pi.h
 *
 * @brief  Header file for the proportional integral controller utility
 *
 * @date   2026-05-19
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

/**
 * @brief   Static PI controller tuning
 */
typedef struct {
  float kp;      /**< Proportional gain */
  float ki;      /**< Integral gain */
  float out_min; /**< Output lower limit */
  float out_max; /**< Output upper limit */
} pi_config_t;

/**
 * @brief   PI controller instance (tuning plus running state)
 */
typedef struct {
  pi_config_t cfg;  /**< Tuning constants */
  float integrator; /**< Integrator state */
} pi_t;

/**
 * @brief   Initialize a PI controller and clear its integrator
 * @param   pi Controller to initialize
 * @param   cfg Tuning constants to copy in
 */
void pi_init(pi_t *pi, const pi_config_t *cfg);

/**
 * @brief   Run one PI update with dynamic anti windup
 * @details The integrator bounds are derived from the output limits minus the
 *          proportional term, so the summed output stays within [out_min,
 * out_max] without a separate saturation stage
 * @param   pi Controller to update
 * @param   error Setpoint minus measurement
 * @param   dt Timestep (s)
 * @return  Controller output (clamped to the configured limits)
 */
float pi_run(pi_t *pi, float error, float dt);

/** @} */
