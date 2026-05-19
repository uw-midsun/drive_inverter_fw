#pragma once

/************************************************************************************************
 * @file   observer.h
 *
 * @brief  Header file for the fused encoder and sensorless position observer
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "smo.h"

/**
 * @defgroup Observer
 * @brief    Drive Inverter Observer
 * @{
 */

/**
 * @brief   Fused position observer state
 */
typedef struct {
  float theta_est; /**< Estimated electrical angle (rad) */
  float omega_est; /**< Estimated electrical speed (rad/s) */

  float Rs;           /**< Stator resistance (ohms) */
  float Ls;           /**< Stator inductance (henries) */
  float flux_linkage; /**< Permanent magnet flux linkage (Wb) */

  float v_alpha_prev; /**< Previous cycle alpha axis voltage command (V) */
  float v_beta_prev;  /**< Previous cycle beta axis voltage command (V) */

  smo_state_t smo; /**< Sliding mode observer backend */

  float correction_gain; /**< Speed scheduled encoder correction gain */

  uint8_t warmup_cycles; /**< Skip the SMO until voltage history is valid */
} observer_state_t;

/**
 * @brief   Initialize the fused position observer
 * @param   obs Pointer to the observer state
 * @param   Rs Stator resistance (ohms)
 * @param   Ls Stator inductance (henries)
 * @param   flux_linkage Permanent magnet flux linkage (Wb)
 */
void observer_init(observer_state_t *obs, float Rs, float Ls, float flux_linkage);

/**
 * @brief   Run one fused observer step
 * @param   obs Pointer to the observer state
 * @param   i_alpha Measured alpha axis current (A)
 * @param   i_beta Measured beta axis current (A)
 * @param   encoder_el_angle Encoder derived electrical angle (rad)
 * @param   dt Control loop period (s)
 */
void observer_step(observer_state_t *obs, float i_alpha, float i_beta, float encoder_el_angle, float dt);

/**
 * @brief   Store the latest voltage command for the next observer step
 * @param   obs Pointer to the observer state
 * @param   v_alpha Alpha axis voltage command (V)
 * @param   v_beta Beta axis voltage command (V)
 */
void observer_update_voltage(observer_state_t *obs, float v_alpha, float v_beta);

/** @} */
