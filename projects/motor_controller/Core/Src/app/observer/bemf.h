#pragma once

/************************************************************************************************
 * @file   bemf.h
 *
 * @brief  Header file for the back EMF observer
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Observer
 * @brief    Drive Inverter Observer
 * @{
 */

/**
 * @brief   Back EMF observer state
 */
typedef struct {
  float i_alpha_prev; /**< Previous cycle alpha axis current */
  float i_beta_prev;  /**< Previous cycle beta axis current */
  float e_alpha_lpf;  /**< Filtered alpha axis back EMF */
  float e_beta_lpf;   /**< Filtered beta axis back EMF */
  float lpf_alpha;    /**< Back EMF low pass filter coefficient */
} bemf_state_t;

/**
 * @brief   Initialize the back EMF observer
 * @param   b Pointer to the observer state
 */
void bemf_init(bemf_state_t *b);

/**
 * @brief   Run one back EMF observer step
 * @param   b Pointer to the observer state
 * @param   v_alpha_prev Previous cycle alpha axis voltage command (V)
 * @param   v_beta_prev Previous cycle beta axis voltage command (V)
 * @param   i_alpha Measured alpha axis current (A)
 * @param   i_beta Measured beta axis current (A)
 * @param   Rs Stator resistance (ohms)
 * @param   Ls Stator inductance (henries)
 * @param   flux_linkage Motor permanent magnet flux linkage (Wb)
 * @param   dt Control loop period (s)
 * @param   theta_out Output estimated electrical angle (rad)
 * @param   omega_out Output estimated electrical speed (rad/s)
 */
void bemf_step(bemf_state_t *b, float v_alpha_prev, float v_beta_prev, float i_alpha, float i_beta, float Rs, float Ls, float flux_linkage, float dt, float *theta_out, float *omega_out);

/** @} */
