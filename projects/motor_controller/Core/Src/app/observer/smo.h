#pragma once

/************************************************************************************************
 * @file   smo.h
 *
 * @brief  Header file for the sliding mode observer backend
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
 * @brief   Sliding mode observer state
 */
typedef struct {
  float i_alpha_hat;    /**< Estimated alpha axis current */
  float i_beta_hat;     /**< Estimated beta axis current */
  float e_alpha_hat;    /**< Estimated alpha axis BEMF */
  float e_beta_hat;     /**< Estimated beta axis BEMF */
  float gain;           /**< Sliding mode switching gain */
  float lpf_alpha;      /**< BEMF low pass filter coefficient */
  float sigmoid_gain;   /**< Sigmoid switching function gain */
  float pll_kp;         /**< PLL proportional gain */
  float pll_ki;         /**< PLL integral gain */
  float pll_integrator; /**< PLL integrator state */
  float theta;          /**< PLL tracked electrical angle */
} smo_state_t;

/**
 * @brief   Initialize the sliding mode observer
 * @param   s Pointer to the observer state
 * @param   flux_linkage Motor permanent magnet flux linkage (Wb)
 */
void smo_init(smo_state_t *s, float flux_linkage);

/**
 * @brief   Run one sliding mode observer step
 * @param   s Pointer to the observer state
 * @param   v_alpha_prev Previous cycle alpha axis voltage command (V)
 * @param   v_beta_prev Previous cycle beta axis voltage command (V)
 * @param   i_alpha Measured alpha axis current (A)
 * @param   i_beta Measured beta axis current (A)
 * @param   Rs Stator resistance (ohms)
 * @param   Ls Stator inductance (henries)
 * @param   dt Control loop period (s)
 * @param   theta_out Output estimated electrical angle (rad)
 * @param   omega_out Output estimated electrical speed (rad/s)
 */
void smo_step(smo_state_t *s, float v_alpha_prev, float v_beta_prev, float i_alpha, float i_beta, float Rs, float Ls, float dt, float *theta_out, float *omega_out);

/** @} */
