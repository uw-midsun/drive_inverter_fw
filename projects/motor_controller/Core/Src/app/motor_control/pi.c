/************************************************************************************************
 * @file   pi.c
 *
 * @brief  Source file for the proportional integral controller utility
 *
 * @date   2026-05-19
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "pi.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

void pi_init(pi_t *pi, const pi_config_t *cfg) {
  pi->cfg = *cfg;
  pi->integrator = 0.0f;
}

float pi_run(pi_t *pi, float error, float dt) {
  const float p_term = pi->cfg.kp * error;
  const float ki_dt = pi->cfg.ki * dt;

  /* Dynamic integrator bounds so p_term + integrator stays within the limits */
  const float i_max = pi->cfg.out_max - p_term;
  const float i_min = pi->cfg.out_min - p_term;

  /* Integrate then clamp to the dynamic bounds */
  float i_term = pi->integrator + ki_dt * error;
  if (i_term > i_max)
    i_term = i_max;
  else if (i_term < i_min)
    i_term = i_min;

  pi->integrator = i_term;

  /* Guaranteed within [out_min, out_max] by the dynamic integrator clamp */
  return p_term + i_term;
}

/** @} */
