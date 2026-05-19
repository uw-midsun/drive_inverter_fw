/************************************************************************************************
 * @file    observer.c
 *
 * @brief   Source file for the fused encoder and sensorless position observer
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <math.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "math_defs.h"
#include "observer.h"

/**
 * @defgroup Observer
 * @brief    Drive Inverter Observer
 * @{
 */

/* Fusion of the sensorless and sensored estimates. The encoder correction
 * gain is scheduled linearly against electrical speed (rad/s) */
#define FUSION_SPEED_LOW                                                       \
  50.0f                          /**< Below this speed trust the encoder fully \
                                  */
#define FUSION_SPEED_HIGH 300.0f /**< Above this speed trust the SMO fully */
#define FUSION_GAIN_MAX 0.8f     /**< Encoder correction gain at low speed */
#define FUSION_GAIN_MIN 0.0f     /**< Encoder correction gain at high speed */

#define OBSERVER_WARMUP_CYCLES 2           /**< Pure encoder steps before the SMO is valid */
#define ENCODER_CORRECTION_LIMIT_RAD 0.52f /**< Reject encoder glitches over ~30 deg */

void observer_init(observer_state_t *obs, float Rs, float Ls, float flux_linkage) {
  obs->theta_est = 0.0f;
  obs->omega_est = 0.0f;
  obs->Rs = Rs;
  obs->Ls = Ls;
  obs->flux_linkage = flux_linkage;
  obs->v_alpha_prev = 0.0f;
  obs->v_beta_prev = 0.0f;
  obs->correction_gain = FUSION_GAIN_MAX;
  obs->warmup_cycles = 0;

  smo_init(&obs->smo, flux_linkage);
}

void observer_step(observer_state_t *obs, float i_alpha, float i_beta, float encoder_el_angle, float dt) {
  /* Pure encoder until the voltage history is valid */
  if (obs->warmup_cycles < OBSERVER_WARMUP_CYCLES) {
    obs->warmup_cycles++;
    obs->theta_est = encoder_el_angle;
    obs->omega_est = 0.0f;
    return;
  }

  float raw_theta = 0.0f;
  float raw_omega = 0.0f;

  smo_step(&obs->smo, obs->v_alpha_prev, obs->v_beta_prev, i_alpha, i_beta, obs->Rs, obs->Ls, dt, &raw_theta, &raw_omega);

  obs->theta_est = raw_theta;
  obs->omega_est = raw_omega;

  /* Speed scheduled encoder correction gain */
  float speed_abs = fabsf(obs->omega_est);
  if (speed_abs < FUSION_SPEED_LOW) {
    obs->correction_gain = FUSION_GAIN_MAX;
  } else if (speed_abs > FUSION_SPEED_HIGH) {
    obs->correction_gain = FUSION_GAIN_MIN;
  } else {
    float t = (speed_abs - FUSION_SPEED_LOW) / (FUSION_SPEED_HIGH - FUSION_SPEED_LOW);
    obs->correction_gain = FUSION_GAIN_MAX + t * (FUSION_GAIN_MIN - FUSION_GAIN_MAX);
  }

  /* Apply the encoder correction, rejecting large glitches */
  float error = wrap_to_pm_pi(encoder_el_angle - obs->theta_est);
  error = clampf(error, -ENCODER_CORRECTION_LIMIT_RAD, ENCODER_CORRECTION_LIMIT_RAD);

  obs->theta_est += obs->correction_gain * error;
  obs->theta_est = wrap_angle(obs->theta_est);
}

void observer_update_voltage(observer_state_t *obs, float v_alpha, float v_beta) {
  obs->v_alpha_prev = v_alpha;
  obs->v_beta_prev = v_beta;
}

/** @} */
