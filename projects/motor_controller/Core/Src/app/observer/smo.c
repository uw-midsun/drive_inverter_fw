/************************************************************************************************
 * @file    smo.c
 *
 * @brief   Source file for the sliding mode observer backend
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <math.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "math_defs.h"
#include "smo.h"

/**
 * @defgroup Observer
 * @brief    Drive Inverter Observer
 * @{
 */

/* Estimate the current vector, compare to the real current vector. The
 * correction needed to keep them matched is the BEMF, low pass filter that
 * and feed it into a PLL to recover the angle */

#define SMO_GAIN_FLUX_SCALE 20.0f        /**< Switching gain as a multiple of flux linkage */
#define SMO_BEMF_LPF_ALPHA 0.02f         /**< BEMF extraction low pass coefficient */
#define SMO_SIGMOID_GAIN 100.0f          /**< Sigmoid switching function steepness */
#define SMO_PLL_KP 400.0f                /**< PLL proportional gain */
#define SMO_PLL_KI 40000.0f              /**< PLL integral gain */
#define SMO_CURRENT_EST_LIMIT 200.0f     /**< Estimated current saturation (A) */
#define SMO_PLL_INTEGRATOR_LIMIT 5000.0f /**< PLL integrator saturation (rad/s) */

static inline float smo_sigmoid(float x, float gain) {
  return x / (fabsf(x) + (1.0f / gain));
}

void smo_init(smo_state_t *s, float flux_linkage) {
  s->i_alpha_hat = 0.0f;
  s->i_beta_hat = 0.0f;
  s->e_alpha_hat = 0.0f;
  s->e_beta_hat = 0.0f;
  s->gain = SMO_GAIN_FLUX_SCALE * flux_linkage;
  s->lpf_alpha = SMO_BEMF_LPF_ALPHA;
  s->sigmoid_gain = SMO_SIGMOID_GAIN;
  s->pll_kp = SMO_PLL_KP;
  s->pll_ki = SMO_PLL_KI;
  s->pll_integrator = 0.0f;
  s->theta = 0.0f;
}

void smo_step(smo_state_t *s, float v_alpha_prev, float v_beta_prev, float i_alpha, float i_beta, float Rs, float Ls, float dt, float *theta_out, float *omega_out) {
  /* Current estimation error */
  float err_alpha = s->i_alpha_hat - i_alpha;
  float err_beta = s->i_beta_hat - i_beta;

  /* Sliding mode switching signals */
  float z_alpha = s->gain * smo_sigmoid(err_alpha, s->sigmoid_gain);
  float z_beta = s->gain * smo_sigmoid(err_beta, s->sigmoid_gain);

  /* Current observer (Euler integration) */
  float di_alpha_hat = (v_alpha_prev - Rs * s->i_alpha_hat - s->e_alpha_hat - z_alpha) / Ls;
  float di_beta_hat = (v_beta_prev - Rs * s->i_beta_hat - s->e_beta_hat - z_beta) / Ls;

  s->i_alpha_hat += di_alpha_hat * dt;
  s->i_beta_hat += di_beta_hat * dt;
  s->i_alpha_hat = clampf(s->i_alpha_hat, -SMO_CURRENT_EST_LIMIT, SMO_CURRENT_EST_LIMIT);
  s->i_beta_hat = clampf(s->i_beta_hat, -SMO_CURRENT_EST_LIMIT, SMO_CURRENT_EST_LIMIT);

  /* Extract BEMF from the sliding signals via low pass filter */
  s->e_alpha_hat = s->lpf_alpha * z_alpha + (1.0f - s->lpf_alpha) * s->e_alpha_hat;
  s->e_beta_hat = s->lpf_alpha * z_beta + (1.0f - s->lpf_alpha) * s->e_beta_hat;

  /* PLL angle tracking */
  float sin_th = sinf(s->theta);
  float cos_th = cosf(s->theta);
  float pll_error = -s->e_alpha_hat * cos_th - s->e_beta_hat * sin_th;

  s->pll_integrator += s->pll_ki * pll_error * dt;
  s->pll_integrator = clampf(s->pll_integrator, -SMO_PLL_INTEGRATOR_LIMIT, SMO_PLL_INTEGRATOR_LIMIT);
  float omega = s->pll_integrator + s->pll_kp * pll_error;
  s->theta += omega * dt;
  s->theta = wrap_angle(s->theta);

  *theta_out = s->theta;
  *omega_out = omega;
}

/** @} */
