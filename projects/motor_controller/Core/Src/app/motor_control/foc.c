/************************************************************************************************
 * @file    foc.c
 *
 * @brief   Source file for the field oriented control loop
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <math.h>

/* Inter-component Headers */
#include "hrtim.h"

/* Intra-component Headers */
#include "foc.h"
#include "pi.h"
#include "transform_utils.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

/* ADC counts to amps: (Vref / full scale) / sensor sensitivity */
#define ADC_VREF 3.3f          /**< ADC reference voltage (V) */
#define ADC_FULL_SCALE 4095.0f /**< 12 bit ADC full scale (counts) */
#define ADC_SCALE (ADC_VREF / ADC_FULL_SCALE)
#define CC6922SG_SENS 0.0132f /**< Current sensor sensitivity (V/A) */

/* PWM compare register full count. HRTIM_PERIOD is set in the hrtim.h user
 * block */
#define PWM_PERIOD HRTIM_PERIOD

#define MC_VBUS_DEFAULT 30.0f /**< Assumed bus voltage until measured (V) */

#define PI_CURRENT_KP 1.0f   /**< d and q axis current loop proportional gain */
#define PI_CURRENT_KI 200.0f /**< d and q axis current loop integral gain */

#define CURRENT_LPF_ALPHA 0.5f   /**< Phase current first order low pass coefficient */
#define PHASE_C_SENSE_SIGN -1.0f /**< Phase C current sensor polarity is reversed */

#define SVPWM_SECTOR_COUNT 6 /**< Number of space vector sectors */
#define SVPWM_COEFF_COUNT 4  /**< Per sector coefficients { A, B, C, D } */

motor_controller_t mc = {
  .vbus = MC_VBUS_DEFAULT,
  .mode = MC_DISABLED,
  .el_angle = 0.0f,
  .current_scale = ADC_SCALE / CC6922SG_SENS,
};

static float svpwm_coeffs[SVPWM_SECTOR_COUNT][SVPWM_COEFF_COUNT] = { 0 }; /* computed at init */

volatile uint16_t phase_current_adc[ADC_CURRENT_COUNT] = { 0 };
volatile uint16_t phase_current_ref_adc[ADC_CURRENT_COUNT] = { 0 };

static void foc_compute_svpwm_coeffs(void) {
  /* svpwm_coeffs[s][0..3] = { A, B, C, D } where
   * A = 0.5 * cos(phi1), B = 0.5 * sin(phi1)
   * C = 0.5 * cos(phi2), D = 0.5 * sin(phi2)
   * phi1, phi2 are the two active vector angles for sector s
   * (s = 0..5 maps to sectors 1..6)
   *
   * Call once at startup (or when Ts changes). Stored as fractions of Ts
   * (half angle factors)
   *
   * cos and sin for the six active vector angles:
   * 0, 60, 120, 180, 240, 300 degrees */
  const float cosv[SVPWM_SECTOR_COUNT] = { 1.0f, 0.5f, -0.5f, -1.0f, -0.5f, 0.5f };
  const float sinv[SVPWM_SECTOR_COUNT] = { 0.0f, HALF_SQRT3_F, HALF_SQRT3_F, 0.0f, -HALF_SQRT3_F, -HALF_SQRT3_F };

  for (int s = 0; s < SVPWM_SECTOR_COUNT; ++s) {
    const int next = (s + 1) % SVPWM_SECTOR_COUNT;
    svpwm_coeffs[s][0] = 0.5f * cosv[s];
    svpwm_coeffs[s][1] = 0.5f * sinv[s];
    svpwm_coeffs[s][2] = 0.5f * cosv[next];
    svpwm_coeffs[s][3] = 0.5f * sinv[next];
  }
}

void foc_init(void) {
  const float vmax = mc.vbus * INVERSE_SQRT3_F;

  const pi_config_t current_pi = {
    .kp = PI_CURRENT_KP,
    .ki = PI_CURRENT_KI,
    .out_min = -vmax,
    .out_max = vmax,
  };
  pi_init(&mc.pi_d, &current_pi);
  pi_init(&mc.pi_q, &current_pi);

  mc.cmd.id_ref = 0.0f;
  mc.cmd.iq_ref = 0.0f;

  foc_compute_svpwm_coeffs();
  observer_init(&mc.observer, MOTOR_RS, MOTOR_LS, MOTOR_FLUX_LINKAGE);
}

static void foc_read_phase_currents(void) {
  const int16_t a_diff = (int16_t)(phase_current_adc[ADC_CSA] - phase_current_ref_adc[ADC_CSA]);
  const int16_t c_diff = (int16_t)(phase_current_adc[ADC_CSC] - phase_current_ref_adc[ADC_CSC]);

  mc.phase_currents.i_a += CURRENT_LPF_ALPHA * ((float)a_diff * mc.current_scale - mc.phase_currents.i_a);
  mc.phase_currents.i_c += CURRENT_LPF_ALPHA * ((float)c_diff * mc.current_scale * PHASE_C_SENSE_SIGN - mc.phase_currents.i_c);

  mc.phase_currents.i_b = -(mc.phase_currents.i_a + mc.phase_currents.i_c);
}

static void foc_update_trig(void) {
  mc.trig.sin_theta = sinf(mc.el_angle);
  mc.trig.cos_theta = cosf(mc.el_angle);
}

static void foc_clamp_voltage_vector(void) {
  /* Clamp the commanded voltage vector to the circle |V| <= Vbus / sqrt(3) */
  const float vmax = mc.vbus * INVERSE_SQRT3_F;
  const float mag_sq = mc.volt_ab.v_alpha * mc.volt_ab.v_alpha + mc.volt_ab.v_beta * mc.volt_ab.v_beta;
  const float vmax_sq = vmax * vmax;

  if (mag_sq > vmax_sq) {
    const float scale = vmax / sqrtf(mag_sq);
    mc.volt_ab.v_alpha *= scale;
    mc.volt_ab.v_beta *= scale;
  }
}

static void foc_svpwm(void) {
  /* Convert alpha beta to a normalized reference vector (relative to Vdc/2) */
  const float vbus_half = mc.vbus * 0.5f;
  const float a = mc.volt_ab.v_alpha / vbus_half;
  const float b = mc.volt_ab.v_beta / vbus_half;

  /* Sector detection (projections) */
  float wX = b;
  float wY = 0.5f * (b + a);
  float wZ = 0.5f * (b - a);

  uint8_t sector;
  if (wY < 0) {
    if (wZ < 0)
      sector = 5;
    else if (wX <= 0)
      sector = 4;
    else
      sector = 3;
  } else {
    if (wZ >= 0)
      sector = 2;
    else if (wX <= 0)
      sector = 6;
    else
      sector = 1;
  }

  /* T1, T2 as fractions of the PWM period using precomputed coeffs */
  const float *c = svpwm_coeffs[sector - 1]; /* c[0]=A_f, c[1]=B_f, c[2]=C_f, c[3]=D_f */
  float T1_frac = c[0] * a + c[1] * b;
  float T2_frac = c[2] * a + c[3] * b;

  /* Clamp and ensure T0 >= 0 */
  if (T1_frac < 0.0f) T1_frac = 0.0f;
  if (T2_frac < 0.0f) T2_frac = 0.0f;
  if (T1_frac + T2_frac > 1.0f) {
    float s = 1.0f / (T1_frac + T2_frac);
    T1_frac *= s;
    T2_frac *= s;
  }

  float T0_frac = 1.0f - T1_frac - T2_frac;
  if (T0_frac < 0.0f) T0_frac = 0.0f;
  const float halfT0 = 0.5f * T0_frac;

  /* Per sector mapping to Ta, Tb, Tc (fractions of the full period) */
  float Ta_frac, Tb_frac, Tc_frac;
  switch (sector) {
    case 1:
      Ta_frac = halfT0 + T1_frac + T2_frac;
      Tb_frac = halfT0 + T2_frac;
      Tc_frac = halfT0;
      break;
    case 2:
      Ta_frac = halfT0 + T1_frac;
      Tb_frac = halfT0 + T1_frac + T2_frac;
      Tc_frac = halfT0;
      break;
    case 3:
      Ta_frac = halfT0;
      Tb_frac = halfT0 + T1_frac + T2_frac;
      Tc_frac = halfT0 + T2_frac;
      break;
    case 4:
      Ta_frac = halfT0;
      Tb_frac = halfT0 + T1_frac;
      Tc_frac = halfT0 + T1_frac + T2_frac;
      break;
    case 5:
      Ta_frac = halfT0 + T2_frac;
      Tb_frac = halfT0;
      Tc_frac = halfT0 + T1_frac + T2_frac;
      break;
    default: /* sector 6 */
      Ta_frac = halfT0 + T1_frac + T2_frac;
      Tb_frac = halfT0;
      Tc_frac = halfT0 + T1_frac;
      break;
  }

  mc.duty.d_a = Ta_frac;
  mc.duty.d_b = Tb_frac;
  mc.duty.d_c = Tc_frac;
}

static void foc_write_duty(void) {
  uint16_t a_counts = (uint16_t)(mc.duty.d_a * PWM_PERIOD);
  uint16_t b_counts = (uint16_t)(mc.duty.d_b * PWM_PERIOD);
  uint16_t c_counts = (uint16_t)(mc.duty.d_c * PWM_PERIOD);

  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, a_counts);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, b_counts);
  __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, c_counts);
}

void foc_step(const float dt, float encoder_el_angle) {
  foc_read_phase_currents();
  clarke_transform(mc.phase_currents.i_a, mc.phase_currents.i_b, &mc.alpha_beta.i_alpha, &mc.alpha_beta.i_beta);

  observer_step(&mc.observer, mc.alpha_beta.i_alpha, mc.alpha_beta.i_beta, encoder_el_angle, dt);
  mc.el_angle = mc.observer.theta_est;

  foc_update_trig();
  park_transform(mc.alpha_beta.i_alpha, mc.alpha_beta.i_beta, mc.trig.sin_theta, mc.trig.cos_theta, &mc.dq.id, &mc.dq.iq);

  const float err_d = mc.cmd.id_ref - mc.dq.id;
  const float err_q = mc.cmd.iq_ref - mc.dq.iq;

  mc.cmd.ud = pi_run(&mc.pi_d, err_d, dt);
  mc.cmd.uq = pi_run(&mc.pi_q, err_q, dt);

  inverse_park_transform(mc.cmd.ud, mc.cmd.uq, mc.trig.sin_theta, mc.trig.cos_theta, &mc.volt_ab.v_alpha, &mc.volt_ab.v_beta);
  foc_clamp_voltage_vector();

  observer_update_voltage(&mc.observer, mc.volt_ab.v_alpha, mc.volt_ab.v_beta);

  foc_svpwm();
  foc_write_duty();
}

void foc_open_loop_step(void) {
  foc_update_trig();
  inverse_park_transform(mc.cmd.ud, mc.cmd.uq, mc.trig.sin_theta, mc.trig.cos_theta, &mc.volt_ab.v_alpha, &mc.volt_ab.v_beta);
  foc_clamp_voltage_vector();
  foc_svpwm();
  foc_write_duty();
}

/** @} */
