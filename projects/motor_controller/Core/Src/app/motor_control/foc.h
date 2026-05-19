#pragma once

/************************************************************************************************
 * @file   foc.h
 *
 * @brief  Header file for the field oriented control loop
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */
#include "hrtim.h"
#include "main.h"

/* Intra-component Headers */
#include "math_defs.h"
#include "motor_config.h"
#include "observer.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

/* ADC and current sense */
#define ADC_VREF 3.3f
#define ADC_SCALE (ADC_VREF / 4095.0f)
#define CC6922SG_SENS 0.0132f

/* Control loop (HRTIM_FREQ_HZ and HRTIM_PERIOD defined in hrtim.h user block) */
#define CONTROL_DT (1.0f / (float)HRTIM_FREQ_HZ)
#define PWM_PERIOD HRTIM_PERIOD

/**
 * @brief   Per phase measured currents
 */
typedef struct {
  float i_a; /**< Phase A current (A) */
  float i_b; /**< Phase B current (A) */
  float i_c; /**< Phase C current (A) */
} phase_currents_t;

/**
 * @brief   Stationary frame current vector
 */
typedef struct {
  float i_alpha; /**< Alpha axis current (A) */
  float i_beta;  /**< Beta axis current (A) */
} alpha_beta_t;

/**
 * @brief   Stationary frame voltage vector
 */
typedef struct {
  float v_alpha; /**< Alpha axis voltage (V) */
  float v_beta;  /**< Beta axis voltage (V) */
} volt_alpha_beta_t;

/**
 * @brief   Rotating frame current vector
 */
typedef struct {
  float id; /**< Direct axis current (A) */
  float iq; /**< Quadrature axis current (A) */
} direct_quadrature_t;

/**
 * @brief   Cached rotor angle trig terms
 */
typedef struct {
  float sin_theta; /**< Sine of the electrical angle */
  float cos_theta; /**< Cosine of the electrical angle */
} foc_trig_t;

/**
 * @brief   Per phase voltage commands
 */
typedef struct {
  float v_a; /**< Phase A voltage (V) */
  float v_b; /**< Phase B voltage (V) */
  float v_c; /**< Phase C voltage (V) */
} v_abc_t;

/**
 * @brief   Per phase PWM duty cycles
 */
typedef struct {
  float d_a; /**< Phase A duty (fraction of period) */
  float d_b; /**< Phase B duty (fraction of period) */
  float d_c; /**< Phase C duty (fraction of period) */
} duty_t;

/**
 * @brief   PI controller state
 */
typedef struct {
  float kp;         /**< Proportional gain */
  float ki;         /**< Integral gain */
  float integrator; /**< Integrator state */
  float out_min;    /**< Output lower limit */
  float out_max;    /**< Output upper limit */
} pi_t;

/**
 * @brief   FOC current and voltage commands
 */
typedef struct {
  float id_ref; /**< Direct axis current reference (A) */
  float iq_ref; /**< Quadrature axis current reference (A) */
  float ud;     /**< Direct axis voltage command (V) */
  float uq;     /**< Quadrature axis voltage command (V) */
} foc_commands_t;

/**
 * @brief   Motor controller operating mode
 */
typedef enum {
  MC_DISABLED = 0,  /**< Outputs disabled */
  MC_FOC = 1,       /**< Closed loop field oriented control */
  MC_OPEN_LOOP = 2, /**< Open loop voltage drive */
} mc_mode_t;

/**
 * @brief   Top level motor controller state
 */
typedef struct {
  float vbus;          /**< DC bus voltage (V) */
  mc_mode_t mode;      /**< Operating mode */
  float el_angle;      /**< Electrical angle (rad) */
  float current_scale; /**< ADC counts to amps scale factor */

  phase_currents_t phase_currents; /**< Measured phase currents */
  foc_trig_t trig;                 /**< Cached rotor angle trig */
  alpha_beta_t alpha_beta;         /**< Stationary frame current */
  direct_quadrature_t dq;          /**< Rotating frame current */
  volt_alpha_beta_t volt_ab;       /**< Stationary frame voltage */
  v_abc_t v_abc;                   /**< Per phase voltage commands */
  duty_t duty;                     /**< Per phase PWM duty */

  foc_commands_t cmd; /**< Current and voltage commands */
  pi_t pi_d;          /**< Direct axis current PI */
  pi_t pi_q;          /**< Quadrature axis current PI */

  observer_state_t observer; /**< Fused position observer */
} motor_controller_t;

extern motor_controller_t mc;

extern volatile uint16_t current_buf[2];
extern volatile uint16_t current_ref_buf[2];

/**
 * @brief   Initialize the motor controller and observer
 */
void foc_init(void);

/**
 * @brief   Run one closed loop FOC step
 * @param   dt Control loop period (s)
 * @param   encoder_el_angle Encoder derived electrical angle (rad)
 */
void foc_step(float dt, float encoder_el_angle);

/**
 * @brief   Run one open loop voltage drive step
 */
void foc_open_loop_step(void);

/** @} */
