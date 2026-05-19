/************************************************************************************************
 * @file    app.c
 *
 * @brief   Source file for the drive inverter application supervisor
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */
#include "adc.h"
#include "hrtim.h"
#include "main.h"
#include "stm32g4xx_ll_gpio.h"

/* Intra-component Headers */
#include "app.h"
#include "can_bus.h"
#include "can_schema.h"
#include "foc.h"
#include "motor_interface.h"

/**
 * @defgroup App
 * @brief    Drive Inverter Application Layer
 * @{
 */

#define DRIVE_WATCHDOG_MS                                                                       \
  300U                         /**< Fault if no drive frame within this window (~3 missed 100ms \
                                  frames) */
#define STATUS_TX_MS 100U      /**< CAN status broadcast period (~10 Hz) */
#define MC_MAX_CURRENT_A 10.0f /**< Maps drive current_limit [0,1] pu onto iq_ref (A) */
#define PHASE_EN_SETTLE_MS 1U  /**< Gate driver enable settle delay (ms) */
#define OPEN_LOOP_UQ_V 3.0f    /**< Fixed q axis voltage used in open loop mode (V) */

/* All app owned state in one place */
typedef struct {
  app_state_t fsm;
  can_tx_t tx;             /* status frames, broadcast every STATUS_TX_MS */
  can_rx_t rx;             /* auto filled by can_rx_dispatch() */
  uint32_t last_status_tx; /* HAL_GetTick() of the last broadcast */
} drive_inverter_state_t;

static const can_bus_config_t s_can_cfg = {
  .base_mc = CAN_BASE_MC_DEFAULT,
  .base_dc = CAN_BASE_DC_DEFAULT,
};
static drive_inverter_state_t s_drive_inverter; /* zero init: unsourced tx fields stay 0 */

static void app_start_adc(void) {
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)phase_current_ref_adc, ADC_CURRENT_COUNT);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)phase_current_adc, ADC_CURRENT_COUNT);
}

static void app_start_hrtim(void) {
  HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TE1);
}

void app_init(void) {
  motor_interface_init();
  app_start_adc();
  foc_init();
  app_start_hrtim();
  can_bus_init(&s_can_cfg);

  LL_GPIO_SetOutputPin(PHASE_EN_GPIO_Port, PHASE_EN_Pin);
  HAL_Delay(PHASE_EN_SETTLE_MS);

  mc.mode = MC_DISABLED;
  s_drive_inverter.fsm = APP_IDLE;
}

/* Map the latest drive command (current_limit pu) onto the FOC iq ref */
static void app_apply_drive(void) {
  float cl = s_drive_inverter.rx.current_limit;
  if (cl < 0.0f) cl = 0.0f;
  if (cl > 1.0f) cl = 1.0f;
  mc.cmd.iq_ref = cl * MC_MAX_CURRENT_A;
}

/* Drive watchdog and arming state machine */
static void app_run_state(void) {
  bool fresh = s_drive_inverter.rx.fresh;
  s_drive_inverter.rx.fresh = false;
  if (fresh) {
    app_apply_drive();
  }

  bool stale = (HAL_GetTick() - can_bus_last_dc_tick()) > DRIVE_WATCHDOG_MS;
  bool bus_ok = can_bus_ok();

  switch (s_drive_inverter.fsm) {
    case APP_IDLE:
      if (fresh && bus_ok) {
        mc.mode = MC_FOC;
        s_drive_inverter.fsm = APP_RUNNING;
      }
      break;

    case APP_RUNNING:
      if (stale || !bus_ok) {
        mc.cmd.iq_ref = 0.0f;
        mc.mode = MC_DISABLED;
        s_drive_inverter.fsm = APP_FAULT;
      }
      break;

    case APP_FAULT:
      /* Recover only once the bus is healthy and frames are flowing */
      if (!stale && bus_ok) {
        s_drive_inverter.fsm = APP_IDLE;
      }
      break;

    case APP_INIT:
    default:
      break;
  }
}

/* Snapshot mc and sensor signals into the tx struct and broadcast. Fields
 * with no source yet (velocity, bemf, rails, temps, fan, odometer,
 * observer/position_error) stay at their zero init value */
static void app_populate_status(void) {
  s_drive_inverter.tx.bus_voltage = mc.vbus;
  s_drive_inverter.tx.phase_ib = mc.phase_currents.i_b;
  s_drive_inverter.tx.phase_ic = mc.phase_currents.i_c;
  s_drive_inverter.tx.motor_vd = mc.cmd.ud;
  s_drive_inverter.tx.motor_vq = mc.cmd.uq;
  s_drive_inverter.tx.motor_id = mc.dq.id;
  s_drive_inverter.tx.motor_iq = mc.dq.iq;
  s_drive_inverter.tx.encoder_position = motor_interface_get_position_rad();
  s_drive_inverter.tx.el_angle = mc.el_angle;

  can_tx_all(&s_drive_inverter.tx, s_can_cfg.base_mc);
}

void app_step(void) {
  can_bus_run(); /* drain last cycle's queued status frames */

  can_msg_t msg;
  while (can_bus_receive(&msg)) {
    can_rx_dispatch(&s_drive_inverter.rx, &msg, s_can_cfg.base_dc);
  }

  app_run_state();

  uint32_t now = HAL_GetTick();
  if ((now - s_drive_inverter.last_status_tx) >= STATUS_TX_MS) {
    s_drive_inverter.last_status_tx = now;
    app_populate_status();
  }
}

void app_control_isr(void) {
  if (mc.mode == MC_FOC) {
    foc_step(CONTROL_DT, motor_interface_get_position_rad());
  } else if (mc.mode == MC_OPEN_LOOP) {
    mc.el_angle = motor_interface_get_position_rad();
    mc.cmd.uq = OPEN_LOOP_UQ_V;
    foc_open_loop_step();
  }
}

app_state_t app_state(void) {
  return s_drive_inverter.fsm;
}

/** @} */
