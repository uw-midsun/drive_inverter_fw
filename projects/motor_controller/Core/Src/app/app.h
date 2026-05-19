#pragma once

/************************************************************************************************
 * @file   app.h
 *
 * @brief  Header file for the drive inverter application supervisor
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup App
 * @brief    Drive Inverter Application Layer
 * @{
 */

/**
 * @brief   High level application state
 */
typedef enum {
  APP_INIT = 0, /**< Peripherals up, motor disabled */
  APP_IDLE,     /**< Armed, waiting for a fresh drive command */
  APP_RUNNING,  /**< Drive command fresh, FOC active */
  APP_FAULT,    /**< Drive watchdog timeout or CAN bus error */
} app_state_t;

/**
 * @brief   Initialize peripherals, the motor controller and the CAN bus
 */
void app_init(void);

/**
 * @brief   Supervisor tick, call continuously from the main loop
 * @details Self paces via HAL_GetTick: drains CAN rx into the rx struct, runs
 *          the drive watchdog and arming state machine, and broadcasts CAN
 *          status frames every STATUS_TX_MS
 */
void app_step(void);

/**
 * @brief   Fast control path, call from HRTIM1_Master_IRQHandler
 * @details Runs at HRTIM_FREQ_HZ (~20.75 kHz) after the peripheral IT is
 *          acked. Reads the encoder and runs FOC or open loop per mc.mode
 */
void app_control_isr(void);

/**
 * @brief   Get the current application state
 * @return  Current app_state_t
 */
app_state_t app_state(void);

/** @} */
