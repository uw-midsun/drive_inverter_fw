#pragma once

/************************************************************************************************
 * @file   app.h
 *
 * @brief  Header file for the drive position sensor application supervisor
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup App
 * @brief    Drive Position Sensor Application Layer
 * @{
 */

/**
 * @brief   High level application status
 */
typedef enum {
  APP_STATUS_OK = 0,     /**< Sensor pipeline healthy */
  APP_STATUS_SPI_ERROR,  /**< MLX90382 SPI transfer failed */
  APP_STATUS_UART_ERROR, /**< WS22 host link transmit failed */
  APP_STATUS_ADC_ERROR,  /**< Thermistor ADC conversion failed */
} AppStatus;

/**
 * @brief   Initialize the thermistor and MLX90382 and start the first frame
 * @details Halts in Error_Handler on any bring up failure
 */
void app_init(void);

/**
 * @brief   Run one supervisor iteration, call continuously from the main loop
 * @details Drains a completed angle frame, samples the thermistor, emits the
 *          WS22 packet and rearms the next SPI DMA frame
 */
void app_run(void);

/**
 * @brief   Get the latest application status
 * @return  Current AppStatus
 */
AppStatus app_status(void);

/** @} */
