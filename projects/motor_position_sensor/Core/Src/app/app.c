/************************************************************************************************
 * @file    app.c
 *
 * @brief   Source file for the drive position sensor application supervisor
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */
#include "main.h"

/* Intra-component Headers */
#include "app.h"
#include "mlx90382.h"
#include "thermistor.h"
#include "ws22.h"

/**
 * @defgroup App
 * @brief    Drive Position Sensor Application Layer
 * @{
 */

static bool s_device_type_sent = false;
static volatile AppStatus s_app_status = APP_STATUS_OK;

void app_init(void) {
  if (thermistor_init() != HAL_OK) {
    Error_Handler();
  }

  if (mlx90382_init() != HAL_OK) {
    Error_Handler();
  }

  if (mlx90382_kick() != HAL_OK) {
    Error_Handler();
  }
}

void app_run(void) {
  uint16_t angle_raw;

  if (mlx90382_take_reading(&angle_raw)) {
    uint16_t thermistor_raw;
    HAL_StatusTypeDef tx_status;

    if (thermistor_read(&thermistor_raw) != HAL_OK) {
      s_app_status = APP_STATUS_ADC_ERROR;
    }

    if (!s_device_type_sent) {
      tx_status = ws22_send_device_type(angle_raw);
      s_device_type_sent = true;
    } else {
      tx_status = ws22_send_data(thermistor_raw, angle_raw);
    }

    if (tx_status != HAL_OK) {
      s_app_status = APP_STATUS_UART_ERROR;
    } else if (s_app_status != APP_STATUS_ADC_ERROR) {
      s_app_status = APP_STATUS_OK;
    }
  }

  /* Rearm the next frame, back off briefly only if the HAL start failed */
  if (mlx90382_kick() == HAL_ERROR) {
    s_app_status = APP_STATUS_SPI_ERROR;
    HAL_Delay(1U);
  }
}

AppStatus app_status(void) {
  return s_app_status;
}

/** @} */
