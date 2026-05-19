/************************************************************************************************
 * @file    thermistor.c
 *
 * @brief   Source file for the motor thermistor ADC reader
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>

/* Inter-component Headers */
#include "adc.h"

/* Intra-component Headers */
#include "thermistor.h"

/**
 * @defgroup Sensors
 * @brief    Drive Position Sensor peripherals
 * @{
 */

#define THERMISTOR_ADC_CHANNEL ADC_CHANNEL_9
#define ADC_TIMEOUT_MS 10U

/**
 * @brief   Thermistor reader state
 */
typedef struct {
  volatile uint32_t error_count; /**< Cumulative ADC error count */
} thermistor_data_t;

static thermistor_data_t s_thermistor;

HAL_StatusTypeDef thermistor_init(void) {
  ADC_ChannelConfTypeDef channel_config = { 0 };

  channel_config.Channel = THERMISTOR_ADC_CHANNEL;
  channel_config.Rank = ADC_REGULAR_RANK_1;
  channel_config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

  if (HAL_ADC_ConfigChannel(&hadc1, &channel_config) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef thermistor_read(uint16_t *value) {
  if (value == NULL) {
    return HAL_ERROR;
  }

  *value = 0U;

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    s_thermistor.error_count++;
    return HAL_ERROR;
  }

  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    s_thermistor.error_count++;
    return HAL_ERROR;
  }

  *value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return HAL_OK;
}

uint32_t thermistor_error_count(void) {
  return s_thermistor.error_count;
}

/** @} */
