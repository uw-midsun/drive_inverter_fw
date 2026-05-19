/************************************************************************************************
 * @file    ws22.c
 *
 * @brief   Source file for the WaveSculptor22 position packet protocol
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stddef.h>

/* Inter-component Headers */
#include "usart.h"

/* Intra-component Headers */
#include "ws22.h"

/**
 * @defgroup Comms
 * @brief    Drive Position Sensor host link
 * @{
 */

#define WS22_PACKET_SIZE 4U
#define WS22_SENSOR_DEVICE_TYPE 5U

#define MASK_6B 0x3FU
#define MASK_7B 0x7FU

/**
 * @brief   WaveSculptor22 link state
 */
typedef struct {
  uint8_t packet[2U][WS22_PACKET_SIZE]; /**< Double buffer storage */
  volatile uint8_t tx_index;            /**< Buffer currently transmitting */
  volatile uint8_t fill_index;          /**< Buffer being filled */
  volatile bool tx_busy;                /**< UART transfer in flight */
  volatile bool packet_pending;         /**< Fill buffer holds a fresh packet */
  volatile uint32_t error_count;        /**< Cumulative UART error count */
} ws22_comms_data_t;

static ws22_comms_data_t s_ws22 = {
  .tx_index = 0U,
  .fill_index = 1U,
};

static void ws22_build_device_type_packet(uint8_t *packet, uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] = (uint8_t)((0x02U << 6U) | (WS22_SENSOR_DEVICE_TYPE & MASK_6B));
  packet[1U] = 0x00U;
  packet[2U] = (uint8_t)((angle_14b >> 7U) & MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & MASK_7B);
}

static void ws22_build_data_packet(uint8_t *packet, uint16_t thermistor_raw, uint16_t angle_raw) {
  uint16_t angle_14b = (uint16_t)(angle_raw >> 2U);

  packet[0U] = (uint8_t)((0x03U << 6U) | ((thermistor_raw >> 10U) & MASK_6B));
  packet[1U] = (uint8_t)((thermistor_raw >> 3U) & MASK_7B);
  packet[2U] = (uint8_t)((angle_14b >> 7U) & MASK_7B);
  packet[3U] = (uint8_t)(angle_14b & MASK_7B);
}

static void ws22_queue_packet(const uint8_t *packet) {
  uint32_t index;

  __disable_irq();
  for (index = 0U; index < WS22_PACKET_SIZE; ++index) {
    s_ws22.packet[s_ws22.fill_index][index] = packet[index];
  }
  s_ws22.packet_pending = true;
  __enable_irq();
}

static HAL_StatusTypeDef ws22_kick_tx(void) {
  HAL_StatusTypeDef hal_status = HAL_OK;
  uint8_t *next_buffer = NULL;

  __disable_irq();
  if (!s_ws22.tx_busy && s_ws22.packet_pending) {
    uint8_t previous_tx_index = s_ws22.tx_index;

    s_ws22.tx_index = s_ws22.fill_index;
    s_ws22.fill_index = previous_tx_index;
    s_ws22.packet_pending = false;
    s_ws22.tx_busy = true;
    next_buffer = s_ws22.packet[s_ws22.tx_index];
  }
  __enable_irq();

  if (next_buffer != NULL) {
    hal_status = HAL_UART_Transmit_IT(&huart2, next_buffer, WS22_PACKET_SIZE);
    if (hal_status != HAL_OK) {
      __disable_irq();
      s_ws22.tx_busy = false;
      __enable_irq();
      s_ws22.error_count++;
    }
  }

  return hal_status;
}

HAL_StatusTypeDef ws22_send_device_type(uint16_t angle_raw) {
  uint8_t packet[WS22_PACKET_SIZE];

  ws22_build_device_type_packet(packet, angle_raw);
  ws22_queue_packet(packet);
  return ws22_kick_tx();
}

HAL_StatusTypeDef ws22_send_data(uint16_t thermistor_raw, uint16_t angle_raw) {
  uint8_t packet[WS22_PACKET_SIZE];

  ws22_build_data_packet(packet, thermistor_raw, angle_raw);
  ws22_queue_packet(packet);
  return ws22_kick_tx();
}

uint32_t ws22_error_count(void) {
  return s_ws22.error_count;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    s_ws22.tx_busy = false;
    if (ws22_kick_tx() != HAL_OK) {
      s_ws22.error_count++;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    s_ws22.tx_busy = false;
    s_ws22.error_count++;
  }
}

/** @} */
