/************************************************************************************************
 * @file    can_bus.c
 *
 * @brief   Source file for the FDCAN2 ring buffered bus driver
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <string.h>

/* Inter-component Headers */
#include "fdcan.h"
#include "main.h"

/* Intra-component Headers */
#include "can_bus.h"

/**
 * @defgroup CAN
 * @brief    Drive Inverter CAN Interface
 * @{
 */

/*
 * CubeMX (fdcan.c) owns hfdcan2 and calls HAL_FDCAN_Init with the
 * corrected 500 kbps timing (prescaler=20, TS1=13, TS2=3)
 * CubeMX (stm32g4xx_it.c) owns FDCAN2_IT0_IRQHandler to HAL_FDCAN_IRQHandler
 *
 * This file adds: a global accept all filter, notification activation,
 * HAL_FDCAN_Start, an ISR driven RX ring buffer, and a software TX ring
 * pumped into the 3 deep hardware FIFO by can_bus_run()
 *
 * Call order in main.c (USER CODE sections):
 *   MX_FDCAN2_Init();      CubeMX generated, already in the init block
 *   can_bus_init(&cfg);    must follow immediately
 */

#define CAN_RX_BUF_MASK (CAN_RX_BUF_SIZE - 1U)
#define CAN_TX_BUF_MASK (CAN_TX_BUF_SIZE - 1U)

/* All driver state in one place:
 *   rx_* : the ISR is the producer, the main loop the consumer, so volatile
 *   tx_* : both ends run in the main context, so plain
 *   head/tail are word aligned: R/W is atomic on Cortex M */
typedef struct {
  can_bus_config_t cfg;

  can_msg_t rx_buf[CAN_RX_BUF_SIZE];
  volatile uint32_t rx_head;
  volatile uint32_t rx_tail;

  can_msg_t tx_buf[CAN_TX_BUF_SIZE];
  uint32_t tx_head;
  uint32_t tx_tail;

  volatile uint32_t last_dc_tick;
} can_bus_t;

static can_bus_t s_can_bus;

/* FDCAN DataLength field encodes the byte count in bits [19:16] */
static inline uint32_t can_bus_len_to_dlc(uint8_t len) {
  return (uint32_t)len << 16U;
}
static inline uint8_t can_bus_dlc_to_len(uint32_t dlc) {
  return (uint8_t)(dlc >> 16U);
}

/* Public API */

void can_bus_init(const can_bus_config_t *cfg) {
  s_can_bus.cfg = *cfg;

  /* Accept all standard and extended frames to FIFO0 */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  /* Route the RX FIFO0 interrupt to IT line 0 */
  if (HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
}

bool can_bus_transmit(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len > 8U) {
    len = 8U;
  }

  uint32_t head = s_can_bus.tx_head;
  uint32_t next = (head + 1U) & CAN_TX_BUF_MASK;
  if (next == s_can_bus.tx_tail) {
    return false; /* software TX ring full so we drop */
  }

  can_msg_t *slot = &s_can_bus.tx_buf[head & CAN_TX_BUF_MASK];
  slot->id = id;
  slot->len = len;
  memcpy(slot->data, data, len);

  s_can_bus.tx_head = next;
  return true;
}

void can_bus_run(void) {
  while (s_can_bus.tx_tail != s_can_bus.tx_head && HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U) {
    const can_msg_t *slot = &s_can_bus.tx_buf[s_can_bus.tx_tail & CAN_TX_BUF_MASK];

    FDCAN_TxHeaderTypeDef hdr = {
      .Identifier = slot->id,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = can_bus_len_to_dlc(slot->len),
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0U,
    };

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &hdr, (uint8_t *)slot->data) != HAL_OK) {
      break; /* leave it queued, retry next can_bus_run() */
    }
    s_can_bus.tx_tail = (s_can_bus.tx_tail + 1U) & CAN_TX_BUF_MASK;
  }
}

bool can_bus_receive(can_msg_t *out) {
  uint32_t tail = s_can_bus.rx_tail;
  if (tail == s_can_bus.rx_head) {
    return false;
  }
  *out = s_can_bus.rx_buf[tail & CAN_RX_BUF_MASK];
  s_can_bus.rx_tail = (tail + 1U) & CAN_RX_BUF_MASK;
  return true;
}

bool can_bus_ok(void) {
  FDCAN_ProtocolStatusTypeDef ps;
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan2, &ps) != HAL_OK) {
    return false;
  }

  return ps.BusOff == 0U && ps.ErrorPassive == 0U;
}

uint32_t can_bus_last_dc_tick(void) {
  return s_can_bus.last_dc_tick;
}

void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan) {
  (void)hfdcan;

  FDCAN_RxHeaderTypeDef hdr;
  uint8_t raw[8] = { 0 };

  if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &hdr, raw) != HAL_OK) {
    return;
  }

  uint32_t head = s_can_bus.rx_head;
  uint32_t next = (head + 1U) & CAN_RX_BUF_MASK;
  if (next == s_can_bus.rx_tail) {
    return; /* ring buffer full, so we drop */
  }

  uint8_t len = can_bus_dlc_to_len(hdr.DataLength);
  if (len > 8U) {
    len = 8U;
  }

  can_msg_t *slot = &s_can_bus.rx_buf[head & CAN_RX_BUF_MASK];
  slot->id = hdr.Identifier;
  slot->len = len;
  memcpy(slot->data, raw, len);

  s_can_bus.rx_head = next;

  if (hdr.Identifier >= s_can_bus.cfg.base_dc && hdr.Identifier < s_can_bus.cfg.base_dc + 0x10U) {
    s_can_bus.last_dc_tick = HAL_GetTick();
  }
}

/** @} */
