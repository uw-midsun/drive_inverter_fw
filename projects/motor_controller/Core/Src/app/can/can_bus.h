#pragma once

/************************************************************************************************
 * @file   can_bus.h
 *
 * @brief  Header file for the FDCAN2 ring buffered bus driver
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdbool.h>
#include <stdint.h>

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup CAN
 * @brief    Drive Inverter CAN Interface
 * @{
 */

#define CAN_RX_BUF_SIZE 32U /**< RX ring buffer depth, power of 2 */
#define CAN_TX_BUF_SIZE 32U /**< TX ring buffer depth, power of 2 */

/**
 * @brief   CAN base address configuration
 */
typedef struct {
  uint32_t base_mc; /**< Motor controller base address (default 0x400) */
  uint32_t base_dc; /**< Driver controls base address (default 0x500) */
} can_bus_config_t;

/**
 * @brief   Single CAN frame
 */
typedef struct {
  uint32_t id;     /**< Standard frame identifier */
  uint8_t data[8]; /**< Frame payload */
  uint8_t len;     /**< Payload length in bytes */
} can_msg_t;

/**
 * @brief   Initialize FDCAN2, configure filters, start the peripheral and enable RX IRQ
 * @param   cfg Pointer to the base address configuration
 */
void can_bus_init(const can_bus_config_t *cfg);

/**
 * @brief   Enqueue a frame into the software TX ring
 * @param   id Standard frame identifier
 * @param   data Pointer to the payload
 * @param   len Payload length in bytes
 * @return  true if queued, false if the ring is full and the frame was dropped
 */
bool can_bus_transmit(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief   Drain the software TX ring into the hardware FIFO while slots are free
 */
void can_bus_run(void);

/**
 * @brief   Pop one received frame
 * @param   out Pointer to receive the frame
 * @return  true if a frame was returned, false if the ring is empty
 */
bool can_bus_receive(can_msg_t *out);

/**
 * @brief   Report bus health
 * @return  true if no bus off or error passive condition
 */
bool can_bus_ok(void);

/**
 * @brief   Timestamp of the last frame received from base_dc
 * @return  HAL_GetTick() value at the last driver controls frame
 */
uint32_t can_bus_last_dc_tick(void);

/** @} */
