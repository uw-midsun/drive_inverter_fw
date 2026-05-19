/************************************************************************************************
 * @file    retarget.c
 *
 * @brief   Source file for retargeting printf to UART4
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <errno.h>
#include <sys/stat.h>

/* Inter-component Headers */
#include "usart.h"

/* Intra-component Headers */
#include "retarget.h"

/**
 * @defgroup BSP
 * @brief    Drive Inverter Board Support Package
 * @{
 */

/* Retarget _write to UART4 for printf output */
extern UART_HandleTypeDef huart4;

int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart4, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/** @} */
