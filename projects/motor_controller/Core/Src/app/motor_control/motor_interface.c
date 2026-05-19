/************************************************************************************************
 * @file    motor_interface.c
 *
 * @brief   Source file for the WaveSculptor encoder interface
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stdio.h>

/* Inter-component Headers */
#include "dma.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_usart.h"
#include "usart.h"

/* Intra-component Headers */
#include "math_defs.h"
#include "motor_interface.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

/**
 * @brief   Persisted encoder calibration record
 */
typedef struct {
  uint16_t offset;       /**< Encoder zero offset in counts */
  uint8_t reverse;       /**< Direction reversal flag */
  uint8_t lut_valid;     /**< Set when the LUT is valid */
  int16_t lut[LUT_SIZE]; /**< Encoder linearization LUT */
} encoder_cal_data_t;

/**
 * @brief   WaveSculptor encoder interface state
 */
typedef struct {
  uint8_t dma_buf[WS22_DMA_BUF_SIZE]; /**< UART5 RX circular DMA buffer */
  motor_sensor_data_t sensor;         /**< Latest decoded sensor frame */
  uint16_t encoder_offset;            /**< Encoder zero offset in counts */
  uint8_t reverse_direction;          /**< Direction reversal flag */
  uint8_t lut_valid;                  /**< Set when the LUT is valid */
  int16_t lut[LUT_SIZE];              /**< Encoder linearization LUT */
} motor_interface_data_t;

static motor_interface_data_t s_mi = { 0 };

void motor_interface_init(void) {
  /* Start UART5 RX DMA in circular mode */
  HAL_UART_Receive_DMA(&huart5, s_mi.dma_buf, WS22_DMA_BUF_SIZE);

  /* Disable DMA interrupts for this channel (adjust DMAx/CHy if needed) */
  LL_DMA_DisableIT_HT(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);

  motor_interface_load_calibration();
}

static inline uint32_t motor_interface_dma_write_index(void) {
  uint32_t remaining = LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
  return (WS22_DMA_BUF_SIZE - remaining) & (WS22_DMA_BUF_SIZE - 1);
}

static int motor_interface_find_last_sync(void) {
  uint32_t w = motor_interface_dma_write_index();

  for (int i = 0; i < WS22_DMA_BUF_SIZE; i++) {
    uint32_t idx = (w + WS22_DMA_BUF_SIZE - 1 - i) & (WS22_DMA_BUF_SIZE - 1);

    uint8_t b0 = s_mi.dma_buf[idx];

    /* Sync byte: bit7 = 1 */
    if (b0 & 0x80) {
      uint8_t b1 = s_mi.dma_buf[(idx + 1) & (WS22_DMA_BUF_SIZE - 1)];
      uint8_t b2 = s_mi.dma_buf[(idx + 2) & (WS22_DMA_BUF_SIZE - 1)];
      uint8_t b3 = s_mi.dma_buf[(idx + 3) & (WS22_DMA_BUF_SIZE - 1)];

      /* Data bytes: bit7 = 0 */
      if (!(b1 & 0x80) && !(b2 & 0x80) && !(b3 & 0x80)) return (int)idx; /* frame start */
    }
  }

  return -1; /* no valid frame found */
}

static void motor_interface_parse_frame(void) {
  int idx = motor_interface_find_last_sync();
  if (idx < 0) return;

  uint8_t b0 = s_mi.dma_buf[idx];
  uint8_t b1 = s_mi.dma_buf[(idx + 1) & (WS22_DMA_BUF_SIZE - 1)];
  uint8_t b2 = s_mi.dma_buf[(idx + 2) & (WS22_DMA_BUF_SIZE - 1)];
  uint8_t b3 = s_mi.dma_buf[(idx + 3) & (WS22_DMA_BUF_SIZE - 1)];

  uint8_t msgType = (b0 >> 6) & 0x03;

  if (msgType == 0b10) {
    /* Device type packet */
    s_mi.sensor.device_type = b0 & 0x3F;
    s_mi.sensor.valid = 1;
  } else if (msgType == 0b11) {
    /* Temperature and position packet */
    s_mi.sensor.temperature = ((uint16_t)(b0 & 0x3F) << 7) | (b1 & 0x7F);
    s_mi.sensor.position = ((uint16_t)(b2 & 0x7F) << 7) | (b3 & 0x7F);
    s_mi.sensor.valid = 1;
  }
}

static void motor_interface_apply_calibration(const encoder_cal_data_t *cal) {
  s_mi.encoder_offset = cal->offset & ENCODER_MASK;
  s_mi.reverse_direction = cal->reverse ? 1 : 0;
  s_mi.lut_valid = cal->lut_valid ? 1 : 0;

  /* if (lut_valid) { */
  /*   for (int i = 0; i < LUT_SIZE; i++) */
  /*     s_mi.lut[i] = cal->lut[i]; */
  /* } */
}

void motor_interface_set_offset(uint16_t off) {
  s_mi.encoder_offset = off & ENCODER_MASK;
}

void motor_interface_set_reverse(uint8_t rev) {
  s_mi.reverse_direction = rev ? 1 : 0;
}

void motor_interface_set_lut(const int16_t *lut) {
  for (int i = 0; i < LUT_SIZE; i++) s_mi.lut[i] = lut[i];

  s_mi.lut_valid = 1;
}

void motor_interface_load_calibration(void) {
  const encoder_cal_data_t *cal = (const encoder_cal_data_t *)CALIB_FLASH_ADDR;

  /* Very simple validity check: lut_valid must be 0 or 1, reverse 0 or 1 */
  if (cal->lut_valid <= 1 && cal->reverse <= 1) {
    motor_interface_apply_calibration(cal);
  }
}

void motor_interface_save_calibration(void) {
  encoder_cal_data_t cal;

  if (s_mi.encoder_offset != cal.offset) {
    printf("Saving calibration: offset = %u counts\r\n", s_mi.encoder_offset);
    cal.offset = s_mi.encoder_offset;
  }

  if (s_mi.reverse_direction != cal.reverse) {
    printf("Saving calibration: reverse = %u\r\n", s_mi.reverse_direction);
    cal.reverse = s_mi.reverse_direction;
  }

  if (s_mi.lut_valid) {
    printf("Saving calibration: LUT:\r\n");
    cal.lut_valid = s_mi.lut_valid;

    for (int i = 0; i < LUT_SIZE; i++) {
      printf("  LUT[%d] = %+6d\r\n", i, s_mi.lut[i]);
      cal.lut[i] = s_mi.lut[i];
    }
  } else {
    cal.lut_valid = 0;
    printf("Saving calibration: lut_valid = %u\r\n", s_mi.lut_valid);
  }

  HAL_FLASH_Unlock();

  /* Erase one page */
  FLASH_EraseInitTypeDef erase = { 0 };
  uint32_t pageError = 0;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = (CALIB_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = 1;

  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return;
  }

  /* Program as 64 bit double words */
  uint64_t *src = (uint64_t *)&cal;
  uint32_t addr = CALIB_FLASH_ADDR;
  uint32_t bytes = sizeof(encoder_cal_data_t);

  for (uint32_t i = 0; i < (bytes + 7) / 8; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
      break;
    }
    addr += 8;
  }

  HAL_FLASH_Lock();
}

uint16_t motor_interface_get_position(void) {
  motor_interface_parse_frame();
  uint16_t raw = s_mi.sensor.position & ENCODER_MASK;

  if (s_mi.reverse_direction) raw = (ENCODER_COUNTS - raw) & ENCODER_MASK;

  raw = (raw - s_mi.encoder_offset) & ENCODER_MASK;

  if (s_mi.lut_valid) {
    uint32_t idx = raw >> 5; /* 256 entry LUT */
    uint32_t frac = raw & 0x1F;

    int16_t o0 = s_mi.lut[idx];
    int16_t o1 = s_mi.lut[(idx + 1) & 0xFF];

    int32_t interp = o0 + (((int32_t)(o1 - o0) * frac) >> 5);

    raw = (raw + interp) & ENCODER_MASK;
  }

  return raw;
}

float motor_interface_get_position_rad(void) {
  return (float)motor_interface_get_position() / ENCODER_COUNTS * PI2_F;
}

uint16_t motor_interface_get_position_raw(void) {
  motor_interface_parse_frame();

  uint16_t raw = s_mi.sensor.position & ENCODER_MASK;

  /* Apply direction reversal only */
  if (s_mi.reverse_direction) raw = (ENCODER_COUNTS - raw) & ENCODER_MASK;

  return raw;
}

motor_sensor_data_t motor_interface_get_data(void) {
  motor_interface_parse_frame();
  return s_mi.sensor;
}

/** @} */
