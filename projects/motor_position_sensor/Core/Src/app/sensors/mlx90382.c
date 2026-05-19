/************************************************************************************************
 * @file    mlx90382.c
 *
 * @brief   Source file for the MLX90382 magnetic angle sensor driver
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <stddef.h>

/* Inter-component Headers */
#include "main.h"
#include "spi.h"

/* Intra-component Headers */
#include "mlx90382.h"

/**
 * @defgroup Sensors
 * @brief    Drive Position Sensor peripherals
 * @{
 */

#define MLX90382_CS_Pin GPIO_PIN_12
#define MLX90382_CS_GPIO_Port GPIOB

#define SPI_TIMEOUT_MS 10U

#define MLX90382_RR_BASE 0xCCU
#define MLX90382_RW_BASE 0x78U
#define MLX90382_FR_COMMAND 0x00U

#define MLX90382_MAX_REG_ADDR 0x025EU
#define MLX90382_REG_CONFIG 0x0200U
#define MLX90382_REG_FADDR01 0x0230U
#define MLX90382_REG_FADDR23 0x0232U
#define MLX90382_REG_FR_CONFIG 0x0234U

#define MLX90382_CONFIG_SENSING_MODE_MASK 0x0007U
#define MLX90382_CONFIG_GPIO_IF_MASK 0x0018U
#define MLX90382_SENSING_MODE_YZ 0x0002U
#define MLX90382_GPIO_IF_SPI_BUS 0x0010U

#define MLX90382_FR_PATTERN 0x000AU
#define MLX90382_FR_FS_ENABLE 0x0010U
#define MLX90382_FR_CRC_DISABLE 0x0000U
#define MLX90382_FR_CONFIG_5BYTE_FRAME (MLX90382_FR_PATTERN | MLX90382_FR_FS_ENABLE | MLX90382_FR_CRC_DISABLE)

#define MLX90382_FRAME_TRANSFER_SIZE 5U

/**
 * @brief   MLX90382 driver state
 */
typedef struct {
  uint8_t frame_tx[MLX90382_FRAME_TRANSFER_SIZE]; /**< DMA transmit frame */
  uint8_t frame_rx[MLX90382_FRAME_TRANSFER_SIZE]; /**< DMA receive frame */
  volatile bool dma_busy;                         /**< SPI DMA transfer in flight */
  volatile bool dma_done;                         /**< Unread reading available */
  volatile uint16_t reading;                      /**< Latest raw angle word */
  volatile uint32_t error_count;                  /**< Cumulative SPI error count */
} mlx90382_data_t;

static mlx90382_data_t s_mlx = {
  .frame_tx = { MLX90382_FR_COMMAND, 0x00U, 0x00U, 0x00U, 0x00U },
};

static void mlx90382_chip_select(bool selected) {
  HAL_GPIO_WritePin(MLX90382_CS_GPIO_Port, MLX90382_CS_Pin, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint16_t mlx90382_to_spi_addr(uint16_t reg_addr) {
  return (uint16_t)(reg_addr >> 1U);
}

static HAL_StatusTypeDef mlx90382_register_read(uint16_t reg_addr, uint16_t *data) {
  HAL_StatusTypeDef hal_status;
  uint16_t spi_addr;
  uint8_t tx_buffer[5U];
  uint8_t rx_buffer[5U];

  if ((data == NULL) || (reg_addr > MLX90382_MAX_REG_ADDR)) {
    return HAL_ERROR;
  }

  spi_addr = mlx90382_to_spi_addr(reg_addr);

  tx_buffer[0U] = (uint8_t)(MLX90382_RR_BASE | ((spi_addr >> 8U) & 0x01U));
  tx_buffer[1U] = (uint8_t)(spi_addr & 0xFFU);
  tx_buffer[2U] = 0x00U;
  tx_buffer[3U] = 0x00U;
  tx_buffer[4U] = 0x00U;

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, sizeof(tx_buffer), SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  if (hal_status != HAL_OK) {
    return hal_status;
  }

  *data = (uint16_t)(((uint16_t)rx_buffer[3U] << 8U) | rx_buffer[4U]);
  return HAL_OK;
}

static HAL_StatusTypeDef mlx90382_register_write(uint16_t reg_addr, uint16_t data) {
  HAL_StatusTypeDef hal_status;
  uint16_t spi_addr;
  uint8_t tx_buffer[4U];
  uint8_t rx_buffer[4U];

  if (reg_addr > MLX90382_MAX_REG_ADDR) {
    return HAL_ERROR;
  }

  spi_addr = mlx90382_to_spi_addr(reg_addr);

  tx_buffer[0U] = (uint8_t)(MLX90382_RW_BASE | ((spi_addr >> 8U) & 0x01U));
  tx_buffer[1U] = (uint8_t)(spi_addr & 0xFFU);
  tx_buffer[2U] = (uint8_t)((data >> 8U) & 0xFFU);
  tx_buffer[3U] = (uint8_t)(data & 0xFFU);

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive(&hspi2, tx_buffer, rx_buffer, sizeof(tx_buffer), SPI_TIMEOUT_MS);
  mlx90382_chip_select(false);

  return hal_status;
}

HAL_StatusTypeDef mlx90382_init(void) {
  HAL_StatusTypeDef hal_status;
  uint16_t config_reg = 0U;

  hal_status = mlx90382_register_read(MLX90382_REG_CONFIG, &config_reg);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  config_reg &= (uint16_t)~MLX90382_CONFIG_SENSING_MODE_MASK;
  config_reg |= MLX90382_SENSING_MODE_YZ;

  config_reg &= (uint16_t)~MLX90382_CONFIG_GPIO_IF_MASK;
  config_reg |= MLX90382_GPIO_IF_SPI_BUS;

  hal_status = mlx90382_register_write(MLX90382_REG_CONFIG, config_reg);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FADDR01, 0x0000U);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FADDR23, 0x0000U);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  hal_status = mlx90382_register_write(MLX90382_REG_FR_CONFIG, MLX90382_FR_CONFIG_5BYTE_FRAME);
  if (hal_status != HAL_OK) {
    return hal_status;
  }

  return HAL_OK;
}

static uint16_t mlx90382_parse_frame_word(const uint8_t *frame) {
  return (uint16_t)(((uint16_t)frame[3U] << 8U) | frame[4U]);
}

HAL_StatusTypeDef mlx90382_kick(void) {
  HAL_StatusTypeDef hal_status;

  if (s_mlx.dma_busy || s_mlx.dma_done) {
    return HAL_BUSY;
  }

  s_mlx.dma_busy = true;

  mlx90382_chip_select(true);
  hal_status = HAL_SPI_TransmitReceive_DMA(&hspi2, s_mlx.frame_tx, s_mlx.frame_rx, MLX90382_FRAME_TRANSFER_SIZE);
  if (hal_status != HAL_OK) {
    mlx90382_chip_select(false);
    s_mlx.dma_busy = false;
    s_mlx.error_count++;
  }

  return hal_status;
}

bool mlx90382_take_reading(uint16_t *angle_raw) {
  if (!s_mlx.dma_done) {
    return false;
  }

  *angle_raw = (uint16_t)s_mlx.reading;
  s_mlx.dma_done = false;
  return true;
}

uint32_t mlx90382_error_count(void) {
  return s_mlx.error_count;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    mlx90382_chip_select(false);
    s_mlx.reading = mlx90382_parse_frame_word(s_mlx.frame_rx);
    s_mlx.dma_busy = false;
    s_mlx.dma_done = true;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    mlx90382_chip_select(false);
    s_mlx.dma_busy = false;
    s_mlx.dma_done = false;
    s_mlx.error_count++;
  }
}

/** @} */
