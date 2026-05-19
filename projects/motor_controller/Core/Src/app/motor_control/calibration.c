/************************************************************************************************
 * @file    calibration.c
 *
 * @brief   Source file for encoder offset and linearization calibration
 *
 * @date    2026-05-18
 * @author  Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */
#include <math.h>
#include <stdio.h>

/* Inter-component Headers */

/* Intra-component Headers */
#include "calibration.h"
#include "foc.h"
#include "motor_interface.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

#define CALIB_OFFSET_SETTLE_MS 1000 /**< Initial rotor settle before the offset sweep (ms) */
#define CALIB_SWEEP_STEP_MS 250     /**< Per step settle during the full sweep (ms) */

static int16_t calibrate_wrap_diff(int32_t diff) {
  /* Bring into [0, ENCODER_COUNTS) */
  diff %= (int32_t)ENCODER_COUNTS;
  if (diff < 0) diff += (int32_t)ENCODER_COUNTS;

  /* Now map to [-ENCODER_COUNTS/2, +ENCODER_COUNTS/2) */
  if (diff > (int32_t)(ENCODER_COUNTS / 2U)) diff -= (int32_t)ENCODER_COUNTS;

  return (int16_t)diff;
}

void calibrate_offset(const float Ud, const uint16_t steps, const uint16_t delay) {
  printf("\r\n=== BEGIN OFFSET CALIBRATION ===\r\n");

  mc.cmd.ud = Ud;

  int64_t offset_sum = 0;
  int16_t offset = 0;
  mc.el_angle = -PI2_F * POLE_PAIRS / steps;
  foc_open_loop_step();
  HAL_Delay(CALIB_OFFSET_SETTLE_MS);

  for (uint16_t i = 0; i < steps; i++) {
    float theta_elec = (PI2_F * i * POLE_PAIRS) / steps;
    mc.el_angle = theta_elec;
    foc_open_loop_step();
    HAL_Delay(delay);

    uint16_t raw = motor_interface_get_position_raw();
    uint16_t cmd = i * ENCODER_COUNTS * POLE_PAIRS / steps;
    cmd = cmd % ENCODER_COUNTS;

    offset = raw - cmd;
    offset = calibrate_wrap_diff(offset);

    offset_sum += offset;

    printf("Step: %4u | Raw %5u, | Offset %4d\r\n", i, raw, offset);
  }

  int16_t offset_avg = offset_sum / steps;
  printf("Average Offset = %+6d\r\n", offset_avg);
  motor_interface_set_offset((offset_avg + ENCODER_COUNTS) & ENCODER_MASK);
}

void calibrate_run(const float Ud) {
  printf("\r\n=== BEGIN FULL MECHANICAL CALIBRATION ===\r\n");

  mc.cmd.ud = Ud;

  uint16_t raw_samples[CAL_SAMPLES];
  int16_t diff_samples[CAL_SAMPLES];

  /* 1. Sweep a full mechanical revolution */
  for (int i = 0; i < CAL_SAMPLES; i++) {
    /* Electrical angle ranges 0 to 2pi * POLE_PAIRS */
    float theta_elec = (2.0f * PI_F * POLE_PAIRS * i) / CAL_SAMPLES;
    mc.el_angle = theta_elec;

    foc_open_loop_step();
    HAL_Delay(CALIB_SWEEP_STEP_MS); /* shorter delay is OK with oversampling */

    uint16_t raw = motor_interface_get_position_raw();
    raw_samples[i] = raw;

    /* Commanded encoder counts for this electrical angle */
    uint16_t cmd = (uint16_t)lroundf(theta_elec * (ENCODER_COUNTS / (2.0f * PI_F)));

    int32_t diff32 = (int32_t)raw - (int32_t)cmd;
    int16_t diff = calibrate_wrap_diff(diff32);

    diff_samples[i] = diff;

    printf("Sample %4d | Raw %4u | Cmd %4u | Diff32 %6ld | Diff %+6d\r\n", i, raw, cmd, diff32, diff);
  }

  /* 2. Compute the global offset */
  int32_t sum = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) sum += diff_samples[i];
  int16_t avg = sum / CAL_SAMPLES;

  printf("\r\nGlobal offset (avg diff) = %+6d\r\n", avg);

  /* 3. Compute residuals */
  int16_t residual[CAL_SAMPLES];
  for (int i = 0; i < CAL_SAMPLES; i++) residual[i] = diff_samples[i] - avg;

  printf("\r\nResiduals computed.\r\n");

  /* 4. Re bin oversampled data into LUT_SIZE bins */
  int16_t lut[LUT_SIZE];

  printf("\r\n=== LUT RE-BINNING ===\r\n");

  for (int bin = 0; bin < LUT_SIZE; bin++) {
    uint32_t raw_lo = (bin * ENCODER_COUNTS) / LUT_SIZE;
    uint32_t raw_hi = ((bin + 1) * ENCODER_COUNTS) / LUT_SIZE;

    int32_t acc = 0;
    int count = 0;

    for (int i = 0; i < CAL_SAMPLES; i++) {
      uint16_t r = raw_samples[i];

      if (r >= raw_lo && r < raw_hi) {
        acc += residual[i];
        count++;
      }
    }

    if (count == 0)
      lut[bin] = 0;
    else
      lut[bin] = acc / count;

    printf("LUT[%3d] | raw_lo %4lu | raw_hi %4lu | count %3d | value %+6d\r\n", bin, (unsigned long)raw_lo, (unsigned long)raw_hi, count, lut[bin]);
  }

  /* 5. Store the LUT */
  motor_interface_set_lut(lut);
  motor_interface_save_calibration();

  printf("\r\n=== CALIBRATION COMPLETE ===\r\n");
  printf("Offset = %+6d counts\r\n", avg);
}

void calibrate_clear(void) {
  printf("\r\n=== CLEAR CALIBRATION ===\r\n");

  /* 1. Zero offset */
  motor_interface_set_offset(0);

  /* 2. Zero LUT */
  int16_t lut[LUT_SIZE];
  for (int i = 0; i < LUT_SIZE; i++) lut[i] = 0;

  motor_interface_set_lut(lut);

  /* 3. Save to flash */
  motor_interface_save_calibration();

  printf("Offset cleared.\r\n");
  printf("LUT cleared.\r\n");
  printf("Calibration reset to identity.\r\n");
}

/** @} */
