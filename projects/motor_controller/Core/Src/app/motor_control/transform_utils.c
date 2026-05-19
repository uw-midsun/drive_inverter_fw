/************************************************************************************************
 * @file   transform_utils.c
 *
 * @brief  Source file for stateless Clarke and Park transform utilities
 *
 * @date   2026-05-19
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */
#include "transform_utils.h"

#include "math_defs.h"

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

void clarke_transform(float i_a, float i_b, float *alpha, float *beta) {
  *alpha = i_a;
  *beta = i_a * INVERSE_SQRT3_F + i_b * INVERSE_2SQRT3_F;
}

void park_transform(float alpha, float beta, float sin_th, float cos_th, float *d, float *q) {
  *d = alpha * cos_th + beta * sin_th;
  *q = -alpha * sin_th + beta * cos_th;
}

void inverse_park_transform(float d, float q, float sin_th, float cos_th, float *alpha, float *beta) {
  *alpha = d * cos_th - q * sin_th;
  *beta = d * sin_th + q * cos_th;
}

/** @} */
