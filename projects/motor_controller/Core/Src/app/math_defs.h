#pragma once

/************************************************************************************************
 * @file   math_defs.h
 *
 * @brief  Header file for shared fixed constants and math helpers
 *
 * @date   2026-05-18
 * @author Midnight Sun Team #24
 ************************************************************************************************/

/* Standard library Headers */

/* Inter-component Headers */

/* Intra-component Headers */

/**
 * @defgroup Motor_Control
 * @brief    Drive Inverter Motor Control
 * @{
 */

#define SQRT3_F 1.732050807568877f
#define HALF_SQRT3_F 0.8660254037844386f
#define INVERSE_SQRT3_F 0.5773502691896258f
#define INVERSE_2SQRT3_F 1.154700538379252f
#define PI_F 3.141592653589793f
#define PI2_F 6.283185307179586f

/**
 * @brief   Wrap an angle into [0, 2pi)
 * @param   x Input angle (rad)
 * @return  Wrapped angle (rad)
 */
static inline float wrap_angle(float x) {
  for (int i = 0; i < 3; i++) {
    if (x >= PI2_F)
      x -= PI2_F;
    else if (x < 0.0f)
      x += PI2_F;
    else
      return x;
  }
  if (x < 0.0f || x >= PI2_F) x = 0.0f;
  return x;
}

/**
 * @brief   Wrap an angle into [-pi, pi]
 * @param   x Input angle (rad)
 * @return  Wrapped angle (rad)
 */
static inline float wrap_to_pm_pi(float x) {
  for (int i = 0; i < 3; i++) {
    if (x > PI_F)
      x -= PI2_F;
    else if (x < -PI_F)
      x += PI2_F;
    else
      return x;
  }
  if (x < -PI_F || x > PI_F) x = 0.0f;
  return x;
}

/**
 * @brief   Clamp a value to the inclusive range [lo, hi]
 * @param   val Input value
 * @param   lo Lower bound
 * @param   hi Upper bound
 * @return  Clamped value
 */
static inline float clampf(float val, float lo, float hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

/** @} */
