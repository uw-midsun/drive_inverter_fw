#pragma once

/************************************************************************************************
 * @file   transform_utils.h
 *
 * @brief  Header file for stateless Clarke and Park transform utilities
 *
 * @date   2026-05-19
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

/**
 * @brief   Two phase Clarke transform (balanced system)
 * @details Maps phase currents into the stationary alpha beta frame assuming a
 *          balanced load, so phase C is implied by i_c = -i_a - i_b
 * @param   i_a Phase A current (A)
 * @param   i_b Phase B current (A)
 * @param   alpha Alpha axis result (A)
 * @param   beta Beta axis result (A)
 */
void clarke_transform(float i_a, float i_b, float *alpha, float *beta);

/**
 * @brief   Park transform (stationary alpha beta to rotating dq)
 * @param   alpha Alpha axis input
 * @param   beta Beta axis input
 * @param   sin_th Sine of the electrical angle
 * @param   cos_th Cosine of the electrical angle
 * @param   d Direct axis result
 * @param   q Quadrature axis result
 */
void park_transform(float alpha, float beta, float sin_th, float cos_th, float *d, float *q);

/**
 * @brief   Inverse Park transform (rotating dq to stationary alpha beta)
 * @param   d Direct axis input
 * @param   q Quadrature axis input
 * @param   sin_th Sine of the electrical angle
 * @param   cos_th Cosine of the electrical angle
 * @param   alpha Alpha axis result
 * @param   beta Beta axis result
 */
void inverse_park_transform(float d, float q, float sin_th, float cos_th, float *alpha, float *beta);

/** @} */
