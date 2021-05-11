/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef MISC_QUATERNION_H
#define MISC_QUATERNION_H

#include <stddef.h>
#include <stdlib.h>
#include "MathLib.h"

#define devec(x) (&x[1]) // devectorize from dimension 4 to dimension 3 (take bottom 3 elements) - works only on vector (float array)

// Quaternion helper functions for computations with quaternions of the format
//   q = {q0, q1, q2, q3} = {w, x, y, z} = {w, v}
// Hence a 4-dimensional vector where the scalar value is first and the vector part are the 3 bottom elements
//   w = q0
//   v = {q1, q2, q3}

void Quaternion_Phi(const float q[4], const float p[4], float result[4]); // result = q o p = Phi(q)*p
void Quaternion_devecPhi(const float q[4], const float p[4], float result[3]); // result = V * q o p = V*Phi(q)*p
void Quaternion_PhiT(const float q[4], const float p[4], float result[4]); // result = q* o p = Phi(q)^T*p
void Quaternion_devecPhiT(const float q[4], const float p[4], float result[3]); // result = V * q* o p = V*Phi(q)^T*p
void Quaternion_mat_Phi(const float q[4], float mat[4*4]);
void Quaternion_mat_PhiVec(const float q[4], float mat[4*3]);
void Quaternion_mat_PhiT(const float q[4], float mat[4*4]);
void Quaternion_mat_devecPhiT(const float q[4], float mat[3*4]);
void Quaternion_Gamma(const float p[4], const float q[4], float result[4]); // result = q o p = Gamma(p)*q
void Quaternion_GammaT(const float p[4], const float q[4], float result[4]); // result = q o p* = Gamma(p)^T*q
void Quaternion_mat_Gamma(const float p[4], float mat[4*4]); // mat = Gamma(p)
void Quaternion_mat_GammaT(const float p[4], float mat[4*4]); // mat = Gamma(p)'
void Quaternion_mat_devecGammaT(const float p[4], float mat[3*4]); // mat = devec*Gamma(p)'
void Quaternion_Conjugate(const float q[4], float result[4]); // result = q*
void Quaternion_Conjugate(float q[4]); // q = q*
void Quaternion_Negate(float q[4]); // q = -q
void Quaternion_Print(const float q[4]);
void Quaternion_Normalize(const float q[4], float q_out[4]);
void Quaternion_Normalize(float q[4]);
void Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll, float q[4]);
void Quaternion_quat2eul_zyx(const float q[4], float yaw_pitch_roll[3]);
void Quaternion_RotateVector_Body2Inertial(const float q[4], const float v[3], float v_out[3]);
void Quaternion_RotateVector_Inertial2Body(const float q[4], const float v[3], float v_out[3]);
void Quaternion_AngleClamp(const float q[4], const float angleMax, float q_clamped[4]);
void Quaternion_GetAngularVelocity_Inertial(const float q[4], const float dq[4], float omega_inertial_out[3]);
void Quaternion_GetAngularVelocity_Body(const float q[4], const float dq[4], float omega_body_out[3]);
void Quaternion_GetDQ_FromInertial(const float q[4], const float omega_inertial[3], float dq[4]);
void Quaternion_GetDQ_FromBody(const float q[4], const float omega_body[3], float dq[4]);
void Quaternion_Integration_Body(const float q[4], const float omega_body[3], const float dt, float q_out[4]);
void Quaternion_Integration_Inertial(const float q[4], const float omega_inertial[3], const float dt, float q_out[4]);

void HeadingIndependentReferenceManual(const float q_ref[4], const float q[4], float q_ref_out[4]);
void HeadingIndependentQdot(const float dq[4], const float q[4], float q_dot_out[4]);
float HeadingFromQuaternion(const float q[4]);
void HeadingQuaternion(const float q[4], float q_heading[4]);

float invSqrt(float x);

#endif
