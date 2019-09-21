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
 
#include "Quaternion.h"

#include <arm_math.h>
#include <math.h>
#include <stdlib.h>

// Quaternion class for computations with quaternions of the format
//   q = {q0, q1, q2, q3} = {s, v}
// Hence a 4-dimensional vector where the scalar value is first and the vector part are the 3 bottom elements
//   s = q0
//   v = {q1, q2, q3}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* result = q o p = Phi(q)*p */
void Quaternion_Phi(const float q[4], const float p[4], float result[4])
{
	/*Phi = @(q)[q(0) -q(1) -q(2) -q(3);     % for q o p = Phi(q) * p
				 q(1) q(0)  -q(3) q(2);
				 q(2) q(3)  q(0)  -q(1);
				 q(3) -q(2) q(1)  q(0)];
	*/
	result[0] = q[0]*p[0] - q[1]*p[1] - q[2]*p[2] - q[3]*p[3];
	result[1] = q[1]*p[0] + q[0]*p[1] - q[3]*p[2] + q[2]*p[3];
	result[2] = q[2]*p[0] + q[3]*p[1] + q[0]*p[2] - q[1]*p[3];
	result[3] = q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3];
}

/* result = V * q o p = V*Phi(q)*p */
void Quaternion_devecPhi(const float q[4], const float p[4], float result[3])
{
	// V (devec) removes the first row of the result
	/*Phi = @(q)[q(0) -q(1) -q(2) -q(3);     % for q o p = Phi(q) * p
				 q(1) q(0)  -q(3) q(2);
				 q(2) q(3)  q(0)  -q(1);
				 q(3) -q(2) q(1)  q(0)];
	*/
	result[0] = q[1]*p[0] + q[0]*p[1] - q[3]*p[2] + q[2]*p[3];
	result[1] = q[2]*p[0] + q[3]*p[1] + q[0]*p[2] - q[1]*p[3];
	result[2] = q[3]*p[0] - q[2]*p[1] + q[1]*p[2] + q[0]*p[3];
}

/* result = q* o p = Phi(q)^T*p */
void Quaternion_PhiT(const float q[4], const float p[4], float result[4])
{
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
				 -q(1) q(0)  q(3) -q(2);
				 -q(2) -q(3)  q(0)  q(1);
				 -q(3) q(2) -q(1)  q(0)];
	*/
	result[0] = q[0]*p[0] + q[1]*p[1] + q[2]*p[2] + q[3]*p[3];
	result[1] = -q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3];
	result[2] = -q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3];
	result[3] = -q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3];
}

/* result = V * q* o p = V*Phi(q)^T*p */
void Quaternion_devecPhiT(const float q[4], const float p[4], float result[3])
{
		// V (devec) removes the first row of the result
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
				 -q(1) q(0)  q(3) -q(2);
				 -q(2) -q(3)  q(0)  q(1);
				 -q(3) q(2) -q(1)  q(0)];
	*/
	result[0] = -q[1]*p[0] + q[0]*p[1] + q[3]*p[2] - q[2]*p[3];
	result[1] = -q[2]*p[0] - q[3]*p[1] + q[0]*p[2] + q[1]*p[3];
	result[2] = -q[3]*p[0] + q[2]*p[1] - q[1]*p[2] + q[0]*p[3];
}

/* mat = Phi(q) */
void Quaternion_mat_Phi(const float q[4], float mat[4*4])
{
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
				  -q(1) q(0)  q(3) -q(2);
				  -q(2) -q(3)  q(0)  q(1);
				  -q(3) q(2) -q(1)  q(0)];
	*/
	mat[0]  = q[0];   mat[1]  = -q[1];   mat[2]  = -q[2];   mat[3]  = -q[3];
	mat[4]  = q[1];  mat[5]  = q[0];   mat[6]  = -q[3];  mat[7]  = q[2];
	mat[8]  = q[2];  mat[9]  = q[3];   mat[10] = q[0];   mat[11] = -q[1];
	mat[12] = q[3];  mat[13] = -q[2];  mat[14] = q[1];   mat[15] = q[0];
}

/* mat = Phi(q) * Λ  */
void Quaternion_mat_PhiVec(const float q[4], float mat[4*3])
{
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
				  -q(1) q(0)  q(3) -q(2);
				  -q(2) -q(3)  q(0)  q(1);
				  -q(3) q(2) -q(1)  q(0)];
	*/
	mat[0]  = -q[1];   mat[1]  = -q[2];   mat[2]  = -q[3];
	mat[3]  = q[0];   mat[4]  = -q[3];  mat[5]  = q[2];
	mat[6]  = q[3];   mat[7] = q[0];   mat[8] = -q[1];
	mat[9] = -q[2];  mat[10] = q[1];   mat[11] = q[0];
}

/* mat = Phi(q)^T */
void Quaternion_mat_PhiT(const float q[4], float mat[4*4])
{
	/*Phi^T = @(q)[q(0) q(1) q(2) q(3);     % for q o p = Phi(q) * p
				  -q(1) q(0)  q(3) -q(2);
				  -q(2) -q(3)  q(0)  q(1);
				  -q(3) q(2) -q(1)  q(0)];
	*/
	mat[0]  = q[0];   mat[1]  = q[1];   mat[2]  = q[2];   mat[3]  = q[3];
	mat[4]  = -q[1];  mat[5]  = q[0];   mat[6]  = q[3];  mat[7]  = -q[2];
	mat[8]  = -q[2];  mat[9]  = -q[3];   mat[10] = q[0];   mat[11] = q[1];
	mat[12] = -q[3];  mat[13] = q[2];  mat[14] = -q[1];   mat[15] = q[0];
}

/* mat = devec*Phi(q)^T */
void Quaternion_mat_devecPhiT(const float q[4], float mat[3*4])
{
	/*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
					-p(1) p(0) -p(3) p(2);
					-p(2) p(3) p(0) -p(1);
					-p(3) -p(2) p(1) p(0)];
	*/
	mat[0]  = -q[1];  mat[1]  = q[0];   mat[2]  = q[3];  mat[3]  = -q[2];
	mat[4]  = -q[2];  mat[5]  = -q[3];   mat[6] = q[0];   mat[7] = q[1];
	mat[8] = -q[3];  mat[9] = q[2];  mat[10] = -q[1];   mat[11] = q[0];
}

/* result = q o p = Gamma(p)*q */
void Quaternion_Gamma(const float p[4], const float q[4], float result[4])
{
	/*Gamma = @(p)[p(0) -p(1) -p(2) -p(3);   % for q o p = Gamma(p) * q
				   p(1) p(0) p(3) -p(2);
				   p(2) -p(3) p(0) p(1);
				   p(3) p(2) -p(1) p(0)];
	*/
	result[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
	result[1] = p[1]*q[0] + p[0]*q[1] + p[3]*q[2] - p[2]*q[3];
	result[2] = p[2]*q[0] - p[3]*q[1] + p[0]*q[2] + p[1]*q[3];
	result[3] = p[3]*q[0] + p[2]*q[1] - p[1]*q[2] + p[0]*q[3];
}

/* result = q o p* = Gamma(p)^T*q */
void Quaternion_GammaT(const float p[4], const float q[4], float result[4])
{
	/*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
					-p(1) p(0) -p(3) p(2);
					-p(2) p(3) p(0) -p(1);
					-p(3) -p(2) p(1) p(0)];
	*/
	result[0] = p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
	result[1] = -p[1]*q[0] + p[0]*q[1] - p[3]*q[2] + p[2]*q[3];
	result[2] = -p[2]*q[0] + p[3]*q[1] + p[0]*q[2] - p[1]*q[3];
	result[3] = -p[3]*q[0] - p[2]*q[1] + p[1]*q[2] + p[0]*q[3];
}

/* result = V * q o p* = V*Gamma(p)^T*q */
void Quaternion_devecGammaT(const float p[4], const float q[4], float result[3])
{
	/*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
					-p(1) p(0) -p(3) p(2);
					-p(2) p(3) p(0) -p(1);
					-p(3) -p(2) p(1) p(0)];
	*/
	result[0] = -p[1]*q[0] + p[0]*q[1] - p[3]*q[2] + p[2]*q[3];
	result[1] = -p[2]*q[0] + p[3]*q[1] + p[0]*q[2] - p[1]*q[3];
	result[2] = -p[3]*q[0] - p[2]*q[1] + p[1]*q[2] + p[0]*q[3];
}

/* mat = Gamma(p) */
void Quaternion_mat_Gamma(const float p[4], float mat[4*4])
{
	/*Gamma = @(p)[p(0) -p(1) -p(2) -p(3);   % for q o p = Gamma(p) * q
				   p(1) p(0) p(3) -p(2);
				   p(2) -p(3) p(0) p(1);
				   p(3) p(2) -p(1) p(0)];
	*/
	mat[0]  = p[0];  mat[1]  = -p[1];  mat[2]  = -p[2];  mat[3]  = -p[3];
	mat[4]  = p[1];  mat[5]  = p[0];   mat[6]  = p[3];   mat[7]  = -p[2];
	mat[8]  = p[2];  mat[9]  = -p[3];  mat[10] = p[0];   mat[11] = p[1];
	mat[12] = p[3];  mat[13] = p[2];   mat[14] = -p[1];  mat[15] = p[0];
}

/* mat = Gamma(p)^T */
void Quaternion_mat_GammaT(const float p[4], float mat[4*4])
{
	/*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
					-p(1) p(0) -p(3) p(2);
					-p(2) p(3) p(0) -p(1);
					-p(3) -p(2) p(1) p(0)];
	*/
	mat[0]  = p[0];   mat[1]  = p[1];   mat[2]  = p[2];   mat[3]  = p[3];
	mat[4]  = -p[1];  mat[5]  = p[0];   mat[6]  = -p[3];  mat[7]  = p[2];
	mat[8]  = -p[2];  mat[9]  = p[3];   mat[10] = p[0];   mat[11] = -p[1];
	mat[12] = -p[3];  mat[13] = -p[2];  mat[14] = p[1];   mat[15] = p[0];
}

/* mat = devec*Gamma(p)^T */
void Quaternion_mat_devecGammaT(const float p[4], float mat[3*4])
{
	/*Gamma^T = @(p)[p(0) p(1) p(2) p(3);   % for q o p = Gamma(p) * q
					-p(1) p(0) -p(3) p(2);
					-p(2) p(3) p(0) -p(1);
					-p(3) -p(2) p(1) p(0)];
	*/
	mat[0]  = -p[1];  mat[1]  = p[0];   mat[2]  = -p[3];  mat[3]  = p[2];
	mat[4]  = -p[2];  mat[5]  = p[3];   mat[6]  = p[0];   mat[7]  = -p[1];
	mat[8]  = -p[3];  mat[9]  = -p[2];  mat[10] = p[1];   mat[11] = p[0];
}

/* result = q* */
void Quaternion_Conjugate(const float q[4], float result[4])
{
	result[0] = q[0];
	result[1] = -q[1];
	result[2] = -q[2];
	result[3] = -q[3];
}

/* q = q* */
void Quaternion_Conjugate(float q[4])
{
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

/* q = -q */
void Quaternion_Negate(float q[4])
{
	q[0] = -q[0];
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

void Quaternion_Print(const float q[4])
{
	/*Serial.print("  ");
	Serial.printf("%7.4f\n", q[0]);
	Serial.print("  ");
	Serial.printf("%7.4f\n", q[1]);
	Serial.print("  ");
	Serial.printf("%7.4f\n", q[2]);
	Serial.print("  ");
	Serial.printf("%7.4f\n", q[3]);*/
}

void Quaternion_Normalize(const float q[4], float q_out[4])
{
	float normFactor = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q_out[0] = normFactor * q[0];
	q_out[1] = normFactor * q[1];
	q_out[2] = normFactor * q[2];
	q_out[3] = normFactor * q[3];
}

void Quaternion_Normalize(float q[4])
{
	float normFactor = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
	q[0] *= normFactor;
	q[1] *= normFactor;
	q[2] *= normFactor;
	q[3] *= normFactor;
}

void Quaternion_eul2quat_zyx(const float yaw, const float pitch, const float roll, float q[4])
{
	const float cx = cosf(roll/2);
	const float cy = cosf(pitch/2);
	const float cz = cosf(yaw/2);
	const float sx = sinf(roll/2);
	const float sy = sinf(pitch/2);
	const float sz = sinf(yaw/2);

	q[0] = cz*cy*cx+sz*sy*sx;
	q[1] = cz*cy*sx-sz*sy*cx;
	q[2] = cz*sy*cx+sz*cy*sx;
	q[3] = sz*cy*cx-cz*sy*sx;
}

void Quaternion_quat2eul_zyx(const float q[4], float yaw_pitch_roll[3])
{
	// Normalize quaternion
	float q_normalized[4];
	Quaternion_Normalize(q, q_normalized);

	float qw = q_normalized[0];
	float qx = q_normalized[1];
	float qy = q_normalized[2];
	float qz = q_normalized[3];

  float aSinInput = -2*(qx*qz-qw*qy);
	aSinInput = fmaxf(fminf(aSinInput, 1.f), -1.f);

  yaw_pitch_roll[0] = atan2( 2*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qy - qz*qz ); // yaw
  yaw_pitch_roll[1] = asin( aSinInput ); // pitch
	yaw_pitch_roll[2] = atan2( 2*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz ); // roll
}

/* Rotate vector within body frame into inertial frame */
void Quaternion_RotateVector_Body2Inertial(const float q[4], const float v[3], float v_out[3])
{
	// v_out = devec * q o dev*v o q* = devec * Phi(q) * Gamma(q)^T * vec*v
	v_out[0] = (q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])*v[0] + (2*q[1]*q[2] - 2*q[0]*q[3])					 *v[1] + (2*q[0]*q[2] + 2*q[1]*q[3])					*v[2];
	v_out[1] = (2*q[0]*q[3] + 2*q[1]*q[2])					  *v[0] + (q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3])*v[1] + (2*q[2]*q[3] - 2*q[0]*q[1])					*v[2];
	v_out[2] = (2*q[1]*q[3] - 2*q[0]*q[2])					  *v[0] + (2*q[0]*q[1] + 2*q[2]*q[3])					 *v[1] + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*v[2];
}

/* Rotate vector within inertial frame into body frame */
void Quaternion_RotateVector_Inertial2Body(const float q[4], const float v[3], float v_out[3])
{
	// v_out = devec * q* o dev*v o q = devec * Phi(q)^T * Gamma(q) * vec*v
	v_out[0] = (q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])*v[0] + (2*q[0]*q[3] + 2*q[1]*q[2])					 *v[1] + (2*q[1]*q[3] - 2*q[0]*q[2])					*v[2];
	v_out[1] = (2*q[1]*q[2] - 2*q[0]*q[3])					  *v[0] + (q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3])*v[1] + (2*q[0]*q[1] + 2*q[2]*q[3])					*v[2];
	v_out[2] = (2*q[0]*q[2] + 2*q[1]*q[3])					  *v[0] + (2*q[2]*q[3] - 2*q[0]*q[1])					 *v[1] + (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*v[2];
}

void Quaternion_AngleClamp(const float q[4], const float angleMax, float q_clamped[4])
{
	// Bound/clamp quaternion rotation amount by angle
	float cosAngle, sinAngle, currentAngle, clampedAngle; // tan = sin/cos
	cosAngle = q[0];
	sinAngle = sqrtf(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); // norm of rotation axis

	if (sinAngle == 0) {
		// Return unit quaternion if there is no tilt, hence norm is zero
		q_clamped[0] = 1;
		q_clamped[1] = 0;
		q_clamped[2] = 0;
		q_clamped[3] = 0;
		return;
	}

	currentAngle = atan2f(sinAngle, cosAngle) * 2;
	clampedAngle = fminf(fmaxf(currentAngle, -angleMax), angleMax);

	// Form clamped quaternion
	q_clamped[0] = cosf(clampedAngle / 2);
	q_clamped[1] = (q[1] / sinAngle) * sinf(clampedAngle / 2);
	q_clamped[2] = (q[2] / sinAngle) * sinf(clampedAngle / 2);
	q_clamped[3] = (q[3] / sinAngle) * sinf(clampedAngle / 2);
}

void Quaternion_GetAngularVelocity_Inertial(const float q[4], const float dq[4], float omega_inertial_out[3])
{
	// dq = 1/2 * q_omega_inertial o q
	// q_omega_inertial = 2*dq o inv(q)
	// We therefore have:
	// omega_body = devec * 2 * Gamma(q)' * dq
	Quaternion_devecGammaT(q, dq, omega_inertial_out);
	arm_scale_f32(omega_inertial_out, 2.0f, omega_inertial_out, 3);
}

void Quaternion_GetAngularVelocity_Body(const float q[4], const float dq[4], float omega_body_out[3])
{
	// dq = 1/2 * q o q_omega_body
	// q_omega_body = 2*inv(q) o dq
	// We therefore have:
	// omega_body = devec * 2 * Phi(q)' * dq
	Quaternion_devecPhiT(q, dq, omega_body_out);
	arm_scale_f32(omega_body_out, 2.0f, omega_body_out, 3);
}

void Quaternion_GetDQ_FromInertial(const float q[4], const float omega_inertial[3], float dq[4])
{
	// dq = 1/2 * q_omega_body o q
	// dq = 1/2 * Gamma(q) * [0;q_omega_inertial];
	float omega_q[4] = {0, omega_inertial[0], omega_inertial[1], omega_inertial[2]};
	Quaternion_Phi(q, omega_q, dq); // Gamma(q) * [0;omega_inertial]
	arm_scale_f32(dq, 0.5f, dq, 4);
}

void Quaternion_GetDQ_FromBody(const float q[4], const float omega_body[3], float dq[4])
{
	// dq = 1/2 * q o q_omega_body
	// dq = 1/2 * Phi(q) * [0;q_omega_body];
	float omega_q[4] = {0, omega_body[0], omega_body[1], omega_body[2]};
	Quaternion_Phi(q, omega_q, dq); // Phi(q) * [0;omega_body]
	arm_scale_f32(dq, 0.5f, dq, 4);
}

void Quaternion_Integration_Body(const float q[4], const float omega_body[3], const float dt, float q_out[4])
{
	/* Quaternion Exponential method
	 * q_out = q o exp(1/2*dt*q_omeg)
	 * q_omeg = [0,omeg_x,omeg_y,omeg_z]
	 */
	float omega_norm = sqrtf(omega_body[0]*omega_body[0] + omega_body[1]*omega_body[1] + omega_body[2]*omega_body[2]);
	float q_exp[4];

	if (omega_norm > 0) {
		float sinOmeg = sinf(0.5f * dt * omega_norm);
		q_exp[0] = cosf(0.5f * dt * omega_norm); // scalar part
		q_exp[1] = sinOmeg * omega_body[0] / omega_norm;
		q_exp[2] = sinOmeg * omega_body[1] / omega_norm;
		q_exp[3] = sinOmeg * omega_body[2] / omega_norm;
	} else {
		// unit quaternion since the angular velocity is zero (no movement)
		q_exp[0] = 1.0f;
		q_exp[1] = 0.0f;
		q_exp[2] = 0.0f;
		q_exp[3] = 0.0f;
	}

	// q_out = Phi(q) o q_exp
	Quaternion_Phi(q, q_exp, q_out);
}

void Quaternion_Integration_Inertial(const float q[4], const float omega_inertial[3], const float dt, float q_out[4])
{
	/* Quaternion Exponential method
	 * q_out = exp(1/2*dt*q_omeg) o q
	 * q_omeg = [0,omeg_x,omeg_y,omeg_z]
	 */
	float omega_norm = sqrtf(omega_inertial[0]*omega_inertial[0] + omega_inertial[1]*omega_inertial[1] + omega_inertial[2]*omega_inertial[2]);
	float q_exp[4];

	if (omega_norm > 0) {
		float sinOmeg = sinf(0.5f * dt * omega_norm);
		q_exp[0] = cosf(0.5f * dt * omega_norm); // scalar part
		q_exp[1] = sinOmeg * omega_inertial[0] / omega_norm;
		q_exp[2] = sinOmeg * omega_inertial[1] / omega_norm;
		q_exp[3] = sinOmeg * omega_inertial[2] / omega_norm;
	} else {
		// unit quaternion since the angular velocity is zero (no movement)
		q_exp[0] = 1.0f;
		q_exp[1] = 0.0f;
		q_exp[2] = 0.0f;
		q_exp[3] = 0.0f;
	}

	// q_out = Gamma(q) o q_exp
	Quaternion_Gamma(q, q_exp, q_out);
}

void HeadingIndependentReferenceManual(const float q_ref[4], const float q[4], float q_ref_out[4])
{
  // OBS!!! MAYBE THIS CODE SHOULD BE DONE DIFFERENTLY BY EXTRACTING HEADING QUATERNION BY EXTRACTING AND USING THE X-AXIS VECTOR DIRECTION (see "HeadingQuaternion2.m")

  /* Derive tilt and heading from combined quaternion */
  // Z unit vector of Body in Inertial frame
  // I_e_Z = devec * Phi(q) * Gamma(q)' * [0;0;0;1];
  // Extract direction in which this Z vector is pointing (project down to XY-plane)
  // direction = I_e_Z(1:2);  % direction = [eye(2), zeros(2,1)] * I_e_Z;
  float direction[2];
  direction[0] = 2*q[0]*q[2] + 2*q[1]*q[3];
  direction[1] = 2*q[2]*q[3] - 2*q[0]*q[1];

  // Tilt amount corresponds to sin^-1 of the length of this vector
  float normDirection = sqrtf(direction[0]*direction[0] + direction[1]*direction[1]);
  float tilt = asinf(normDirection);

  // normalize direction vector before forming tilt quaternion
  if (normDirection != 0) {
	direction[0] = direction[0] / normDirection;
	direction[1] = direction[1] / normDirection;
  } else {
	direction[0] = 0;
	direction[1] = 0;
  }

  // Tilt quaternion describes the current (heading independent) tilt of the robot
  float q_tilt[4];
  q_tilt[0] = cosf(tilt/2);
  q_tilt[1] = sinf(tilt/2) * -direction[1];
  q_tilt[2] = sinf(tilt/2) * direction[0];
  q_tilt[3] = 0;

  // Remove the tilt from the current quaternion to extract the heading part of the quaternion
  float q_heading[4];
  Quaternion_PhiT(q_tilt, q, q_heading); // q_heading = Phi(q_tilt)' * q;

  /* Derive tilt from quaternion reference */
  // Z unit vector of Body in Inertial frame
  // I_e_Z = devec * Phi(q_ref) * Gamma(q_ref)' * [0;0;0;1];
  // Extract direction in which this Z vector is pointing (project down to XY-plane)
  // direction = I_e_Z(1:2); //direction = [eye(2), zeros(2,1)] * I_e_Z;
  float direction_ref[2];
  direction_ref[0] = 2*q_ref[0]*q_ref[2] + 2*q_ref[1]*q_ref[3];
  direction_ref[1] = 2*q_ref[2]*q_ref[3] - 2*q_ref[0]*q_ref[1];
  // Tilt amount corresponds to sin^-1 of the length of this vector
  float normDirectionRef = sqrtf(direction_ref[0]*direction_ref[0] + direction_ref[1]*direction_ref[1]);
  float tilt_ref = asinf(normDirectionRef);

  // normalize direction vector before forming tilt quaternion
  if (normDirectionRef != 0) {
	direction_ref[0] = direction_ref[0] / normDirectionRef;
	direction_ref[1] = direction_ref[1] / normDirectionRef;
  } else {
	direction_ref[0] = 0;
	direction_ref[1] = 0;
  }

  // Tilt quaternion describes the current (heading independent) tilt of the robot
  float q_tilt_ref[4];
  q_tilt_ref[0] = cosf(tilt_ref/2);
  q_tilt_ref[1] = sinf(tilt_ref/2) * -direction_ref[1];
  q_tilt_ref[2] = sinf(tilt_ref/2) * direction_ref[0];
  q_tilt_ref[3] = 0;

  // Remove the tilt from the current quaternion to extract the heading part of the quaternion
  float q_heading_ref[4];
  Quaternion_PhiT(q_tilt_ref, q_ref, q_heading_ref); // q_heading_ref = Phi(q_tilt_ref)' * q_ref;

  /* Calculate reference quaternion by multiplying with the desired reference */
  // We multiply on the right side since the heading quaternion is given
  // around the Z-axis in the body frame - thus in the frame of the desired tilt angle
  //Quaternion_Phi(q_tilt_ref, q_heading, q_ref_out); // q_ref_out = Phi(q_tilt_ref) * q_heading;     % if desired tilt reference is given in inertial heading frame
  Quaternion_Phi(q_heading, q_tilt_ref, q_ref_out); // q_ref_out = Phi(q_heading) * q_tilt_ref;      % if desired tilt reference is given in body heading frame
}

void HeadingIndependentQdot(const float dq[4], const float q[4], float q_dot_out[4])
{
  /* omega = 2*Phi(q)'*dq    % body
	 removeYaw = [eye(3), zeros(3,1); zeros(1,3), 0];
	 dq_withoutYaw = SimplifyWithQuatConstraint(1/2 * Phi(q) * removeYaw * 2*Phi(q)' * dq, q)
  */
  /*q_dot_out[0] = dq[0]*q[0]*q[0] + dq[3]*q[3]*q[0] + dq[0]*q[1]*q[1] - dq[2]*q[3]*q[1] + dq[0]*q[2]*q[2] + dq[1]*q[3]*q[2];
  q_dot_out[1] = dq[1]*q[0]*q[0] - dq[3]*q[2]*q[0] + dq[1]*q[1]*q[1] + dq[2]*q[2]*q[1] + dq[1]*q[3]*q[3] + dq[0]*q[2]*q[3];
  q_dot_out[2] = dq[2]*q[0]*q[0] + dq[3]*q[1]*q[0] + dq[2]*q[2]*q[2] + dq[1]*q[1]*q[2] + dq[2]*q[3]*q[3] - dq[0]*q[1]*q[3];
  q_dot_out[3] = dq[3]*q[1]*q[1] + dq[2]*q[0]*q[1] + dq[3]*q[2]*q[2] - dq[1]*q[0]*q[2] + dq[3]*q[3]*q[3] + dq[0]*q[0]*q[3];*/

  /* No body angular velocity */
  /* omega = devec*2*Phi(q)'*dq    % body
	 omega_noYaw = [omega(1:2); 0]
	 dq_noYaw = 1/2 * Phi(q) * vec*omega_noYaw
  */
  /*q_dot_out[0] = dq[0]*q[1]*q[1] + dq[0]*q[2]*q[2] - dq[1]*q[0]*q[1] - dq[2]*q[0]*q[2] + dq[1]*q[2]*q[3] - dq[2]*q[1]*q[3];
  q_dot_out[1] = dq[1]*q[0]*q[0] + dq[1]*q[3]*q[3] - dq[0]*q[0]*q[1] + dq[0]*q[2]*q[3] - dq[3]*q[0]*q[2] - dq[3]*q[1]*q[3];
  q_dot_out[2] = dq[2]*q[0]*q[0] + dq[2]*q[3]*q[3] - dq[0]*q[0]*q[2] - dq[0]*q[1]*q[3] + dq[3]*q[0]*q[1] - dq[3]*q[2]*q[3];
  q_dot_out[3] = dq[3]*q[1]*q[1] + dq[3]*q[2]*q[2] - dq[1]*q[0]*q[2] + dq[2]*q[0]*q[1] - dq[1]*q[1]*q[3] - dq[2]*q[2]*q[3];*/
  // The second method is only slightly different from the first, in the sense that it forces the q0 component of omega to be 0 (sort of a rectification)

  /* No inertial yaw angular velocity
	 omega = 2*devec*Gamma(q)'*dq   % inertial
	 omega(3) = 0;
	 dq_withoutYaw = SimplifyWithQuatConstraint(1/2 * Gamma(q) * vec * omega, q)
  */
  q_dot_out[0] = dq[0]*q[1]*q[1] + dq[0]*q[2]*q[2] - dq[1]*q[0]*q[1] - dq[2]*q[0]*q[2] - dq[1]*q[2]*q[3] + dq[2]*q[1]*q[3];
  q_dot_out[1] = dq[1]*q[0]*q[0] + dq[1]*q[3]*q[3] - dq[0]*q[0]*q[1] - dq[0]*q[2]*q[3] + dq[3]*q[0]*q[2] - dq[3]*q[1]*q[3];
  q_dot_out[2] = dq[2]*q[0]*q[0] + dq[2]*q[3]*q[3] - dq[0]*q[0]*q[2] + dq[0]*q[1]*q[3] - dq[3]*q[0]*q[1] - dq[3]*q[2]*q[3];
  q_dot_out[3] = dq[3]*q[1]*q[1] + dq[3]*q[2]*q[2] + dq[1]*q[0]*q[2] - dq[2]*q[0]*q[1] - dq[1]*q[1]*q[3] - dq[2]*q[2]*q[3];
}

float HeadingFromQuaternion(const float q[4])
{
	// Extract body x-axis direction in inertial frame
	// I_e_x = devec * Phi(q) * Gamma(q)' * [0,1,0,0]';
	const float B_e_x[3] = { 1, 0, 0 };
	float I_e_x[3];
	Quaternion_RotateVector_Body2Inertial(q, B_e_x, I_e_x);

	// Now compute heading by taking the x-y components (projecting the x-axis vector down to the xy-plane)
	// heading = atan2(x_vec(2), x_vec(1));
	float heading = atan2(I_e_x[1], I_e_x[0]);

	return heading;
}

void HeadingQuaternion(const float q[4], float q_heading[4])
{
	float heading = HeadingFromQuaternion(q);

	q_heading[0] = cos(heading / 2.0f);
	q_heading[1] = 0;
	q_heading[2] = 0;
	q_heading[3] = sin(heading / 2.0f);

	if (q[0] < 0) // scalar is negative, make sure this is the case in the output quaternion too
		Quaternion_Negate(q_heading);
}
