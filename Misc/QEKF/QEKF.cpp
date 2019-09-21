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
 
#include "QEKF.h"
#include "QEKF_coder.h"
#include "QEKF_initialize.h"
#include "MathLib.h"
#include <math.h>
#include <cmath>
#include <string.h> // for memcpy
 
#include "Quaternion.h"
#include "MathLib.h" // for matrix symmetrization
#include "arm_math.h"

QEKF::QEKF(Parameters& params, Timer * microsTimer) : _params(params), _microsTimer(microsTimer)
{
	Reset();
}

QEKF::QEKF(Parameters& params) : _params(params), _microsTimer(0)
{
	Reset();
}

QEKF::~QEKF()
{
}

void QEKF::Reset()
{
	QEKF_initialize(_params.estimator.QEKF_P_init_diagonal, X, P);

	if (_microsTimer)
		_prevTimerValue = _microsTimer->Get();
	else
		_prevTimerValue = 0;
}

/**
 * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 */
void QEKF::Reset(const float accelerometer[3])
{
	Reset();

	/* Reset quaternion state into certain angle based on accelerometer measurement */
	// Based on Freescale Application Note: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
	const float mu = 0.0001; // regularization factor
	float roll = atan2f(accelerometer[1], sqrtf(accelerometer[2]*accelerometer[2] + mu*accelerometer[0]*accelerometer[0]));
	float pitch = atan2f(-accelerometer[0], sqrtf(accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]));
	Quaternion_eul2quat_zyx(0, pitch, roll, &X[0]);
}

/**
 * @brief 	Reset attitude estimator to an angle based on an accelerometer measurement
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 */
void QEKF::Reset(const float accelerometer[3], const float heading)
{
	Reset();

	/* Reset quaternion state into certain angle based on accelerometer measurement */
	// Based on Freescale Application Note: https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
	const float mu = 0.0001; // regularization factor
	float roll = atan2f(accelerometer[1], sqrtf(accelerometer[2]*accelerometer[2] + mu*accelerometer[0]*accelerometer[0]));
	float pitch = atan2f(-accelerometer[0], sqrtf(accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]));
	Quaternion_eul2quat_zyx(heading, pitch, roll, &X[0]);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3])
{
	Step(accelerometer, gyroscope, _params.estimator.EstimateBias);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias)
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(accelerometer, gyroscope, EstimateBias, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer, gyroscope measurements, a heading input/estimate and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param	heading			   Input: heading angle in inertial frame [rad]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const float heading, const bool EstimateBias)
{
	float dt;

	if (!_microsTimer) return; // timer not defined
	dt = _microsTimer->GetDeltaTime(_prevTimerValue);
	_prevTimerValue = _microsTimer->Get();

	Step(accelerometer, gyroscope, heading, EstimateBias, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 * @param	dt             Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const bool EstimateBias, const float dt)
{
	if (_params.estimator.UseXsensIMU) // use MTI covariance
		Step(accelerometer, gyroscope, 0, false, _params.estimator.SensorDrivenQEKF, EstimateBias, false, _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mti, _params.estimator.cov_gyro_mti, _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading, _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled, _params.estimator.AccelerometerVibration_NormLPFtau, _params.estimator.AccelerometerVibration_CovarianceVaryFactor, _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
	else
		Step(accelerometer, gyroscope, 0, false, _params.estimator.SensorDrivenQEKF, EstimateBias, false, _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mpu, _params.estimator.cov_gyro_mpu, _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading, _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled, _params.estimator.AccelerometerVibration_NormLPFtau, _params.estimator.AccelerometerVibration_CovarianceVaryFactor, _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer, gyroscope measurements, a heading input/estimate and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param	heading			   Input: heading angle in inertial frame [rad]
 * @param   EstimateBias       Input: flag to control if gyroscope bias should be estimated
 * @param	dt                 Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const float heading, const bool EstimateBias, const float dt)
{
	if (_params.estimator.UseXsensIMU) // use MTI covariance
		Step(accelerometer, gyroscope, heading, true, _params.estimator.SensorDrivenQEKF, EstimateBias, EstimateBias, _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mti, _params.estimator.cov_gyro_mti, _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading, _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled, _params.estimator.AccelerometerVibration_NormLPFtau, _params.estimator.AccelerometerVibration_CovarianceVaryFactor, _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
	else
		Step(accelerometer, gyroscope, heading, true, _params.estimator.SensorDrivenQEKF, EstimateBias, EstimateBias, _params.estimator.CreateQdotFromQDifference, _params.estimator.cov_acc_mpu, _params.estimator.cov_gyro_mpu, _params.estimator.GyroscopeTrustFactor, _params.estimator.sigma2_omega, _params.estimator.sigma2_heading, _params.estimator.sigma2_bias, _params.estimator.AccelerometerVibration_DetectionEnabled, _params.estimator.AccelerometerVibration_NormLPFtau, _params.estimator.AccelerometerVibration_CovarianceVaryFactor, _params.estimator.AccelerometerVibration_MaxVaryFactor, _params.model.g, dt);
}

/**
 * @brief 	Estimate attitude quaternion given accelerometer and gyroscope measurements and passed time
 * @param	accelerometer[3]   Input: acceleration measurement in body frame [m/s^2]
 * @param	gyroscope[3]       Input: angular velocity measurement in body frame [rad/s]
 * @param	heading            Input: heading angle measurement [rad]
 * @param   UseHeadingForCorrection	 Input: flag to indicate if heading measurement is available
 * @param   SensorDriven       Input: flag to control if QEKF should run in sensor driven mode, disabling smoothing of angular velocity estimate
 * @param   EstimateBias       Input: flag to control if gyroscope x/y axis bias should be estimated
 * @param   EstimateYawBias    Input: flag to control if gyroscope z-axis bias should be estimated
 * @param   CreateQdotFromDifference  Input: flag to control if qdot estimate is generated by differentiating q estimate
 * @param   cov_acc            Input: accelerometer sensor covariance matrix
 * @param   cov_gyro           Input: gyroscope sensor covariance matrix
 * @param   GyroscopeTrustFactor	  Input: tuning factor for accelerometer-gyroscope trust ratio - increase value to trust gyroscope measurements more
 * @param   sigma2_omega       Input: smoothing parameter for angular velocity
 * @param   sigma2_heading     Input: variance on heading input
 * @param   sigma2_bias        Input: bias variance (random walk)
 * @param   AccelerometerVibrationDetectionEnabled  	Input: reduce trust in accelerometer measurements during periods with large vibrations
 * @param   AccelerometerVibrationNormLPFtau  			Input: low-pass filter for vibration detector
 * @param   AccelerometerVibrationCovarianceVaryFactor  Input: exponentially scaled factor to decrease gyroscope covariance (to decrease trust in accelerometer measurement) with during vibration periods
 * @param   AccelerometerCovarianceMaxVaryFactor  		Input: maximum ratio of decrease in gyroscope covariance
 * @param   g                  Input: gravity constant [m/s^2]
 * @param	dt                 Input: time passed since last estimate
 */
void QEKF::Step(const float accelerometer[3], const float gyroscope[3], const float heading, const bool UseHeadingForCorrection, const bool SensorDriven, const bool EstimateBias, const bool EstimateYawBias, const bool CreateQdotFromDifference, const float cov_acc[9], const float cov_gyro[9], const float GyroscopeTrustFactor, const float sigma2_omega, const float sigma2_heading, const float sigma2_bias, const bool AccelerometerVibrationDetectionEnabled, const float AccelerometerVibrationNormLPFtau, const float AccelerometerVibrationCovarianceVaryFactor, const float AccelerometerCovarianceMaxVaryFactor, const float g, const float dt)
{
	if (dt == 0) return; // no time has passed

	float X_prev[10];
	memcpy(X_prev, X, sizeof(X_prev));

	float P_prev[10*10];
	memcpy(P_prev, P, sizeof(P_prev));

	_QEKF(X_prev, P_prev,
		 gyroscope, accelerometer,
		 heading, UseHeadingForCorrection,
		 dt,
		 SensorDriven, // true == sensor driven Kalman filter
		 EstimateBias, EstimateYawBias,
		 true,  // true == normalize accelerometer
		 cov_gyro, cov_acc, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias,
		 AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau, AccelerometerVibrationCovarianceVaryFactor, AccelerometerCovarianceMaxVaryFactor,
		 g,
		 X, P);

	Math_SymmetrizeSquareMatrix(P, sizeof(X)/sizeof(float));

	if (CreateQdotFromDifference) {
	  X[4] = (X[0] - X_prev[0]) / dt; // dq[0]
	  X[5] = (X[1] - X_prev[1]) / dt; // dq[1]
	  X[6] = (X[2] - X_prev[2]) / dt; // dq[2]
	  X[7] = (X[3] - X_prev[3]) / dt; // dq[3]
	}
}

/**
 * @brief 	Get estimated attitude quaternion
 * @param	q[4]		Output: estimated attitude quaternion
 */
void QEKF::GetQuaternion(float q[4])
{
	q[0] = X[0];
	q[1] = X[1];
	q[2] = X[2];
	q[3] = X[3];
}

/**
 * @brief 	Get estimated attitude quaternion derivative
 * @param	dq[4]		Output: estimated attitude quaternion derivative
 */
void QEKF::GetQuaternionDerivative(float dq[4])
{
	/* Body angular velocity */
	/* dq = 1/2 * Phi(q) * [0;omega]; */
	float omega_q[4] = { 0, X[4], X[5], X[6] };
	Quaternion_Phi(&X[0], omega_q, dq); // Phi(q) * [0;omega]
	arm_scale_f32(dq, 0.5f, dq, 4);

	/*dq[0] = X[4];
	dq[1] = X[5];
	dq[2] = X[6];
	dq[3] = X[7];*/
}

/**
 * @brief 	Get estimated gyroscope bias
 * @param	bias[3]		Output: estimated gyroscope bias
 */
void QEKF::GetGyroBias(float bias[3])
{
	bias[0] = X[7];
	bias[1] = X[8];
	bias[2] = X[9];
}

/**
 * @brief 	Get covariance matrix of estimated quaternion
 * @param	Cov_q[4*4]		Output: quaternion estimate covariance
 */
void QEKF::GetQuaternionCovariance(float Cov_q[4*4])
{
	for (int m = 0; m < 4; m++) {
	  for (int n = 0; n < 4; n++) {
		Cov_q[4*m + n] = P[10*m + n];
	  }
	}
}

/**
 * @brief 	Get covariance matrix of estimated quaternion derivative
 * @param	Cov_dq[4*4]		Output: quaternion derivative estimate covariance
 */
void QEKF::GetQuaternionDerivativeCovariance(float Cov_dq[4*4])
{
	/*for (int m = 0; m < 4; m++) {
	  for (int n = 0; n < 4; n++) {
		Cov_dq[4*m + n] = P[10*m + n + (10*4 + 4)];
	  }
	}*/

	// OBS. The covariance of the quaternion derivative estimate is not stored in the estimator covariance, since it is not part of the state vector
	// Hence we need to transform the covariance of the angular velocity estimate into a covariance of the quaternion derivative estimate

	/* Cov_dq = (1/2 * Phi(q) * vec) * Cov_omega * (1/2 * Phi(q) * vec)' */
	/* Cov_dq = T(q) * Cov_omega * T(q)' */
	float Cov_omega[3*3]; arm_matrix_instance_f32 Cov_omega_; arm_mat_init_f32(&Cov_omega_, 3, 3, Cov_omega);
	GetAngularVelocityCovariance(Cov_omega);

	// Compute transformation matrix, T(q)
	float T_q[4*3]; arm_matrix_instance_f32 T_q_; arm_mat_init_f32(&T_q_, 4, 3, T_q);
	Quaternion_mat_PhiVec(&X[0], T_q);
	arm_scale_f32(T_q, 0.5f, T_q, 4*3);

	// Compute transpose, T(q)'
	float T_q_T[3*4]; arm_matrix_instance_f32 T_q_T_; arm_mat_init_f32(&T_q_T_, 3, 4, T_q_T);
	arm_mat_trans_f32(&T_q_, &T_q_T_);

	// Compute right part of transformation   -->   tmp = Cov_omega * T(q)'
	float tmp[3*4]; arm_matrix_instance_f32 tmp_; arm_mat_init_f32(&tmp_, 3, 4, tmp);
	arm_mat_mult_f32(&Cov_omega_, &T_q_T_, &tmp_);

	// Compute output   -->  Cov_dq = T(q) * tmp
	arm_matrix_instance_f32 Cov_dq_; arm_mat_init_f32(&Cov_dq_, 4, 4, Cov_dq);
	arm_mat_mult_f32(&T_q_, &tmp_, &Cov_dq_);

	Math_SymmetrizeSquareMatrix(Cov_dq, 4);
}

/**
 * @brief 	Get covariance matrix of estimated angular velocity
 * @param	Cov_omega[3*3]		Output: angular velocity estimate covariance
 */
void QEKF::GetAngularVelocityCovariance(float Cov_omega[3*3])
{
	for (int m = 0; m < 3; m++) {
	  for (int n = 0; n < 3; n++) {
		  Cov_omega[3*m + n] = P[10*m + n + (10*4 + 4)];
	  }
	}
}

/**
 * @brief 	Get covariance matrix of estimated gyroscope bias
 * @param	Cov_bias[3*3]		Output: gyroscope bias estimate covariance
 */
void QEKF::GetBiasCovariance(float Cov_bias[3*3])
{
	for (int m = 0; m < 3; m++) {
	  for (int n = 0; n < 3; n++) {
		  Cov_bias[3*m + n] = P[10*m + n + (10*7 + 7)];
	  }
	}
}

bool QEKF::UnitTest(void)
{
	const float g = 9.82f;

	const float cov_gyro_mpu[9] = {0.2529E-03,   -0.0064E-03,    0.1981E-03,
								  -0.0064E-03,    0.9379E-03,   -0.0038E-03,
								   0.1981E-03,   -0.0038E-03,    1.6828E-03};
	const float cov_acc_mpu[9] = {0.4273E-03,    0.0072E-03,    0.0096E-03,
								  0.0072E-03,    0.4333E-03,    0.0041E-03,
								  0.0096E-03,    0.0041E-03,    1.0326E-03};

	const bool SensorDriven = true;
	const float sigma2_bias = 1E-11;
	const float sigma2_omega = 1E-4;
	const float sigma2_heading = powf(deg2rad(1) / 3.0f, 2);
	const float GyroscopeTrustFactor = 1.0;
	const bool EstimateBias = true;
	const bool EstimateYawBias = true;
	const bool CreateQdotFromDifference = false;
	const bool UseHeadingForCorrection = true;
	const float VibrationDetectionAmount = 1.0;
	const bool VibrationDetectionEnabled = false;
	const float VibrationNormLPFtau = 0.5;
	const float VibrationCovarianceVaryFactor = 2.0;
	const float VibrationMaxVaryFactor = 10000;

	const float QEKF_P_init_diagonal[11] = {1E-5, 1E-5, 1E-5, 1E-7,   1E-7, 1E-7, 1E-7, 1E-7,   1E-5, 1E-5, 1E-5};

	QEKF_initialize(QEKF_P_init_diagonal, X, P); // reset

	const float Accelerometer[3] = {0.05, 0, 9.82};
	const float Gyroscope[3] = {1.0, 0.5, 0.09};
	const float heading = 0.4;

	const float dt = 1.0 / 200.0; // 400 Hz

	Step(Accelerometer, Gyroscope, heading, UseHeadingForCorrection, SensorDriven, EstimateBias, EstimateYawBias, CreateQdotFromDifference, cov_acc_mpu, cov_gyro_mpu, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias, VibrationDetectionEnabled, VibrationNormLPFtau, VibrationCovarianceVaryFactor, VibrationMaxVaryFactor, g, dt);
	Step(Accelerometer, Gyroscope, heading, UseHeadingForCorrection, SensorDriven, EstimateBias, EstimateYawBias, CreateQdotFromDifference, cov_acc_mpu, cov_gyro_mpu, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias, VibrationDetectionEnabled, VibrationNormLPFtau, VibrationCovarianceVaryFactor, VibrationMaxVaryFactor, g, dt);

	float q[4];
	GetQuaternion(q);

	float dq[4];
	GetQuaternionDerivative(dq);

	float Cov_q[4*4];
	GetQuaternionCovariance(Cov_q);

	float bias[3];
	bias[0] = X[8];
	bias[1] = X[9];
	bias[2] = X[10];

	const float q_expected[4] = {999.9893e-03, 0.8631e-03, 0.0246e-03, 4.5302e-03};
	if (!(Math_Round(q[0], 3) == Math_Round(q_expected[0], 3) &&
		  Math_Round(q[1], 7) == Math_Round(q_expected[1], 7) &&
		  Math_Round(q[2], 7) == Math_Round(q_expected[2], 7) &&
		  Math_Round(q[3], 7) == Math_Round(q_expected[3], 7)))
		return false;

	const float dq_expected[4] = {0.0180e-03, 0.2838339, 0.0631545, -0.0189311};
	if (!(Math_Round(dq[0], 7) == Math_Round(dq_expected[0], 7) &&
		  Math_Round(dq[1], 6) == Math_Round(dq_expected[1], 6) &&
		  Math_Round(dq[2], 6) == Math_Round(dq_expected[2], 6) &&
		  Math_Round(dq[3], 6) == Math_Round(dq_expected[3], 6)))
		return false;

	const float bias_expected[3] = {0.0401830, 0.0085002, -0.0032197};
	if (!(Math_Round(bias[0], 5) == Math_Round(bias_expected[0], 5) &&
		  Math_Round(bias[1], 5) == Math_Round(bias_expected[1], 5) &&
		  Math_Round(bias[2], 5) == Math_Round(bias_expected[2], 5)))
		return false;

	const float Cov_q_diag_expected[4] = {0.9262118e-05, 0.8441292e-05, 0.8419634e-05, 0.0098245e-05};
	if (!(Math_Round(Cov_q[0], 9) == Math_Round(Cov_q_diag_expected[0], 9) &&
		  Math_Round(Cov_q[1+4], 9) == Math_Round(Cov_q_diag_expected[1], 9) &&
		  Math_Round(Cov_q[2+8], 9) == Math_Round(Cov_q_diag_expected[2], 9) &&
		  Math_Round(Cov_q[3+12], 9) == Math_Round(Cov_q_diag_expected[3], 9)))
		return false;

	return true;
}
