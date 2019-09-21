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
 
#include "IMU.h"
#include "Debug.h"
#include "MathLib.h"
#include <arm_math.h>
#include "svd.h"

// Class for sensor abstraction, sampling and calibration
// Should eg. configure MPU-9250 interrupt

#if 0
void MPU9250_Sampling(void const * argument)
{
	SPI * spi = new SPI(SPI::PORT_SPI3, MPU9250_SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250<SPI,MPU9250_SPI> * imu = new MPU9250<SPI,MPU9250_SPI>(spi);

	imu->Configure(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
	imu->setFilt(DLPF_BANDWIDTH_250HZ, DLPF_BANDWIDTH_184HZ, 8);
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	while (1) {
		imu->WaitForNewData();
		imu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	}
}
#endif

void IMU::CorrectMeasurement(Measurement_t& measurement, bool correctAccelerometerBias, bool correctAccelerometerScale, bool correctGyroBias, bool correctAlignment)
{
	if (correctAccelerometerBias && calibration_.acc_bias_valid) {
		measurement.Accelerometer[0] -= calibration_.acc_bias[0];
		measurement.Accelerometer[1] -= calibration_.acc_bias[1];
		measurement.Accelerometer[2] -= calibration_.acc_bias[2];
	}
	if (correctAccelerometerScale && calibration_.acc_scale_valid) {
		measurement.Accelerometer[0] *= calibration_.acc_scale[0];
		measurement.Accelerometer[1] *= calibration_.acc_scale[1];
		measurement.Accelerometer[2] *= calibration_.acc_scale[2];
	}
	if (correctGyroBias && calibration_.gyro_bias_valid) {
		measurement.Gyroscope[0] -= calibration_.gyro_bias[0];
		measurement.Gyroscope[1] -= calibration_.gyro_bias[1];
		measurement.Gyroscope[2] -= calibration_.gyro_bias[2];
	}
	if (correctAlignment && calibration_.imu_calibration_matrix_valid) {
		rotateImuMeasurement(measurement.Gyroscope[0],
							 measurement.Gyroscope[1],
							 measurement.Gyroscope[2],
							 measurement.Accelerometer[0],
							 measurement.Accelerometer[1],
							 measurement.Accelerometer[2],
							 calibration_.imu_calibration_matrix);
	}
}

void IMU::AttachEEPROM(EEPROM * eeprom)
{
	if (!eeprom) return;
	eeprom_ = eeprom;
	//eeprom_->EnableSection(eeprom_->sections.imu_calibration, sizeof(calibration_)); // enable IMU calibration section in EEPROM --> moved to MainTask since it has to be enabled before calling eeprom->Initialize()
	LoadCalibrationFromEEPROM();
}

void IMU::LoadCalibrationFromEEPROM(void)
{
	if (!eeprom_) return;
	eeprom_->ReadData(eeprom_->sections.imu_calibration, (uint8_t *)&calibration_, sizeof(calibration_));
	ValidateCalibration();
}

void IMU::ValidateCalibration(void)
{
	ValidateCalibrationMatrix();
	// Validate the other parts of the calibration
}

void IMU::ValidateCalibrationMatrix(void)
{
	const unsigned int ValidationPrecision = 2; // decimal places checked

	if (!calibration_.imu_calibration_matrix_valid) return; // the calibration flag is not even set, so nothing to validate

	calibration_.imu_calibration_matrix_valid = false; // set the flag to false until we succeed with all checks

	// We perform a crude validation by ensuring that the calibration matrix is orthogonal : R*R' = I and R'*R = I
	float * R = calibration_.imu_calibration_matrix; arm_matrix_instance_f32 R_; arm_mat_init_f32(&R_, 3, 3, R);
	float R_T[3*3]; arm_matrix_instance_f32 R_T_; arm_mat_init_f32(&R_T_, 3, 3, R_T);
	arm_mat_trans_f32(&R_, &R_T_); // transpose the matrix

	float I1[3*3]; arm_matrix_instance_f32 I1_; arm_mat_init_f32(&I1_, 3, 3, I1);
	float I2[3*3]; arm_matrix_instance_f32 I2_; arm_mat_init_f32(&I2_, 3, 3, I2);
	arm_mat_mult_f32(&R_, &R_T_, &I1_); // R*R'
	arm_mat_mult_f32(&R_T_, &R_, &I2_); // R'*R

	// Verify that the diagonal elements are 1 (or very close to 1) and that any off-diagonal elements are close to 0
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == j) { // diagonal element, should be close to 1
				if (Math_Round(I1[3*i+j], ValidationPrecision) != 1.0 ||
					Math_Round(I2[3*i+j], ValidationPrecision) != 1.0)
					return;
			} else { // off-diagonal element
				if (Math_Round(I1[3*i+j], ValidationPrecision) != 0.0 ||
					Math_Round(I2[3*i+j], ValidationPrecision) != 0.0)
					return;
			}
		}
	}

	// Matrix has been validated as being a proper rotation matrix (at least it is orthogonal)
	calibration_.imu_calibration_matrix_valid = true;
}

bool IMU::isCalibrated()
{
	return (isAccelerometerCalibrated() && isGyroscopeCalibrated() && isAlignmentCalibrated());
}

bool IMU::isAccelerometerCalibrated()
{
	return (calibration_.acc_bias_valid && calibration_.acc_scale_valid);
}

bool IMU::isGyroscopeCalibrated()
{
	return calibration_.gyro_bias_valid;
}

bool IMU::isAlignmentCalibrated()
{
	return calibration_.imu_calibration_matrix_valid;
}

void IMU::SetCalibration(const float accelerometer_bias[3], const float accelerometer_scale[3], const float gyroscope_bias[3], const float calibration_matrix[3*3], bool storeInEEPROM)
{
	memcpy(calibration_.acc_bias, accelerometer_bias, sizeof(calibration_.acc_bias));
	calibration_.acc_bias_valid = true;

	memcpy(calibration_.acc_scale, accelerometer_scale, sizeof(calibration_.acc_scale));
	calibration_.acc_scale_valid = true;

	memcpy(calibration_.gyro_bias, gyroscope_bias, sizeof(calibration_.gyro_bias));
	calibration_.gyro_bias_valid = true;

	memcpy(calibration_.imu_calibration_matrix, calibration_matrix, sizeof(calibration_.imu_calibration_matrix));
	calibration_.imu_calibration_matrix_valid = true;

	ValidateCalibration();

	if (storeInEEPROM && eeprom_) { // if EEPROM is configured, store new calibration in EEPROM
		eeprom_->WriteData(eeprom_->sections.imu_calibration, (uint8_t *)&calibration_, sizeof(calibration_));
	}
}

void IMU::Calibrate(bool storeInEEPROM)
{
	Measurement_t meas;
	Debug::print("Calibrating IMU\n");
	osDelay(1000);

	float avg_acc[3] = {0.0f, 0.0f, 0.0f};
	int num_samples = 10;
	Debug::print("Getting "); Debug::printf("%d", num_samples); Debug::print(" accelerometer samples:\n");
	for (int i = 0; i < num_samples; ++i) {
		Get(meas);
		arm_add_f32(meas.Accelerometer, avg_acc, avg_acc, 3);
		Debug::printf("%f", meas.Accelerometer[0]); Debug::print("\t");
		Debug::printf("%f", meas.Accelerometer[1]); Debug::print("\t");
		Debug::printf("%f\n", meas.Accelerometer[2]);
		osDelay(100);
	}
	arm_scale_f32(avg_acc, 1.f/num_samples, avg_acc, 3);

	float avg_gyro[3] = {0.0f, 0.0f, 0.0f};
	num_samples = 3000;
	Debug::print("Getting "); Debug::printf("%d", num_samples); Debug::print(" gyro samples:\n");
	for (int i = 0; i < num_samples; ++i) {
		Get(meas);
		arm_add_f32(meas.Gyroscope, avg_gyro, avg_gyro, 3);
		Debug::printf("%f", meas.Gyroscope[0]); Debug::print("\t");
		Debug::printf("%f", meas.Gyroscope[1]); Debug::print("\t");
		Debug::printf("%f\n", meas.Gyroscope[2]);
		osDelay(1);
	}
	arm_scale_f32(avg_gyro, 1.f/num_samples, calibration_.gyro_bias, 3);

	calibrateImu(reference_acc_vector_, avg_acc, calibration_.imu_calibration_matrix);
	calibration_.imu_calibration_matrix_valid = true;
	calibration_.gyro_bias_valid = true;

	Debug::print("Resulting calibration matrix:\n");
	Debug::printf("%f", calibration_.imu_calibration_matrix[0]); Debug::print("\t");
	Debug::printf("%f", calibration_.imu_calibration_matrix[1]); Debug::print("\t");
	Debug::printf("%f\n", calibration_.imu_calibration_matrix[2]);
	Debug::printf("%f", calibration_.imu_calibration_matrix[3]); Debug::print("\t");
	Debug::printf("%f", calibration_.imu_calibration_matrix[4]); Debug::print("\t");
	Debug::printf("%f\n", calibration_.imu_calibration_matrix[5]);
	Debug::printf("%f", calibration_.imu_calibration_matrix[6]); Debug::print("\t");
	Debug::printf("%f", calibration_.imu_calibration_matrix[7]); Debug::print("\t");
	Debug::printf("%f\n", calibration_.imu_calibration_matrix[8]);

	/* We have now calibrated, but we need to verify that the calibration is valid */
	ValidateCalibration();
	if (!isCalibrated()) {
		Debug::print("Calibration failed: Could not validate calibration\n\n");
		osDelay(5000);
		return;
	}

	if (storeInEEPROM && eeprom_) { // if EEPROM is configured, store new calibration in EEPROM
		eeprom_->WriteData(eeprom_->sections.imu_calibration, (uint8_t *)&calibration_, sizeof(calibration_));
	}

	/*Debug::print("Testing:\n");
	Get(meas);
	Debug::print("Unadjusted:\n");
	Debug::print("Acc:\t"); Debug::printf("%f", meas.Accelerometer[0]); Debug::print("\t");
	Debug::printf("%f", meas.Accelerometer[1]); Debug::print("\t"); Debug::printf("%f", meas.Accelerometer[2]); Debug::print("\n");
	Debug::print("Gyro:\t"); Debug::printf("%f", meas.Gyroscope[0]); Debug::print("\t");
	Debug::printf("%f", meas.Gyroscope[1]); Debug::print("\t"); Debug::printf("%f\n", meas.Gyroscope[2]);
	adjustImuMeasurement(meas.Gyroscope[0], meas.Gyroscope[1], meas.Gyroscope[2], meas.Accelerometer[0], meas.Accelerometer[1], meas.Accelerometer[2], calibration_.imu_calibration_matrix, calibration_.gyro_bias);
	Debug::print("Adjusted:\n");
	Debug::print("Acc:\t"); Debug::printf("%f", meas.Accelerometer[0]); Debug::print("\t");
	Debug::printf("%f", meas.Accelerometer[1]); Debug::print("\t"); Debug::printf("%f", meas.Accelerometer[2]); Debug::print("\n");
	Debug::print("Gyro:\t"); Debug::printf("%f", meas.Gyroscope[0]); Debug::print("\t");
	Debug::printf("%f", meas.Gyroscope[1]); Debug::print("\t"); Debug::printf("%f\n", meas.Gyroscope[2]);
	Debug::print("\n");*/
}

void IMU::CalibrateAccelerometer(bool storeInEEPROM)
{
	Debug::print("Calibrating Accelerometer - tilt accelerometer slowly such that measurements are taken at all sides\n");
	osDelay(1000);

	float acc_min[3];
	float acc_max[3];

	uint32_t steadyCount = 0;
	Measurement_t meas;
	TickType_t startTime = xTaskGetTickCount();
	TickType_t endTime = startTime + configTICK_RATE_HZ * ACCELEROMETER_CALIBRATION_TIME;
	while (xTaskGetTickCount() < endTime)
	{
		Get(meas);

		if (fabsf(vector_length(meas.Accelerometer) - ACCELEROMETER_GRAVITY_NORM) < 0.8/* && vector_length(meas.Gyroscope) < 0.5*/) {
			steadyCount++;
			if (steadyCount > ACCELEROMETER_CALIBRATION_STEADY_TIME*ACCELEROMETER_CALIBRATION_SAMPLE_RATE) {
				if (meas.Accelerometer[0] < acc_min[0])
					acc_min[0] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_min[0] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[0];

				if (meas.Accelerometer[1] < acc_min[1])
					acc_min[1] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_min[1] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[1];

				if (meas.Accelerometer[2] < acc_min[2])
					acc_min[2] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_min[2] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[2];

				if (meas.Accelerometer[0] > acc_max[0])
					acc_max[0] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_max[0] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[0];

				if (meas.Accelerometer[1] > acc_max[1])
					acc_max[1] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_max[1] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[1];

				if (meas.Accelerometer[2] > acc_max[2])
					acc_max[2] = ACCELEROMETER_CALIBRATION_LPF_COEFF_A * acc_max[2] + ACCELEROMETER_CALIBRATION_LPF_COEFF_B * meas.Accelerometer[2];
			}
		} else {
			steadyCount = 0;
		}

		/* Compute calibration values */
		for (int i = 0; i < 3; i++) {
			calibration_.acc_bias[i] = (acc_min[i] + acc_max[i]) / 2.0;
			calibration_.acc_scale[i] = 2 * ACCELEROMETER_GRAVITY_NORM / (acc_max[i] - acc_min[i]);
		}

		if (steadyCount == 0)
			Debug::print(" === MOVE SLOWLY ===\n");
		else
			Debug::print(" === CALIBRATING ===\n");
		Debug::printf("Accelerometer min: {  %.2f,  %.2f,  %.2f  }\n", acc_min[0], acc_min[1], acc_min[2]);
		Debug::printf("Accelerometer max: {  %.2f,  %.2f,  %.2f  }\n", acc_max[0], acc_max[1], acc_max[2]);
		Debug::printf("Accelerometer bias: {  %.2f,  %.2f,  %.2f  }\n", calibration_.acc_bias[0], calibration_.acc_bias[1], calibration_.acc_bias[2]);
		Debug::printf("Accelerometer scale: {  %.2f,  %.2f,  %.2f  }\n", calibration_.acc_scale[0], calibration_.acc_scale[1], calibration_.acc_scale[2]);
		Debug::print("\n");

		osDelay(1000/ACCELEROMETER_CALIBRATION_SAMPLE_RATE);
	}

	calibration_.acc_bias_valid = true;
	calibration_.acc_scale_valid = true;

	Debug::print(" === Finished accelerometer calibration ===\n");
	Debug::printf("Accelerometer bias: {  %.7f,  %.7f,  %.7f  }\n", calibration_.acc_bias[0], calibration_.acc_bias[1], calibration_.acc_bias[2]);
	Debug::printf("Accelerometer scale: {  %.7f,  %.7f,  %.7f  }\n", calibration_.acc_scale[0], calibration_.acc_scale[1], calibration_.acc_scale[2]);

	/* We have now calibrated, but we need to verify that the calibration is valid */
	ValidateCalibration();
	if (!isAccelerometerCalibrated()) {
		Debug::print("Calibration failed: Could not validate calibration\n\n");
		osDelay(5000);
		return;
	}

	if (storeInEEPROM && eeprom_) { // if EEPROM is configured, store new calibration in EEPROM
		eeprom_->WriteData(eeprom_->sections.imu_calibration, (uint8_t *)&calibration_, sizeof(calibration_));
	}

	osDelay(5000);
}

float IMU::vector_length(const float v[3])
{
  return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void IMU::calibrateImu(const float desired_acc_vector[3],
				  const float actual_acc_vector[3],
				  float calibration_matrix[9])
{
  // Scale vectors to unity
  // arm_scale_f32 does not take a const vector, but it does not modify the
  // source vector (hence the const_cast)
  Debug::print("desired_acc_vector:\t"); Debug::printf("%f", desired_acc_vector[0]); Debug::print("\t");
  Debug::printf("%f", desired_acc_vector[1]); Debug::print("\t"); Debug::printf("%f", desired_acc_vector[2]); Debug::print("\n");
  float len_des = vector_length(desired_acc_vector);
  Debug::print("len_des:\t"); Debug::printf("%f\n", len_des);
  float d[3];
  arm_scale_f32(const_cast<float*>(desired_acc_vector), 1.f/len_des, d, 3);
  Debug::print("d:\t"); Debug::printf("%f", d[0]); Debug::print("\t");
  Debug::printf("%f", d[1]); Debug::print("\t"); Debug::printf("%f", d[2]); Debug::print("\n");

  Debug::print("actual_acc_vector:\t"); Debug::printf("%f", actual_acc_vector[0]); Debug::print("\t");
  Debug::printf("%f", actual_acc_vector[1]); Debug::print("\t"); Debug::printf("%f", actual_acc_vector[2]); Debug::print("\n");
  float len_act = vector_length(actual_acc_vector);
  Debug::print("len_act:\t"); Debug::printf("%f\n", len_act);
  float a[3];
  arm_scale_f32(const_cast<float*>(actual_acc_vector), 1.f/len_act, a, 3);
  Debug::print("a:\t"); Debug::printf("%f", a[0]); Debug::print("\t");
  Debug::printf("%f", a[1]); Debug::print("\t"); Debug::printf("%f", a[2]); Debug::print("\n");
  // Find rotation matrix R between vectors, s.t. Ra=d
  // See: https://math.stackexchange.com/a/476311

  // Cross product: v = a x d
  float v[3] = {a[1]*d[2]-a[2]*d[1],
				a[2]*d[0]-a[0]*d[2],
				a[0]*d[1]-a[1]*d[0]};
  Debug::print("v:\t"); Debug::printf("%f", v[0]); Debug::print("\t");
  Debug::printf("%f", v[1]); Debug::print("\t"); Debug::printf("%f", v[2]); Debug::print("\n");
  // Sine between vectors: s = ||v||
  float s = vector_length(v);
  Debug::print("s:\t"); Debug::printf("%f\n", s);
  // Cosine between vectors: c = a . b
  float c;
  arm_dot_prod_f32(a, d, 3, &c);
  Debug::print("c:\t"); Debug::printf("%f\n", c);
  // R = I + [v]_x + [v]_x^2 (1-c)/(s^2), where [v]_x is the skew-symmetric
  // cross product matrix of v
  // It is not applicable if a and b point into exactly opposite directions,
  // which is unlikely so we do not handle it.
  float R[9] = {1.f, 0.f, 0.f,
					 0.f, 1.f, 0.f,
					 0.f, 0.f, 1.f,};
  arm_matrix_instance_f32 R_;
  arm_mat_init_f32(&R_, 3, 3, R);
  float v_x_data[9] = { 0.f, -v[2],  v[1],
					   v[2],   0.f, -v[0],
					  -v[1],  v[0],   0.f};
  arm_matrix_instance_f32 temp;
  arm_mat_init_f32(&temp, 3, 3, v_x_data);
  arm_mat_add_f32(&R_, &temp, &R_);
  arm_mat_mult_f32(&temp, &temp, &temp);
  arm_mat_scale_f32(&temp, (1.f - c) / (s * s), &temp);
  arm_mat_add_f32(&R_, &temp, &R_);

  // Regularize rotation matrix by making an SVD decomposition and forcing singular values to be 1
  float U[3*3]; arm_matrix_instance_f32 U_; arm_mat_init_f32(&U_, 3, 3, (float32_t *)U);
  float S[3*3]; arm_matrix_instance_f32 S_; arm_mat_init_f32(&S_, 3, 3, (float32_t *)S);
  float V[3*3]; arm_matrix_instance_f32 V_; arm_mat_init_f32(&V_, 3, 3, (float32_t *)V);

  /*svd(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8],
	  U[0], U[1], U[2], U[3], U[4], U[5], U[6], U[7], U[8],
	  S[0], S[1], S[2], S[3], S[4], S[5], S[6], S[7], S[8],
	  V[0], V[1], V[2], V[3], V[4], V[5], V[6], V[7], V[8]);*/
  svd(R, U, S, V);

  // Compute the regularized rotation matrix
  float V_T[3*3]; arm_matrix_instance_f32 V_T_; arm_mat_init_f32(&V_T_, 3, 3, (float32_t *)V_T);
  arm_mat_trans_f32(&V_, &V_T_);
  arm_mat_mult_f32(&U_, &V_T_, &R_); // R = U * V'

  // Return the matrix
  memcpy(calibration_matrix, R, 9*sizeof(R[0]));
}

void IMU::rotateImuMeasurement(float& gx, float& gy, float& gz,
						  float& ax, float& ay, float& az,
						  const float calibration_matrix[9])
{
  arm_matrix_instance_f32 R;
  arm_mat_init_f32(&R, 3, 3, const_cast<float*>(calibration_matrix));

  arm_matrix_instance_f32 g;
  float g_data[3] = {gx, gy, gz};
  arm_mat_init_f32(&g, 3, 1, g_data);
  arm_mat_mult_f32(&R, &g, &g);
  gx = g_data[0]; gy = g_data[1]; gz = g_data[2];

  arm_matrix_instance_f32 a;
  float a_data[3] = {ax, ay, az};
  arm_mat_init_f32(&a, 3, 1, a_data);
  arm_mat_mult_f32(&R, &a, &a);
  ax = a_data[0]; ay = a_data[1]; az = a_data[2];
}
