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
 
#ifndef DEVICES_IMU_H
#define DEVICES_IMU_H

#include "cmsis_os.h"
#include "EEPROM.h"

class IMU
{
	private:
		const float ACCELEROMETER_CALIBRATION_TIME = 120; // seconds
		const float ACCELEROMETER_CALIBRATION_SAMPLE_RATE = 100; // Hz
		const float ACCELEROMETER_CALIBRATION_LPF_COEFF_A = 0.904837418035960; // corresponds to 0.3 seconds settling time at 100 Hz sampling
		const float ACCELEROMETER_CALIBRATION_LPF_COEFF_B = 0.095162581964040;
		const float ACCELEROMETER_CALIBRATION_STEADY_TIME = 1.0; // seconds
		const float ACCELEROMETER_GRAVITY_NORM = 9.82; // m/s^2

	public:
		typedef struct Measurement_t {
			float Accelerometer[3];
			float Gyroscope[3];
			float Magnetometer[3];
		} Measurement_t;

		typedef struct Estimates_t {
			float q[4];
			float dq[4];
		} Estimates_t;

		typedef struct calibration_t {
			float imu_calibration_matrix[9];
			bool imu_calibration_matrix_valid = false;
			float gyro_bias[3];
			bool gyro_bias_valid = false;
			float acc_bias[3];
			bool acc_bias_valid = false;
			float acc_scale[3];
			bool acc_scale_valid = false;
		};

	public:
		virtual ~IMU() {};

		virtual uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY) { return pdFALSE; };
		virtual void Get(Measurement_t& measurement) {};
		virtual void GetEstimates(Estimates_t& estimates) {}; // if supported, eg. by Xsens IMU

		void Calibrate(bool storeInEEPROM = true);
		void CalibrateAccelerometer(bool storeInEEPROM = true);
		void SetCalibration(const float accelerometer_bias[3], const float accelerometer_scale[3], const float gyroscope_bias[3], const float calibration_matrix[3*3], bool storeInEEPROM = true);
		void CorrectMeasurement(Measurement_t& measurement, bool correctAccelerometerBias = false, bool correctAccelerometerScale = false, bool correctGyroBias = true, bool correctAlignment = true);

		bool isCalibrated();
		bool isAccelerometerCalibrated();
		bool isGyroscopeCalibrated();
		bool isAlignmentCalibrated();

		void AttachEEPROM(EEPROM * eeprom);

	private:
		void LoadCalibrationFromEEPROM(void);
		void ValidateCalibration(void);
		void ValidateCalibrationMatrix(void);
		float vector_length(const float v[3]);
		void calibrateImu(const float desired_acc_vector[3], const float actual_acc_vector[3], float calibration_matrix[9]);
		void rotateImuMeasurement(float& gx, float& gy, float& gz, float& ax, float& ay, float& az, const float calibration_matrix[9]);

	private:
		EEPROM * eeprom_ = 0;
		calibration_t calibration_;

		const float reference_acc_vector_[3] = {0.0f, 0.0f, 9.82f};

};
	
	
#endif
