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

/* OBS. Remember to keep this file consistent across the Embedded Firmware (Kugle-Embedded) and ROS Driver (Kugle-ROS) */

#ifndef LSPC_MESSAGE_TYPES_HPP
#define LSPC_MESSAGE_TYPES_HPP

#include <cstdint>

namespace lspc
{
	namespace ParameterLookup {
		typedef enum: uint8_t
		{
			_unknown = 0x00,
			_bool = 0x01,
			_float,
			_uint8,
			_uint16,
			_uint32
		} ValueType_t;
		typedef enum: uint8_t
		{
			unknown = 0x00,
			debug = 0x01,
			behavioural,
			controller,
			estimator,
			model,
			test
		} type_t;

		typedef enum: uint8_t
		{
			EnableDumpMessages = 0x01,
			EnableRawSensorOutput,
			UseFilteredIMUinRawSensorOutput,
			DisableMotorOutput
		} debug_t;

		typedef enum: uint8_t
		{
			IndependentHeading = 0x01,
			YawVelocityBraking,
			StepTestEnabled,
			SineTestEnabled,
			CircleTestEnabled,
			PowerButtonMode
		} behavioural_t;

		typedef enum: uint8_t
		{
			ControllerSampleRate = 0x01,
			type,
			mode,
			EnableTorqueLPF,
			TorqueLPFtau,
			MotorFailureDetection,
			EnableTorqueSaturation,
			TorqueMax,
			TorqueRampUp,
			TorqueRampUpTime,
			DisableQdot,
			DisableQdotInEquivalentControl,
			DisableOmegaXYInEquivalentControl,
			AngularVelocityClampsEnabled,
			AngularVelocityClamps,
			ManifoldType,
			K,
			Kx,
			Ky,
			Kz,
			Kv_x,
			Kv_y,
			Kvi_x,
			Kvi_y,
			gamma,
			ContinousSwitching,
			EquivalentControl,
			eta,
			epsilon,
			LQR_K,
			LQR_MaxYawError,
			VelocityControl_AccelerationLimit,
			VelocityControl_UseOmegaRef,
			VelocityController_MaxTilt,
			VelocityController_MaxIntegralCorrection,
			VelocityController_VelocityClamp,
			VelocityController_IntegralGain,
			VelocityController_AngleLPFtau,
			VelocityController_OmegaLPFtau
		} controller_t;

		typedef enum: uint8_t
		{
			EstimatorSampleRate = 0x01,
			EnableSensorLPFfilters,
			EnableSoftwareLPFfilters,
			SoftwareLPFcoeffs_a,
			SoftwareLPFcoeffs_b,
			CreateQdotFromQDifference,
			UseMadgwick,
			EstimateBias,
			SensorDrivenQEKF,
			UseCoRvelocity,
			UseVelocityEstimator,
			EnableVelocityLPF,
			EnableWheelSlipDetector,
			UseQdotInVelocityEstimator,
			EstimateCOM,
			EstimateCOMminVelocity,
			MaxCOMDeviation,
			MadgwickBeta,
			GyroCov_Tuning_Factor,
			AccelCov_Tuning_Factor,
			cov_gyro_mpu,
			cov_acc_mpu,
			sigma2_bias,
			sigma2_omega,
			sigma2_heading,
			GyroscopeTrustFactor,
			eta_encoder,
			eta_accelerometer,
			var_acc_bias,
			var_acceleration
		} estimator_t;

		typedef enum: uint8_t
		{
			l = 0x01,
			COM_X,
			COM_Y,
			COM_Z,
			CoR,
			g,
			rk,
			Mk,
			Jk,
			rw,
			Mw,
			i_gear,
			Jow,
			Jm,
			Jw,
			Mb,
			Jbx,
			Jby,
			Jbz,
			Bvk,
			Bvm,
			Bvb,
			EncoderTicksPrRev,
			TicksPrRev,
			SaturationTorqueOfMaxOutputTorque
		} model_t;

		typedef enum: uint8_t
		{
			tmp = 0x01,
			tmp2
		} test_t;
	}

	namespace ParameterTypes {
		typedef enum: uint8_t {
			UNKNOWN_CONTROLLER = 0x00,
			LQR_CONTROLLER = 0x01,
			SLIDING_MODE_CONTROLLER
		} controllerType_t;

		typedef enum: uint8_t {
			OFF = 0x00,
			QUATERNION_CONTROL,
			VELOCITY_CONTROL,
			PATH_FOLLOWING,
			UNKNOWN_MODE = 0xFF
		} controllerMode_t;

		typedef enum: uint8_t {
			POWER_OFF = 0x00, // default
			START_STOP_QUATERNION_CONTROL,
			START_STOP_VELOCITY_CONTROL,
			UNKNOWN_BUTTON_MODE = 0xFF
		} powerButtonMode_t;

		typedef enum: uint8_t {
			Q_DOT_INERTIAL_MANIFOLD = 0x00,
			Q_DOT_BODY_MANIFOLD,
			OMEGA_INERTIAL_MANIFOLD,
			OMEGA_BODY_MANIFOLD, // default/suggested
			VELOCITY_AND_Q_DOT_MANIFOLD,
			UNKNOWN_MANIFOLD = 0xFF
		} slidingManifoldType_t;

		typedef enum: uint8_t {
			BODY_FRAME = 0x00,
			INERTIAL_FRAME,
			HEADING_FRAME,
			UNKNOWN_FRAME = 0xFF
		} referenceFrame_t;
	}

	namespace MessageTypesFromPC
	{
		typedef enum MessageTypesFromPC: uint8_t
		{
			Test = 0x01,
			GetParameter = 0x02,
			SetParameter = 0x03,
			StoreParameters = 0x04,
			DumpParameters = 0x05,
			SystemSettings = 0x10,
			EstimatorSettings = 0x11,
			ControllerSettings = 0x12,
			YawCorrection = 0x20,
			PositionCorrection = 0x21,
			QuaternionReference = 0x30,
			AngularVelocityReference = 0x31,
			BalanceControllerReference = 0x32,
			VelocityReference = 0x33,
			MPCpathReference = 0x34,
			CalibrateIMU = 0xE0,
			CPUload = 0xE1,
			RestartController = 0xE2,
			EnterBootloader = 0xF0,
			Reboot = 0xF1,
			Debug = 0xFF
		} MessageTypesFromPC_t;

		typedef struct
		{
			ParameterLookup::type_t type;
			uint8_t param;
		} GetParameter_t;

		typedef struct
		{
			ParameterLookup::type_t type;
			uint8_t param;
			ParameterLookup::ValueType_t valueType;
			uint8_t arraySize;
			//void * valuePtr;  // after arraySize the parameter values are inserted in little-endian format with value[0] first (if array)
		} SetParameter_t;

		typedef struct
		{
			uint16_t estimate_msg_prescaler;
		} EstimatorSettings_t;

		typedef struct
		{
			ParameterTypes::controllerMode_t mode;
			ParameterTypes::controllerType_t type;
		} ControllerSettings_t;

		typedef struct
		{
			float yaw;
		} YawCorrection_t;

		typedef struct
		{
			float x;
			float y;
		} PositionCorrection_t;

		typedef struct
		{
			struct q_t
			{
				float w;
				float x;
				float y;
				float z;
			} q;
		} QuaternionReference_t;

		typedef struct
		{
			ParameterTypes::referenceFrame_t frame;
			struct omega_t
			{
				float x;
				float y;
				float z;
			} omega;
		} AngularVelocityReference_t;

		typedef struct
		{
			ParameterTypes::referenceFrame_t frame; // defines the frame of the angular velocity
			struct q_t
			{
				float w;
				float x;
				float y;
				float z;
			} q;
			struct omega_t
			{
				float x;
				float y;
				float z;
			} omega;
		} BalanceControllerReference_t;

		typedef struct
		{
			ParameterTypes::referenceFrame_t frame;
			struct vel_t
			{
				float x;
				float y;
				float yaw;
			} vel;
		} VelocityReference_t;

		typedef struct
		{
			float desired_velocity;
			float desired_heading;
			float path_length;
			float coeffs_x[10];
			float coeffs_y[10];
		} MPCpathReference_t;

		typedef struct
		{
			uint32_t magic_key;
			bool calibrate_accelerometer;
		} CalibrateIMU_t;

		typedef struct
		{
			uint32_t magic_key;
		} RestartController_t;

		typedef struct
		{
			uint32_t magic_key;
		} EnterBootloader_t;

		typedef struct
		{
			uint32_t magic_key;
		} Reboot_t;
	}

	namespace MessageTypesToPC
	{
		typedef enum MessageTypesToPC: uint8_t
		{
			Test = 0x01,
			GetParameter = 0x02,
			SetParameterAck = 0x03,
			StoreParametersAck = 0x04,
			DumpParameters = 0x05,
			SystemInfo = 0x10,
			StateEstimates = 0x11,
			ControllerInfo = 0x12,
			AttitudeControllerInfo = 0x13,
			VelocityControllerInfo = 0x14,
			ControllerDebug = 0x15,
			MPCinfo = 0x20,
			PredictedMPCtrajectory = 0x21,
			RawSensor_IMU_MPU9250 = 0x30,
			RawSensor_IMU_MTI200 = 0x31,
			RawSensor_Encoders = 0x32,
			RawSensor_Battery = 0x33,
			CalibrateIMUAck = 0xE0,
			CPUload = 0xE1,
			RestartControllerAck = 0xE2,
			MathDump = 0xFA, // publish array of floats (parsed by PC and dumped into tabulated .txt file in "~/kugle_dump/")
			SensorDump = 0xFB,
			CovarianceDump = 0xFC,
			Debug = 0xFF
		} MessageTypesToPC_t;

		typedef struct
		{
			ParameterLookup::type_t type;
			uint8_t param;
			ParameterLookup::ValueType_t valueType;
			uint8_t arraySize;
			//void * valuePtr;  // after arraySize the parameter values are inserted in little-endian format with value[0] first (if array)
		} GetParameter_t;

		typedef struct
		{
			ParameterLookup::type_t type;
			uint8_t param;
			bool acknowledged;
		} SetParameterAck_t;

		typedef struct
		{
			bool acknowledged;
		} StoreParametersAck_t;

		typedef struct
		{
			uint16_t parameters_size_bytes;
			uint8_t packages_to_follow;
		} DumpParameters_t;

		typedef struct
		{
			float time;
			float battery_pct;
			float current_consumption;
		} SystemInfo_t;

		typedef struct
		{
			float time;
			struct q_t
			{
				float w;
				float x;
				float y;
				float z;
			} q;
			struct dq_t
			{
				float w;
				float x;
				float y;
				float z;
			} dq;
			struct pos_t
			{
				float x;
				float y;
			} pos;
			struct vel_t
			{
				float x;
				float y;
			} vel;
			struct COM_t
			{
				float x;
				float y;
				float z;
			} COM;
		} StateEstimates_t;

		typedef struct
		{
			float time;
			ParameterTypes::controllerType_t type;
			ParameterTypes::controllerMode_t mode;
			float torque1;
			float torque2;
			float torque3;
			float compute_time;
			float delivered_torque1;
			float delivered_torque2;
			float delivered_torque3;
		} ControllerInfo_t;		

		typedef struct
		{
			float time;
		} AttitudeControllerInfo_t;

		typedef struct
		{
			float time;
		} VelocityControllerInfo_t;

		typedef struct
		{
			float time;
			struct orient_t
			{
				float roll;
				float pitch;
				float yaw;
			} orient;
			struct orient_ref_t
			{
				float roll;
				float pitch;
				float yaw;
			} orient_ref;
			struct orient_integral_t
			{
				float roll;
				float pitch;
				float yaw;
			} orient_integral;
			struct omega_t
			{
				float x;
				float y;
				float z;
			} omega;
			struct omega_ref_t
			{
				float x;
				float y;
				float z;
			} omega_ref;
			struct vel_t
			{
				float x;
				float y;
			} vel;
			struct vel_kinematics_t
			{
				float x;
				float y;
			} vel_kinematics;
			struct vel_ref_t
			{
				float x;
				float y;
			} vel_ref;
			float torque[3];
			float S[3];
		} ControllerDebug_t;		
		
		typedef struct
		{
			float time;
		} MPCinfo_t;

		typedef struct
		{
			float time;
			uint8_t horizon_index;
			struct q_t
			{
				float w;
				float x;
				float y;
				float z;
			} q;
			struct dq_t
			{
				float w;
				float x;
				float y;
				float z;
			} dq;
			struct pos_t
			{
				float x;
				float y;
			} pos;
			struct vel_t
			{
				float x;
				float y;
			} vel;
		} PredictedMPCtrajectory_t;

		typedef struct
		{
			float time;
			struct accelerometer_t
			{
				float x;
				float y;
				float z;
				float cov[9]; // stored in row-major format
			} accelerometer;
			struct gyroscope_t
			{
				float x;
				float y;
				float z;
				float cov[9]; // stored in row-major format
			} gyroscope;
			struct magnetometer_t
			{
				float x;
				float y;
				float z;
				float cov[9]; // stored in row-major format
			} magnetometer;
		} RawSensor_IMU_MPU9250_t;

		typedef struct
		{
			float time;
			struct accelerometer_t
			{
				float x;
				float y;
				float z;
			} accelerometer;
			struct gyroscope_t
			{
				float x;
				float y;
				float z;
			} gyroscope;
			struct magnetometer_t
			{
				float x;
				float y;
				float z;
			} magnetometer;
		} RawSensor_IMU_MTI200_t;

		typedef struct
		{
			float time;
			float angle1;
			float angle2;
			float angle3;
		} RawSensor_Encoders_t;

		typedef struct
		{
			float time;
			float vbat1;
			float vbat2;
			float current1;
			float current2;
			float pct1;
			float pct2;
		} RawSensor_Battery_t;

		typedef struct
		{
			bool acknowledged;
		} CalibrateIMUAck_t;

		typedef struct
		{
			bool acknowledged;
		} RestartControllerAck_t;
	}

} // namespace lspc

#endif // LSPC_MESSAGE_TYPES_HPP
