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
 
#ifndef DEVICES_ESCON_H
#define DEVICES_ESCON_H

#include "stm32h7xx_hal.h"
#include <Motor/Motor.hpp>
#include <PWM/PWM.hpp>
#include <IO/IO.hpp>
#include <ADC/ADC.hpp>

class ESCON : private Motor
{
	private:
		const double M_PI = 3.14159265358979323846264338327950288;

		const int ESCON_PWM_FREQUENCY	= 1000;		// 4 kHz
		const int ESCON_PWM_RANGE = 2000;			// 0-2000, corresponding to 0.1% resolution

		const float ESCON_MAX_RAD_PR_SEC; // rad/s
		const float ESCON_MAX_AMP_SETPOINT;	 // A
		const float MOTOR_TORQUE_CONSTANT;  // Nm/A

	public:
		ESCON(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed); // minimal operation general constructor
		ESCON(PWM * TorqueSetpoint, IO * EnablePin, Encoder * encoder, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed, ADC * CurrentFeedback, ADC * VelocityFeedback, IO * DirectionFeedbackPin); // extended operation general constructor
		ESCON(uint8_t MotorIndex, float MaxCurrent, float TorqueConstant, float GearRatio, uint16_t EncoderTicksPrRev, float MaxMotorSpeed); // platform specific constructor
		~ESCON();

		void Enable();
		void Disable();

		bool SetTorque(float torqueNewtonMeter);
		float GetAppliedTorque();
		float GetCurrent();
		float GetAngle();
		float GetVelocity();

		int32_t GetEncoderRaw();

	private:
		PWM * _torqueSetpoint; 		// DIG_IN_1 on ESCON
		IO * _enablePin;			// DIG_IN_4 on ESCON
		ADC * _currentFeedback;		// AN_OUT_1 on ESCON
		ADC * _velocityFeedback;	// AN_OUT_2 on ESCON
		IO * _directionFeedbackPin;	// DIG_IN_3 on ESCON

		// Future use objects
		// PWM * _DIG_IN_2;
		// IO * _DIG_IN_3;
		// DAC * _AN_IN_1; // for furture use when SPI DAC library has been implemented and tested
		// DAC * _AN_IN_2;

		bool _deleteObjectsAtDestruction;
};
	
	
#endif
