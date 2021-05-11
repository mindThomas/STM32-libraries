/* Copyright (C) 2018- Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 
#pragma once

#include <MotorDriver/MotorDriver.hpp>
#include <Encoder/Encoder.hpp>
#include <CurrentSense/CurrentSense.hpp>

class Motor
{
	private:
		const double M_PI{3.14159265358979323846264338327950288};

		uint16_t EncoderTicksPrRev_{0};	 // ticks/rev on encoder side - hence one revolution on motor shaft (before gearing)
		float GearingRatio_{1};
		float R_a_{1};					 // coil/internal resistance
		float L_a_{1};					 // coil/internal inductance
		float K_t_{1};					 // torque constant
		float K_e_{1};					 // back-emf constant
		float B_{0};					 // viscous friction coefficient

	public:
		Motor();
		~Motor();

		void Enable();
		void Disable();

		bool SetOutputTorque(float torqueNewtonMeter);
		float GetAppliedOutputTorque();
		float GetAngle();
		int32_t GetEncoderRaw();

	private:
		MotorDriver * _motorDriver;
		CurrentSense * _currentSense;
		Encoder * _encoder;		
		
};
	
	
#endif
