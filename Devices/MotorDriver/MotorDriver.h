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
 
#ifndef DEVICES_MOTOR_DRIVER_H
#define DEVICES_MOTOR_DRIVER_H

class MotorDriver
{

	public:
		MotorDriver();	
		virtual ~MotorDriver() {};

		virtual void Enable() {};
		virtual void Disable() {};

		virtual bool SetCurrent(float current) { return false; };

		virtual bool SetTorque(float torqueNewtonMeter) { return false; };
		virtual float GetAppliedTorque() { return 0; };
		virtual float GetCurrent() { return 0; };

	private:
		
};
	
	
#endif
