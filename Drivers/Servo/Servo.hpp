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
 
#ifndef DRIVERS_SERVO_H
#define DRIVERS_SERVO_H

#include <PWM/PWM.hpp>

class Servo : private PWM
{
	
public:
	Servo(timer_t timer, pwm_channel_t channel, float min = -1.0f, float max = 1.0f, float min_ms = 1.0f, float max_ms = 2.0f, uint16_t range_resolution_steps = 2048);
	
	void Set(float value);
	void Disable();

private:	
	float _min_ms;
	float _min_value;
	float _max_ms;	
	float _max_value;
	float _ms_resolution;
	
};
	
	
#endif
