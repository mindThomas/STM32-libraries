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
 
#pragma once

#include <PWM/PWM.hpp>

class Servo : private PWM
{
	
public:
	Servo(timer_t timer, pwm_channel_t channel, float min = -1.0f, float max = 1.0f, float min_ms = 1.0f, float max_ms = 2.0f);
	
	void Set(float value);
	void Disable();

private:	
	const float _min_ms;
    const float _min_value;
    const float _max_ms;
    const float _max_value;
    const float _ms_resolution;
	
};