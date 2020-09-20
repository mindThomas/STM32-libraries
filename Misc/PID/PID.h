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
 
#ifndef MODULES_CONTROLLERS_PID_H
#define MODULES_CONTROLLERS_PID_H

#include <stddef.h>
#include <stdlib.h>

#include "Timer.h"

class PID
{
	public:
		PID(const float Kp, const float Ki, const float Kd, Timer * microsTimer);
		PID(const float Kp, const float Ki = 0.0f, const float Kd = 0.0f);
		~PID();

		void Reset(void);
		void SetPID(float Kp, float Ki, float Kd);
		float Step(const float state, const float ref, bool integrator_enabled = true);
		float Step(const float state, const float ref, const float dt, bool integrator_enabled = true);

	private:
		float Kp_;
		float Ki_;
		float Kd_;

		Timer * _microsTimer;
		uint32_t _prevTimerValue;

		float prev_error_;
		float integral_;
};
	
	
#endif
