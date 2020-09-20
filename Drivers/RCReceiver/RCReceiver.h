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
 
#ifndef DRIVERS_RCRECEIVER_H
#define DRIVERS_RCRECEIVER_H

#include "InputCapture.h"

class RCReceiver : private InputCapture
{
	
private:
	const float RECEIVER_PERIOD = 0.020;    // 50 Hz = 20 ms
	const float RECEIVER_PERIOD_TOLERANCE = 0.002;    // +/- 2 ms tolerance

public:
	RCReceiver(InputCapture::timer_t timer, InputCapture::ic_channel_t channel, float min_ms = 1.0f, float max_ms = 2.0f);
	//~RCReceiver(); // use base-class destructor

	float Get(bool ClearAfterReading = false);
	bool isActive(void);

private:
	bool VerifyPeriod(void);

private:
	float _min;
	float _max;

	float _prev_value;

};
	
	
#endif
