/* Copyright (C) 2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 
#ifndef MODULES_SPEEDCONTROLLER_H
#define MODULES_SPEEDCONTROLLER_H

#include "cmsis_os.h" // for processing task

#include "LSPC.hpp"
#include "Timer.h"
#include "Servo.h"
#include "Encoder.h"
#include "PID.h"

class SpeedController
{
	private:
		const uint32_t SPEED_CONTROLLER_THREAD_STACK = 512;
		const float SAMPLE_RATE = 1;
		const uint32_t TicksPrRev = 4096;

		const float KP = 1;
		const float KI = 0;
		const float KD = 0;

	public:
		SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t speedControllerPriority);
		~SpeedController();

	public:
		void Enable();
		void Disable();
		void SetSpeed(float speed);

	private:
		TaskHandle_t _speedControllerTaskHandle;

		LSPC * _lspc;
		Timer& _microsTimer;
		Servo& _motor;
		Encoder& _encoder;
		PID _controller;

		bool _enabled;
		float _speed;

	private:
		static void SpeedControllerThread(void * pvParameters);
		
};
	
	
#endif
