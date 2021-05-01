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
#include "FirstOrderLPF.h"

#include <functional>

class SpeedController
{
	private:
		const uint32_t SPEED_CONTROLLER_THREAD_STACK = 512;
		const float SAMPLE_RATE = 20; // Hz
		const uint32_t TicksPrRev = 1;

		const float KP = 0.03;
		const float KI = 0.02;
		const float KD = 0;
		const float LPF_FREQ = 5.0; // Hz

		const float MIN_SPEED = 0.2f; // rad/s (anything below this will be detected as a desire to stop/brake)
		const float MIN_INTEGRATOR_SPEED = 4.0f; // rad/s (anything below this will be detected as a desire to stop/brake)
		const float DEADBAND = 0.2154f; // deadband value (should come from exponential fit)

	public:
		SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t EncoderTicksPrRev, uint32_t speedControllerPriority);
		SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t EncoderTicksPrRev, std::function<float(float)> feedforward, uint32_t speedControllerPriority);
		~SpeedController();

	public:
		void Enable();
		void Disable();
		void SetSpeed(float speed);

		void SetPID(float P, float I, float D);

	private:
		TaskHandle_t _speedControllerTaskHandle;

		LSPC * _lspc;
		Timer& _microsTimer;
		Servo& _motor;
		Encoder& _encoder;

		PID _controller;
		FirstOrderLPF _speedLPF;

		std::function<float(float)> _feedforward;

		bool _enabled;
		float _speed;

		enum {
			STOPPED,
			FORWARD,
			BACKWARD
		} _driving_direction;

	private:
		static void SpeedControllerThread(void * pvParameters);
		
	public:
		static int32_t EncoderDiff;
		static float SpeedRaw;
		static float SpeedFiltered;
		static float SpeedSetpoint;
		static float MotorOutput;

};
	
	
#endif
