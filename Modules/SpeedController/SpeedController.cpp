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
 
#include "SpeedController.h"
#include "cmsis_os.h" // for processing task
#include "MathLib.h"

#include "Debug.h"

SpeedController::SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t speedControllerPriority) : _speedControllerTaskHandle(0), _lspc(lspc), _microsTimer(microsTimer), _motor(motor), _encoder(encoder), _controller(KP, KI, KD, &microsTimer), _enabled(false), _speed(0)
{
	xTaskCreate(SpeedController::SpeedControllerThread, (char *)"Speed Controller", SPEED_CONTROLLER_THREAD_STACK, (void*) this, speedControllerPriority, &_speedControllerTaskHandle);
}

SpeedController::~SpeedController()
{
	if (_speedControllerTaskHandle)
		vTaskDelete(_speedControllerTaskHandle); // stop task
}

void SpeedController::Enable(void)
{
	_enabled = true;
}

void SpeedController::Disable(void)
{
	_enabled = false;
	_speed = 0;
}

void SpeedController::SetSpeed(float speed)
{
	_speed = speed;
}

void SpeedController::SpeedControllerThread(void * pvParameters)
{
	SpeedController * controller = (SpeedController *)pvParameters;
	TickType_t xLastWakeTime;
	uint32_t prevTimerValue; // used for measuring dt
	uint32_t prevTimerValue2; // used for measuring dt

	/* Motor speed variables */
	volatile float dpsi = 0;
	volatile int32_t prevEncoderTicks = 0;
	volatile float prevMotorAngle = 0;

	/* Controller loop time / sample rate */
	TickType_t loopWaitTicks = configTICK_RATE_HZ / controller->SAMPLE_RATE;

	/* Load initialized objects for easier access */
	Timer& microsTimer = controller->_microsTimer;
	Servo& motor = controller->_motor;
	Encoder& encoder = controller->_encoder;
	PID& pid = controller->_controller;

	/* Main loop */
	xLastWakeTime = xTaskGetTickCount();
	prevTimerValue = microsTimer.Get();
	prevTimerValue2 = HAL_tic();
	while (1)
	{
		/* Wait until time has been reached to make control loop periodic */
		vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

		if (!controller->_enabled) {
			motor.Disable();
			while (!controller->_enabled) osDelay(10);
			// Calibrate/enable servos
			motor.Set(0);
			osDelay(500);
		}

		volatile float dt = microsTimer.GetDeltaTime(prevTimerValue);
		prevTimerValue = microsTimer.Get();

		/* Estimate velocity from encoders */
		volatile int32_t encoderTicks = encoder.Get();
		volatile float EncoderDiffMeas = (float)(encoderTicks - prevEncoderTicks);
		prevEncoderTicks = encoderTicks;

		float EncoderConversionRatio = 2.f * M_PI / controller->TicksPrRev;
		dpsi = EncoderConversionRatio * EncoderDiffMeas;

		/* Compute control */
		float output = pid.Step(dpsi, controller->_speed);

		/* Saturate output */
		if (output > 1.0f) output = 1.0f;
		if (output < -1.0f) output = -1.0f;

		/* Set output */
		motor.Set(output);
	}
}
