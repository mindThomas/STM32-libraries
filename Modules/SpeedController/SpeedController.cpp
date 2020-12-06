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
#include <math.h>

#include "Debug.h"

int32_t SpeedController::EncoderDiff = 0;
float SpeedController::SpeedRaw;
float SpeedController::SpeedFiltered;
float SpeedController::SpeedSetpoint;
float SpeedController::MotorOutput;

SpeedController::SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t EncoderTicksPrRev, uint32_t speedControllerPriority) : _speedControllerTaskHandle(0), _lspc(lspc), _microsTimer(microsTimer), _motor(motor), _encoder(encoder), TicksPrRev(EncoderTicksPrRev), _controller(KP, KI, KD, &microsTimer), _speedLPF(1.0f / SAMPLE_RATE, LPF_FREQ), _enabled(false), _speed(0)
{
	motor.Disable();
	xTaskCreate(SpeedController::SpeedControllerThread, (char *)"Speed Controller", SPEED_CONTROLLER_THREAD_STACK, (void*) this, speedControllerPriority, &_speedControllerTaskHandle);
}

SpeedController::SpeedController(LSPC * lspc, Timer& microsTimer, Servo& motor, Encoder& encoder, uint32_t EncoderTicksPrRev, std::function<float(float)> feedforward, uint32_t speedControllerPriority)
	: SpeedController(lspc, microsTimer, motor, encoder, EncoderTicksPrRev, speedControllerPriority)
{
	_feedforward = feedforward;
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

void SpeedController::SetPID(float P, float I, float D)
{
	_controller.SetPID(P, I, D);
}

void SpeedController::SpeedControllerThread(void * pvParameters)
{
	SpeedController * controller = (SpeedController *)pvParameters;
	TickType_t xLastWakeTime;
	uint32_t prevTimerValue; // used for measuring dt
	uint32_t prevTimerValue2; // used for measuring dt

	/* Motor speed variables */
	volatile float dpsi_filtered = 0;
	volatile int32_t prevEncoderTicks = 0;
	volatile float prevMotorAngle = 0;

	/* Controller loop time / sample rate */
	TickType_t loopWaitTicks = configTICK_RATE_HZ / controller->SAMPLE_RATE;

	/* Load initialized objects for easier access */
	Timer& microsTimer = controller->_microsTimer;
	Servo& motor = controller->_motor;
	Encoder& encoder = controller->_encoder;
	PID& pid = controller->_controller;

	/* Wait until the controller is enabled */
	while (!controller->_enabled) osDelay(10);
	/* Enable motor by calibrating ESC */
	motor.Disable();
	osDelay(100);
	motor.Set(0);
	osDelay(100);
	motor.Disable();
	osDelay(100);
	motor.Set(0);
	osDelay(1000);

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
			/* Enable motor by calibrating ESC */
			motor.Disable();
			osDelay(100);
			motor.Set(0);
			osDelay(100);
			motor.Disable();
			osDelay(100);
			motor.Set(0);
			osDelay(1000);
		}

		volatile float dt = microsTimer.GetDeltaTime(prevTimerValue);
		prevTimerValue = microsTimer.Get();

		/* Estimate velocity from encoders */
		volatile int32_t encoderTicks = encoder.Get();
		controller->EncoderDiff = encoderTicks - prevEncoderTicks;
		volatile float EncoderDiffMeas = (float)(encoderTicks - prevEncoderTicks);
		prevEncoderTicks = encoderTicks;

		float EncoderConversionRatio = M_2PI / controller->TicksPrRev;
		float dpsi = EncoderConversionRatio * EncoderDiffMeas / dt;
		dpsi_filtered = controller->_speedLPF.Filter(dpsi);

		controller->SpeedRaw = dpsi;
		controller->SpeedFiltered = dpsi_filtered;
		controller->SpeedSetpoint = controller->_speed;

		/* Compute control */
		float output = pid.Step(dpsi_filtered, controller->_speed, (fabs(controller->_speed) > controller->MIN_INTEGRATOR_SPEED) );
		if (controller->_feedforward && fabs(controller->_speed) > controller->MIN_SPEED) {
			output += controller->_feedforward(controller->_speed);
		}

		/* Control the direction change and avoid sudden brakes due to computed control output changing sign */
		if (controller->_speed > controller->MIN_SPEED) // driving forward
			controller->_driving_direction = FORWARD;
		else if (controller->_speed < -controller->MIN_SPEED) // driving backward
			controller->_driving_direction = BACKWARD;
		else if (fabsf(dpsi_filtered) < controller->MIN_SPEED)
			controller->_driving_direction = STOPPED;

		if (controller->_driving_direction == FORWARD && output < controller->DEADBAND) output = controller->DEADBAND;
		if (controller->_driving_direction == BACKWARD && output > -controller->DEADBAND) output = -controller->DEADBAND;
		if (controller->_driving_direction == STOPPED) pid.Reset();

		/* Saturate output */
		if (output > 1.0f) output = 1.0f;
		if (output < -1.0f) output = -1.0f;

		/* Set output */
		motor.Set(output);

		controller->MotorOutput = output;
	}
}
