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
 
#ifndef PERIPHIRALS_TIMER_H
#define PERIPHIRALS_TIMER_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "cmsis_os.h" // for semaphore support

class Timer
{
	private:
		const uint16_t TIMER_DEFAULT_MAXVALUE = 0xFFFF;

	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER6,
			TIMER7,
			TIMER8,
			TIMER17,
		} timer_t;

	public:
		Timer(timer_t timer, uint32_t frequency); // frequency defines the timer count frequency
		~Timer();

		void ConfigureTimerPeripheral();
		void RegisterInterruptSoft(uint32_t frequency, void (*TimerCallbackSoft)(void * param), void * parameter = (void*)0);
		void RegisterInterrupt(uint32_t frequency, void (*TimerCallback)(void * param), void * parameter = (void*)0);
		void RegisterInterrupt(uint32_t frequency, SemaphoreHandle_t semaphore);
		void SetMaxValue(uint16_t maxValue);

		uint32_t Get();
		float GetTime();
		void Reset();
		void Wait(uint32_t MicrosToWait);
		float GetDeltaTime(uint32_t prevTimerValue);

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			uint32_t frequency;
			uint16_t maxValue;
			uint32_t counterOffset;
			TIM_HandleTypeDef handle;
			TaskHandle_t callbackTaskHandle;
			void (*TimerCallback)(void * param);
			void * TimerCallbackParameter;
			SemaphoreHandle_t callbackSemaphore;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER6;
		static hardware_resource_t * resTIMER7;
		static hardware_resource_t * resTIMER8;
		static hardware_resource_t * resTIMER17;

		void (*_TimerCallbackSoft)(void * param);
		void * _TimerCallbackSoftParameter;

	private:
		hardware_resource_t * _hRes;
		SemaphoreHandle_t _waitSemaphore;

	public:
		static void InterruptHandler(Timer::hardware_resource_t * timer);
		static void CallbackThread(void * pvParameters);

};
	
	
#endif
