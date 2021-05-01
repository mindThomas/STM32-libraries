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

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_tim.h"

#ifdef USE_FREERTOS
#include "cmsis_os.h" // for semaphore support
#endif

class Timer
{
	private:
		const uint16_t TIMER_DEFAULT_MAXVALUE = 0xFFFF;

	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER2,
			TIMER14,
			TIMER16,
			TIMER17
		} timer_t;

	public:
		Timer(timer_t timer, uint32_t frequency); // frequency defines the timer count frequency
		~Timer();

		void ConfigureTimerPeripheral();
		void RegisterInterrupt(uint32_t frequency, void (*TimerCallback)());
	#ifdef USE_FREERTOS
		void RegisterInterruptSoft(uint32_t frequency, void (*TimerCallbackSoft)());
		void RegisterInterrupt(uint32_t frequency, SemaphoreHandle_t semaphore);
	#endif
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
			TIM_TypeDef * instance;
			bool waitingFlag;
			void (*TimerCallback)();
		#ifdef USE_FREERTOS
			TaskHandle_t callbackTaskHandle;
			SemaphoreHandle_t callbackSemaphore;
		#endif
		} hardware_resource_t;

		static hardware_resource_t * resTIMER1;
		//static hardware_resource_t * resTIMER2;
		//static hardware_resource_t * resTIMER3;
		static hardware_resource_t * resTIMER14;
		//static hardware_resource_t * resTIMER16;
		static hardware_resource_t * resTIMER17;

	#ifdef USE_FREERTOS
		void (*_TimerCallbackSoft)();
	#endif

	private:
		hardware_resource_t * _hRes;
	#ifdef USE_FREERTOS
		SemaphoreHandle_t _waitSemaphore;
	#endif

	public:
		static void InterruptHandler(Timer::hardware_resource_t * timer);
	#ifdef USE_FREERTOS
		static void CallbackThread(void * pvParameters);
	#endif

};
	
	
#endif
