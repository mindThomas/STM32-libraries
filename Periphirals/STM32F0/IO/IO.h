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
 
#ifndef PERIPHIRALS_IO_H
#define PERIPHIRALS_IO_H

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"

#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#endif

#define GPIO_PIN_0		LL_GPIO_PIN_0
#define GPIO_PIN_1		LL_GPIO_PIN_1
#define GPIO_PIN_2		LL_GPIO_PIN_2
#define GPIO_PIN_3		LL_GPIO_PIN_3
#define GPIO_PIN_4		LL_GPIO_PIN_4
#define GPIO_PIN_5		LL_GPIO_PIN_5
#define GPIO_PIN_6		LL_GPIO_PIN_6
#define GPIO_PIN_7		LL_GPIO_PIN_7
#define GPIO_PIN_8		LL_GPIO_PIN_8
#define GPIO_PIN_9		LL_GPIO_PIN_9
#define GPIO_PIN_10		LL_GPIO_PIN_10
#define GPIO_PIN_11		LL_GPIO_PIN_11
#define GPIO_PIN_12		LL_GPIO_PIN_12
#define GPIO_PIN_13		LL_GPIO_PIN_13
#define GPIO_PIN_14		LL_GPIO_PIN_14
#define GPIO_PIN_15		LL_GPIO_PIN_15

class IO
{
	public:
		typedef enum interrupt_trigger_t {
			TRIGGER_RISING = 0,
			TRIGGER_FALLING,
			TRIGGER_BOTH
		} interrupt_trigger_t;

		typedef enum pull_t {
			PULL_NONE = 0,
			PULL_UP,
			PULL_DOWN,
		} pull_t;

	public:
		IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin); // configure as output
		IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, pull_t pull); // configure as input
		~IO();

#ifdef USE_FREERTOS
		void RegisterInterrupt(interrupt_trigger_t trigger, SemaphoreHandle_t semaphore);
#endif
		void RegisterInterrupt(interrupt_trigger_t trigger, void (*InterruptCallback)(void * params), void * callbackParams = 0);
		void DeregisterInterrupt();

		void Set(bool state);
		bool Read();
		void High();
		void Low();
		void Toggle();

		void ChangeToInput(pull_t pull);
		void ChangeToOutput(bool state = false);
		void ChangeToOpenDrain(bool state = false);

	private:
		void ConfigurePin(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, bool isInput, bool isOpenDrain, pull_t pull);
		uint16_t getPinIndex();

	public:
		void (*_InterruptCallback)(void * params);
		void * _InterruptCallbackParams;
#ifdef USE_FREERTOS
		SemaphoreHandle_t _InterruptSemaphore;
#endif

		static IO * interruptObjects[16]; // we only have 16 interrupt lines

	private:
		GPIO_TypeDef * _GPIO;
		uint32_t _pin;
		bool _isInput;
		bool _isOpenDrain;
		pull_t _pull;

	private:
		void ConfigureInterrupt(interrupt_trigger_t level);
		void DisableInterrupt();

	public:
		static void InterruptHandler(IO * io);
};
	
	
#endif
