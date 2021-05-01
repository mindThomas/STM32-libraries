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
 
#ifndef PERIPHIRALS_ENCODER_H
#define PERIPHIRALS_ENCODER_H

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_tim.h"

class Encoder
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER2,
			TIMER3,
		} timer_t;

	public:
		Encoder(timer_t timer);
		~Encoder();

		void ConfigureEncoderGPIO();
		void ConfigureEncoderPeripheral();

		int32_t Get();

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			TIM_TypeDef * instance;
			int32_t offsetValue;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER2;
		static hardware_resource_t * resTIMER3;
	
	private:
		hardware_resource_t * _hRes;

	public:
		static void InterruptHandler(Encoder::hardware_resource_t * encoder);
};
	
	
#endif
