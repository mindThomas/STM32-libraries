/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"

class Encoder
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER4,
		} timer_t;

	public:
		Encoder(timer_t timer, bool invertDirection = false);
		~Encoder();

		void ConfigureEncoderGPIO();
		void ConfigureEncoderPeripheral();

		int32_t Get();

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			TIM_HandleTypeDef handle;
			int32_t offsetValue;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER4;
	
	private:
		hardware_resource_t * _hRes;
		bool _invertDirection;

	public:
		static void InterruptHandler(Encoder::hardware_resource_t * encoder);
};
	
	
#endif
