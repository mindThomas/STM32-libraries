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
 
#pragma once

#include <IO/IO.hpp>

class QuadratureKnob
{
	public:
		QuadratureKnob(GPIO_TypeDef * GPIOx_A, uint32_t GPIO_Pin_A, GPIO_TypeDef * GPIOx_B, uint32_t GPIO_Pin_B);
		~QuadratureKnob();

		int32_t Get();

	public:
		IO * sigA;
		IO * sigB;

		int32_t value;
		uint8_t oldAB;

	public:
		static void InterruptHandler(void * params);
	
	private:
		static const int8_t EncStates[16];
};
