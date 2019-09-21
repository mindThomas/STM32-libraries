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
 
#ifndef DEVICES_WS2811_H
#define DEVICES_WS2811_H

#include "stm32h7xx_hal.h"

class WS2811
{
	private:
		// Signal timing according to WS2811 datasheet
		const uint16_t t0_High_ns = 250;
		const uint16_t t0_Low_ns = 1000;
		const uint16_t t1_High_ns = 600;
		const uint16_t t1_Low_ns = 650;
		const uint16_t tPeriod_ns = 1250;

		const uint8_t DUTY_MAX = 50;
		const uint8_t DUTY0 = DUTY_MAX * t0_High_ns / (t0_High_ns + t0_Low_ns);
		const uint8_t DUTY1 = DUTY_MAX * t1_High_ns / (t1_High_ns + t1_Low_ns);

	public:
		WS2811();
		~WS2811();

		void InterruptHandler(void);

		void SetColor(uint8_t R, uint8_t G, uint8_t B);

	private:
		void ConfigureTimerPeripheral(uint32_t frequency, uint16_t maxValue);
		void ConfigureTimerGPIO(void);
		void ConfigureTimerChannel(void);
		void ConfigureDMA(void);


	private:
		TIM_HandleTypeDef _handle;
		DMA_HandleTypeDef _DMA_handle;
		ALIGN_32BYTES( uint32_t _ccBuffer[25]);  // 32-bytes Alignement is needed for cache maintenance purpose
												// And 24 bits (values) needed for one single LED

	public:
		static WS2811 * Handle; // library currently only developed for a single device, whose handle/pointer is stored in this variable

};
	
	
#endif
