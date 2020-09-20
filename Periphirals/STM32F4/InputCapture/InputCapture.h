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
 
#ifndef PERIPHIRALS_INPUTCAPTURE_H
#define PERIPHIRALS_INPUTCAPTURE_H

#include "stm32f4xx_hal.h"

class InputCapture
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER4
		} timer_t;

		typedef enum ic_channel_t {
			CH1 = 1,
			CH2 = 2,
			CH3 = 4,
			CH4 = 8
		} ic_channel_t;

	public:
		InputCapture(timer_t timer, ic_channel_t channel, float maxTime);
		InputCapture(timer_t timer, ic_channel_t channel);
		~InputCapture();

		void InitPeripheral(timer_t timer, ic_channel_t channel, float maxTime);
		void ConfigureTimerPeripheral();
		void ConfigureTimerGPIO();
		void ConfigureTimerChannel();

		float GetLowTime(void);
		float GetHighTime(void);
		float GetPeriodTime(void);
		void Clear(void);

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			float maxTime;
			float tickTime;
			uint8_t configuredChannels; // each bit indicate whether the corresponding channel is configured and in use by another object
			TIM_HandleTypeDef handle;
			int32_t inputCaptures_low[4]; // one value for each possible channel (up to 4 channels)
			int32_t inputCaptures_high[4]; // one value for each possible channel (up to 4 channels)
			int32_t inputCaptures[4]; // one value for each possible channel (up to 4 channels)
			GPIO_TypeDef * channelPorts[4];
			uint32_t channelPins[4];
		} hardware_resource_t;

		static hardware_resource_t * resTIMER4;
	
	private:
		hardware_resource_t * _hRes;
		ic_channel_t _channel;
		uint32_t _channelHAL;

	public:
		static void InterruptHandler(InputCapture::hardware_resource_t * timer);
};
	
	
#endif
