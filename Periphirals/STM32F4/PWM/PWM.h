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
 
#ifndef PERIPHIRALS_PWM_H
#define PERIPHIRALS_PWM_H

#include "stm32f4xx_hal.h"

class PWM
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER1,
			TIMER8,
			TIMER9,
			TIMER12,
		} timer_t;

		typedef enum pwm_channel_t {
			CH1 = 1,
			CH2 = 2,
			CH3 = 4,
			CH4 = 8
		} pwm_channel_t;

	public:
		PWM(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue, bool invert = false);
		PWM(timer_t timer, pwm_channel_t channel, bool invert);
		PWM(timer_t timer, pwm_channel_t channel);
		~PWM();

		void InitPeripheral(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue, bool invert);
		void ConfigureTimerPeripheral();
		void ConfigureTimerGPIO();
		void ConfigureTimerChannel(bool invert);

		void Set(float value);
		void SetRaw(uint16_t value);

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			uint32_t frequency;
			uint16_t maxValue;
			uint8_t configuredChannels; // each bit indicate whether the corresponding channel is configured and in use by another object
			TIM_HandleTypeDef handle;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER1;
		static hardware_resource_t * resTIMER8;
		static hardware_resource_t * resTIMER9;
		static hardware_resource_t * resTIMER12;

	private:
		hardware_resource_t * _hRes;
		pwm_channel_t _channel;
		bool _complementaryOutput; // Channel xN ?
		uint32_t _channelHAL;
};
	
	
#endif
