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

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_tim.h"

#define TIM_SET_COMPARE(__INSTANCE__, __CHANNEL__, __COMPARE__) \
(*(__IO uint32_t *)(&((__INSTANCE__)->CCR1) + ((__CHANNEL__) >> 2U)) = (__COMPARE__))   // modified from stm32f0xx_hal_tim.h

class PWM
{
	public:
		typedef enum timer_t {
			TIMER_UNDEFINED = 0,
			TIMER1,
			TIMER2,
			TIMER3,
			TIMER14,
			TIMER16,
			TIMER17
		} timer_t;

		typedef enum pwm_channel_t {
			CH1 = 1,
			CH2 = 2,
			CH3 = 4,
			CH4 = 8
		} pwm_channel_t;

		typedef enum pwm_hal_channel_t { // borrowed from stm32f0xx_hal_tim.h
			HAL_CH1 = 0x00,
			HAL_CH2 = 0x04,
			HAL_CH3 = 0x08,
			HAL_CH4 = 0x0C
		} pwm_hal_channel_t;

	public:
		PWM(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue);
		PWM(timer_t timer, pwm_channel_t channel);
		~PWM();

		void InitPeripheral(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue);
		void ConfigureTimerPeripheral();
		void ConfigureTimerGPIO();
		void ConfigureTimerChannel();

		void Set(float value);
		void SetRaw(uint16_t value);

	public:
		typedef struct hardware_resource_t {
			timer_t timer;
			uint32_t frequency;
			uint16_t maxValue;
			uint8_t configuredChannels; // each bit indicate whether the corresponding channel is configured and in use by another object
			TIM_TypeDef * instance;
		} hardware_resource_t;

		static hardware_resource_t * resTIMER1;
		static hardware_resource_t * resTIMER2;
		static hardware_resource_t * resTIMER3;
		static hardware_resource_t * resTIMER14;
		static hardware_resource_t * resTIMER16;
		static hardware_resource_t * resTIMER17;

	private:
		hardware_resource_t * _hRes;
		pwm_channel_t _channel;
		uint32_t _channelLL;
		uint8_t _channelHAL;
};
	
	
#endif
