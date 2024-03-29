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
 
#include "PWM.h"
#include "Debug.h"
#include <string.h> // for memset
#include <math.h> // for roundf
 
PWM::hardware_resource_t * PWM::resTIMER1 = 0;
PWM::hardware_resource_t * PWM::resTIMER8 = 0;
PWM::hardware_resource_t * PWM::resTIMER9 = 0;
PWM::hardware_resource_t * PWM::resTIMER12 = 0;

PWM::PWM(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue, bool invert) : _channel(channel), _channelHAL(0)
{
	InitPeripheral(timer, channel, frequency, maxValue, invert);
}

PWM::PWM(timer_t timer, pwm_channel_t channel) : PWM(timer, channel, 0, 0, false)
{
}

PWM::PWM(timer_t timer, pwm_channel_t channel, bool invert) : PWM(timer, channel, 0, 0, invert)
{
}

PWM::~PWM()
{
	if (!_hRes) return;
	_hRes->configuredChannels &= !_channel;

	// Stop channel
	if (HAL_TIM_PWM_Stop(&_hRes->handle, _channelHAL) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not stop PWM channel");
		return;
	}

	// Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

	if (_hRes->configuredChannels == 0) { // no more channels in use in resource, so delete the resource
		// Delete hardware resource
		timer_t tmpTimer = _hRes->timer;
		delete(_hRes);

		switch (tmpTimer)
		{
			case TIMER1:
				__HAL_RCC_TIM1_CLK_DISABLE();
				resTIMER1 = 0;
				break;
			case TIMER8:
				__HAL_RCC_TIM8_CLK_DISABLE();
				resTIMER8 = 0;
				break;
			case TIMER9:
				__HAL_RCC_TIM9_CLK_DISABLE();
				resTIMER9 = 0;
				break;
			case TIMER12:
				__HAL_RCC_TIM12_CLK_DISABLE();
				resTIMER12 = 0;
				break;
			default:
				ERROR("Undefined timer");
				return;
		}
	}
}

void PWM::InitPeripheral(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue, bool invert)
{
	bool configureResource = false;

	_hRes = 0;

	switch (timer)
	{
		case TIMER1:
			if (!resTIMER1) {
				if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
					resTIMER1 = new PWM::hardware_resource_t;
					memset(resTIMER1, 0, sizeof(PWM::hardware_resource_t));
					configureResource = true;
					_hRes = resTIMER1;
				}
			}
			else {
				_hRes = resTIMER1;
			}
			break;
		case TIMER8:
			if (!resTIMER8) {
				if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
					resTIMER8 = new PWM::hardware_resource_t;
					memset(resTIMER8, 0, sizeof(PWM::hardware_resource_t));
					configureResource = true;
					_hRes = resTIMER8;
				}
			}
			else {
				_hRes = resTIMER8;
			}
			break;
		case TIMER9:
			if (!resTIMER9) {
				if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
					resTIMER9 = new PWM::hardware_resource_t;
					memset(resTIMER9, 0, sizeof(PWM::hardware_resource_t));
					configureResource = true;
					_hRes = resTIMER9;
				}
			}
			else {
				_hRes = resTIMER9;
			}
			break;
		case TIMER12:
			if (!resTIMER12) {
				if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
					resTIMER12 = new PWM::hardware_resource_t;
					memset(resTIMER12, 0, sizeof(PWM::hardware_resource_t));
					configureResource = true;
					_hRes = resTIMER12;
				}
			}
			else {
				_hRes = resTIMER12;
			}
			break;
		default:
			ERROR("Undefined timer");
			return;
	}

	if (configureResource) { // first time configuring peripheral
		_hRes->timer = timer;
		_hRes->frequency = frequency;
		_hRes->maxValue = maxValue;
		_hRes->configuredChannels = 0;

		if (frequency == 0 || maxValue == 0)
		{
			_hRes = 0;
			ERROR("Invalid timer frequency and/or maxValue");
			return;
		}

		ConfigureTimerPeripheral();
	}

	// Ensure that frequency and maxValue matches
	if (!(frequency == 0 && maxValue == 0) && // frequency and maxValue undefined
		(frequency != _hRes->frequency || maxValue != _hRes->maxValue))
	{
		_hRes = 0;
		ERROR("Timer already configured with different frequency and/or max value");
		return;
	}

	// Ensure that the channel is valid and not already in use
	if ((channel % 2) != 0 && channel != 1) {
		_hRes = 0;
		ERROR("Only one timer channel can be configured per object");
		return;
	}
	if (((timer == TIMER1 || timer == TIMER8) && channel > CH4) || // channel 1-4
		((timer == TIMER9 || timer == TIMER12) && channel > CH2)) // only channel 1-2
	{
		_hRes = 0;
		ERROR("Invalid channel for selected timer");
		return;
	}
	if ((_hRes->configuredChannels & (uint8_t)channel) != 0) {
		_hRes = 0;
		ERROR("Channel already configured on selected timer");
		return;
	}

	ConfigureTimerGPIO();
	ConfigureTimerChannel(invert);
}

void PWM::ConfigureTimerPeripheral()
{
	if (!_hRes) return;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
	uint32_t TimerClock = 0;

	if (_hRes->timer == TIMER1) {
		__HAL_RCC_TIM1_CLK_ENABLE();
		_hRes->handle.Instance = TIM1;
		TimerClock = 2*HAL_RCC_GetPCLK2Freq(); // factor 2 due to STM32F4 architecture
	} else if (_hRes->timer == TIMER8) {
		__HAL_RCC_TIM8_CLK_ENABLE();
		_hRes->handle.Instance = TIM8;
		TimerClock = 2*HAL_RCC_GetPCLK2Freq(); // factor 2 due to STM32F4 architecture
	} else if (_hRes->timer == TIMER9) {
		__HAL_RCC_TIM9_CLK_ENABLE();
		_hRes->handle.Instance = TIM9;
		TimerClock = 2*HAL_RCC_GetPCLK2Freq(); // factor 2 due to STM32F4 architecture
	} else if (_hRes->timer == TIMER12) {
		__HAL_RCC_TIM12_CLK_ENABLE();
		_hRes->handle.Instance = TIM12;
		TimerClock = 2*HAL_RCC_GetPCLK1Freq(); // factor 2 due to STM32F4 architecture
	}

	_hRes->handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	_hRes->handle.Init.RepetitionCounter = 0;
	_hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	_hRes->handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	_hRes->handle.Init.Period = _hRes->maxValue - 1; // this will allow duty cycles (CCR register) to go from 0 to maxValue, with maxValue giving 100% duty cycle (fully on)

	// Configure timer prescaler based on desired frequency
	//   fCNT = (ARR+1) * fPERIOD
	//   PSC = (fTIM / fCNT) - 1
	// Added prescaler computation as float such that rounding can happen
	float prescaler = ((float)TimerClock / ((_hRes->handle.Init.Period+1) * _hRes->frequency)) - 1;
	_hRes->handle.Init.Prescaler = roundf(prescaler);

	if (_hRes->handle.Init.Prescaler > 0xFFFF) {
		_hRes = 0;
		ERROR("Timer frequency too slow");
		return;
	}

	if (HAL_TIM_Base_Init(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not initialize timer");
		return;
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&_hRes->handle, &sClockSourceConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure timer clock source");
		return;
	}

	if (HAL_TIM_PWM_Init(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not initialize PWM for timer");
		return;
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&_hRes->handle, &sMasterConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure timer");
		return;
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&_hRes->handle, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure timer break-dead time");
		return;
	}
}

void PWM::ConfigureTimerGPIO()
{
	if (!_hRes) return;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if(_hRes->timer == TIMER1)
	{
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
		/**TIM1 GPIO Configuration
		PA8      ------> TIM1_CH1
		*/
		__HAL_RCC_GPIOA_CLK_ENABLE();
		if (_channel == CH1)
			GPIO_InitStruct.Pin = GPIO_PIN_8;
		else
			return;

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		_complementaryOutput = false;
	}
	else if(_hRes->timer == TIMER8)
	{
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		/**TIM8 GPIO Configuration
		PC6     ------> TIM8_CH1
		PC7     ------> TIM8_CH2
		PC8     ------> TIM8_CH3
		*/
		__HAL_RCC_GPIOC_CLK_ENABLE();
		if (_channel == CH1)
			GPIO_InitStruct.Pin = GPIO_PIN_6;
		else if (_channel == CH2)
			GPIO_InitStruct.Pin = GPIO_PIN_7;
		else if (_channel == CH3)
			GPIO_InitStruct.Pin = GPIO_PIN_8;
		else
			return;

		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		_complementaryOutput = false;
	}
	else if(_hRes->timer == TIMER9)
	{
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
		/**TIM8 GPIO Configuration
		PA2     ------> TIM9_CH1
		PA3     ------> TIM9_CH2
		*/
		__HAL_RCC_GPIOA_CLK_ENABLE();
		if (_channel == CH1)
			GPIO_InitStruct.Pin = GPIO_PIN_2;
		else if (_channel == CH2)
			GPIO_InitStruct.Pin = GPIO_PIN_3;
		else
			return;

		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		_complementaryOutput = false;
	}
	else if(_hRes->timer == TIMER12)
	{
		GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
		/**TIM17 GPIO Configuration
		PB14     ------> TIM12_CH1
		*/
		__HAL_RCC_GPIOB_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_14;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		_complementaryOutput = true;
	}
}

void PWM::ConfigureTimerChannel(bool invert)
{
	if (!_hRes) return;

	TIM_OC_InitTypeDef sConfigOC = {0};

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = invert ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if (_channel == CH1)
		_channelHAL = TIM_CHANNEL_1;
	else if (_channel == CH2)
		_channelHAL = TIM_CHANNEL_2;
	else if (_channel == CH3)
		_channelHAL = TIM_CHANNEL_3;
	else if (_channel == CH4)
		_channelHAL = TIM_CHANNEL_4;
	else
		return;

	// Configure channel
	if (HAL_TIM_PWM_ConfigChannel(&_hRes->handle, &sConfigOC, _channelHAL) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure PWM channel");
		return;
	}

	// Start channel
	if (!_complementaryOutput) {
		if (HAL_TIM_PWM_Start(&_hRes->handle, _channelHAL) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not start PWM channel");
			return;
		}
	} else {
		if (HAL_TIMEx_PWMN_Start(&_hRes->handle, _channelHAL) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not start PWM channel");
			return;
		}
	}
	_hRes->configuredChannels |= _channel;
}

// Set a duty-cycle value between 0-1, where 0 results in an always LOW signal and 1 results in an always HIGH signal
void PWM::Set(float value)
{
	if (value < 0) value = 0.0f;
	if (value > 1) value = 1.0f;

	uint16_t rawValue = _hRes->maxValue * value;
	SetRaw(rawValue);
}

void PWM::SetRaw(uint16_t value)
{
	if (!_hRes) return;
	if (value > _hRes->maxValue) value = _hRes->maxValue;

	if (!_complementaryOutput)
		__HAL_TIM_SET_COMPARE(&_hRes->handle, _channelHAL, value);
	else
		__HAL_TIM_SET_COMPARE(&_hRes->handle, _channelHAL, _hRes->maxValue-value);
}
