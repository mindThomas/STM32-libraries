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
 
#include "Timer.h"
#include "stm32f4xx_hal.h"
#include "Priorities.h"
#include "Debug.h"
#include <string.h> // for memset
#include <cmath>

Timer::hardware_resource_t * Timer::resTIMER11 = 0;
Timer::hardware_resource_t * Timer::resTIMER13 = 0;
Timer::hardware_resource_t * Timer::resTIMER14 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void TIM1_TRG_COM_TIM11_IRQHandler(void);
extern "C" __EXPORT void TIM8_UP_TIM13_IRQHandler(void);
extern "C" __EXPORT void TIM8_TRG_COM_TIM14_IRQHandler(void);

Timer::Timer(timer_t timer, uint32_t frequency) : _TimerCallbackSoft(0), _waitSemaphore(0)
{
	if (timer == TIMER11 && !resTIMER11) {
		resTIMER11 = new Timer::hardware_resource_t;
		memset(resTIMER11, 0, sizeof(Timer::hardware_resource_t));
		_hRes = resTIMER11;
	}
	else if (timer == TIMER13 && !resTIMER13) {
		resTIMER13 = new Timer::hardware_resource_t;
		memset(resTIMER13, 0, sizeof(Timer::hardware_resource_t));
		_hRes = resTIMER13;
	}
	else if (timer == TIMER14 && !resTIMER14) {
		resTIMER14 = new Timer::hardware_resource_t;
		memset(resTIMER14, 0, sizeof(Timer::hardware_resource_t));
		_hRes = resTIMER14;
	}
	else {
		_hRes = 0;
		ERROR("Undefined timer or timer already in use");
		return;
	}

	_hRes->timer = timer;
	_hRes->frequency = frequency;
	_hRes->maxValue = TIMER_DEFAULT_MAXVALUE;
	_hRes->TimerCallback = 0;
	_hRes->callbackTaskHandle = 0;
	_hRes->callbackSemaphore = 0;

	ConfigureTimerPeripheral();
}

Timer::~Timer()
{
	if (!_hRes) return;

	// Stop Encoder interface
	if (HAL_TIM_Base_Stop(&_hRes->handle) != HAL_OK) {
		ERROR("Could not stop encoder");
	}

	__HAL_TIM_DISABLE_IT(&_hRes->handle, TIM_IT_UPDATE);

	timer_t tmpTimer = _hRes->timer;
	delete(_hRes);

	if (tmpTimer == TIMER11) {
		__HAL_RCC_TIM11_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM11_IRQn);
		resTIMER11 = 0;
	}
	else if (tmpTimer == TIMER13) {
		__HAL_RCC_TIM13_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
		resTIMER13 = 0;
	}
	else if (tmpTimer == TIMER14) {
		__HAL_RCC_TIM14_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		resTIMER14 = 0;
	}
	else {
		ERROR("Undefined timer");
		return;
	}
}

void Timer::ConfigureTimerPeripheral()
{
	if (!_hRes) return;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	uint32_t TimerClock = 0;

	if (_hRes->timer == TIMER11) {
		__HAL_RCC_TIM11_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
		_hRes->handle.Instance = TIM11;
		TimerClock = 2*HAL_RCC_GetPCLK2Freq(); // factor 2 due to STM32F4 architecture
	} else if (_hRes->timer == TIMER13) {
		__HAL_RCC_TIM13_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
		_hRes->handle.Instance = TIM3;
		TimerClock = 2*HAL_RCC_GetPCLK1Freq(); // factor 2 due to STM32F4 architecture
	} else if (_hRes->timer == TIMER14) {
		__HAL_RCC_TIM14_CLK_ENABLE();
		HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		_hRes->handle.Instance = TIM14;
		TimerClock = 2*HAL_RCC_GetPCLK1Freq(); // factor 2 due to STM32F4 architecture
	}

	_hRes->handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	_hRes->handle.Init.RepetitionCounter = 0;
	_hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	_hRes->handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	_hRes->handle.Init.Period = _hRes->maxValue;

	// Configure timer prescaler based on desired frequency
	//   fCNT = (ARR+1) * fPERIOD
	//   PSC = (fTIM / fCNT) - 1
	_hRes->handle.Init.Prescaler = (TimerClock / _hRes->frequency) - 1;

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

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&_hRes->handle, &sMasterConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure timer");
		return;
	}

	if (HAL_TIM_Base_Start(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not start timer");
		return;
	}

	// Enable interrupt
	__HAL_TIM_ENABLE_IT(&_hRes->handle, TIM_IT_UPDATE);
}

void Timer::SetMaxValue(uint16_t maxValue)
{
	if (!_hRes) return;
	_hRes->maxValue = maxValue;
	__HAL_TIM_SET_AUTORELOAD(&_hRes->handle, maxValue);
}

uint32_t Timer::Get()
{
	if (!_hRes) return 0;
	return (uint32_t)__HAL_TIM_GET_COUNTER(&_hRes->handle) + _hRes->counterOffset;
}

float Timer::GetTime()
{
	return (float)Get() / (float)_hRes->frequency;
}

void Timer::Reset()
{
	if (_hRes->callbackSemaphore)
		xQueueReset(_hRes->callbackSemaphore);
	__HAL_TIM_SET_COUNTER(&_hRes->handle, 0);
	_hRes->counterOffset = 0;
}

void Timer::Wait(uint32_t MicrosToWait)
{
	if (!_hRes) return;

	if (_hRes->TimerCallback || _TimerCallbackSoft || _hRes->callbackSemaphore != _waitSemaphore) {
		ERROR("Timer interrupt already registered elsewhere");
		return;
	}

	if (!_waitSemaphore) {
		_waitSemaphore = xSemaphoreCreateBinary();
		_hRes->callbackSemaphore = _waitSemaphore;
	}

	float MicrosTimerCountPeriod = 1000000.0f / _hRes->frequency;
	uint16_t CountsToWait = ceilf((float)MicrosToWait / MicrosTimerCountPeriod) - 1;
	Reset();
	SetMaxValue(CountsToWait);

	xSemaphoreTake( _waitSemaphore, ( TickType_t ) portMAX_DELAY );
}

/**
 * @brief 	Return delta time in seconds between now and a previous timer value
 * @param	prevTimerValue  	Previous timer value
 * @return	float				Delta time in seconds
 */
float Timer::GetDeltaTime(uint32_t prevTimerValue)
{
	if (!_hRes) return -1;

	/*uint16_t timerNow = Get();
	uint16_t timerDelta = ((int32_t)timerNow - (int32_t)prevTimerValue) % (_hRes->maxValue + 1);*/
	uint32_t timerDelta;
	uint32_t timerNow = Get();
	if (timerNow > prevTimerValue)
		timerDelta = timerNow - prevTimerValue;
	else
		timerDelta = ((uint32_t)0xFFFFFFFF - prevTimerValue) + timerNow;

	float microsTime = (float)timerDelta / (float)_hRes->frequency;
	return microsTime;
}

void Timer::RegisterInterruptSoft(uint32_t frequency, void (*TimerCallbackSoft)()) // note that the frequency should be a multiple of the configured timer count frequency
{
	if (!_hRes) return;

	uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
	SetMaxValue(interruptValue);

	_TimerCallbackSoft = TimerCallbackSoft;
	xTaskCreate(Timer::CallbackThread, (char *)"Timer callback", 128, (void*) this, 3, &_hRes->callbackTaskHandle);
}

void Timer::RegisterInterrupt(uint32_t frequency, void (*TimerCallback)()) // note that the frequency should be a multiple of the configured timer count frequency
{
	if (!_hRes) return;

	uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
	SetMaxValue(interruptValue);

	_hRes->TimerCallback = TimerCallback;
}

void Timer::RegisterInterrupt(uint32_t frequency, SemaphoreHandle_t semaphore)
{
	if (!_hRes) return;
	if (!semaphore) return;

	uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
	SetMaxValue(interruptValue);

	_hRes->callbackSemaphore = semaphore;
}

void Timer::CallbackThread(void * pvParameters)
{
	Timer * timer = (Timer *)pvParameters;

	while (1) {
		vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

		if (timer->_TimerCallbackSoft)
			timer->_TimerCallbackSoft();
	}
}

void Timer::InterruptHandler(Timer::hardware_resource_t * timer)
{
	/* TIM Update event */
	if(__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_UPDATE) != RESET)
	{
		if(__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_UPDATE) !=RESET)
		{
			__HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_UPDATE);

			timer->counterOffset += (timer->maxValue + 1);

			if (timer->callbackSemaphore) {
				portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
				//xSemaphoreGiveFromISR( timer->callbackSemaphore, &xHigherPriorityTaskWoken );
				xQueueSendFromISR(timer->callbackSemaphore, NULL, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			}

			if (timer->TimerCallback)
				timer->TimerCallback();

			if (timer->callbackTaskHandle) {
				portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(timer->callbackTaskHandle);
				portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
			}
		}
	}
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if (Timer::resTIMER11)
		Timer::InterruptHandler(Timer::resTIMER11); //HAL_TIM_IRQHandler(&htim11);
	else
		TIM11->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_TRIGGER | TIM_IT_BREAK); // clear all interrupts
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	if (Timer::resTIMER13)
		Timer::InterruptHandler(Timer::resTIMER13); //HAL_TIM_IRQHandler(&htim13);
	else
		TIM13->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_TRIGGER | TIM_IT_BREAK); // clear all interrupts
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	if (Timer::resTIMER14)
		Timer::InterruptHandler(Timer::resTIMER14); //HAL_TIM_IRQHandler(&htim14);
	else
		TIM14->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM | TIM_IT_TRIGGER | TIM_IT_BREAK); // clear all interrupts
}
