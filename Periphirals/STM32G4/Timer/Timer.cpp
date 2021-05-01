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

#include "Timer.hpp"
#include <Debug/Debug.h>
#include "Priorities.h"
#include "stm32g4xx_hal.h"
#include <cmath>
#include <string.h> // for memset

Timer::hardware_resource_t* Timer::resTIMER6  = 0;
Timer::hardware_resource_t* Timer::resTIMER7  = 0;
Timer::hardware_resource_t* Timer::resTIMER8  = 0;
Timer::hardware_resource_t* Timer::resTIMER17 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void TIM6_DAC_IRQHandler(void);
extern "C" void TIM7_IRQHandler(void);
extern "C" void TIM8_UP_IRQHandler(void);
extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void);

Timer::Timer(timer_t timer, uint32_t frequency)
    : _TimerCallbackSoft(0)
    , _waitSemaphore(0)
{
    if (timer == TIMER6 && !resTIMER6) {
        resTIMER6 = new Timer::hardware_resource_t;
        memset(resTIMER6, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER6;
    } else if (timer == TIMER7 && !resTIMER7) {
        resTIMER7 = new Timer::hardware_resource_t;
        memset(resTIMER7, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER7;
    } else if (timer == TIMER8 && !resTIMER8) {
        resTIMER8 = new Timer::hardware_resource_t;
        memset(resTIMER8, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER8;
    } else if (timer == TIMER17 && !resTIMER17) {
        resTIMER17 = new Timer::hardware_resource_t;
        memset(resTIMER17, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER17;
    } else {
        _hRes = 0;
        ERROR("Undefined timer or timer already in use");
        return;
    }

    _hRes->timer              = timer;
    _hRes->frequency          = frequency;
    _hRes->maxValue           = TIMER_DEFAULT_MAXVALUE;
    _hRes->TimerCallback      = 0;
    _hRes->callbackTaskHandle = 0;
    _hRes->callbackSemaphore  = 0;

    ConfigureTimerPeripheral();
}

Timer::~Timer()
{
    if (!_hRes)
        return;

    // Stop Encoder interface
    if (HAL_TIM_Base_Stop(&_hRes->handle) != HAL_OK) {
        ERROR("Could not stop encoder");
    }

    __HAL_TIM_DISABLE_IT(&_hRes->handle, TIM_IT_UPDATE);

    timer_t tmpTimer = _hRes->timer;
    delete (_hRes);

    if (tmpTimer == TIMER6) {
        __HAL_RCC_TIM6_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
        resTIMER6 = 0;
    } else if (tmpTimer == TIMER7) {
        __HAL_RCC_TIM7_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM7_IRQn);
        resTIMER7 = 0;
    } else if (tmpTimer == TIMER8) {
        __HAL_RCC_TIM8_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM8_UP_IRQn);
        resTIMER8 = 0;
    } else if (tmpTimer == TIMER17) {
        __HAL_RCC_TIM17_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
        resTIMER17 = 0;
    } else {
        ERROR("Undefined timer");
        return;
    }
}

void Timer::ConfigureTimerPeripheral()
{
    if (!_hRes)
        return;

    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};
    uint32_t                TimerClock         = 0;

    if (_hRes->timer == TIMER6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
        _hRes->handle.Instance = TIM6;
        TimerClock             = HAL_RCC_GetPCLK1Freq();
    } else if (_hRes->timer == TIMER7) {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        _hRes->handle.Instance = TIM7;
        TimerClock             = HAL_RCC_GetPCLK1Freq();
    } else if (_hRes->timer == TIMER8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM8_UP_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
        _hRes->handle.Instance = TIM8;
        TimerClock             = HAL_RCC_GetPCLK2Freq();
    } else if (_hRes->timer == TIMER17) {
        __HAL_RCC_TIM17_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, TIMER_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
        _hRes->handle.Instance = TIM17;
        TimerClock             = HAL_RCC_GetPCLK2Freq();
    }

    _hRes->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    _hRes->handle.Init.RepetitionCounter = 0;
    _hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    _hRes->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    _hRes->handle.Init.Period            = _hRes->maxValue;

    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    _hRes->handle.Init.Prescaler = (TimerClock / _hRes->frequency) - 1;

    if (_hRes->handle.Init.Prescaler > 0xFFFF) {
        _hRes = 0;
        ERROR("Timer frequency too slow");
        return;
    }

    if (HAL_TIM_Base_Init(&_hRes->handle) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not initialize timer");
        return;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&_hRes->handle, &sClockSourceConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure timer clock source");
        return;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&_hRes->handle, &sMasterConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure timer");
        return;
    }

    if (HAL_TIM_Base_Start(&_hRes->handle) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not start timer");
        return;
    }

    // Enable interrupt
    __HAL_TIM_ENABLE_IT(&_hRes->handle, TIM_IT_UPDATE);
}

void Timer::SetMaxValue(uint16_t maxValue)
{
    if (!_hRes)
        return;
    _hRes->maxValue = maxValue;
    __HAL_TIM_SET_AUTORELOAD(&_hRes->handle, maxValue);
}

uint32_t Timer::Get()
{
    if (!_hRes)
        return 0;
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
    if (!_hRes)
        return;

    if (_hRes->TimerCallback || _TimerCallbackSoft || _hRes->callbackSemaphore != _waitSemaphore) {
        ERROR("Timer interrupt already registered elsewhere");
        return;
    }

    if (!_waitSemaphore) {
        _waitSemaphore           = xSemaphoreCreateBinary();
        _hRes->callbackSemaphore = _waitSemaphore;
    }

    float    MicrosTimerCountPeriod = 1000000.0f / _hRes->frequency;
    uint16_t CountsToWait           = ceilf((float)MicrosToWait / MicrosTimerCountPeriod) - 1;
    Reset();
    SetMaxValue(CountsToWait);

    xSemaphoreTake(_waitSemaphore, (TickType_t)portMAX_DELAY);
}

/**
 * @brief 	Return delta time in seconds between now and a previous timer value
 * @param	prevTimerValue  	Previous timer value
 * @return	float				Delta time in seconds
 */
float Timer::GetDeltaTime(uint32_t prevTimerValue)
{
    if (!_hRes)
        return -1;

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

void Timer::RegisterInterruptSoft(
  uint32_t frequency, void (*TimerCallbackSoft)(void* param),
  void*    parameter) // note that the frequency should be a multiple of the configured timer count frequency
{
    if (!_hRes)
        return;

    uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
    SetMaxValue(interruptValue);

    _TimerCallbackSoft          = TimerCallbackSoft;
    _TimerCallbackSoftParameter = parameter;
    if (!_hRes->callbackTaskHandle)
        xTaskCreate(Timer::CallbackThread, (char*)"Timer callback", configMINIMAL_STACK_SIZE, (void*)this,
                    TIMER_SOFT_CALLBACK_PRIORITY, &_hRes->callbackTaskHandle);
}

void Timer::RegisterInterrupt(
  uint32_t frequency, void (*TimerCallback)(void* param),
  void*    parameter) // note that the frequency should be a multiple of the configured timer count frequency
{
    if (!_hRes)
        return;

    uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
    SetMaxValue(interruptValue);

    _hRes->TimerCallback          = TimerCallback;
    _hRes->TimerCallbackParameter = parameter;
}

void Timer::RegisterInterrupt(uint32_t frequency, SemaphoreHandle_t semaphore)
{
    if (!_hRes)
        return;
    if (!semaphore)
        return;

    uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
    SetMaxValue(interruptValue);

    _hRes->callbackSemaphore = semaphore;
}

void Timer::CallbackThread(void* pvParameters)
{
    Timer* timer = (Timer*)pvParameters;

    while (1) {
        vTaskSuspend(
          NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

        if (timer->_TimerCallbackSoft)
            timer->_TimerCallbackSoft(timer->_TimerCallbackSoftParameter);
    }
}

void Timer::InterruptHandler(Timer::hardware_resource_t* timer)
{
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_UPDATE);

            timer->counterOffset += (timer->maxValue + 1);

            if (timer->callbackSemaphore) {
                portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
                // xSemaphoreGiveFromISR( timer->callbackSemaphore, &xHigherPriorityTaskWoken );
                xQueueSendFromISR(timer->callbackSemaphore, NULL, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }

            if (timer->TimerCallback)
                timer->TimerCallback(timer->TimerCallbackParameter);

            if (timer->callbackTaskHandle) {
                portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(timer->callbackTaskHandle);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if (Timer::resTIMER6)
        Timer::InterruptHandler(Timer::resTIMER6); // HAL_TIM_IRQHandler(&htim6);
    else
        TIM6->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                               TIM_IT_TRIGGER | TIM_IT_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_IDX | TIM_FLAG_DIR |
                               TIM_FLAG_IERR | TIM_FLAG_TERR); // clear all interrupts
}

void TIM7_IRQHandler(void)
{
    if (Timer::resTIMER7)
        Timer::InterruptHandler(Timer::resTIMER7); // HAL_TIM_IRQHandler(&htim7);
    else
        TIM7->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                               TIM_IT_TRIGGER | TIM_IT_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_IDX | TIM_FLAG_DIR |
                               TIM_FLAG_IERR | TIM_FLAG_TERR); // clear all interrupts
}

void TIM8_UP_IRQHandler(void)
{
    if (Timer::resTIMER8)
        Timer::InterruptHandler(Timer::resTIMER8); // HAL_TIM_IRQHandler(&htim8);
    else
        TIM8->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                               TIM_IT_TRIGGER | TIM_IT_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_IDX | TIM_FLAG_DIR |
                               TIM_FLAG_IERR | TIM_FLAG_TERR); // clear all interrupts
}

void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
    if (Timer::resTIMER17)
        Timer::InterruptHandler(Timer::resTIMER17); // HAL_TIM_IRQHandler(&htim17);
    else
        TIM17->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                                TIM_IT_TRIGGER | TIM_IT_BREAK | TIM_FLAG_BREAK2 | TIM_FLAG_IDX | TIM_FLAG_DIR |
                                TIM_FLAG_IERR | TIM_FLAG_TERR); // clear all interrupts
}
