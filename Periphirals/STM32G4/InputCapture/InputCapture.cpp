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

/* Use input capture to measure time between rising and/or falling edges of an input */

#include "InputCapture.h"
#include "Debug.h"
#include "Priorities.h"
#include <math.h>   // for roundf
#include <string.h> // for memset

InputCapture::hardware_resource_t* InputCapture::resTIMER4 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void TIM4_IRQHandler(void);

InputCapture::InputCapture(timer_t timer, ic_channel_t channel, float maxTime)
    : _channel(channel)
    , _channelHAL(0)
{
    InitPeripheral(timer, channel, maxTime);
}

InputCapture::InputCapture(timer_t timer, ic_channel_t channel)
    : InputCapture(timer, channel, 0)
{}

InputCapture::~InputCapture()
{
    if (!_hRes)
        return;
    _hRes->configuredChannels &= !_channel;

    // Stop channel
    if (HAL_TIM_IC_Stop_IT(&_hRes->handle, _channelHAL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not stop Input Capture channel");
        return;
    }

    __HAL_TIM_DISABLE_IT(&_hRes->handle, TIM_IT_UPDATE);

    // Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

    if (_hRes->configuredChannels == 0) { // no more channels in use in resource, so delete the resource
        // Delete hardware resource
        timer_t tmpTimer = _hRes->timer;
        delete (_hRes);

        switch (tmpTimer) {
            case TIMER4:
                __HAL_RCC_TIM4_CLK_DISABLE();
                HAL_NVIC_DisableIRQ(TIM4_IRQn);
                resTIMER4 = 0;
                break;
            default:
                ERROR("Undefined timer");
                return;
        }
    }
}

void InputCapture::InitPeripheral(timer_t timer, ic_channel_t channel, float maxTime)
{
    bool configureResource = false;

    _hRes = 0;

    switch (timer) {
        case TIMER4:
            if (!resTIMER4) {
                if (maxTime > 0) { // only configure if frequency and maxTime is set
                    resTIMER4 = new InputCapture::hardware_resource_t;
                    memset(resTIMER4, 0, sizeof(InputCapture::hardware_resource_t));
                    configureResource = true;
                    _hRes             = resTIMER4;
                }
            } else {
                _hRes = resTIMER4;
            }
            break;
        default:
            ERROR("Undefined timer");
            return;
    }

    if (configureResource) { // first time configuring peripheral
        _hRes->timer              = timer;
        _hRes->maxTime            = maxTime;
        _hRes->configuredChannels = 0;

        if (maxTime == 0) {
            _hRes = 0;
            ERROR("Invalid timer maxTime");
            return;
        }

        ConfigureTimerPeripheral();
    }

    // Ensure that maxTime matches
    if (maxTime != 0 && // maxTime undefined
        maxTime != _hRes->maxTime) {
        _hRes = 0;
        ERROR("Timer already configured with different max time");
        return;
    }

    // Ensure that the channel is valid and not already in use
    if ((channel % 2) != 0 && channel != 1) {
        _hRes = 0;
        ERROR("Only one timer channel can be configured per object");
        return;
    }
    if (((timer == TIMER4) && (channel < CH3 || channel > CH4))) // channel 3-4
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
    ConfigureTimerChannel();
}

void InputCapture::ConfigureTimerPeripheral()
{
    if (!_hRes)
        return;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    uint32_t               TimerClock         = 0;

    if (_hRes->timer == TIMER4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM4_IRQn, INPUT_CAPTURE_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        _hRes->handle.Instance = TIM4;
        TimerClock             = 2 * HAL_RCC_GetPCLK1Freq(); // factor 2 due to STM32F4 architecture
    }

    _hRes->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    _hRes->handle.Init.RepetitionCounter = 0;
    _hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    _hRes->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    _hRes->handle.Init.Period            = 0xFFFF; // enable count to max value (16-bit timer)

    float tick_time = (float)_hRes->maxTime / (_hRes->handle.Init.Period + 1);
    // float fCNT = 1.0f / tick_time;
    // float fCNT = (float)(_hRes->handle.Init.Period + 1) / (float)_hRes->maxTime;
    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    // Added prescaler computation as float such that rounding can happen
    // float prescaler = ((float)TimerClock / fCNT) - 1;
    float prescaler              = ((float)TimerClock * tick_time) - 1;
    _hRes->handle.Init.Prescaler = roundf(prescaler);

    if (_hRes->handle.Init.Prescaler > 0xFFFF) {
        _hRes = 0;
        ERROR("Timer frequency too slow");
        return;
    }

    _hRes->tickTime = (_hRes->handle.Init.Prescaler + 1) / (float)TimerClock;

    if (HAL_TIM_IC_Init(&_hRes->handle) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not initialize timer");
        return;
    }

    // Enable interrupt
    __HAL_TIM_ENABLE_IT(&_hRes->handle, TIM_IT_UPDATE);
}

void InputCapture::ConfigureTimerGPIO()
{
    if (!_hRes)
        return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_PULLUP;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;

    if (_hRes->timer == TIMER4) {
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        /**TIM4 GPIO Configuration
        PB8      ------> TIM4_CH3
        PB9      ------> TIM4_CH4
        */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        if (_channel == CH3) {
            GPIO_InitStruct.Pin    = GPIO_PIN_8;
            _hRes->channelPorts[2] = GPIOB;
            _hRes->channelPins[2]  = GPIO_PIN_8;
        } else if (_channel == CH4) {
            GPIO_InitStruct.Pin    = GPIO_PIN_9;
            _hRes->channelPorts[3] = GPIOB;
            _hRes->channelPins[3]  = GPIO_PIN_9;
        } else
            return;

        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void InputCapture::ConfigureTimerChannel()
{
    if (!_hRes)
        return;

    TIM_IC_InitTypeDef sICConfig = {0};

    sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
    sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sICConfig.ICFilter    = 10;

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
    if (HAL_TIM_IC_ConfigChannel(&_hRes->handle, &sICConfig, _channelHAL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure Input Capture channel");
        return;
    }

    // Start channel
    if (HAL_TIM_IC_Start_IT(&_hRes->handle, _channelHAL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not start Input Capture channel");
        return;
    }
    _hRes->configuredChannels |= _channel;
}

float InputCapture::GetLowTime(void)
{
    uint32_t timerCount = 0;

    if (_channel == CH1)
        timerCount = _hRes->inputCaptures_low[0];
    else if (_channel == CH2)
        timerCount = _hRes->inputCaptures_low[1];
    else if (_channel == CH3)
        timerCount = _hRes->inputCaptures_low[2];
    else if (_channel == CH4)
        timerCount = _hRes->inputCaptures_low[3];

    return _hRes->tickTime * timerCount;
}

float InputCapture::GetHighTime(void)
{
    uint32_t timerCount = 0;

    if (_channel == CH1)
        timerCount = _hRes->inputCaptures_high[0];
    else if (_channel == CH2)
        timerCount = _hRes->inputCaptures_high[1];
    else if (_channel == CH3)
        timerCount = _hRes->inputCaptures_high[2];
    else if (_channel == CH4)
        timerCount = _hRes->inputCaptures_high[3];

    return _hRes->tickTime * timerCount;
}

float InputCapture::GetPeriodTime(void)
{
    uint32_t timerCount = 0;

    if (_channel == CH1) {
        timerCount = _hRes->inputCaptures_low[0];
        timerCount += _hRes->inputCaptures_high[0];
    } else if (_channel == CH2) {
        timerCount = _hRes->inputCaptures_low[1];
        timerCount += _hRes->inputCaptures_high[1];
    } else if (_channel == CH3) {
        timerCount = _hRes->inputCaptures_low[2];
        timerCount += _hRes->inputCaptures_high[2];
    } else if (_channel == CH4) {
        timerCount = _hRes->inputCaptures_low[3];
        timerCount += _hRes->inputCaptures_high[3];
    }

    return _hRes->tickTime * timerCount;
}

void InputCapture::InterruptHandler(InputCapture::hardware_resource_t* timer)
{
    /* Capture compare 1 event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_CC1) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_CC1) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_CC1);

            uint16_t ICTimerValue = HAL_TIM_ReadCapturedValue(&timer->handle, TIM_CHANNEL_1);
            timer->inputCaptures[0] += ICTimerValue;
            if ((timer->channelPorts[0]->IDR & timer->channelPins[0]) != (uint32_t)GPIO_PIN_RESET) {
                // Transition from Low to High
                // Hence the measured period has been during Low state
                timer->inputCaptures_low[0] = timer->inputCaptures[0];
            } else {
                // Transition from High to Low
                // Hence the measured period has been during High state
                timer->inputCaptures_high[0] = timer->inputCaptures[0];
            }
            timer->inputCaptures[0] = -(int32_t)ICTimerValue;
        }
    }

    /* Capture compare 2 event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_CC2) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_CC2) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_CC2);

            uint16_t ICTimerValue = HAL_TIM_ReadCapturedValue(&timer->handle, TIM_CHANNEL_2);
            timer->inputCaptures[1] += ICTimerValue;
            if ((timer->channelPorts[1]->IDR & timer->channelPins[1]) != (uint32_t)GPIO_PIN_RESET) {
                // Transition from Low to High
                // Hence the measured period has been during Low state
                timer->inputCaptures_low[1] = timer->inputCaptures[1];
            } else {
                // Transition from High to Low
                // Hence the measured period has been during High state
                timer->inputCaptures_high[1] = timer->inputCaptures[1];
            }
            timer->inputCaptures[1] = -(int32_t)ICTimerValue;
        }
    }

    /* Capture compare 3 event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_CC3) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_CC3) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_CC3);

            uint16_t ICTimerValue = HAL_TIM_ReadCapturedValue(&timer->handle, TIM_CHANNEL_3);
            timer->inputCaptures[2] += ICTimerValue;
            if ((timer->channelPorts[2]->IDR & timer->channelPins[2]) != (uint32_t)GPIO_PIN_RESET) {
                // Transition from Low to High
                // Hence the measured period has been during Low state
                timer->inputCaptures_low[2] = timer->inputCaptures[2];
            } else {
                // Transition from High to Low
                // Hence the measured period has been during High state
                timer->inputCaptures_high[2] = timer->inputCaptures[2];
            }
            timer->inputCaptures[2] = -(int32_t)ICTimerValue;
        }
    }

    /* Capture compare 4 event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_CC4) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_CC4) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_CC4);

            uint16_t ICTimerValue = HAL_TIM_ReadCapturedValue(&timer->handle, TIM_CHANNEL_4);
            timer->inputCaptures[3] += ICTimerValue;
            if ((timer->channelPorts[3]->IDR & timer->channelPins[3]) != (uint32_t)GPIO_PIN_RESET) {
                // Transition from Low to High
                // Hence the measured period has been during Low state
                timer->inputCaptures_low[3] = timer->inputCaptures[3];
            } else {
                // Transition from High to Low
                // Hence the measured period has been during High state
                timer->inputCaptures_high[3] = timer->inputCaptures[3];
            }
            timer->inputCaptures[3] = -(int32_t)ICTimerValue;
        }
    }

    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&timer->handle, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&timer->handle, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&timer->handle, TIM_IT_UPDATE);

            timer->inputCaptures[0] += (int32_t)0x10000;
            timer->inputCaptures[1] += (int32_t)0x10000;
            timer->inputCaptures[2] += (int32_t)0x10000;
            timer->inputCaptures[3] += (int32_t)0x10000;
        }
    }
}

void TIM4_IRQHandler(void)
{
    if (InputCapture::resTIMER4)
        InputCapture::InterruptHandler(InputCapture::resTIMER4); // HAL_TIM_IRQHandler(&htim4);
    else
        TIM4->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                               TIM_IT_TRIGGER | TIM_IT_BREAK); // clear all interrupts
}
