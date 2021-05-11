/* Copyright (C) 2018- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "PWM.hpp"

#include <math.h>   // for roundf
#include <string.h> // for memset

#ifdef STM32G4_PWM_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) ((void)0U); // not implemented
#endif

PWM::hardware_resource_t* PWM::resTIMER1  = 0;
PWM::hardware_resource_t* PWM::resTIMER3  = 0;

PWM::PWM(timer_t timer, pwm_channel_t channel, uint32_t frequency)
    : _channel(channel)
    , _channelHAL(0)
{
    InitPeripheral(timer, channel, frequency);
}

PWM::PWM(timer_t timer, pwm_channel_t channel)
    : PWM(timer, channel, 0)
{}

PWM::~PWM()
{
    if (!_hRes)
        return;
    _hRes->configuredChannels &= !_channel;

    // Stop channel
    if (HAL_TIM_PWM_Stop(&_hRes->handle, _channelHAL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not stop PWM channel");
        return;
    }

    // Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

    if (_hRes->configuredChannels == 0) { // no more channels in use in resource, so delete the resource
        // Delete hardware resource
        timer_t tmpTimer = _hRes->timer;
        delete (_hRes);

        switch (tmpTimer) {
            case TIMER1:
                __HAL_RCC_TIM1_CLK_DISABLE();
                resTIMER1 = 0;
                break;
            case TIMER3:
                __HAL_RCC_TIM3_CLK_DISABLE();
                resTIMER3 = 0;
                break;
            default:
                ERROR("Undefined timer");
                return;
        }
    }
}

void PWM::InitPeripheral(timer_t timer, pwm_channel_t channel, uint32_t frequency)
{
    bool configureResource = false;

    _hRes = 0;

    switch (timer) {
        case TIMER1:
            if (!resTIMER1) {
                if (frequency > 0) { // only configure if frequency is set
                    resTIMER1 = new PWM::hardware_resource_t;
                    memset(resTIMER1, 0, sizeof(PWM::hardware_resource_t));
                    configureResource = true;
                    _hRes             = resTIMER1;
                }
            } else {
                _hRes = resTIMER1;
            }
            break;
        case TIMER3:
            if (!resTIMER3) {
                if (frequency > 0) { // only configure if frequency is set
                    resTIMER3 = new PWM::hardware_resource_t;
                    memset(resTIMER3, 0, sizeof(PWM::hardware_resource_t));
                    configureResource = true;
                    _hRes             = resTIMER3;
                }
            } else {
                _hRes = resTIMER3;
            }
            break;
        default:
            ERROR("Undefined timer");
            return;
    }

    if (configureResource) { // first time configuring peripheral
        _hRes->timer              = timer;
        _hRes->frequency          = frequency;
        _hRes->configuredChannels = 0;

        if (frequency == 0) {
            _hRes = 0;
            ERROR("Invalid timer frequency and/or maxValue");
            return;
        }

        ConfigureTimerPeripheral();
    }

    // Ensure that the channel is valid and not already in use
    if ((channel % 2) != 0 && channel != 1) {
        _hRes = 0;
        ERROR("Only one timer channel can be configured per object");
        return;
    }
    if (((timer == TIMER1) && channel > CH3) || // channel 1-3
        ((timer == TIMER3) && channel > CH1)) // only channel 1
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

void PWM::ConfigureTimerPeripheral(bool fixedPrescaler)
{
    if (!_hRes)
        return;

    TIM_ClockConfigTypeDef         sClockSourceConfig   = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    if (_hRes->timer == TIMER1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        _hRes->handle.Instance = TIM1;
    } else if (_hRes->timer == TIMER3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        _hRes->handle.Instance = TIM3;
    }

    _hRes->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    _hRes->handle.Init.RepetitionCounter = 0;
    _hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    _hRes->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

#if 0
    _hRes->handle.Init.Period = _hRes->maxValue - 1; // this will allow duty cycles (CCR register) to go from 0 to
                                                     // maxValue, with maxValue giving 100% duty cycle (fully on)

    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    uint32_t TimerClock = HAL_RCC_GetPCLK2Freq();
    // Added prescaler computation as float such that rounding can happen
    float prescaler              = ((float)TimerClock / ((_hRes->handle.Init.Period + 1) * _hRes->frequency)) - 1;
    _hRes->handle.Init.Prescaler = roundf(prescaler);
#endif

    // Find PWM prescaler to yield the largest possible ARR while still keeping it within 16-bit
    // Based on "A few useful formulas" from the STM32 Timer slides (PDF)
    // fPWM = fTIM / ((ARR+1)*(PSC+1))
    uint32_t TimerClock = HAL_RCC_GetPCLK2Freq();
    // Start with no prescaler: _PWM_prescaler = PSC+1 = 1
    uint32_t ARR = 0x10000;
    if (!fixedPrescaler) {
        _hRes->handle.Init.Prescaler = 0;
        while (ARR > 0xFFFF) {
            _hRes->handle.Init.Prescaler++;
            // Compute ARR and check value
            ARR = TimerClock / (_hRes->frequency * _hRes->handle.Init.Prescaler) - 1;
        }
    } else {
        ARR = TimerClock / (_hRes->frequency * _hRes->handle.Init.Prescaler) - 1;
        if (ARR > 0xFFFF) {
            return; // error, not possible with this prescaler
        }
    }
    _hRes->handle.Init.Period    = ARR;
    _hRes->handle.Init.Prescaler = _hRes->handle.Init.Prescaler - 1;
    _hRes->maxValue = _hRes->handle.Init.Period + 1;

    // Recompute actual frequency
    _hRes->frequency = TimerClock / ((_hRes->handle.Init.Period + 1) * _hRes->handle.Init.Prescaler);

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

    if (HAL_TIM_PWM_Init(&_hRes->handle) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not initialize PWM for timer");
        return;
    }

    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&_hRes->handle, &sMasterConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure timer");
        return;
    }

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.BreakAFMode      = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.Break2AFMode     = TIM_BREAK_AFMODE_INPUT;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&_hRes->handle, &sBreakDeadTimeConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure timer break-dead time");
        return;
    }
}

void PWM::ConfigureTimerGPIO()
{
    if (!_hRes)
        return;

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;

    if (_hRes->timer == TIMER1) {
        GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        PA9     ------> TIM1_CH2
        PA10    ------> TIM1_CH3
        */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        if (_channel == CH1)
            GPIO_InitStruct.Pin = GPIO_PIN_8;
        else if (_channel == CH2)
            GPIO_InitStruct.Pin = GPIO_PIN_9;
        else if (_channel == CH3)
            GPIO_InitStruct.Pin = GPIO_PIN_10;
        else
            return;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        _complementaryOutput = false;
    } else if (_hRes->timer == TIMER3) {
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        /**TIM8 GPIO Configuration
        PC6     ------> TIM3_CH1
        */
        __HAL_RCC_GPIOC_CLK_ENABLE();
        if (_channel == CH1)
            GPIO_InitStruct.Pin = GPIO_PIN_6;
        else
            return;

        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        _complementaryOutput = false;
    }
}

void PWM::ConfigureTimerChannel()
{
    if (!_hRes)
        return;

    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
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
    if (HAL_TIM_PWM_ConfigChannel(&_hRes->handle, &sConfigOC, _channelHAL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure PWM channel");
        return;
    }

    // Start channel
    if (!_complementaryOutput) {
        if (HAL_TIM_PWM_Start(&_hRes->handle, _channelHAL) != HAL_OK) {
            _hRes = 0;
            ERROR("Could not start PWM channel");
            return;
        }
    } else {
        if (HAL_TIMEx_PWMN_Start(&_hRes->handle, _channelHAL) != HAL_OK) {
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
    if (!_hRes)
        return;
    if (value < 0)
        value = 0.0f;
    if (value > 1)
        value = 1.0f;

    uint16_t rawValue = _hRes->maxValue * value;
    SetRaw(rawValue);
}

void PWM::SetRaw(uint16_t value)
{
    if (!_hRes)
        return;
    if (value > _hRes->maxValue)
        value = _hRes->maxValue;

    if (!_complementaryOutput)
        __HAL_TIM_SET_COMPARE(&_hRes->handle, _channelHAL, value);
    else
        __HAL_TIM_SET_COMPARE(&_hRes->handle, _channelHAL, _hRes->maxValue - value);
}

uint16_t PWM::GetMaxValue()
{
    if (!_hRes)
        return 0;

    return _hRes->maxValue;
}