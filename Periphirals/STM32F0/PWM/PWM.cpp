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
#include "Priorities.h"

#include <math.h>   // for roundf
#include <string.h> // for memset

PWM::hardware_resource_t* PWM::resTIMER1  = 0;
PWM::hardware_resource_t* PWM::resTIMER2  = 0;
PWM::hardware_resource_t* PWM::resTIMER3  = 0;
PWM::hardware_resource_t* PWM::resTIMER14 = 0;
PWM::hardware_resource_t* PWM::resTIMER16 = 0;
PWM::hardware_resource_t* PWM::resTIMER17 = 0;

extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler(void);

PWM::PWM(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue)
    : _channel(channel)
    , _channelLL(0)
    , _channelHAL(0)
{
    InitPeripheral(timer, channel, frequency, maxValue);
}

PWM::PWM(timer_t timer, pwm_channel_t channel)
    : PWM(timer, channel, 0, 0)
{}

PWM::~PWM()
{
    if (!_hRes)
        return;
    _hRes->configuredChannels &= !_channel;

    // Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

    if (_hRes->configuredChannels == 0) { // no more channels in use in resource, so delete the resource
        // Delete hardware resource
        timer_t      tmpTimer    = _hRes->timer;
        TIM_TypeDef* tmpInstance = _hRes->instance;
        delete (_hRes);

        switch (tmpTimer) {
            case TIMER1:
                resTIMER1 = 0;
                break;
            case TIMER3:
                resTIMER3 = 0;
                break;
            default:
                ERROR("Undefined timer");
                return;
        }

        LL_TIM_DeInit(tmpInstance);

        if (tmpTimer == TIMER1) {
            LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_TIM1); //__HAL_RCC_TIM1_CLK_DISABLE();
        } else if (tmpTimer == TIMER3) {
            LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM3); //__HAL_RCC_TIM3_CLK_DISABLE();
        } else {
            // TODO: INSERT OTHER TIMERS HERE
        }
    }
}

void PWM::InitPeripheral(timer_t timer, pwm_channel_t channel, uint32_t frequency, uint16_t maxValue)
{
    bool configureResource = false;

    _hRes = 0;

    switch (timer) {
        case TIMER1:
            if (!resTIMER1) {
                if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
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
                if (frequency > 0 && maxValue > 0) { // only configure if frequency and maxValue is set
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
        _hRes->maxValue           = maxValue;
        _hRes->configuredChannels = 0;

        if (frequency == 0 || maxValue == 0) {
            _hRes = 0;
            ERROR("Invalid timer frequency and/or maxValue");
            return;
        }

        ConfigureTimerPeripheral();
    }

    // Ensure that frequency and maxValue matches
    if (!(frequency == 0 && maxValue == 0) && // frequency and maxValue undefined
        (frequency != _hRes->frequency || maxValue != _hRes->maxValue)) {
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
    if (((timer == TIMER1 || timer == TIMER2 || timer == TIMER3) && channel > CH4) ||  // channel 1-4
        ((timer == TIMER14 || timer == TIMER16 || timer == TIMER17) && channel > CH1)) // only channel 1
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

void PWM::ConfigureTimerPeripheral()
{
    if (!_hRes)
        return;

    LL_TIM_InitTypeDef      TIM_InitStruct     = {0};
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

    if (_hRes->timer == TIMER1) {
        LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1); //__HAL_RCC_TIM1_CLK_ENABLE();
        _hRes->instance = TIM1;

        NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, TIMER_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

        /* Enable the update interrupt */
        LL_TIM_EnableIT_UPDATE(_hRes->instance);
    } else if (_hRes->timer == TIMER3) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); //__HAL_RCC_TIM3_CLK_ENABLE();
        _hRes->instance = TIM3;
    } else {
        // TODO: INSERT OTHER TIMERS HERE
    }

    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_CENTER_UP;
    TIM_InitStruct.RepetitionCounter =
      1; // 0==update IRQ on both overflow+underflow event,  1==update IRQ on only overflow event
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.Autoreload    = _hRes->maxValue - 1; // this will allow duty cycles (CCR register) to go from 0 to
                                                        // maxValue, with maxValue giving 100% duty cycle (fully on)

    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    uint32_t TimerClock = clocks.PCLK1_Frequency; // HAL_RCC_GetPCLK1Freq();
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1)
        TimerClock *= 2;
    // Added prescaler computation as float such that rounding can happen
    float prescaler = ((float)TimerClock / (2 * (TIM_InitStruct.Autoreload + 1) * _hRes->frequency)) -
                      1; // factor 1/2 due to center-aligned PWM mode
    TIM_InitStruct.Prescaler = roundf(prescaler);

    if (TIM_InitStruct.Prescaler > 0xFFFF) {
        _hRes = 0;
        ERROR("Timer frequency too slow");
        return;
    }

    if (LL_TIM_Init(_hRes->instance, &TIM_InitStruct) != SUCCESS) {
        _hRes = 0;
        ERROR("Could not initialize timer");
        return;
    }

    // Enable ARR register preload
    LL_TIM_DisableARRPreload(_hRes->instance);

    LL_TIM_SetClockSource(_hRes->instance, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(_hRes->instance, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(_hRes->instance);

    TIM_BDTRInitStruct.OSSRState       = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState       = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime        = 0;
    TIM_BDTRInitStruct.BreakState      = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity   = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(_hRes->instance, &TIM_BDTRInitStruct);

    // Enable all available PWM (output compare) outputs
    LL_TIM_EnableAllOutputs(_hRes->instance);

    // Enable timer
    LL_TIM_EnableCounter(_hRes->instance);
}

void PWM::ConfigureTimerGPIO()
{
    if (!_hRes)
        return;

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode                = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.OutputType          = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull                = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed               = LL_GPIO_SPEED_LOW;

    if (_hRes->timer == TIMER1) {
        GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        PA9     ------> TIM1_CH2
        PA10    ------> TIM1_CH3
        */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        if (_channel == CH1) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        } else if (_channel == CH2) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        } else if (_channel == CH3) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        } else
            return;
    } else if (_hRes->timer == TIMER3) {
        GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
        /**TIM1 GPIO Configuration
        PA6     ------> TIM3_CH1
        PA7     ------> TIM3_CH2
        PB0     ------> TIM3_CH3
        */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
        if (_channel == CH1) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        } else if (_channel == CH2) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
            LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        } else if (_channel == CH3) {
            GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
            LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        } else
            return;
    }
}

void PWM::ConfigureTimerChannel()
{
    if (!_hRes)
        return;

    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0; // init to 0% duty cycle
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    if (_channel == CH1) {
        _channelLL  = LL_TIM_CHANNEL_CH1;
        _channelHAL = HAL_CH1;
    } else if (_channel == CH2) {
        _channelLL  = LL_TIM_CHANNEL_CH2;
        _channelHAL = HAL_CH2;
    } else if (_channel == CH3) {
        _channelLL  = LL_TIM_CHANNEL_CH3;
        _channelHAL = HAL_CH3;
    } else if (_channel == CH4) {
        _channelLL  = LL_TIM_CHANNEL_CH4;
        _channelHAL = HAL_CH4;
    } else
        return;

    // Enable CCR1 register preload
    LL_TIM_OC_EnablePreload(_hRes->instance, _channelLL);

    // Configure channel
    if (LL_TIM_OC_Init(_hRes->instance, _channelLL, &TIM_OC_InitStruct) != SUCCESS) {
        _hRes = 0;
        ERROR("Could not configure PWM channel");
        return;
    }

    LL_TIM_OC_DisableFast(_hRes->instance, _channelLL);

    // Start channel
    LL_TIM_CC_EnableChannel(_hRes->instance, _channelLL);

    // Force update generation
    // LL_TIM_GenerateEvent_UPDATE(_hRes->instance);

    _hRes->configuredChannels |= _channel;
}

// Set a duty-cycle value between 0-1, where 0 results in an always LOW signal and 1 results in an always HIGH signal
void PWM::Set(float value)
{
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

    TIM_SET_COMPARE(_hRes->instance, _channelHAL, value);
}

void PWM::InterruptHandler(PWM::hardware_resource_t* timer)
{
    /* TIM Update event */
    if (LL_TIM_IsActiveFlag_UPDATE(timer->instance) == 1) {
        /* Clear the update interrupt flag*/
        LL_TIM_ClearFlag_UPDATE(timer->instance);

        // Do something here
    }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (PWM::resTIMER1)
        PWM::InterruptHandler(PWM::resTIMER1);
    else
        TIM1->SR = ~(uint32_t)(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                               TIM_SR_TIF | TIM_SR_BIF); // clear all interrupts
}
