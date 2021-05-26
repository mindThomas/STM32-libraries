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

#include "Encoder.hpp"

#include <Priorities.h>
#include <string.h> // for memset

#ifdef STM32G4_ENCODER_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) while(1) { __asm("bkpt #0"); }; // break on error and halt if debugging
#endif

Encoder::hardware_resource_t* Encoder::resTIMER4 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void TIM4_IRQHandler(void);

Encoder::Encoder(timer_t timer, bool invertDirection)
{
    _hRes = 0;
    switch (timer) {
        case TIMER4:
            if (!resTIMER4) {
                resTIMER4 = new Encoder::hardware_resource_t;
                memset(resTIMER4, 0, sizeof(Encoder::hardware_resource_t));
                _hRes = resTIMER4;
            } else {
                ERROR("Encoder already configured and in use");
                return;
            }
            break;
        default:
            ERROR("Undefined timer");
            return;
    }

    _hRes->timer       = timer;
    _hRes->offsetValue = -(0x10000); // a single interrupt will be fired the first time the encoder is enabled - this
                                     // initial offset is thus to compensate for that
    _invertDirection = invertDirection;

    ConfigureEncoderGPIO();
    ConfigureEncoderPeripheral();
}

Encoder::~Encoder()
{
    if (!_hRes)
        return;

    // Stop Encoder interface
    if (HAL_TIM_Encoder_Stop(&_hRes->handle, TIM_CHANNEL_ALL) != HAL_OK) {
        ERROR("Could not stop encoder");
    }

    timer_t tmpTimer = _hRes->timer;
    delete (_hRes);

    if (tmpTimer == TIMER4) {
        /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM4 GPIO Configuration
        PB6     ------> TIM4_CH1
        PB7     ------> TIM4_CH2
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);

        /* TIM4 interrupt DeInit */
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
        resTIMER4 = 0;
    } else {
        ERROR("Undefined encoder timer");
        return;
    }
}

void Encoder::ConfigureEncoderGPIO()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (_hRes->timer == TIMER4) {
        /* Peripheral clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PB6     ------> TIM4_CH1
        PB7     ------> TIM4_CH2
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM4_IRQn, ENCODER_TIMER_OVERFLOW_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

void Encoder::ConfigureEncoderPeripheral()
{
    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    if (_hRes->timer == TIMER4) {
        _hRes->handle.Instance = TIM4;
    }

    _hRes->handle.Init.Prescaler         = 0;
    _hRes->handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    _hRes->handle.Init.Period            = 0xFFFF; // 16-bit timers
    _hRes->handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    _hRes->handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 0;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 0;

    if (HAL_TIM_Encoder_Init(&_hRes->handle, &sConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure encoder");
        return;
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&_hRes->handle, &sMasterConfig) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not configure encoder break-dead time");
        return;
    }

    // Enable Encoder interface
    if (HAL_TIM_Encoder_Start(&_hRes->handle, TIM_CHANNEL_ALL) != HAL_OK) {
        _hRes = 0;
        ERROR("Could not start encoder");
        return;
    }

    // Enable overflow (update) interrupt
    __HAL_TIM_ENABLE_IT(&_hRes->handle, TIM_IT_UPDATE);
}

int32_t Encoder::Get()
{
    if (!_hRes)
        return 0;
    if (_invertDirection)
        return -((uint16_t)__HAL_TIM_GET_COUNTER(&_hRes->handle) + _hRes->offsetValue); // invert direction
    else
        return (uint16_t)__HAL_TIM_GET_COUNTER(&_hRes->handle) + _hRes->offsetValue;
}

void Encoder::InterruptHandler(Encoder::hardware_resource_t* encoder)
{
    /* TIM Update event */
    if (__HAL_TIM_GET_FLAG(&encoder->handle, TIM_FLAG_UPDATE) != RESET) {
        if (__HAL_TIM_GET_IT_SOURCE(&encoder->handle, TIM_IT_UPDATE) != RESET) {
            __HAL_TIM_CLEAR_IT(&encoder->handle, TIM_IT_UPDATE);
            // Overflow detected
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder->handle)) // encoder is turning negative
                encoder->offsetValue -= 0x10000;
            else // encoder is turning positive
                encoder->offsetValue += 0x10000;
        }
    }
}

void TIM4_IRQHandler(void)
{
    if (Encoder::resTIMER4)
        Encoder::InterruptHandler(Encoder::resTIMER4); // HAL_TIM_IRQHandler(&htim4);
    else
        TIM4->SR = ~(uint32_t)(TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_COM |
                               TIM_IT_TRIGGER | TIM_IT_BREAK); // clear all interrupts
}
