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

#include "Encoder.hpp"

#include <Debug/Debug.h>
#include "Priorities.h"
#include <string.h> // for memset

Encoder::hardware_resource_t* Encoder::resTIMER2 = 0;
Encoder::hardware_resource_t* Encoder::resTIMER3 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void TIM2_IRQHandler(void);
extern "C" void TIM3_IRQHandler(void);

Encoder::Encoder(timer_t timer)
{
    _hRes = 0;
    switch (timer) {
        case TIMER2:
            if (!resTIMER2) {
                resTIMER2 = new Encoder::hardware_resource_t;
                memset(resTIMER2, 0, sizeof(Encoder::hardware_resource_t));
                _hRes = resTIMER2;
            } else {
                ERROR("Encoder already configured and in use");
                return;
            }
            break;
        case TIMER3:
            if (!resTIMER3) {
                resTIMER3 = new Encoder::hardware_resource_t;
                memset(resTIMER3, 0, sizeof(Encoder::hardware_resource_t));
                _hRes = resTIMER3;
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

    ConfigureEncoderGPIO();
    ConfigureEncoderPeripheral();
}

Encoder::~Encoder()
{
    if (!_hRes)
        return;

    // Stop Encoder interface
    LL_TIM_DisableIT_UPDATE(_hRes->instance);
    LL_TIM_DeInit(_hRes->instance);

    if (_hRes->timer == TIMER2) {
        NVIC_DisableIRQ(TIM2_IRQn);
        resTIMER2 = 0;
    } else if (_hRes->timer == TIMER3) {
        NVIC_DisableIRQ(TIM3_IRQn);
        resTIMER3 = 0;
    } else {
        ERROR("Undefined encoder timer");
        return;
    }

    delete (_hRes);
}

void Encoder::ConfigureEncoderGPIO()
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_HIGH;

    if (_hRes->timer == TIMER2) {
        /* GPIO clock enable */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

        /**TIM2 GPIO Configuration
        PA15    ------> TIM2_CH1
        PB3     ------> TIM2_CH2
        */
        GPIO_InitStruct.Alternate = LL_GPIO_AF_2;

        GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
        LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void Encoder::ConfigureEncoderPeripheral()
{
    if (!_hRes)
        return;

    LL_TIM_InitTypeDef         TIM_InitStruct        = {0};
    LL_TIM_ENCODER_InitTypeDef TIM_EncoderInitStruct = {0};

    if (_hRes->timer == TIMER2) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); //__HAL_RCC_TIM2_CLK_ENABLE();
        NVIC_SetPriority(TIM2_IRQn, TIMER_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(TIM2_IRQn);
        _hRes->instance = TIM2;
    } else if (_hRes->timer == TIMER3) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3); //__HAL_RCC_TIM3_CLK_ENABLE();
        NVIC_SetPriority(TIM3_IRQn, TIMER_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(TIM3_IRQn);
        _hRes->instance = TIM3;
    }

    TIM_InitStruct.Prescaler         = 0;
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.RepetitionCounter = 0;
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.Autoreload        = 0xFFFF; // 16-bit timers

    if (LL_TIM_Init(_hRes->instance, &TIM_InitStruct) != SUCCESS) {
        _hRes = 0;
        ERROR("Could not initialize timer");
        return;
    }

    TIM_EncoderInitStruct.EncoderMode    = LL_TIM_ENCODERMODE_X4_TI12;
    TIM_EncoderInitStruct.IC1Polarity    = LL_TIM_IC_POLARITY_RISING;
    TIM_EncoderInitStruct.IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    TIM_EncoderInitStruct.IC1Prescaler   = LL_TIM_ICPSC_DIV1;
    TIM_EncoderInitStruct.IC1Filter      = LL_TIM_IC_FILTER_FDIV1;
    TIM_EncoderInitStruct.IC2Polarity    = LL_TIM_IC_POLARITY_RISING;
    TIM_EncoderInitStruct.IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;
    TIM_EncoderInitStruct.IC2Prescaler   = LL_TIM_ICPSC_DIV1;
    TIM_EncoderInitStruct.IC2Filter      = LL_TIM_IC_FILTER_FDIV1;

    if (LL_TIM_ENCODER_Init(_hRes->instance, &TIM_EncoderInitStruct) != SUCCESS) {
        _hRes = 0;
        ERROR("Could not configure encoder");
        return;
    }

    // Enable timer
    LL_TIM_EnableCounter(_hRes->instance);

    // Enable overflow (update) interrupt
    LL_TIM_EnableIT_UPDATE(_hRes->instance);
}

int32_t Encoder::Get()
{
    if (!_hRes)
        return 0;
    return -((uint16_t)LL_TIM_GetCounter(_hRes->instance) +
             _hRes->offsetValue); // invert sign due to direction of motors and assignment of encoder channels A+B
}

void Encoder::InterruptHandler(Encoder::hardware_resource_t* encoder)
{
    /* TIM Update event */
    if (LL_TIM_IsActiveFlag_UPDATE(encoder->instance) == 1) {
        /* Clear the update interrupt flag*/
        LL_TIM_ClearFlag_UPDATE(encoder->instance);

        // Overflow detected
        if (LL_TIM_GetDirection(encoder->instance) == LL_TIM_COUNTERDIRECTION_DOWN) // encoder is turning negative
            encoder->offsetValue -= 0x10000;
        else // encoder is turning positive
            encoder->offsetValue += 0x10000;
    }
}

void TIM2_IRQHandler(void)
{
    if (Encoder::resTIMER2)
        Encoder::InterruptHandler(Encoder::resTIMER2); // HAL_TIM_IRQHandler(&htim2);
    else
        TIM2->SR = ~(uint32_t)(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                               TIM_SR_TIF | TIM_SR_BIF); // clear all interrupts
}

void TIM3_IRQHandler(void)
{
    if (Encoder::resTIMER3)
        Encoder::InterruptHandler(Encoder::resTIMER3); // HAL_TIM_IRQHandler(&htim2);
    else
        TIM3->SR = ~(uint32_t)(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                               TIM_SR_TIF | TIM_SR_BIF); // clear all interrupts
}
