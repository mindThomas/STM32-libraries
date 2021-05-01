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

#include "Debug.h"
#include "Priorities.h"
#include <cmath>
#include <string.h> // for memset

// Timer::hardware_resource_t * Timer::resTIMER1 = 0; // used for PWM
// Timer::hardware_resource_t * Timer::resTIMER2 = 0; // included in Quadrature encoder
// Timer::hardware_resource_t * Timer::resTIMER3 = 0; // used for PWM
Timer::hardware_resource_t* Timer::resTIMER14 = 0;
// Timer::hardware_resource_t * Timer::resTIMER16 = 0; // used as SysTick replacement
Timer::hardware_resource_t* Timer::resTIMER17 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
// extern "C" void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
// extern "C" void TIM2_IRQHandler(void);
// extern "C" void TIM3_IRQHandler(void);
extern "C" void TIM14_IRQHandler(void);
// extern "C" void TIM16_IRQHandler(void);
extern "C" void TIM17_IRQHandler(void);

#ifdef USE_FREERTOS
Timer::Timer(timer_t timer, uint32_t frequency)
    : _TimerCallbackSoft(0)
    , _waitSemaphore(0)
#else
Timer::Timer(timer_t timer, uint32_t frequency)
#endif
{
    if (timer == TIMER14 && !resTIMER14) {
        resTIMER14 = new Timer::hardware_resource_t;
        memset(resTIMER14, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER14;
    } else if (timer == TIMER17 && !resTIMER17) {
        resTIMER17 = new Timer::hardware_resource_t;
        memset(resTIMER17, 0, sizeof(Timer::hardware_resource_t));
        _hRes = resTIMER17;
    } else {
        _hRes = 0;
        ERROR("Undefined timer or timer already in use");
        return;
    }

    _hRes->timer         = timer;
    _hRes->frequency     = frequency;
    _hRes->maxValue      = TIMER_DEFAULT_MAXVALUE;
    _hRes->TimerCallback = 0;
#ifdef USE_FREERTOS
    _hRes->callbackTaskHandle = 0;
    _hRes->callbackSemaphore  = 0;
#endif

    ConfigureTimerPeripheral();
}

Timer::~Timer()
{
    if (!_hRes)
        return;

    // Stop Timer
    LL_TIM_DisableIT_UPDATE(_hRes->instance);
    LL_TIM_DeInit(_hRes->instance);

    if (_hRes->timer == TIMER14) {
        NVIC_DisableIRQ(TIM14_IRQn);
        resTIMER14 = 0;
    } else if (_hRes->timer == TIMER17) {
        NVIC_DisableIRQ(TIM17_IRQn);
        resTIMER17 = 0;
    } else {
        ERROR("Undefined timer");
        return;
    }

    delete (_hRes);
}

void Timer::ConfigureTimerPeripheral()
{
    if (!_hRes)
        return;

    LL_TIM_InitTypeDef TIM_InitStruct = {0};

    if (_hRes->timer == TIMER14) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14); //__HAL_RCC_TIM14_CLK_ENABLE();
        NVIC_SetPriority(TIM14_IRQn, TIMER_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(TIM14_IRQn);
        _hRes->instance = TIM14;
    } else if (_hRes->timer == TIMER17) {
        LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM17); //__HAL_RCC_TIM17_CLK_ENABLE();
        NVIC_SetPriority(TIM17_IRQn, TIMER_INTERRUPT_PRIORITY);
        NVIC_EnableIRQ(TIM17_IRQn);
        _hRes->instance = TIM17;
    }

    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.RepetitionCounter = 0;
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.Autoreload        = _hRes->maxValue;

    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    LL_RCC_ClocksTypeDef clocks;
    LL_RCC_GetSystemClocksFreq(&clocks);
    uint32_t TimerClock = clocks.PCLK1_Frequency; // HAL_RCC_GetPCLK1Freq();
    if (LL_RCC_GetAPB1Prescaler() != LL_RCC_APB1_DIV_1)
        TimerClock *= 2;
    // Added prescaler computation as float such that rounding can happen
    TIM_InitStruct.Prescaler = (TimerClock / _hRes->frequency) - 1;

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

    // Enable timer
    LL_TIM_EnableCounter(_hRes->instance);

    /* Enable the update interrupt */
    LL_TIM_EnableIT_UPDATE(_hRes->instance);
}

void Timer::SetMaxValue(uint16_t maxValue)
{
    if (!_hRes)
        return;
    _hRes->maxValue = maxValue;
    LL_TIM_SetAutoReload(_hRes->instance, maxValue);
}

uint32_t Timer::Get()
{
    if (!_hRes)
        return 0;
    return (uint32_t)LL_TIM_GetCounter(_hRes->instance) + _hRes->counterOffset;
}

float Timer::GetTime()
{
    return (float)Get() / (float)_hRes->frequency;
}

void Timer::Reset()
{
#ifdef USE_FREERTOS
    if (_hRes->callbackSemaphore)
        xQueueReset(_hRes->callbackSemaphore);
#endif
    LL_TIM_SetCounter(_hRes->instance, 0);
    _hRes->counterOffset = 0;
}

void Timer::Wait(uint32_t MicrosToWait)
{
    if (!_hRes)
        return;

#ifdef USE_FREERTOS
    if (_hRes->TimerCallback || _TimerCallbackSoft || _hRes->callbackSemaphore != _waitSemaphore) {
#else
    if (_hRes->TimerCallback) {
#endif
        ERROR("Timer interrupt already registered elsewhere");
        return;
    }

#ifdef USE_FREERTOS
    if (!_waitSemaphore) {
        _waitSemaphore           = xSemaphoreCreateBinary();
        _hRes->callbackSemaphore = _waitSemaphore;
    }
#endif

    float    MicrosTimerCountPeriod = 1000000.0f / _hRes->frequency;
    uint16_t CountsToWait           = ceilf((float)MicrosToWait / MicrosTimerCountPeriod) - 1;
    _hRes->waitingFlag              = true;
    Reset();
    SetMaxValue(CountsToWait);

#ifdef USE_FREERTOS
    xSemaphoreTake(_waitSemaphore, (TickType_t)portMAX_DELAY);
    _hRes->waitingFlag = false;
#else
    while (_hRes->waitingFlag)
        asm("nop");
#endif
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

void Timer::RegisterInterrupt(
  uint32_t frequency,
  void (*TimerCallback)()) // note that the frequency should be a multiple of the configured timer count frequency
{
    if (!_hRes)
        return;

    uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
    SetMaxValue(interruptValue);

    _hRes->TimerCallback = TimerCallback;
}

#ifdef USE_FREERTOS
void Timer::RegisterInterruptSoft(
  uint32_t frequency,
  void (*TimerCallbackSoft)()) // note that the frequency should be a multiple of the configured timer count frequency
{
    if (!_hRes)
        return;

    uint16_t interruptValue = (_hRes->frequency / frequency) - 1;
    SetMaxValue(interruptValue);

    _TimerCallbackSoft = TimerCallbackSoft;
    xTaskCreate(Timer::CallbackThread, (char*)"Timer callback", 128, (void*)this, 3, &_hRes->callbackTaskHandle);
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
            timer->_TimerCallbackSoft();
    }
}
#endif

void Timer::InterruptHandler(Timer::hardware_resource_t* timer)
{
    /* TIM Update event */
    if (LL_TIM_IsActiveFlag_UPDATE(timer->instance) == 1) {
        /* Clear the update interrupt flag*/
        LL_TIM_ClearFlag_UPDATE(timer->instance);

        timer->counterOffset += (timer->maxValue + 1);

        timer->waitingFlag = false;

        if (timer->TimerCallback)
            timer->TimerCallback();

#ifdef USE_FREERTOS
        if (timer->callbackSemaphore) {
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            // xSemaphoreGiveFromISR( timer->callbackSemaphore, &xHigherPriorityTaskWoken );
            xQueueSendFromISR(timer->callbackSemaphore, NULL, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        if (timer->callbackTaskHandle) {
            portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(timer->callbackTaskHandle);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
#endif
    }
}

void TIM14_IRQHandler(void)
{
    if (Timer::resTIMER14)
        Timer::InterruptHandler(Timer::resTIMER14); // HAL_TIM_IRQHandler(&htim17);
    else
        TIM14->SR = ~(uint32_t)(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                                TIM_SR_TIF | TIM_SR_BIF); // clear all interrupts
}

void TIM17_IRQHandler(void)
{
    if (Timer::resTIMER17)
        Timer::InterruptHandler(Timer::resTIMER17); // HAL_TIM_IRQHandler(&htim17);
    else
        TIM17->SR = ~(uint32_t)(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF |
                                TIM_SR_TIF | TIM_SR_BIF); // clear all interrupts
}
