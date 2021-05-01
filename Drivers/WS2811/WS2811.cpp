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

#include "WS2811.h"
#include "Debug.h"
#include "stm32h7xx_hal.h"
#include <math.h>   // for roundf
#include <string.h> // for memset

WS2811* WS2811::Handle = 0;

extern "C" void DMA2_Stream2_IRQHandler(void);

WS2811::WS2811()
{
    if (Handle) {
        ERROR("WS2811 device already in use");
        return;
    }

    memset(_ccBuffer, 0, sizeof(_ccBuffer));

    // Clean cache by writing current cache to memory
    // See https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[0], sizeof(_ccBuffer));
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[0], 32);
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[8], 32);
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[16], 32);
    SCB_CleanDCache();

    Handle = this;

    ConfigureTimerPeripheral(1000000000 / tPeriod_ns, DUTY_MAX);
    ConfigureTimerGPIO();
    ConfigureTimerChannel();
}

WS2811::~WS2811() {}

void WS2811::ConfigureTimerPeripheral(uint32_t frequency, uint16_t maxValue)
{
    TIM_ClockConfigTypeDef         sClockSourceConfig   = {0};
    TIM_MasterConfigTypeDef        sMasterConfig        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM8_CLK_ENABLE();
    _handle.Instance = TIM8;

    _handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    _handle.Init.RepetitionCounter = 0;
    //_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    _handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    _handle.Init.Period = maxValue - 1; // this will allow duty cycles (CCR register) to go from 0 to maxValue, with
                                        // maxValue giving 100% duty cycle (fully on)

    // Configure timer prescaler based on desired frequency
    //   fCNT = (ARR+1) * fPERIOD
    //   PSC = (fTIM / fCNT) - 1
    uint32_t TimerClock = HAL_RCC_GetHCLKFreq();
    // Added prescaler computation as float such that rounding can happen
    float prescaler        = ((float)TimerClock / ((_handle.Init.Period + 1) * frequency)) - 1;
    _handle.Init.Prescaler = roundf(prescaler);

    if (_handle.Init.Prescaler > 0xFFFF) {
        ERROR("Timer frequency too slow");
        return;
    }

    if (HAL_TIM_Base_Init(&_handle) != HAL_OK) {
        ERROR("Could not initialize timer");
        return;
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&_handle, &sClockSourceConfig) != HAL_OK) {
        ERROR("Could not configure timer clock source");
        return;
    }

    ConfigureDMA();

    if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
        ERROR("Could not initialize timer for WS2811");
        return;
    }

    /*sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&_handle, &sMasterConfig) != HAL_OK)
    {
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
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&_handle, &sBreakDeadTimeConfig) != HAL_OK)
    {
        ERROR("Could not configure timer break-dead time");
        return;
    }*/
}

void WS2811::ConfigureTimerGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode             = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull             = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    /* TIM8 GPIO Configuration
         PC7 ------> TIM8_CH2
    */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_7;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void WS2811::ConfigureTimerChannel(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = _ccBuffer[0];
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Configure channel
    if (HAL_TIM_PWM_ConfigChannel(&_handle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        ERROR("Could not configure timer channel");
        return;
    }
}

void WS2811::ConfigureDMA(void)
{
    // See DMA mapping on page 226 in Reference Manual

    /* Enable DMA clock */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /*##- 3- Configure DMA #####################################################*/
    /*********************** Configure DMA parameters ***************************/
    _DMA_handle.Instance     = DMA2_Stream2;
    _DMA_handle.Init.Request = DMA_REQUEST_TIM8_CH2;

    _DMA_handle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    _DMA_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
    _DMA_handle.Init.MemInc              = DMA_MINC_ENABLE;
    _DMA_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    _DMA_handle.Init.MemDataAlignment    = DMA_PDATAALIGN_WORD;
    _DMA_handle.Init.Mode                = DMA_NORMAL;
    _DMA_handle.Init.Priority            = DMA_PRIORITY_HIGH;
    _DMA_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    _DMA_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    _DMA_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
    _DMA_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;

    /* Associate the DMA handle */
    __HAL_LINKDMA(&_handle, hdma[TIM_DMA_ID_CC2], _DMA_handle);

    /* Initialize DMA handle */
    HAL_DMA_Init(_handle.hdma[TIM_DMA_ID_CC2]);

    /* NVIC configuration for DMA transfer complete interrupt */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

void WS2811::SetColor(uint8_t R, uint8_t G, uint8_t B)
{
    uint8_t bit;
    // Send bits in RBG sequence with MSB first for each color

    // Red
    bit = 0x80;
    for (int i = 0; i < 8; i++) {
        if (R & bit)
            _ccBuffer[i] = DUTY1;
        else
            _ccBuffer[i] = DUTY0;
        bit = bit >> 1;
    }

    // Blue
    bit = 0x80;
    for (int i = 0; i < 8; i++) {
        if (B & bit)
            _ccBuffer[i + 8] = DUTY1;
        else
            _ccBuffer[i + 8] = DUTY0;
        bit = bit >> 1;
    }

    // Green
    bit = 0x80;
    for (int i = 0; i < 8; i++) {
        if (G & bit)
            _ccBuffer[i + 16] = DUTY1;
        else
            _ccBuffer[i + 16] = DUTY0;
        bit = bit >> 1;
    }

    _ccBuffer[24] = 0;

    // Clean cache by writing current cache to memory
    // See https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[0], sizeof(_ccBuffer));
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[0], 32);
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[8], 32);
    // SCB_CleanDCache_by_Addr((uint32_t *) &_ccBuffer[16], 32);
    SCB_CleanDCache();

    // Send 24 bits with DMA (normal mode / single time)
    if (HAL_TIM_PWM_Start_DMA(&_handle, TIM_CHANNEL_2, _ccBuffer, 25) != HAL_OK) {
        ERROR("Could not start timer channel");
        return;
    }

    osDelay(1); // wait for data to be set
}

void WS2811::InterruptHandler(void)
{
    HAL_DMA_IRQHandler(_handle.hdma[TIM_DMA_ID_CC2]);
}

void DMA2_Stream2_IRQHandler(void)
{
    if (!WS2811::Handle)
        return;
    WS2811::Handle->InterruptHandler();
}
