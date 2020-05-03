/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 
#include "SyncedPWMADC.h"
#include "stm32g4xx_hal.h"
#include "Debug.h"
#include <string.h> // for memset
#include <math.h> // for roundf

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void DMA1_Channel1_IRQHandler(void);
extern "C" __EXPORT void DMA1_Channel2_IRQHandler(void);
extern "C" __EXPORT void ADC1_2_IRQHandler(void);

void SyncedPWMADC::ConfigureAnalogPins()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN3		[VSENSE1]
    PB1     ------> ADC1_IN12		[VSENSE3]
    PB12    ------> ADC1_IN11		[POTENTIOMETER]
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**ADC2 GPIO Configuration
    PA0     ------> ADC2_IN1		[VBUS]
    PA6     ------> ADC2_IN3		[VSENSE2]
    PA4     ------> ADC2_IN17		[BEMF1]
    PC4     ------> ADC2_IN5		[BEMF2]
    PB11    ------> ADC1/2_IN14		[BEMF3]
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Current sense Ch1 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**OPAMP1 GPIO Configuration
    PA1     ------> OPAMP1_VINP
    PA2     ------> OPAMP1_VOUT
    PA3     ------> OPAMP1_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Current sense Ch2 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**OPAMP2 GPIO Configuration
    PA5     ------> OPAMP2_VINM
    PA6     ------> OPAMP2_VOUT
    PA7     ------> OPAMP2_VINP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Current sense Ch3 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**OPAMP3 GPIO Configuration
    PB0     ------> OPAMP3_VINP
    PB1     ------> OPAMP3_VOUT
    PB2     ------> OPAMP3_VINM
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void SyncedPWMADC::DeInitAnalogPins()
{
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1|GPIO_PIN_12);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_6);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);
}

void SyncedPWMADC::InitOpAmps()
{
	OPAMP_HandleTypeDef hOpAmp = {0};

	hOpAmp.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
	hOpAmp.Init.Mode = OPAMP_PGA_MODE;
	hOpAmp.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	hOpAmp.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hOpAmp.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
	hOpAmp.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	hOpAmp.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	hOpAmp.Init.InternalOutput = ENABLE;

	{
		OPAMP_HandleTypeDef handle = hOpAmp;
		handle.Instance = OPAMP1;
		if (ENABLE_VSENSE1_DEBUG_OPAMP_OUTPUT) {
			handle.Init.InternalOutput = DISABLE; // in debug mode we sample the external output
		}
		if (HAL_OPAMP_Init(&handle) != HAL_OK)
		{
			ERROR("Could not initialize OPAMP1");
		}
		if (HAL_OPAMP_Start(&handle) != HAL_OK)
		{
			ERROR("Could not start OPAMP1");
		}
	}
	{
		OPAMP_HandleTypeDef handle = hOpAmp;
		handle.Instance = OPAMP2;
		if (HAL_OPAMP_Init(&handle) != HAL_OK)
		{
			ERROR("Could not initialize OPAMP2");
		}
		if (HAL_OPAMP_Start(&handle) != HAL_OK)
		{
			ERROR("Could not start OPAMP2");
		}
	}
	{
		OPAMP_HandleTypeDef handle = hOpAmp;
		handle.Instance = OPAMP3;
		if (HAL_OPAMP_Init(&handle) != HAL_OK)
		{
			ERROR("Could not initialize OPAMP3");
		}
		if (HAL_OPAMP_Start(&handle) != HAL_OK)
		{
			ERROR("Could not start OPAMP3");
		}
	}
}

void SyncedPWMADC::DeInitOpAmps()
{
	{
		OPAMP_HandleTypeDef handle = {0};
		handle.Instance = OPAMP1;
		HAL_OPAMP_Stop(&handle);
		HAL_OPAMP_DeInit(&handle);
	}
	{
		OPAMP_HandleTypeDef handle = {0};
		handle.Instance = OPAMP2;
		HAL_OPAMP_Stop(&handle);
		HAL_OPAMP_DeInit(&handle);
	}
	{
		OPAMP_HandleTypeDef handle = {0};
		handle.Instance = OPAMP3;
		HAL_OPAMP_Stop(&handle);
		HAL_OPAMP_DeInit(&handle);
	}
}

void SyncedPWMADC::InitADCs()
{
	ADC_HandleTypeDef hADC_Config;

	/* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    /* Clock config */
    uint32_t SysClock = HAL_RCC_GetHCLKFreq();
    _ADC_Clock = SysClock / 4;
    hADC_Config.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // Take ADC PLL-based clock (168 MHz) and divide with this Prescaler

    /* Common config */
    hADC_Config.Init.Resolution = ADC_RESOLUTION_12B;
    hADC_Config.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hADC_Config.Init.GainCompensation = 0;
    hADC_Config.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hADC_Config.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hADC_Config.Init.LowPowerAutoWait = DISABLE;
    hADC_Config.Init.ContinuousConvMode = DISABLE; // Disabled since we want the Timer to trigger the sample
    hADC_Config.Init.NbrOfConversion = 1;
    hADC_Config.Init.DiscontinuousConvMode = DISABLE;
    hADC_Config.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
    hADC_Config.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hADC_Config.Init.DMAContinuousRequests = ENABLE;
    hADC_Config.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hADC_Config.Init.OversamplingMode = DISABLE;

    hADC1 = hADC_Config;
    hADC1.Instance = ADC1;
	if (HAL_ADC_Init(&hADC1) != HAL_OK)
	{
		ERROR("Could not initialize ADC1");
	}

    hADC2 = hADC_Config;
    hADC2.Instance = ADC2;
	if (HAL_ADC_Init(&hADC2) != HAL_OK)
	{
		ERROR("Could not initialize ADC2");
	}

	/* Configure the ADC multi-mode */
	ADC_MultiModeTypeDef multimode = {0};
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hADC1, &multimode) != HAL_OK)
	{
		ERROR("Could not configure ADC1");
	}

	ADC_ConfigureCurrentSenseSampling();

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

    /* Calibrate the ADCs */
    HAL_ADCEx_Calibration_Start(&hADC1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hADC2, ADC_SINGLE_ENDED);
}

void SyncedPWMADC::ADC_ConfigureCurrentSenseSampling()
{
	bool ShouldRestart = false;
	if (_SamplingEnabled || LL_ADC_REG_IsConversionOngoing(hADC1.Instance) || LL_ADC_REG_IsConversionOngoing(hADC2.Instance))
	{
		ShouldRestart = true;
	}
	if (ShouldRestart) {
		StopSampling();
	}

	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

	/* Channel overview
		PA2     ------> ADC1_IN3					[VSENSE1 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP1			[VSENSE1]
		PB1     ------> ADC1_IN12					[VSENSE3 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP3_ADC2	[VSENSE3]
		PB12    ------> ADC1_IN11					[POTENTIOMETER]
		PA0     ------> ADC2_IN1					[VBUS]
		PA6     ------> ADC2_IN3					[VSENSE2 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP2			[VSENSE2]
		PA4     ------> ADC2_IN17					[BEMF1]
		PC4     ------> ADC2_IN5					[BEMF2]
		PB11    ------> ADC1/2_IN14					[BEMF3]
						ADC_CHANNEL_VREFINT			[VREF]
    */

	/* Configure ADC channels */
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	memset(&Channels, 0, sizeof(Channels)); // reset Channels configuration

	/* Set default Channel trigger configuration */
	/* By default 2 triggers (ON- and OFF-period) is enabled */
	timerSettingsNext.Triggers.numEnabledTriggers = 2;
	timerSettingsNext.Triggers.ON.Enabled = true;
	timerSettingsNext.Triggers.ON.Index = 0;
	timerSettingsNext.Triggers.OFF.Enabled = true;
	timerSettingsNext.Triggers.OFF.Index = 1;
	timerSettingsNext.Changed = false;
	timerSettingsCurrent = timerSettingsNext;

	/* Define sample time */
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	float ADC_SampleTimeTotalCycles = 47.5 + 12.5; // Sampling cycles + Conversion cycles
	float ADC_SampleTime = ADC_SampleTimeTotalCycles / (float)_ADC_Clock;
	uint16_t ADC_SampleTime_SingleChannel_us = (uint16_t)(ceilf(1000000.f * ADC_SampleTime));

	uint8_t numberOfChannels = 0;
	uint8_t ADCidx = 0;

	{ /* Configure channels on ADC1 */
		numberOfChannels = 0;
		ADCidx = 1;

		{ /* Current sense 1 */
			if (!ENABLE_VSENSE1_DEBUG_OPAMP_OUTPUT) {
				sConfig.Channel = ADC_CHANNEL_VOPAMP1; // Current sense 1
				Channels.VSENSE1.ADC = ADCidx;
				Channels.VSENSE1.Index = numberOfChannels;
			} else {
				sConfig.Channel = ADC_CHANNEL_3; // Current sense 1
				Channels.VSENSE1.ADC = ADCidx;
				Channels.VSENSE1.Index = numberOfChannels;
			}
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC1");
			}
			numberOfChannels++;
		}

		{ /* Internal Vref */
			sConfig.Channel = ADC_CHANNEL_VREFINT; // Vref
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VREF.ADC = ADCidx;
			Channels.VREF.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC1");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC1 */
		Channels.ADCnumChannels[0] = numberOfChannels;
		MODIFY_REG(hADC1.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	{ /* Configure channels on ADC2 */
		numberOfChannels = 0;
		ADCidx = 2;

		{ /* Current sense 3 */
			sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2; // Current sense 3
			Channels.VSENSE3.ADC = ADCidx;
			Channels.VSENSE3.Index = numberOfChannels;
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* Vbus */
			sConfig.Channel = ADC_CHANNEL_1; // Vbus
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VBUS.ADC = ADCidx;
			Channels.VBUS.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC2 */
		Channels.ADCnumChannels[1] = numberOfChannels;
		MODIFY_REG(hADC2.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	if (Channels.ADCnumChannels[0] > Channels.ADCnumChannels[1]) {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[0] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[0]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	} else {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[1] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[1]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	}

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	if (ShouldRestart) {
		StartSampling();
	}
}

void SyncedPWMADC::ADC_ConfigureBackEMFSampling()
{
	bool ShouldRestart = false;
	if (_SamplingEnabled || LL_ADC_REG_IsConversionOngoing(hADC1.Instance) || LL_ADC_REG_IsConversionOngoing(hADC2.Instance))
	{
		ShouldRestart = true;
	}
	if (ShouldRestart) {
		StopSampling();
	}

	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

	/* Channel overview
		PA2     ------> ADC1_IN3					[VSENSE1 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP1			[VSENSE1]
		PB1     ------> ADC1_IN12					[VSENSE3 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP3_ADC2	[VSENSE3]
		PB12    ------> ADC1_IN11					[POTENTIOMETER]
		PA0     ------> ADC2_IN1					[VBUS]
		PA6     ------> ADC2_IN3					[VSENSE2 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP2			[VSENSE2]
		PA4     ------> ADC2_IN17					[BEMF1]
		PC4     ------> ADC2_IN5					[BEMF2]
		PB11    ------> ADC1/2_IN14					[BEMF3]
						ADC_CHANNEL_VREFINT			[VREF]
    */

	/* Configure ADC channels */
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	memset(&Channels, 0, sizeof(Channels)); // reset Channels configuration

	/* Set default Channel trigger configuration */
	/* By default 2 triggers (ON- and OFF-period) is enabled */
	timerSettingsNext.Triggers.numEnabledTriggers = 2;
	timerSettingsNext.Triggers.ON.Enabled = true;
	timerSettingsNext.Triggers.ON.Index = 0;
	timerSettingsNext.Triggers.OFF.Enabled = true;
	timerSettingsNext.Triggers.OFF.Index = 1;
	timerSettingsNext.Changed = false;
	timerSettingsCurrent = timerSettingsNext;

	/* Define sample time */
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	float ADC_SampleTimeTotalCycles = 47.5 + 12.5; // Sampling cycles + Conversion cycles
	float ADC_SampleTime = ADC_SampleTimeTotalCycles / (float)_ADC_Clock;
	uint16_t ADC_SampleTime_SingleChannel_us = (uint16_t)(ceilf(1000000.f * ADC_SampleTime));

	uint8_t numberOfChannels = 0;
	uint8_t ADCidx = 0;

	{ /* Configure channels on ADC1 */
		numberOfChannels = 0;
		ADCidx = 1;

		{ /* BEMF3 */
			sConfig.Channel = ADC_CHANNEL_14; // BEMF3
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.BEMF3.ADC = ADCidx;
			Channels.BEMF3.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* Internal Vref */
			sConfig.Channel = ADC_CHANNEL_VREFINT; // Vref
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VREF.ADC = ADCidx;
			Channels.VREF.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC1");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC1 */
		Channels.ADCnumChannels[0] = numberOfChannels;
		MODIFY_REG(hADC1.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	{ /* Configure channels on ADC2 */
		numberOfChannels = 0;
		ADCidx = 2;

		{ /* BEMF1 */
			sConfig.Channel = ADC_CHANNEL_17; // BEMF1
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.BEMF1.ADC = ADCidx;
			Channels.BEMF1.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* Vbus */
			sConfig.Channel = ADC_CHANNEL_1; // Vbus
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VBUS.ADC = ADCidx;
			Channels.VBUS.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC2 */
		Channels.ADCnumChannels[1] = numberOfChannels;
		MODIFY_REG(hADC2.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	if (Channels.ADCnumChannels[0] > Channels.ADCnumChannels[1]) {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[0] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[0]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	} else {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[1] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[1]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	}

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	if (ShouldRestart) {
		StartSampling();
	}
}


void SyncedPWMADC::ADC_ConfigureCoastModeSampling()
{
	bool ShouldRestart = false;
	if (_SamplingEnabled || LL_ADC_REG_IsConversionOngoing(hADC1.Instance) || LL_ADC_REG_IsConversionOngoing(hADC2.Instance))
	{
		ShouldRestart = true;
	}
	if (ShouldRestart) {
		StopSampling();
	}

	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

	/* Channel overview
		PA2     ------> ADC1_IN3					[VSENSE1 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP1			[VSENSE1]
		PB1     ------> ADC1_IN12					[VSENSE3 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP3_ADC2	[VSENSE3]
		PB12    ------> ADC1_IN11					[POTENTIOMETER]
		PA0     ------> ADC2_IN1					[VBUS]
		PA6     ------> ADC2_IN3					[VSENSE2 - when OpAmp external output is enabled]
						ADC_CHANNEL_VOPAMP2			[VSENSE2]
		PA4     ------> ADC2_IN17					[BEMF1]
		PC4     ------> ADC2_IN5					[BEMF2]
		PB11    ------> ADC1/2_IN14					[BEMF3]
						ADC_CHANNEL_VREFINT			[VREF]
    */

	/* Configure ADC channels */
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	memset(&Channels, 0, sizeof(Channels)); // reset Channels configuration

	/* Set default Channel trigger configuration */
	/* By default 2 triggers (ON- and OFF-period) is enabled */
	timerSettingsNext.Triggers.numEnabledTriggers = 2;
	timerSettingsNext.Triggers.ON.Enabled = true;
	timerSettingsNext.Triggers.ON.Index = 0;
	timerSettingsNext.Triggers.OFF.Enabled = true;
	timerSettingsNext.Triggers.OFF.Index = 1;
	timerSettingsNext.Changed = false;
	timerSettingsCurrent = timerSettingsNext;

	/* Define sample time */
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	float ADC_SampleTimeTotalCycles = 47.5 + 12.5; // Sampling cycles + Conversion cycles
	float ADC_SampleTime = ADC_SampleTimeTotalCycles / (float)_ADC_Clock;
	uint16_t ADC_SampleTime_SingleChannel_us = (uint16_t)(ceilf(1000000.f * ADC_SampleTime));

	uint8_t numberOfChannels = 0;
	uint8_t ADCidx = 0;

	{ /* Configure channels on ADC1 */
		numberOfChannels = 0;
		ADCidx = 1;

		{ /* Current sense 1 */
			if (!ENABLE_VSENSE1_DEBUG_OPAMP_OUTPUT) {
				sConfig.Channel = ADC_CHANNEL_VOPAMP1; // Current sense 1
				Channels.VSENSE1.ADC = ADCidx;
				Channels.VSENSE1.Index = numberOfChannels;
			} else {
				sConfig.Channel = ADC_CHANNEL_3; // Current sense 1
				Channels.VSENSE1.ADC = ADCidx;
				Channels.VSENSE1.Index = numberOfChannels;
			}
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC1");
			}
			numberOfChannels++;
		}

		{ /* BEMF3 */
			sConfig.Channel = ADC_CHANNEL_14; // BEMF3
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.BEMF3.ADC = ADCidx;
			Channels.BEMF3.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* Internal Vref */
			sConfig.Channel = ADC_CHANNEL_VREFINT; // Vref
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VREF.ADC = ADCidx;
			Channels.VREF.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC1, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC1");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC1 */
		Channels.ADCnumChannels[0] = numberOfChannels;
		MODIFY_REG(hADC1.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	{ /* Configure channels on ADC2 */
		numberOfChannels = 0;
		ADCidx = 2;

		{ /* Current sense 3 */
			sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2; // Current sense 3
			Channels.VSENSE3.ADC = ADCidx;
			Channels.VSENSE3.Index = numberOfChannels;
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* BEMF1 */
			sConfig.Channel = ADC_CHANNEL_17; // BEMF1
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.BEMF1.ADC = ADCidx;
			Channels.BEMF1.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		{ /* Vbus */
			sConfig.Channel = ADC_CHANNEL_1; // Vbus
			sConfig.Rank = ADC_ChannelIndexToADCRank(numberOfChannels);
			Channels.VBUS.ADC = ADCidx;
			Channels.VBUS.Index = numberOfChannels;
			if (HAL_ADC_ConfigChannel(&hADC2, &sConfig) != HAL_OK)
			{
				ERROR("Could not configure channel on ADC2");
			}
			numberOfChannels++;
		}

		/* Set number of ranks in regular group sequencer of ADC2 */
		Channels.ADCnumChannels[1] = numberOfChannels;
		MODIFY_REG(hADC2.Instance->SQR1, ADC_SQR1_L, (numberOfChannels - (uint8_t)1));
	}


	if (Channels.ADCnumChannels[0] > Channels.ADCnumChannels[1]) {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[0] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[0]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	} else {
		_ADC_SampleTime_Total_us = Channels.ADCnumChannels[1] * ADC_SampleTime_SingleChannel_us;
		if ((ADC_DMA_HALFWORD_MAX_BUFFER_SIZE % Channels.ADCnumChannels[1]) > 0) {
			ERROR("DMA buffer size is not a multiple of the number of channels to sample");
		}
	}

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	if (ShouldRestart) {
		StartSampling();
	}
}

uint32_t SyncedPWMADC::ADC_ChannelIndexToADCRank(uint8_t chIndex)
{
	switch (chIndex) {
		case 0: return ADC_REGULAR_RANK_1;
		case 1: return ADC_REGULAR_RANK_2;
		case 2: return ADC_REGULAR_RANK_3;
		case 3: return ADC_REGULAR_RANK_4;
		case 4: return ADC_REGULAR_RANK_5;
		case 5: return ADC_REGULAR_RANK_6;
		case 6: return ADC_REGULAR_RANK_7;
		case 7: return ADC_REGULAR_RANK_8;
		case 8: return ADC_REGULAR_RANK_9;
		case 9: return ADC_REGULAR_RANK_10;
		case 10: return ADC_REGULAR_RANK_11;
		case 11: return ADC_REGULAR_RANK_12;
		case 12: return ADC_REGULAR_RANK_13;
		case 13: return ADC_REGULAR_RANK_14;
		case 14: return ADC_REGULAR_RANK_15;
		case 15: return ADC_REGULAR_RANK_16;
		default: return 0; // error
	}
}

void SyncedPWMADC::DeInitADCs()
{
	__HAL_RCC_ADC12_CLK_DISABLE();
}

void SyncedPWMADC::InitDMAs()
{
	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

    /* ADC1 DMA Init */
    /* ADC1 Init */
	hDMA_ADC1.Instance = DMA1_Channel1;
    hDMA_ADC1.Init.Request = DMA_REQUEST_ADC1;
    hDMA_ADC1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hDMA_ADC1.Init.PeriphInc = DMA_PINC_DISABLE;
    hDMA_ADC1.Init.MemInc = DMA_MINC_ENABLE;
    hDMA_ADC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hDMA_ADC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hDMA_ADC1.Init.Mode = DMA_NORMAL;
    hDMA_ADC1.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    if (!SAMPLE_IN_EVERY_PWM_CYCLE) {
    	hDMA_ADC1.Init.Mode = DMA_NORMAL; // wait for DMA to be started/triggered and the fill the buffer ONCE
    } else {
    	hDMA_ADC1.Init.Mode = DMA_CIRCULAR; // we want to sample at every PWM period, so it should just run continuously in a circular fashion
    	timerSettingsNext.samplingInterval = 1;
    	timerSettingsNext.samplingNumSamples = 1;
    }

    if (HAL_DMA_Init(&hDMA_ADC1) != HAL_OK)
    {
    	ERROR("Could not initialize ADC1 DMA");
    }

    __HAL_LINKDMA(&hADC1, DMA_Handle, hDMA_ADC1);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hDMA_ADC2.Instance = DMA1_Channel2;
    hDMA_ADC2.Init.Request = DMA_REQUEST_ADC2;
    hDMA_ADC2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hDMA_ADC2.Init.PeriphInc = DMA_PINC_DISABLE;
    hDMA_ADC2.Init.MemInc = DMA_MINC_ENABLE;
    hDMA_ADC2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hDMA_ADC2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hDMA_ADC2.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    if (!SAMPLE_IN_EVERY_PWM_CYCLE) {
    	hDMA_ADC2.Init.Mode = DMA_NORMAL; // wait for DMA to be started/triggered and the fill the buffer ONCE
    } else {
    	hDMA_ADC2.Init.Mode = DMA_CIRCULAR; // we want to sample at every PWM period, so it should just run continuously in a circular fashion
    	timerSettingsNext.samplingInterval = 1;
    	timerSettingsNext.samplingNumSamples = 1;
    }

    if (HAL_DMA_Init(&hDMA_ADC2) != HAL_OK)
    {
    	ERROR("Could not initialize ADC2 DMA");
    }

    __HAL_LINKDMA(&hADC2, DMA_Handle, hDMA_ADC2);

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

void SyncedPWMADC::DeInitDMAs()
{
	HAL_DMA_DeInit(hADC1.DMA_Handle);
	HAL_DMA_DeInit(hADC1.DMA_Handle);
}

void SyncedPWMADC::StartADC()
{
	if (_ADCEnabled) return; // ADC already running
	_ADCEnabled = true;

	ADC_Start(&hADC1);
	ADC_Start(&hADC2);
}


void SyncedPWMADC::StopADC()
{
	if (!_ADCEnabled) return; // sampling already stopped
	_ADCEnabled = false;

	LL_ADC_REG_StopConversion(hADC1.Instance);
	LL_ADC_REG_StopConversion(hADC2.Instance);

	ADC_Stop(&hADC1);
	ADC_Stop(&hADC2);
}

void SyncedPWMADC::ADC_Start(ADC_HandleTypeDef * hadc)
{
	// Based on "HAL_ADC_Start_DMA"
	/* Perform ADC enable and conversion start if no conversion is on going */
	if (LL_ADC_REG_IsConversionOngoing(hadc->Instance) == 0UL)
	{
		/* Start conversion if ADC is effectively enabled */
		if (ADC_Enable(hadc) == HAL_OK)
		{
	        /* Set ADC state                                                        */
	        /* - Clear state bitfield related to regular group conversion results   */
	        /* - Set state bitfield related to regular operation                    */
			ADC_STATE_CLR_SET(hadc->State,
							  HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,
							  HAL_ADC_STATE_REG_BUSY);

			CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);

			ADC_CLEAR_ERRORCODE(hadc);


			/* Set the DMA transfer complete callback */
			hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

			/* Set the DMA half transfer complete callback */
			hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

			/* Set the DMA error callback */
			hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;

			/* Clear regular group conversion flag and overrun flag               */
			__HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

			/* Start ADC group regular conversion */
			//LL_ADC_REG_StartConversion(hadc->Instance);
		} else {
			ERROR("Could not start ADC");
		}
	} else {
		ERROR("Could not start ADC");
	}
}

void SyncedPWMADC::ADC_Stop(ADC_HandleTypeDef * hadc)
{
	/* 1. Stop potential ADC group regular conversion on going */
	/* And Disable ADC peripheral if conversions are effectively stopped */
	if (ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP) == HAL_OK)
	{
		/* Disable ADC DMA (ADC DMA configuration of continous requests is kept) */
		CLEAR_BIT(hadc->Instance->CFGR, ADC_CFGR_DMAEN);

		/* Disable ADC overrun interrupt */
		__HAL_ADC_DISABLE_IT(hadc, ADC_IT_OVR);

		/* 2. Disable the ADC peripheral */
		/* And Check if ADC is effectively disabled */
		if (ADC_Disable(hadc) == HAL_OK)
		{
			/* Set ADC state */
			ADC_STATE_CLR_SET(hadc->State,
							  HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,
							  HAL_ADC_STATE_READY);
		}
	} else {
		ERROR("Could not stop ADC");
	}
}

void SyncedPWMADC::StartSampling()
{
	if (!_ADCEnabled) { // ADCs not enabled
		StartADC();
	}

	// Enable the sampling synchronized to the timer
	__HAL_TIM_CLEAR_IT(&hTimer, TIM_IT_UPDATE); // important to clear the interrupt before enabling since the update flag might be set without the interrupt having been enabled, and thus the interrupt would end up firing right after enabling it
	__HAL_TIM_ENABLE_IT(&hTimer, TIM_IT_UPDATE);
}

void SyncedPWMADC::StopSampling()
{
	// Disable the sampling synchronized to the timer
	__HAL_TIM_DISABLE_IT(&hTimer, TIM_IT_UPDATE);

	if (!_SamplingEnabled) return; // sampling already stopped

	if (_ADCEnabled) { // Stop the enabled ADCs
		StopADC();
	}

	/* Disable ADC DMA (ADC DMA configuration of continous requests is kept) */
	CLEAR_BIT(hADC1.Instance->CFGR, ADC_CFGR_DMAEN);
	CLEAR_BIT(hADC2.Instance->CFGR, ADC_CFGR_DMAEN);

	_SamplingEnabled = false;

	if (hDMA_ADC1.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&hDMA_ADC1);
	}
	if (hDMA_ADC2.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&hDMA_ADC2);
	}

    __HAL_ADC_CLEAR_FLAG(&hADC1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
    __HAL_ADC_CLEAR_FLAG(&hADC2, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

    _DMA_ADC1_ongoing = 0;
    _DMA_ADC2_ongoing = 0;
}

void SyncedPWMADC::RestartSampling()
{
	if (_SamplingEnabled)
	{
		StopSampling();
		StartSampling();
	}
}

// Not used in DMA mode. So this interrupt would generally not be called
void ADC1_2_IRQHandler(void)
{
	if (SyncedPWMADC::globalObject) {
		  HAL_ADC_IRQHandler(&SyncedPWMADC::globalObject->hADC1);
		  HAL_ADC_IRQHandler(&SyncedPWMADC::globalObject->hADC2);
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	if (SyncedPWMADC::globalObject)
		HAL_DMA_IRQHandler(&SyncedPWMADC::globalObject->hDMA_ADC1); // The HAL interrupt routine called here is fast, so it's OK to use it.
}

void DMA1_Channel2_IRQHandler(void)
{
	if (SyncedPWMADC::globalObject)
		HAL_DMA_IRQHandler(&SyncedPWMADC::globalObject->hDMA_ADC2); // The HAL interrupt routine called here is fast, so it's OK to use it.
}
