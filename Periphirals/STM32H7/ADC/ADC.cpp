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
 
#include "ADC.h"
#include "stm32h7xx_hal.h"
#include "Debug.h"
#include <string.h> // for memset

ADC::hardware_resource_t * ADC::resADC1 = 0;
ADC::hardware_resource_t * ADC::resADC2 = 0;
ADC::hardware_resource_t * ADC::resADC3 = 0;
float ADC::ADC_REF_CORR = 1.0f;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
extern "C" __EXPORT void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
extern "C" __EXPORT void DMA1_Stream0_IRQHandler(void);
extern "C" __EXPORT void DMA1_Stream1_IRQHandler(void);
extern "C" __EXPORT void DMA1_Stream2_IRQHandler(void);

ADC::ADC(adc_t adc, uint32_t channel, uint32_t resolution) : _channel(channel)
{
	InitPeripheral(adc, resolution);
}
ADC::ADC(adc_t adc, uint32_t channel) : _channel(channel)
{
	InitPeripheral(adc, ADC_DEFAULT_RESOLUTION);
}

ADC::~ADC()
{
	if (!_hRes) return;
	_hRes->map_channel2bufferIndex[_channel] = 0xFF; // mark as unconfigured
	_hRes->numberOfConfiguredChannels--;

	// Stop ADC
	if (HAL_ADC_Stop_DMA(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not stop ADC");
		return;
	}

	ERROR("Deinit (destruction) of ADC with DMA enabled has not been implemented yet!");
	return;

	// Missing deinit of GPIO, eg. HAL_GPIO_DeInit(GPIOF, GPIO_PIN_3)

	if (_hRes->numberOfConfiguredChannels == 0) { // no more channels in use in resource, so delete the resource
		// Delete hardware resource
		timer_t tmpADC = _hRes->adc;
		delete(_hRes);

		switch (tmpADC)
		{
			case ADC_1:
				if (!resADC2)
					__HAL_RCC_ADC12_CLK_DISABLE();
				resADC1 = 0;
				break;
			case ADC_2:
				if (!resADC1)
					__HAL_RCC_ADC12_CLK_DISABLE();
				resADC2 = 0;
				break;
			case ADC_3:
				__HAL_RCC_ADC3_CLK_DISABLE();
				resADC3 = 0;
				break;
			default:
				ERROR("Undefined ADC");
				return;
		}
	}
}

void ADC::InitPeripheral(adc_t adc, uint32_t resolution)
{
	bool configureResource = false;

	_hRes = 0;

	switch (adc)
	{
		case ADC_1:
			if (!resADC1) {
				resADC1 = new ADC::hardware_resource_t;
				memset(resADC1, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC1;
			}
			else {
				_hRes = resADC1;
			}
			break;
		case ADC_2:
			if (!resADC2) {
				resADC2 = new ADC::hardware_resource_t;
				memset(resADC2, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC2;
			}
			else {
				_hRes = resADC2;
			}
			break;
		case ADC_3:
			if (!resADC3) {
				resADC3 = new ADC::hardware_resource_t;
				memset(resADC3, 0, sizeof(ADC::hardware_resource_t));
				configureResource = true;
				_hRes = resADC3;
			}
			else {
				_hRes = resADC3;
			}
			break;
		default:
			ERROR("Undefined ADC");
			return;
	}

	if (configureResource) { // first time configuring peripheral
		_hRes->adc = adc;
		_hRes->resolution = resolution;
		_hRes->numberOfConfiguredChannels = 0;
		memset(_hRes->map_channel2bufferIndex, 0xFF, sizeof(_hRes->map_channel2bufferIndex));
		memset(_hRes->buffer, 0, sizeof(_hRes->buffer));

		if (adc == ADC_3) {
			// configure VREFINT channel as the first one, such that this channel is always read
			_hRes->map_channel2bufferIndex[ADC_CHANNEL_VREFINT] = 0;
			_hRes->numberOfConfiguredChannels = 1;
		}
	}

	// Ensure that the channel is valid and not already in use
	if (_hRes->map_channel2bufferIndex[_channel] != 0xFF) {
		_hRes = 0;
		ERROR("Channel already configured on selected ADC");
		return;
	}
	_hRes->map_channel2bufferIndex[_channel] = _hRes->numberOfConfiguredChannels;
	_hRes->numberOfConfiguredChannels++;

	if (_hRes->resolution != resolution) {
		_hRes = 0;
		ERROR("ADC already in used with different resolution");
		return;
	}

	ConfigureADCPeripheral();
	ConfigureADCGPIO();
	ConfigureADCChannels();
}

void ADC::ConfigureADCPeripheral()
{
	if (!_hRes) return;

	/* ADC Periph interface clock configuration */
	__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_CLKP);

	if (_hRes->adc == ADC_1) {
		__HAL_RCC_ADC12_CLK_ENABLE();
		_hRes->handle.Instance = ADC1;
	} else if (_hRes->adc == ADC_2) {
		__HAL_RCC_ADC12_CLK_ENABLE();
		_hRes->handle.Instance = ADC2;
	} else if (_hRes->adc == ADC_3) {
		__HAL_RCC_ADC3_CLK_ENABLE();
		_hRes->handle.Instance = ADC3;
	}

	// Stop ADC to be able to configure channel
	if (ADC_IS_CONVERSION_ONGOING_REGULAR(&_hRes->handle) != RESET) {
		if (HAL_ADC_Stop_DMA(&_hRes->handle) != HAL_OK)
		{
			_hRes = 0;
			ERROR("Could not stop ADC");
			return;
		}
	}

	/*if (_hRes->adc == ADC_1) {
		__HAL_RCC_ADC12_FORCE_RESET();
		__HAL_RCC_ADC12_RELEASE_RESET();
	} else if (_hRes->adc == ADC_2) {
		__HAL_RCC_ADC12_FORCE_RESET();
		__HAL_RCC_ADC12_RELEASE_RESET();
	} else if (_hRes->adc == ADC_3) {
		__HAL_RCC_ADC3_FORCE_RESET();
		__HAL_RCC_ADC3_RELEASE_RESET();
	}*/

	if (HAL_ADC_DeInit(&_hRes->handle) != HAL_OK)
	{
		/* ADC de-initialization Error */
		_hRes = 0;
		ERROR("ADC de-initialization Error ");
		return;
	}

	if (_hRes->resolution == ADC_RESOLUTION_8B) {
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_8B; /* 8-bit resolution for converted data */
		_hRes->range = ((uint32_t)1 << 8) - 1;
	} else if (_hRes->resolution == ADC_RESOLUTION_10B) {
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_10B; /* 10-bit resolution for converted data */
		_hRes->range = ((uint32_t)1 << 10) - 1;
	} else if (_hRes->resolution == ADC_RESOLUTION_12B) {
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_12B; /* 12-bit resolution for converted data */
		_hRes->range = ((uint32_t)1 << 12) - 1;
	} else if (_hRes->resolution == ADC_RESOLUTION_14B) {
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_14B; /* 14-bit resolution for converted data */
		_hRes->range = ((uint32_t)1 << 14) - 1;
	} else if (_hRes->resolution == ADC_RESOLUTION_16B) {
		_hRes->handle.Init.Resolution = ADC_RESOLUTION_16B; /* 16-bit resolution for converted data */
		_hRes->range = ((uint32_t)1 << 16) - 1;
	} else {
		_hRes = 0;
		ERROR("Incorrect ADC resolution");
		return;
	}

#if 0
	_hRes->handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; // ADC clock input is configured to be 80 MHz, divided by 4 gives an ADC clock of 20 MHz
	_hRes->handle.Init.ScanConvMode = ADC_SCAN_ENABLE; /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	_hRes->handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV; /* EOC flag picked-up to indicate conversion end */
	_hRes->handle.Init.LowPowerAutoWait = DISABLE;
	_hRes->handle.Init.ContinuousConvMode = ENABLE; /* Continuous mode enabled (automatic conversion restart after each conversion) */
	_hRes->handle.Init.NbrOfConversion = 1;
	_hRes->handle.Init.DiscontinuousConvMode = DISABLE;
	_hRes->handle.Init.ExternalTrigConv = ADC_SOFTWARE_START; /* Software start to trig the 1st conversion manually, without external event */
	_hRes->handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	_hRes->handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR; /* Regular Conversion data stored in DR register only */
	_hRes->handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN; /* DR register is overwritten with the last conversion result in case of overrun */
	_hRes->handle.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	_hRes->handle.Init.BoostMode = DISABLE; /* Boost mode can be disabled (to save power) since ADC clock frequency is less than or equal to 20 MHz */
	_hRes->handle.Init.OversamplingMode = DISABLE;
#endif

	_hRes->handle.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;      /* Synchronous clock mode, input ADC clock divided by 4*/
	//_hRes->handle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4; // ADC clock input is configured to be 80 MHz, divided by 4 gives an ADC clock of 20 MHz
	//_hRes->handle.Init.Resolution               = ADC_RESOLUTION_16B;            /* 16-bit resolution for converted data */
	_hRes->handle.Init.EOCSelection             = ADC_EOC_SEQ_CONV;           /* EOC flag picked-up to indicate conversion end */
	_hRes->handle.Init.LowPowerAutoWait         = DISABLE;                       /* Auto-delayed conversion feature disabled */
	_hRes->handle.Init.ContinuousConvMode       = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
	_hRes->handle.Init.NbrOfConversion          = _hRes->numberOfConfiguredChannels;                             /* Parameter discarded because sequencer is disabled */
	if (_hRes->handle.Init.NbrOfConversion > 1)
		_hRes->handle.Init.ScanConvMode         = ENABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	else
		_hRes->handle.Init.ScanConvMode         = DISABLE;                        /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	_hRes->handle.Init.DiscontinuousConvMode    = DISABLE;                       /* Parameter discarded because sequencer is disabled */
	_hRes->handle.Init.NbrOfDiscConversion      = 1;                             /* Parameter discarded because sequencer is disabled */
	_hRes->handle.Init.ExternalTrigConv         = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
	_hRes->handle.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	_hRes->handle.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR; /* ADC DMA circular requested */
	_hRes->handle.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
	_hRes->handle.Init.OversamplingMode         = DISABLE;                       /* No oversampling */
	_hRes->handle.Init.BoostMode                = ENABLE;                        /* Enable Boost mode as ADC clock frequency is bigger than 20 MHz */

	ConfigureDMA();

	if (HAL_ADC_Init(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not initialize ADC");
		return;
	}

	/*Configure the ADC multi-mode */
	/*ADC_MultiModeTypeDef multimode = {0};
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&_hRes->handle, &multimode) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure ADC");
		return;
	}*/

	/* Run the ADC calibration in single-ended mode */
	if (HAL_ADCEx_Calibration_Start(&_hRes->handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not calibrate ADC");
		return;
	}

	/*
	// Start continuous ADC conversion
	if (HAL_ADC_Start(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not start ADC");
		return;
	}*/
}

void ADC::ConfigureADCGPIO()
{
	if (!_hRes) return;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;

	if (_hRes->adc == ADC_1)
	{
		/**ADC1 GPIO Configuration
		PC5     ------> ADC1_INP8
		PF11     ------> ADC1_INP2
		*/
		if (_channel == ADC_CHANNEL_8) {
			GPIO_InitStruct.Pin = GPIO_PIN_5;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_2) {
			GPIO_InitStruct.Pin = GPIO_PIN_11;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
	else if (_hRes->adc == ADC_2)
	{
		/**ADC2 GPIO Configuration
		PC0     ------> ADC2_INP10
		PC1     ------> ADC2_INP11
		*/
		if (_channel == ADC_CHANNEL_10) {
			GPIO_InitStruct.Pin = GPIO_PIN_0;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_11) {
			GPIO_InitStruct.Pin = GPIO_PIN_1;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
	else if (_hRes->adc == ADC_3)
	{
		/**ADC3 GPIO Configuration
		PF3     ------> ADC3_INP5
		PF4     ------> ADC3_INP9
		PF5     ------> ADC3_INP4
		PF10     ------> ADC3_INP6
		PC2_C     ------> ADC3_INP0
		*/
		if (_channel == ADC_CHANNEL_5) {
			GPIO_InitStruct.Pin = GPIO_PIN_3;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_9) {
			GPIO_InitStruct.Pin = GPIO_PIN_4;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_4) {
			GPIO_InitStruct.Pin = GPIO_PIN_5;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_6) {
			GPIO_InitStruct.Pin = GPIO_PIN_10;
			__HAL_RCC_GPIOF_CLK_ENABLE();
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
		}
		else if (_channel == ADC_CHANNEL_0) {
			GPIO_InitStruct.Pin = GPIO_PIN_2;
			__HAL_RCC_GPIOC_CLK_ENABLE();
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		}
		else {
			_hRes = 0;
			ERROR("Invalid ADC channel");
			return;
		}
	}
}

void ADC::ConfigureADCChannels()
{
	if (!_hRes) return;

	ADC_ChannelConfTypeDef sConfig = {0};

	/*sConfig.Channel = _channel;
	sConfig.Rank = ADC_REGULAR_RANK_1; // Rank of sampled channel number ADCx_CHANNEL
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; // Sampling time (number of clock cycles unit)
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&_hRes->handle, &sConfig) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not configure ADC channel");
		return;
	}

	// Start continuous ADC conversion
	if (HAL_ADC_Start(&_hRes->handle) != HAL_OK)
	{
		_hRes = 0;
		ERROR("Could not start ADC");
		return;
	}*/

	/* Loop through configured channels and configure rank according to map */
	for (int channel = 0; channel < sizeof(_hRes->map_channel2bufferIndex); channel++) {
		if (_hRes->map_channel2bufferIndex[channel] != 0xFF) {
			sConfig.Channel      = channel;                /* Sampled channel number */
			sConfig.Rank         = _hRes->map_channel2bufferIndex[channel] + 1; // ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL */
			sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;   /* Sampling time (number of clock cycles unit) */
			if (channel == ADC_CHANNEL_VBAT_DIV4 || channel == ADC_CHANNEL_TEMPSENSOR || channel == ADC_CHANNEL_VREFINT)
				sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;   /* Sampling time (number of clock cycles unit) */
			sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel */
			sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction */
			sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled */
			if (HAL_ADC_ConfigChannel(&_hRes->handle, &sConfig) != HAL_OK)
			{
				_hRes = 0;
				ERROR("Could not configure ADC channel");
				return;
			}
		}
	}

	  _hRes->bufferSize = 2*_hRes->numberOfConfiguredChannels; // 16-bit pr. channel = 2 bytes pr. channel

	  /* ### - 4 - Start conversion in DMA mode ################################# */
	  //if (StartDMA(&_hRes->handle,
	  if (HAL_ADC_Start_DMA(&_hRes->handle,
							(uint32_t *)_hRes->buffer,
							_hRes->numberOfConfiguredChannels  // just sample the number of channels into the buffer
						   ) != HAL_OK)
	  {
			_hRes = 0;
			ERROR("Could not start ADC");
			return;
	  }
}

// Return ADC reading between 0-1, where 1 corresponds to the ADC's Analog reference (Aref)
float ADC::Read()
{
	int32_t reading = ReadRaw();
	if (reading >= 0) {
		float converted = (float)reading / _hRes->range;
		return converted;
	} else {
		return 0;
	}
}

int32_t ADC::ReadRaw()
{
	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
	SCB_InvalidateDCache_by_Addr((uint32_t *) &_hRes->buffer[0], _hRes->bufferSize);
	return _hRes->buffer[_hRes->map_channel2bufferIndex[_channel]];
}

void ADC::ConfigureDMA(void)
{
	// See DMA mapping on page 226 in Reference Manual
	// Note that the below DMA and Stream seems incorrect when compared to this table?!

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable DMA clock */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*##- 3- Configure DMA #####################################################*/
	/*********************** Configure DMA parameters ***************************/
	if (_hRes->adc == ADC_1) {
		_hRes->DMA_handle.Instance = DMA1_Stream0;
		_hRes->DMA_handle.Init.Request = DMA_REQUEST_ADC1;
	} else if (_hRes->adc == ADC_2) {
		_hRes->DMA_handle.Instance = DMA1_Stream1;
		_hRes->DMA_handle.Init.Request = DMA_REQUEST_ADC2;
	} else if (_hRes->adc == ADC_3) {
		_hRes->DMA_handle.Instance = DMA1_Stream2;
		_hRes->DMA_handle.Init.Request = DMA_REQUEST_ADC3;
	}
	_hRes->DMA_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	_hRes->DMA_handle.Init.PeriphInc           = DMA_PINC_DISABLE;
	_hRes->DMA_handle.Init.MemInc              = DMA_MINC_ENABLE;
	_hRes->DMA_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	_hRes->DMA_handle.Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
	_hRes->DMA_handle.Init.Mode                = DMA_CIRCULAR;
	_hRes->DMA_handle.Init.Priority            = DMA_PRIORITY_MEDIUM;
	_hRes->DMA_handle.Init.FIFOMode 		   = DMA_FIFOMODE_DISABLE;
	/* Deinitialize  & Initialize the DMA for new transfer */
	HAL_DMA_DeInit(&_hRes->DMA_handle);
	HAL_DMA_Init(&_hRes->DMA_handle);

	/* Associate the DMA handle */
	__HAL_LINKDMA(&_hRes->handle, DMA_Handle, _hRes->DMA_handle);

	/* NVIC configuration for DMA Input data interrupt */
	/*if (_hRes->adc == ADC_1) {
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	} else if (_hRes->adc == ADC_2) {
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	} else if (_hRes->adc == ADC_3) {
		HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	}*/
}

/**
  * @brief  Enable ADC, start conversion of regular group and transfer result through DMA.
  * @note   Interruptions enabled in this function:
  *         overrun (if applicable), DMA half transfer, DMA transfer complete.
  *         Each of these interruptions has its dedicated callback function.
  * @note   Case of multimode enabled (when multimode feature is available): HAL_ADC_Start_DMA()
  *         is designed for single-ADC mode only. For multimode, the dedicated
  *         HAL_ADCEx_MultiModeStart_DMA() function must be used.
  * @param  hadc: ADC handle
  * @param  pData: Destination Buffer address.
  * @param  Length: Length of data to be transferred from ADC peripheral to memory (in bytes)
  * @retval HAL status.
  */
// Based on HAL_ADC_Start_DMA - but interrupt part removed
HAL_StatusTypeDef ADC::StartDMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
{
  HAL_StatusTypeDef tmp_hal_status = HAL_OK;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_INSTANCE(hadc->Instance));

  /* Perform ADC enable and conversion start if no conversion is on going */
  if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
  {
	/* Process locked */
	__HAL_LOCK(hadc);

	/* Ensure that dual regular conversions are not enabled or unavailable.   */
	/* Otherwise, dedicated API HAL_ADCEx_MultiModeStart_DMA() must be used.  */
	if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET)
	{
	  /* Enable the ADC peripheral */
	  tmp_hal_status = ADC_Enable(hadc);

	  /* Start conversion if ADC is effectively enabled */
	  if (tmp_hal_status == HAL_OK)
	  {
		/* State machine update: Check if an injected conversion is ongoing */
		if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
		  /* Reset ADC error code fields related to regular conversions only */
		  CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
		}
		else
		{
		  /* Set ADC error code to none */
		  ADC_CLEAR_ERRORCODE(hadc);
		}
		/* Clear HAL_ADC_STATE_READY and regular conversion results bits, set HAL_ADC_STATE_REG_BUSY bit */
		ADC_STATE_CLR_SET(hadc->State,
			              (HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP),
			              HAL_ADC_STATE_REG_BUSY);

		/* Reset HAL_ADC_STATE_MULTIMODE_SLAVE bit
		   - by default if ADC is Master or Independent or if multimode feature is not available
		   - if multimode setting is set to independent mode (no dual regular or injected conversions are configured) */
		if (ADC12_NONMULTIMODE_OR_MULTIMODEMASTER(hadc))
		{
		  CLEAR_BIT(hadc->State, HAL_ADC_STATE_MULTIMODE_SLAVE);
		}

		/* Set the DMA transfer complete callback */
		hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

		/* Set the DMA half transfer complete callback */
		hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

		/* Set the DMA error callback */
		hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


		/* Manage ADC and DMA start: ADC overrun interruption, DMA start,     */
		/* ADC start (in case of SW start):                                   */

		/* Clear regular group conversion flag and overrun flag               */
		/* (To ensure of no unknown state from potential previous ADC         */
		/* operations)                                                        */
		__HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

		/* With DMA, overrun event is always considered as an error even if
		   hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore,
		   ADC_IT_OVR is enabled.  */
		__HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

		/* Start the DMA channel */
		HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

		/* Enable conversion of regular group.                                  */
		/* Process unlocked */
		__HAL_UNLOCK(hadc);
		/* If software start has been selected, conversion starts immediately.  */
		/* If external trigger has been selected, conversion will start at next */
		/* trigger event.                                                       */
		SET_BIT(hadc->Instance->CR, ADC_CR_ADSTART);
	  }
	  else
	  {
		/* Process unlocked */
		__HAL_UNLOCK(hadc);
	  }  /* if (tmp_hal_status == HAL_OK) */
	}
	else
	{
	  tmp_hal_status = HAL_ERROR;
	  /* Process unlocked */
	  __HAL_UNLOCK(hadc);
	} /* if (ADC_IS_DUAL_REGULAR_CONVERSION_ENABLE(hadc) == RESET) */

  }
  else
  {
	tmp_hal_status = HAL_BUSY;
  }

  /* Return function status */
  return tmp_hal_status;
}

#if 0
/**
  * @brief ADC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();
  /* ADC Periph clock disable
   (automatically reset all ADC instances of the ADC common group) */
  __HAL_RCC_ADC12_CLK_DISABLE();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize the ADC Channel GPIO pin */
  HAL_GPIO_DeInit(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN);
}
#endif

/**
  * @brief  Conversion complete callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADC::hardware_resource_t * adc = 0;
	if (hadc->Instance == ADC1)
		adc = ADC::resADC1;
	else if (hadc->Instance == ADC2)
		adc = ADC::resADC2;
	else if (hadc->Instance == ADC3)
		adc = ADC::resADC3;

	if (!adc) {
		ERROR("ADC interrupt for unconfigured ADC");
		return;
	}

	/* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
	SCB_InvalidateDCache_by_Addr((uint32_t *) &adc->buffer[0], adc->bufferSize);
}

/**
  * @brief  Conversion DMA half-transfer callback in non-blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADC::hardware_resource_t * adc = 0;
	if (hadc->Instance == ADC1)
		adc = ADC::resADC1;
	else if (hadc->Instance == ADC2)
		adc = ADC::resADC2;
	else if (hadc->Instance == ADC3)
		adc = ADC::resADC3;

	if (!adc) {
		ERROR("ADC interrupt for unconfigured ADC");
		return;
	}

	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
	SCB_InvalidateDCache_by_Addr((uint32_t *) &adc->buffer[adc->bufferSize/2], adc->bufferSize);
}

void DMA1_Stream0_IRQHandler(void)
{
	if (!ADC::resADC1) return;
	HAL_DMA_IRQHandler(ADC::resADC1->handle.DMA_Handle);
}

void DMA1_Stream1_IRQHandler(void)
{
	if (!ADC::resADC2) return;
	HAL_DMA_IRQHandler(ADC::resADC2->handle.DMA_Handle);
}

void DMA1_Stream2_IRQHandler(void)
{
	if (!ADC::resADC3) return;
	HAL_DMA_IRQHandler(ADC::resADC3->handle.DMA_Handle);
}

float ADC::GetCoreTemp(void) {
	// requires the ADC channel of the current object to be configured as ADC_CHANNEL_TEMPSENSOR
	if (_channel != ADC_CHANNEL_TEMPSENSOR) return 0;

    int32_t raw_value = ReadRaw();
    float core_temp_avg_slope = (ADC_CAL2 - ADC_CAL1) / 80.0;
    return (((float)raw_value * ADC_REF_CORR - ADC_CAL1) / core_temp_avg_slope) + 30.0f;
}

float ADC::GetCoreVBAT(void) {
	// requires the ADC channel of the current object to be configured as ADC_CHANNEL_VBAT_DIV4
	if (_channel != ADC_CHANNEL_VBAT_DIV4) return 0;

    uint16_t raw_value = ReadRaw();
    return raw_value * VBAT_DIV * (VREF_CAL_VDD / (float)(((uint32_t)1 << ADC_CAL_BITS) - 1)) * ADC_REF_CORR;
}

float ADC::GetCoreVREF(void) {
	// requires the ADC channel of the current object to be configured as ADC_CHANNEL_VREFINT
	if (_channel != ADC_CHANNEL_VREFINT) return 0;

    uint32_t raw_value = (uint32_t)ReadRaw();

    // update the reference correction factor
    ADC_REF_CORR = ((float)VREFIN_CAL) / ((float)raw_value);

    return VREFIN_CAL * (VREF_CAL_VDD / (float)(((uint32_t)1 << ADC_CAL_BITS) - 1));
}


uint16_t ADC::GetVREFINTraw(void)
{
	if (!ADC::resADC3) return 0; // only available if ADC3 is configured

	/* Invalidate Data Cache to get the updated content of the SRAM on the second half of the ADC converted data buffer: 32 bytes */
	SCB_InvalidateDCache_by_Addr((uint32_t *) &ADC::resADC3->buffer[0], ADC::resADC3->bufferSize);
	return ADC::resADC3->buffer[ADC::resADC3->map_channel2bufferIndex[ADC_CHANNEL_VREFINT]];
}

float ADC::GetVREF(void)
{
	if (!ADC::resADC3) return 0; // only available if ADC3 is configured

	// Compute VREF+ value based on VREFINT measurement and calibration
	// VREFINT calibration is taken with 3.3V VREF+
	//    (VREFINT_MEAS / RANGE) * VREF_CURR = (VREFINT_CAL / RANGE) * 3.3V
	return VREF_CAL_VDD * ((float)(VREFIN_CAL) / ADC_CAL_RANGE) / ((float)GetVREFINTraw() / ADC::resADC3->range);
}

// Return ADC reading in volt based on VREFINT correction (if ADC3 is enabled), otherwise based on 3.3V VREF+ assumption
float ADC::ReadVoltage()
{
	float percentual = Read();

	float voltage;
	if (ADC::resADC3) {
		// VREF available, use to compute voltage
		voltage = GetVREF() * percentual;
	} else {
		voltage = VREF_CAL_VDD * percentual;
	}

	return voltage;
}
