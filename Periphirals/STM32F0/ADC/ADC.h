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
 
#ifndef PERIPHIRALS_ADC_H
#define PERIPHIRALS_ADC_H

#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_dma.h"

/* Macro to get variable aligned on 32-bytes,needed for cache maintenance purpose */
#if defined   (__GNUC__)        /* GNU Compiler */
  #define ALIGN_32BYTES(buf)  buf __attribute__ ((aligned (32)))
#elif defined (__ICCARM__)    /* IAR Compiler */
  #define ALIGN_32BYTES(buf) _Pragma("data_alignment=32") buf
#elif defined   (__CC_ARM)      /* ARM Compiler */
  #define ALIGN_32BYTES(buf) __align(32) buf
#endif

class ADC
{
	private:
		const uint32_t ADC_DEFAULT_RESOLUTION = LL_ADC_RESOLUTION_12B;

		/* Core temperature sensor definitions */
		const uint16_t CORE_TEMP_V25		=	943;  /* (0.76v/3.3v)*(2^ADC resoultion) */
		const uint16_t CORE_TEMP_AVG_SLOPE	=	3;    /* (2.5mv/3.3v)*(2^ADC resoultion) */

		// scale and calibration values for VBAT and VREF
		const float VREF_CAL_VDD = 3.3f; // VREF+ voltage used while calibrating the VREFIN_CAL
		const uint16_t &VREFIN_CAL = *((uint16_t*)(VREFINT_CAL_ADDR)); // ADC_CAL_ADDRESS = (0x1FF1E860);
		const uint32_t &ADC_CAL1		=	*((uint16_t*)(TEMPSENSOR_CAL1_ADDR));
		const uint32_t &ADC_CAL2		=	*((uint16_t*)(TEMPSENSOR_CAL2_ADDR));
		const uint8_t ADC_CAL_BITS		=	16;
		const uint32_t ADC_CAL_RANGE	=	((uint32_t)1 << ADC_CAL_BITS) - 1;
		const uint8_t VBAT_DIV			=	4;

	public:
		typedef enum adc_t {
			ADC_UNDEFINED = 0,
			ADC_1
		} adc_t;

		typedef enum adc_channel_t {
			ADC_CHANNEL_0 = 0,
			ADC_CHANNEL_1,
			ADC_CHANNEL_2,
			ADC_CHANNEL_3,
			ADC_CHANNEL_4,
			ADC_CHANNEL_5,
			ADC_CHANNEL_6,
			ADC_CHANNEL_7,
			ADC_CHANNEL_8,
			ADC_CHANNEL_9,
			ADC_CHANNEL_10,
			ADC_CHANNEL_11,
			ADC_CHANNEL_12,
			ADC_CHANNEL_13,
			ADC_CHANNEL_14,
			ADC_CHANNEL_15,
			ADC_CHANNEL_16,
			ADC_CHANNEL_17,
			ADC_CHANNEL_TEMPSENSOR = 16,
			ADC_CHANNEL_VREFINT = 17,
			ADC_CHANNEL_VBAT = 18
		} adc_channel_t;

	public:
		ADC(adc_t adc, adc_channel_t channel);
		ADC(adc_t adc, adc_channel_t channel, uint32_t resolution);
		~ADC();

		void InitPeripheral(adc_t adc, uint32_t resolution);
		void ConfigureADCPeripheral();
		void ConfigureADCGPIO();
		void ConfigureADCChannels();
		void ConfigureDMA(void);

		float Read();
		int32_t ReadRaw();
		int32_t Read_mVolt();

		float ReadVoltage();

	public:
		typedef struct hardware_resource_t {
			adc_t adc;
			uint32_t resolution;
			uint32_t range;
			uint8_t numberOfConfiguredChannels;
			ADC_TypeDef * instance;
			DMA_TypeDef * DMA;
			ALIGN_32BYTES (uint16_t buffer[16]); // 32-bytes Alignement is needed for cache maintenance purpose
			uint8_t bufferSize;
			uint8_t map_channel2bufferIndex[20];
		} hardware_resource_t;

		static hardware_resource_t * resADC1;
		static float ADC_REF_CORR;

		static uint16_t GetCoreTemperature(void);
		static uint16_t GetVREFINTraw(void);
		static float GetVREF(void);

	private:
		hardware_resource_t * _hRes;
		adc_channel_t _channel;
};
	
	
#endif
