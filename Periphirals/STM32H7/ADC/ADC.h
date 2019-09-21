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

#include "stm32h7xx_hal.h"

class ADC
{
	private:
		const uint32_t ADC_DEFAULT_RESOLUTION = ADC_RESOLUTION_12B;

		/* Core temperature sensor definitions */
		const uint16_t CORE_TEMP_V25		=	943;  /* (0.76v/3.3v)*(2^ADC resoultion) */
		const uint16_t CORE_TEMP_AVG_SLOPE	=	3;    /* (2.5mv/3.3v)*(2^ADC resoultion) */

		// scale and calibration values for VBAT and VREF
		const float VREF_CAL_VDD = 3.3f; // VREF+ voltage used while calibrating the VREFIN_CAL
		const uint16_t &VREFIN_CAL = *((uint16_t*)(0x1FF1E860)); // ADC_CAL_ADDRESS = (0x1FF1E860);
		//const uint32_t * ADC_CAL1		=	((uint16_t*)(0x1FF1E820));
		//const uint32_t * ADC_CAL2		=	((uint16_t*)(0x1FF1E840));
		const uint32_t &ADC_CAL1		=	*((uint16_t*)(0x1FF1E820));
		const uint32_t &ADC_CAL2		=	*((uint16_t*)(0x1FF1E840));
		const uint8_t ADC_CAL_BITS		=	16;
		const uint32_t ADC_CAL_RANGE	=	((uint32_t)1 << ADC_CAL_BITS) - 1;
		const uint8_t VBAT_DIV			=	4;

	public:
		typedef enum adc_t {
			ADC_UNDEFINED = 0,
			ADC_1,
			ADC_2,
			ADC_3,
		} adc_t;

	public:
		ADC(adc_t adc, uint32_t channel);
		ADC(adc_t adc, uint32_t channel, uint32_t resolution);
		~ADC();

		void InitPeripheral(adc_t adc, uint32_t resolution);
		void ConfigureADCPeripheral();
		void ConfigureADCGPIO();
		void ConfigureADCChannels();
		void ConfigureDMA(void);
		HAL_StatusTypeDef StartDMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);

		float Read();
		int32_t ReadRaw();

		float GetCoreTemp(void);
		float GetCoreVBAT(void);
		float GetCoreVREF(void);
		uint16_t GetVREFINTraw(void);
		float GetVREF(void);
		float ReadVoltage();

	public:
		typedef struct hardware_resource_t {
			adc_t adc;
			uint32_t resolution;
			uint32_t range;
			uint8_t numberOfConfiguredChannels;
			ADC_HandleTypeDef handle;
			DMA_HandleTypeDef DMA_handle;
			ALIGN_32BYTES (uint16_t buffer[16]); // 32-bytes Alignement is needed for cache maintenance purpose
			uint8_t bufferSize;
			uint8_t map_channel2bufferIndex[20];
		} hardware_resource_t;

		static hardware_resource_t * resADC1;
		static hardware_resource_t * resADC2;
		static hardware_resource_t * resADC3;
		static float ADC_REF_CORR;

	private:
		hardware_resource_t * _hRes;
		uint32_t _channel;
};
	
	
#endif
