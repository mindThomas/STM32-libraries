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
 
#ifndef PERIPHIRALS_SYNCEDPWMADC_H
#define PERIPHIRALS_SYNCEDPWMADC_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_hal_tim_ex.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"
#include "stm32g4xx_hal_dma.h"

#include "IO.h"
#include <unordered_map>

/* To Do
 *  - Allow switching between Brake mode and Coast mode
 *  - Define and use NVIC priorities from Priorities.h file
 */

/* Macro to get variable aligned on 32-bytes,needed for cache maintenance purpose */
#if defined   (__GNUC__)        /* GNU Compiler */
  #define ALIGN_32BYTES(buf)  buf __attribute__ ((aligned (32)))
#elif defined (__ICCARM__)    /* IAR Compiler */
  #define ALIGN_32BYTES(buf) _Pragma("data_alignment=32") buf
#elif defined   (__CC_ARM)      /* ARM Compiler */
  #define ALIGN_32BYTES(buf) __align(32) buf
#endif

// When a DMA sampling is started/triggered it will fill up the buffer. Note that this should b
// OBS. This should be a multiple of the number of channels sampled, otherwise it will overflow
#define ADC_DMA_HALFWORD_BUFFER_SIZE 4


class SyncedPWMADC
{
	#include "SyncedPWMADC_Config.h"

	private:
		const uint16_t ADC_SAMPLE_TIME_OVERHEAD_US = 1; // add 1 us ADC sample overhead time on top of the pre-computed ADC sample+conversion time

		// Debugging parameters
		const bool DEBUG_MODE_ENABLED = true;
		const bool ENABLE_DEBUG_OPAMP_OUTPUT = false; // OBS. Current sense sampling will only work properly when driving forward
		const bool SAMPLE_IN_EVERY_PWM_CYCLE = false;

	public:
		SyncedPWMADC();
		~SyncedPWMADC();

	public:
		typedef enum {
			BRAKE,
			COAST
		} OperatingMode_t;

	private:
		// PWM parameters
		uint32_t _PWM_frequency; // Hz
		uint32_t _PWM_prescaler;
		uint16_t _PWM_maxValue;
		float _PWM_dutyCycle;
		bool _Direction; // Forward = true

		// Sampling parameters
		uint16_t _samplingInterval; // requires SAMPLE_IN_EVERY_PWM_CYCLE=false - number of PWM periods/cycles between each sample capture. If set to 1 it will sample in every PWM period.

		// ADC samples
		ALIGN_32BYTES (uint16_t _ADC1_buffer[ADC_DMA_HALFWORD_BUFFER_SIZE]){0}; // 32-bytes Alignement is needed for cache maintenance purpose
		ALIGN_32BYTES (uint16_t _ADC2_buffer[ADC_DMA_HALFWORD_BUFFER_SIZE]){0}; // 32-bytes Alignement is needed for cache maintenance purpose

		OperatingMode_t _OperatingMode{BRAKE};

		typedef struct {
			uint8_t ADC{0};
			uint8_t Index{0}; // rank (when in the sequence is this channel sampled)
		} SampleLocation;
		struct {
			SampleLocation VSENSE1;
			SampleLocation VSENSE2;
			SampleLocation VSENSE3;
			SampleLocation POTENTIOMETER;
			SampleLocation VBUS;
			SampleLocation BEMF1;
			SampleLocation BEMF2;
			SampleLocation BEMF3;
			SampleLocation VREF;
			uint8_t ADCnumChannels[2]; // number of channels sampled for each ADC
		} Channels;

		typedef struct {
			float ValueON{0.0f};
			float ValueOFF{0.0f};
			uint32_t Timestamp{0}; // recent update timestamp
		} Sample;
		struct {
			Sample Vref;
			Sample CurrentSense;
			Sample Bemf;
			Sample Vbus;
		} Samples;

	public:
		bool _DMA_ADC1_ongoing; // should be moved to private
		bool _DMA_ADC2_ongoing; // should be moved to private

	public:
		void SetOperatingMode(OperatingMode_t mode);
		void SetPWMFrequency(uint32_t frequency);
		void SetDutyCycle(float duty);

		void SetTimerFrequencyWith50pctDutyCycle(uint32_t freq, bool direction); // Forward: direction = true
		void SetTimerFrequencyAndDutyCycle_MiddleSampling(uint32_t freq, float dutyPct);
		void SetTimerFrequencyAndDutyCycle_EndSampling(uint32_t freq, float dutyPct);
		void SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(uint32_t freq, float dutyPct);

	public:
		static void TriggerSample(SyncedPWMADC * obj);
		static void SamplingCompleted(SyncedPWMADC * obj, uint8_t ADC);
		static void TimerCaptureCallback(SyncedPWMADC * obj);

	// Debug properties
	public:
		void Debug_SetSamplingPin(IO * pin);
	private:
		IO * samplingPin{0};

	public:
		static SyncedPWMADC * globalObject;
};
	
	
#endif
