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

// When a DMA sampling is started/triggered it will fill up the buffer.
// Maximum number of ADC samples the DMA buffer can hold
// _samplingNumSamples*NUM_CHANNELS*NUM_TRIGGERS <= ADC_DMA_HALFWORD_MAX_BUFFER_SIZE
#define ADC_DMA_HALFWORD_MAX_BUFFER_SIZE 16


class SyncedPWMADC
{
	#include "SyncedPWMADC_Config.h"

	private:
		const uint16_t ADC_SAMPLE_TIME_OVERHEAD_US = 1; // add 1 us ADC sample overhead time on top of the pre-computed ADC sample+conversion time
		const uint16_t ADC_PWM_RIPPLE_TIME_US = 8; // transient ripple from PWM level change - keep ADC sampling (whole ADC capture period) away from PWM level changes by at least this amount

		// Debugging parameters
		const bool DEBUG_MODE_ENABLED = true;
		const bool ENABLE_DEBUG_OPAMP_OUTPUT = true; // OBS. Current sense sampling will only work properly when driving forward
		const bool SAMPLE_IN_EVERY_PWM_CYCLE = false; // setting this to true effectively sets "_samplingInterval=1" and "_samplingInterval=1"

	public:
		SyncedPWMADC(uint32_t frequency = 10000, float maxDuty = 0.98);
		~SyncedPWMADC();

	public:
		typedef enum {
			BRAKE,
			COAST
		} OperatingMode_t;
		typedef struct {
			uint32_t Timestamp{0}; // recent update timestamp
			float ValueON{0.0f};
			float ValueOFF{0.0f};
		} Sample;
		typedef struct {
			uint32_t Timestamp{0}; // recent update timestamp
			float Value{0.0f};
		} SampleSingle;

	private:
		// PWM parameters
		//uint32_t _PWM_frequency; // Hz
		//uint32_t _PWM_prescaler;
		//float _PWM_dutyCycle;
		//bool _Direction; // Forward = true
		float _PWM_maxDuty;

		// Sampling parameters
		uint16_t _samplingInterval; // requires SAMPLE_IN_EVERY_PWM_CYCLE=false - number of PWM periods/cycles between each sample capture. If set to 1 it will sample in every PWM period.
		uint16_t _samplingNumSamples; // number of PWM period samples to capture after each sampling interval
									  // _samplingNumSamples <= _samplingInterval

		// ADC samples
		ALIGN_32BYTES (uint16_t _ADC1_buffer[ADC_DMA_HALFWORD_MAX_BUFFER_SIZE]){0}; // 32-bytes Alignement is needed for cache maintenance purpose
		ALIGN_32BYTES (uint16_t _ADC2_buffer[ADC_DMA_HALFWORD_MAX_BUFFER_SIZE]){0}; // 32-bytes Alignement is needed for cache maintenance purpose

		OperatingMode_t _OperatingMode{BRAKE};

		typedef struct {
			uint8_t ADC;
			uint8_t Index; // rank (when in the sequence is this channel sampled)
		} SampleLocation;
		typedef struct {
			bool Enabled;
			uint8_t Index; // index in sample sequence of PWM period, e.g. ON-period has Index=0 and OFF-period has Index=1 (partly redundant since this can be inferred from SampleLocation, but makes it easier for lookup)
			uint32_t Location; // timer count
		} TriggerLocation;
		typedef struct {
			uint8_t numEnabledTriggers; // number of enabled triggers within each PWM period (normally there is 2 triggers pr. PWM period = ON- and OFF-trigger)
			TriggerLocation ON;
			TriggerLocation OFF;
		} Triggers_t;
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
			uint32_t Frequency; // Hz
			uint32_t Prescaler;
			float DutyCycle;
			bool Direction; // Forward = true
			Triggers_t Triggers;
		} TimerSettings_t;
		TimerSettings_t timerSettingsCurrent;
		TimerSettings_t timerSettingsNext;

		struct {
			Sample Vref;
			Sample CurrentSense;
			Sample Bemf;
			Sample Vbus;
		} Samples;

		struct {
			uint16_t End; // end count (ARR+1)
			uint16_t Sample; // sample time in timer counts
			uint16_t Ripple;
		} PredefinedCounts;

	public:
		bool _DMA_ADC1_ongoing; // should be moved to private
		bool _DMA_ADC2_ongoing; // should be moved to private

	#ifdef USE_FREERTOS
		SemaphoreHandle_t _sampleAvailable;
	#else
		bool _sampleAvailable;
	#endif

	public:
		void SetOperatingMode(OperatingMode_t mode);
		void SetPWMFrequency(uint32_t frequency);
		void SetDutyCycle(float duty);
		float GetCurrentDutyCycle();

		void SetDutyCycle_MiddleSampling(float dutyPct);
		void SetDutyCycle_EndSampling(float dutyPct);
		void SetDutyCycle_MiddleSamplingOnce(float dutyPct);

		Sample GetCurrent();
		Sample GetVin();
		Sample GetBemf();
		SampleSingle GetPotentiometer();

		void WaitForNewSample();

	private:
		void RecomputePredefinedCounts();

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
