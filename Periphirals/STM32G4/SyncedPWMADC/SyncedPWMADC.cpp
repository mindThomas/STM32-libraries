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
 
#ifndef USE_FREERTOS
#include <malloc.h>
#endif

SyncedPWMADC * SyncedPWMADC::globalObject = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
extern "C" __EXPORT void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
extern "C" __EXPORT void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

SyncedPWMADC::SyncedPWMADC()
{
	if (globalObject) {
		ERROR("Synced PWM/ADC object has already been created once");
		return;
	}
	globalObject = this;

	// Default settings
	_PWM_frequency = 5000;
	_PWM_maxValue = 1000;
	_PWM_dutyCycle = 0.5;
	_Direction = true;
	_samplingInterval = 1; // sample every PWM period
	_DMA_ADC1_ongoing = false;
	_DMA_ADC2_ongoing = false;
	_TimerEnabled = false;
	_SamplingEnabled = false;

	ConfigureDigitalPins();
	ConfigureAnalogPins();
	InitOpAmps();
	InitADCs(); // configures ADC in Current sense mode
	InitDMAs();
	InitTimer(); // configures PWM in Brake mode

	StartSampling();
	StartPWM();
}

SyncedPWMADC::~SyncedPWMADC()
{
	StopPWM();
	StopSampling();

	DeInitTimer();
	DeInitDMAs();
	DeInitADCs();
	DeInitOpAmps();
	DeInitAnalogPins();
	DeInitDigitalPins();
}

void SyncedPWMADC::SetOperatingMode(OperatingMode_t mode)
{
	_OperatingMode = mode;

	if (_OperatingMode == BRAKE) {
		ADC_ConfigureCurrentSenseSampling();
		Timer_ConfigureBrakeMode();
	}
	else if (_OperatingMode == COAST) {
		ADC_ConfigureBackEMFSampling();
		Timer_ConfigureCoastMode();
	}
}

void SyncedPWMADC::SetPWMFrequency(uint32_t frequency)
{
	_PWM_frequency = frequency;
}

void SyncedPWMADC::SetDutyCycle(float duty)
{
	_PWM_dutyCycle = duty;

	if (_PWM_dutyCycle > 1.f)
		_PWM_dutyCycle = 1.0f;
	else if (_PWM_dutyCycle < -1.f)
		_PWM_dutyCycle = -1.0f;

	if (_PWM_dutyCycle > 0)
		_Direction = true;
	else if (_PWM_dutyCycle < 0)
		_Direction = false;
}


void SyncedPWMADC::SetTimerFrequencyWith50pctDutyCycle(uint32_t freq, bool direction)
{
	const uint32_t PCLK = HAL_RCC_GetPCLK2Freq();
	const uint16_t ADC_SAMPLE_TIME_US = _ADC_SampleTime_Total_us + ADC_SAMPLE_TIME_OVERHEAD_US;

	uint32_t ARR = (PCLK / (uint32_t)(freq*_PWM_prescaler)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&hTimer, ARR);

	// Recompute actual frequency
	_PWM_frequency = PCLK / ((ARR+1) * _PWM_prescaler);

	_Direction = direction;
	if (direction)
		_PWM_dutyCycle = 0.5;
	else
		_PWM_dutyCycle = -0.5;

	uint16_t END = ARR + 1;
	uint16_t CENTER = END / 2;
	uint16_t ADCsampleCnt = (END*_PWM_frequency) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = CENTER - ADCsampleCnt;
	uint16_t sampleLocation2 = END - ADCsampleCnt;

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (_PWM_dutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, CENTER);
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, CENTER);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0);
	}

	if (_OperatingMode == COAST) {
		SetCoastModeTimerConfiguration(_Direction);
	}
}

void SyncedPWMADC::SetTimerFrequencyAndDutyCycle_MiddleSampling(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = HAL_RCC_GetPCLK2Freq();
	const uint16_t ADC_SAMPLE_TIME_US = _ADC_SampleTime_Total_us + ADC_SAMPLE_TIME_OVERHEAD_US;

	uint32_t ARR = (PCLK / (uint32_t)(freq*_PWM_prescaler)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&hTimer, ARR);

	// Recompute actual frequency
	_PWM_frequency = PCLK / ((ARR+1) * _PWM_prescaler);
	_PWM_dutyCycle = dutyPct;
	if (_PWM_dutyCycle > 1.f)
		_PWM_dutyCycle = 1.0f;
	else if (_PWM_dutyCycle < -1.f)
		_PWM_dutyCycle = -1.0f;

	if (_PWM_dutyCycle > 0)
		_Direction = true;
	else if (_PWM_dutyCycle < 0)
		_Direction = false;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = roundf((float)END * fabsf(_PWM_dutyCycle));
	uint16_t ADCsampleCnt = (END*_PWM_frequency) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	uint16_t sampleLocation1 = 1;
	uint16_t sampleLocation2 = 0;

	if (ADCsampleCnt < CENTER/2) {
		sampleLocation1 = CENTER/2 - ADCsampleCnt; // sample location - at the midpoint of the ON-period minus the sample duration
	} // else sampleLocation1 = 1;
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);

	if (((END - CENTER) / 2 + CENTER - ADCsampleCnt) > (sampleLocation1 + ADCsampleCnt)) {
		sampleLocation2 = (END - CENTER) / 2 + CENTER - ADCsampleCnt;  // sample location - at the midpoint of the OFF-period minus the sample duration
	}
	else if ((END - ADCsampleCnt) > (sampleLocation1 + ADCsampleCnt)) {
		sampleLocation2 = (END - ADCsampleCnt);
	} // else sampleLocation2 = 0; // disabled
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2); // sample location - at the midpoint of the OFF-period minus the sample duration

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2); // sample location - at the midpoint of the OFF-period minus the sample duration
	}

	if (_PWM_dutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, CENTER); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, CENTER); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		SetCoastModeTimerConfiguration(_Direction);
	}
}

void SyncedPWMADC::SetTimerFrequencyAndDutyCycle_EndSampling(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = HAL_RCC_GetPCLK2Freq();
	const uint16_t ADC_SAMPLE_TIME_US = _ADC_SampleTime_Total_us + ADC_SAMPLE_TIME_OVERHEAD_US;

	uint32_t ARR = (PCLK / (uint32_t)(freq*_PWM_prescaler)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&hTimer, ARR);

	// Recompute actual frequency
	_PWM_frequency = PCLK / ((ARR+1) * _PWM_prescaler);
	_PWM_dutyCycle = dutyPct;
	if (_PWM_dutyCycle > 1.f)
		_PWM_dutyCycle = 1.0f;
	else if (_PWM_dutyCycle < -1.f)
		_PWM_dutyCycle = -1.0f;

	if (_PWM_dutyCycle > 0)
		_Direction = true;
	else if (_PWM_dutyCycle < 0)
		_Direction = false;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = roundf((float)END * fabsf(_PWM_dutyCycle));
	uint16_t ADCsampleCnt = (END*_PWM_frequency) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = 1;
	uint16_t sampleLocation2 = 0;

	if (CENTER > ADCsampleCnt) {
		sampleLocation1 = CENTER - ADCsampleCnt;
	}
	if (END > ADCsampleCnt) {
		sampleLocation2 = END - ADCsampleCnt;
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (_PWM_dutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, CENTER); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, CENTER); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		SetCoastModeTimerConfiguration(_Direction);
	}
}

void SyncedPWMADC::SetTimerFrequencyAndDutyCycle_MiddleSamplingOnce(uint32_t freq, float dutyPct)
{
	const uint32_t PCLK = HAL_RCC_GetPCLK2Freq();
	const uint16_t ADC_SAMPLE_TIME_US = _ADC_SampleTime_Total_us + ADC_SAMPLE_TIME_OVERHEAD_US;

	uint32_t ARR = (PCLK / (uint32_t)(freq*_PWM_prescaler)) - 1;
	__HAL_TIM_SET_AUTORELOAD(&hTimer, ARR);

	// Recompute actual frequency
	_PWM_frequency = PCLK / ((ARR+1) * _PWM_prescaler);
	_PWM_dutyCycle = dutyPct;
	if (_PWM_dutyCycle > 1.f)
		_PWM_dutyCycle = 1.0f;
	else if (_PWM_dutyCycle < -1.f)
		_PWM_dutyCycle = -1.0f;

	if (_PWM_dutyCycle > 0)
		_Direction = true;
	else if (_PWM_dutyCycle < 0)
		_Direction = false;

	uint16_t END = ARR - 1; // minus one to fix problem with 100% duty cycle issue (should have been +1)
	uint16_t CENTER = roundf((float)END * fabsf(_PWM_dutyCycle));
	uint16_t ADCsampleCnt = (END*_PWM_frequency) / (1000000 / ADC_SAMPLE_TIME_US); // convert ADC sample time (ADC_SAMPLE_TIME_US) to percentage of PWM frequency (freq) and multiply with timer count (END)

	uint16_t sampleLocation = 1;

	if (dutyPct > 0.5) {
		sampleLocation = CENTER/2 - ADCsampleCnt; // sample location - at the midpoint of the ON-period minus the sample duration
	} else {
		sampleLocation = (END - CENTER) / 2 + CENTER - ADCsampleCnt;  // sample location - at the midpoint of the OFF-period minus the sample duration
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, 0); // Only sample once

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, 0); // Only sample once
	}

	if (_PWM_dutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, CENTER); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, CENTER); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		SetCoastModeTimerConfiguration(_Direction);
	}
}

// ToDo: OBS! It's a very bad idea to put these callbacks in here.
// Consider to enable callback registration mechanism within HAL library or to handle the interrupt manually.
// Capture-Compare interrupt - used for debugging purposes
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (SyncedPWMADC::globalObject && htim->Instance == TIM1) {
		SyncedPWMADC::TimerCaptureCallback(SyncedPWMADC::globalObject);
	}
}

void SyncedPWMADC::TimerCaptureCallback(SyncedPWMADC * obj)
{
	if (!obj) return;

	if (obj->DEBUG_MODE_ENABLED && obj->samplingPin) {
		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE || obj->_DMA_ADC1_ongoing || obj->_DMA_ADC2_ongoing) {
			obj->samplingPin->High();
		}
	}
}

// Update interrupt - used to trigger DMA if SAMPLING_INTERVAL > 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (SyncedPWMADC::globalObject && htim->Instance == TIM1) {
		SyncedPWMADC::TriggerSample(SyncedPWMADC::globalObject);
	}
}

void SyncedPWMADC::TriggerSample(SyncedPWMADC * obj)
{
	if (!obj) return;

	/* Called by the Timer Update interrupt vector to trigger/initiate a DMA sample */
	if (obj->SAMPLE_IN_EVERY_PWM_CYCLE) return; // this method can only be used when sample triggering mode is manual (not happening at every PWM period)
										   // actually the method should not even have been called

	if (obj->_DMA_ADC1_ongoing || obj->_DMA_ADC2_ongoing) return; // Previous conversion not finished yet
	if (obj->hDMA_ADC1.State != HAL_DMA_STATE_READY || obj->hDMA_ADC2.State != HAL_DMA_STATE_READY) return;

	/* Start/Trigger the ADC1 DMA */
	if (obj->hDMA_ADC1.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC1);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC1, (uint32_t)&obj->hADC1.Instance->DR, (uint32_t)&obj->_ADC1_buffer[0], ADC_DMA_HALFWORD_BUFFER_SIZE);
	LL_ADC_REG_StartConversion(obj->hADC1.Instance);
	obj->_DMA_ADC1_ongoing = 1;

	/* Start/Trigger the ADC2 DMA */
	if (obj->hDMA_ADC2.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC2);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC2, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC2, (uint32_t)&obj->hADC2.Instance->DR, (uint32_t)&obj->_ADC2_buffer[0], ADC_DMA_HALFWORD_BUFFER_SIZE);
	LL_ADC_REG_StartConversion(obj->hADC2.Instance);
	obj->_DMA_ADC2_ongoing = 1;
}

// ToDo: OBS! It's a very bad idea to put these callbacks in here.
// Consider to enable callback registration mechanism within HAL library or to handle the interrupt manually.
// ADC conversion completed interrupt
// Called from the ADC_DMAConvCplt when the DMA conversion is done
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (SyncedPWMADC::globalObject)
	{
		if (hadc->Instance == ADC1) {
			SyncedPWMADC::SamplingCompleted(SyncedPWMADC::globalObject, 1);
		} else if (hadc->Instance == ADC2) {
			SyncedPWMADC::SamplingCompleted(SyncedPWMADC::globalObject, 2);
		}
	}
}

void SyncedPWMADC::SamplingCompleted(SyncedPWMADC * obj, uint8_t ADC)
{
	if (!obj) return;

	if (obj->DEBUG_MODE_ENABLED && obj->samplingPin) {
		obj->samplingPin->Low();
	}

	if (!obj->SAMPLE_IN_EVERY_PWM_CYCLE) {
		if (ADC == 1) {
			obj->_DMA_ADC1_ongoing = 0;
			LL_ADC_REG_StopConversion(obj->hADC1.Instance);
		}
		else if (ADC == 2) {
			obj->_DMA_ADC2_ongoing = 0;
			LL_ADC_REG_StopConversion(obj->hADC2.Instance);
		}
	}

	if (obj->SAMPLE_IN_EVERY_PWM_CYCLE ||
	   (obj->_DMA_ADC1_ongoing == 0 && obj->_DMA_ADC2_ongoing == 0)) {
		// Compute Vref voltage (based on __LL_ADC_CALC_VREFANALOG_VOLTAGE - with 12-bit ADC resolution)
		uint16_t * buf;
		uint8_t nc;

		if (obj->Channels.VREF.ADC) {
			if (obj->Channels.VREF.ADC == 1) {
				buf = obj->_ADC1_buffer;
				nc = obj->Channels.ADCnumChannels[0];
			} else {
				buf = obj->_ADC2_buffer;
				nc = obj->Channels.ADCnumChannels[1];
			}
			obj->Samples.Vref.ValueON = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index];
			obj->Samples.Vref.ValueOFF = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index + nc];
			obj->Samples.Vref.Timestamp = HAL_GetHighResTick();
		}

		if (obj->_Direction) { // Forward
			// Current sense from VSENSE1 since CH1 is LOW at all times
			if (obj->Channels.VSENSE1.ADC) {
				if (obj->Channels.VSENSE1.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				obj->Samples.CurrentSense.ValueON = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VSENSE1.Index] / 4096.f;
				obj->Samples.CurrentSense.ValueOFF = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VSENSE1.Index + nc] / 4096.f;
				obj->Samples.CurrentSense.Timestamp = HAL_GetHighResTick();
			}
		} else { // Backward
			// Current sense from VSENSE3 since CH3 is LOW at all times
			if (obj->Channels.VSENSE3.ADC) {
				if (obj->Channels.VSENSE3.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				obj->Samples.CurrentSense.ValueON = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VSENSE3.Index] / 4096.f;
				obj->Samples.CurrentSense.ValueOFF = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VSENSE3.Index + nc] / 4096.f;
				obj->Samples.CurrentSense.Timestamp = HAL_GetHighResTick();
			}
		}

		if (obj->_Direction) { // Forward
			// In forward direction it is CH3,CH3N that changes.
			// During COAST mode CH3N is forced to be LOW at all times, causing LOW PWM to deactivate the CH3,CH3N MOSFET
			// Back-EMF should thus be sampled from OUT3 = BEMF3
			if (obj->Channels.BEMF3.ADC) {
				if (obj->Channels.BEMF3.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				float v_bemf = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.BEMF3.Index] / 4096.f;
				obj->Samples.Bemf.ValueON = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;

				v_bemf = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.BEMF3.Index + nc] / 4096.f;
				obj->Samples.Bemf.ValueOFF = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				obj->Samples.Bemf.Timestamp = HAL_GetHighResTick();
			}
		} else { // Backward
			// In backward direction it is CH1,CH1N that changes.
			// During COAST mode CH1N is forced to be LOW at all times, causing LOW PWM to deactivate the CH1,CH1N MOSFET
			// Back-EMF should thus be sampled from OUT1 = BEMF1
			if (obj->Channels.BEMF1.ADC) {
				if (obj->Channels.BEMF1.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				float v_bemf = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.BEMF1.Index] / 4096.f;
				obj->Samples.Bemf.ValueON = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;

				v_bemf = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.BEMF1.Index + nc] / 4096.f;
				obj->Samples.Bemf.ValueOFF = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				obj->Samples.Bemf.Timestamp = HAL_GetHighResTick();
			}
		}

		if (obj->Channels.VBUS.ADC) {
			if (obj->Channels.VBUS.ADC == 1) {
				buf = obj->_ADC1_buffer;
				nc = obj->Channels.ADCnumChannels[0];
			} else {
				buf = obj->_ADC2_buffer;
				nc = obj->Channels.ADCnumChannels[1];
			}

			float v_vbus = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VBUS.Index] / 4096.f;
			obj->Samples.Vbus.ValueON = v_vbus * (18.f+169.f)/18.f;

			v_vbus = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VBUS.Index + nc] / 4096.f;
			obj->Samples.Vbus.ValueOFF = v_vbus * (18.f+169.f)/18.f;
			obj->Samples.Vbus.Timestamp = HAL_GetHighResTick();
		}

		nc = 0;
	}
}

void SyncedPWMADC::Debug_SetSamplingPin(IO * pin)
{
	if (DEBUG_MODE_ENABLED && pin) {
		samplingPin = pin;
	}
}
