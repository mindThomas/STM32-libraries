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
extern "C" __EXPORT void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

SyncedPWMADC::SyncedPWMADC(uint32_t frequency, float maxDuty)
{
	if (globalObject) {
		ERROR("Synced PWM/ADC object has already been created once");
		return;
	}
	globalObject = this;

	// Default settings
	timerSettingsNext.Frequency = frequency;
	timerSettingsNext.DutyCycle = 0;
	timerSettingsNext.Direction = true;
	_PWM_maxDuty = maxDuty;
	if (SAMPLE_IN_EVERY_PWM_CYCLE) {
		_samplingInterval = 1; // sample every PWM period
	} else {
		_samplingInterval = 2; // sample every second PWM period (with SAMPLE_IN_EVERY_PWM_CYCLE=false, this is the fastest possible)
	}
	_samplingNumSamples = 1; // capture samples from one PWM period after every interval (hence every PWM period)
	_DMA_ADC1_ongoing = false;
	_DMA_ADC2_ongoing = false;
	_TimerEnabled = false;
	_SamplingEnabled = false;

	ConfigureDigitalPins();
	ConfigureAnalogPins();
	InitOpAmps();
	InitADCs(); // configures ADC in Current sense mode
	InitDMAs();
	InitTimer(frequency); // configures PWM in Brake mode
	RecomputePredefinedCounts();

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
		Timer_ConfigureCoastMode(timerSettingsCurrent.Direction);
	}
}

void SyncedPWMADC::SetPWMFrequency(uint32_t frequency)
{
	Timer_Configure(frequency);
	RecomputePredefinedCounts(); // recompute at different frequency
}

void SyncedPWMADC::SetDutyCycle(float duty)
{
	timerSettingsNext.DutyCycle = duty;

	if (timerSettingsNext.DutyCycle > _PWM_maxDuty)
		timerSettingsNext.DutyCycle = _PWM_maxDuty;
	else if (timerSettingsNext.DutyCycle < -_PWM_maxDuty)
		timerSettingsNext.DutyCycle = -_PWM_maxDuty;

	if (timerSettingsNext.DutyCycle > 0)
		timerSettingsNext.Direction = true;
	else if (timerSettingsNext.DutyCycle < 0)
		timerSettingsNext.Direction = false;
}

void SyncedPWMADC::RecomputePredefinedCounts()
{
	const uint32_t ADC_SAMPLE_TIME_US = _ADC_SampleTime_Total_us + ADC_SAMPLE_TIME_OVERHEAD_US;
	const uint16_t PWM_RIPPLE_TIME_US = ADC_PWM_RIPPLE_TIME_US;

	// Duty cycle is defined as:
	// Duty Cycle = (CCR+1) / (ARR+1)
	// So 100% duty cycle is achieved by setting CCR=ARR
	// Note that CCR is the Capture Compare Register value, computed as
	// CCR = (Duty * (ARR+1)) - 1
	PredefinedCounts.End = hTimer.Init.Period + 1; // ARR+1

	// Sample_DutyPct = ADC_SAMPLE_TIME_US*1e-6 / (1/PWM_frequency)
	float Sample_DutyPct = (float)((uint32_t)(timerSettingsNext.Frequency) * ADC_SAMPLE_TIME_US) / 1000000.f;
	PredefinedCounts.Sample = (uint16_t)(ceilf( fabsf(Sample_DutyPct)*PredefinedCounts.End ));

	float Ripple_DutyPct = (float)((uint32_t)(timerSettingsNext.Frequency) * PWM_RIPPLE_TIME_US) / 1000000.f;
	PredefinedCounts.Ripple = (uint16_t)(ceilf( fabsf(Ripple_DutyPct)*PredefinedCounts.End ));
}

void SyncedPWMADC::SetDutyCycle_MiddleSampling(float dutyPct)
{
	timerSettingsNext.DutyCycle = dutyPct;

	if (timerSettingsNext.DutyCycle > _PWM_maxDuty)
		timerSettingsNext.DutyCycle = _PWM_maxDuty;
	else if (timerSettingsNext.DutyCycle < -_PWM_maxDuty)
		timerSettingsNext.DutyCycle = -_PWM_maxDuty;

	if (timerSettingsNext.DutyCycle > 0)
		timerSettingsNext.Direction = true;
	else if (timerSettingsNext.DutyCycle < 0)
		timerSettingsNext.Direction = false;

	const uint16_t& End_Count = PredefinedCounts.End;
	const uint16_t& Ripple_Count = PredefinedCounts.Ripple;
	const uint16_t& Sample_Count = PredefinedCounts.Sample;
	// Duty cycle is defined as:
	// Duty Cycle = (CCR+1) / (ARR+1)
	// So 100% duty cycle is achieved by setting CCR=ARR
	const uint16_t Duty_Count = (uint16_t)(roundf( fabsf(timerSettingsNext.DutyCycle)*End_Count )); // note that from testing I have identified that it is more accurate not to subtract -1 here

	// Sanity check
	if (End_Count <= Sample_Count) {
		ERROR("PWM frequency is too fast for ADC sampling at the configured ADC sample time");
	}
	if (End_Count <= Ripple_Count) {
		ERROR("PWM frequency is faster than the expected ripple");
	}

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = 0;
	uint16_t sampleLocation2 = 0;

	// Find mid-point in ON period without LOW-to-HIGH switching ripple
	if ((Duty_Count+Ripple_Count)/2 > (Ripple_Count+Sample_Count)) {
		sampleLocation1 = (Duty_Count+Ripple_Count)/2 - Sample_Count;
	}
	// Find mid-point in OFF period without HIGH-to-LOW switching ripple
	if (((End_Count+Duty_Count+Ripple_Count)/2 - Sample_Count) > (Duty_Count+Ripple_Count)) {
		sampleLocation2 = (End_Count+Duty_Count+Ripple_Count)/2 - Sample_Count;
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (timerSettingsNext.DutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.Triggers.numEnabledTriggers = 0;
	if (sampleLocation1 > 0) {
		timerSettingsNext.Triggers.ON.Enabled = true;
		timerSettingsNext.Triggers.ON.Index = 0;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.ON.Enabled = false;
	}
	if (sampleLocation2 > 0) {
		timerSettingsNext.Triggers.OFF.Enabled = true;
		timerSettingsNext.Triggers.OFF.Index = timerSettingsNext.Triggers.numEnabledTriggers;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.OFF.Enabled = false;
	}
	timerSettingsNext.Triggers.ON.Location = sampleLocation1;
	timerSettingsNext.Triggers.OFF.Location = sampleLocation2;
}

void SyncedPWMADC::SetDutyCycle_EndSampling(float dutyPct)
{
	timerSettingsNext.DutyCycle = dutyPct;

	if (timerSettingsNext.DutyCycle > _PWM_maxDuty)
		timerSettingsNext.DutyCycle = _PWM_maxDuty;
	else if (timerSettingsNext.DutyCycle < -_PWM_maxDuty)
		timerSettingsNext.DutyCycle = -_PWM_maxDuty;

	if (timerSettingsNext.DutyCycle > 0)
		timerSettingsNext.Direction = true;
	else if (timerSettingsNext.DutyCycle < 0)
		timerSettingsNext.Direction = false;

	const uint16_t& End_Count = PredefinedCounts.End;
	const uint16_t& Ripple_Count = PredefinedCounts.Ripple;
	const uint16_t& Sample_Count = PredefinedCounts.Sample;
	// Duty cycle is defined as:
	// Duty Cycle = (CCR+1) / (ARR+1)
	// So 100% duty cycle is achieved by setting CCR=ARR
	const uint16_t Duty_Count = (uint16_t)(roundf( fabsf(timerSettingsNext.DutyCycle)*End_Count )); // note that from testing I have identified that it is more accurate not to subtract -1 here

	// Sanity check
	if (End_Count <= Sample_Count) {
		ERROR("PWM frequency is too fast for ADC sampling at the configured ADC sample time");
	}
	if (End_Count <= Ripple_Count) {
		ERROR("PWM frequency is faster than the expected ripple");
	}

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = 0;
	uint16_t sampleLocation2 = 0;

	if (Duty_Count > (Ripple_Count+Sample_Count)) {
		sampleLocation1 = Duty_Count - Sample_Count;
	}
	if ((End_Count - Sample_Count) > (Duty_Count+Ripple_Count)) {
		sampleLocation2 = End_Count - Sample_Count;
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (timerSettingsNext.DutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.Triggers.numEnabledTriggers = 0;
	if (sampleLocation1 > 0) {
		timerSettingsNext.Triggers.ON.Enabled = true;
		timerSettingsNext.Triggers.ON.Index = 0;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.ON.Enabled = false;
	}
	if (sampleLocation2 > 0) {
		timerSettingsNext.Triggers.OFF.Enabled = true;
		timerSettingsNext.Triggers.OFF.Index = timerSettingsNext.Triggers.numEnabledTriggers;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.OFF.Enabled = false;
	}
	timerSettingsNext.Triggers.ON.Location = sampleLocation1;
	timerSettingsNext.Triggers.OFF.Location = sampleLocation2;
}


void SyncedPWMADC::SetDutyCycle_MiddleSamplingOnce(float dutyPct)
{
	timerSettingsNext.DutyCycle = dutyPct;

	if (timerSettingsNext.DutyCycle > _PWM_maxDuty)
		timerSettingsNext.DutyCycle = _PWM_maxDuty;
	else if (timerSettingsNext.DutyCycle < -_PWM_maxDuty)
		timerSettingsNext.DutyCycle = -_PWM_maxDuty;

	if (timerSettingsNext.DutyCycle > 0)
		timerSettingsNext.Direction = true;
	else if (timerSettingsNext.DutyCycle < 0)
		timerSettingsNext.Direction = false;

	const uint16_t& End_Count = PredefinedCounts.End;
	const uint16_t& Ripple_Count = PredefinedCounts.Ripple;
	const uint16_t& Sample_Count = PredefinedCounts.Sample;
	// Duty cycle is defined as:
	// Duty Cycle = (CCR+1) / (ARR+1)
	// So 100% duty cycle is achieved by setting CCR=ARR
	const uint16_t Duty_Count = (uint16_t)(roundf( fabsf(timerSettingsNext.DutyCycle)*End_Count )); // note that from testing I have identified that it is more accurate not to subtract -1 here

	// Sanity check
	if (End_Count <= Sample_Count) {
		ERROR("PWM frequency is too fast for ADC sampling at the configured ADC sample time");
	}
	if (End_Count <= Ripple_Count) {
		ERROR("PWM frequency is faster than the expected ripple");
	}

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = 0;
	uint16_t sampleLocation2 = 0;

	// Find mid-point in ON period without LOW-to-HIGH switching ripple
	if ((Duty_Count+Ripple_Count)/2 > (Ripple_Count+Sample_Count)) {
		sampleLocation1 = (Duty_Count+Ripple_Count)/2 - Sample_Count;
	}
	// Find mid-point in OFF period without HIGH-to-LOW switching ripple
	if (((End_Count+Duty_Count+Ripple_Count)/2 - Sample_Count) > (Duty_Count+Ripple_Count)) {
		sampleLocation2 = (End_Count+Duty_Count+Ripple_Count)/2 - Sample_Count;
	}

	// Sample EITHER in the middle of the ON-period or in the middle of the OFF-period
	if (timerSettingsNext.DutyCycle >= 0.5 && sampleLocation1 > 0) {
		sampleLocation2 = 0; // since the ON-period is longest, remove sampling from the OFF-period
	} else {
		sampleLocation1 = 0; // since the OFF-period is longest, remove sampling from the ON-period
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (timerSettingsNext.DutyCycle > 0) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_OperatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.Triggers.numEnabledTriggers = 0;
	if (sampleLocation1 > 0) {
		timerSettingsNext.Triggers.ON.Enabled = true;
		timerSettingsNext.Triggers.ON.Index = 0;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.ON.Enabled = false;
	}
	if (sampleLocation2 > 0) {
		timerSettingsNext.Triggers.OFF.Enabled = true;
		timerSettingsNext.Triggers.OFF.Index = timerSettingsNext.Triggers.numEnabledTriggers;
		timerSettingsNext.Triggers.numEnabledTriggers++;
	} else {
		timerSettingsNext.Triggers.OFF.Enabled = false;
	}
	timerSettingsNext.Triggers.ON.Location = sampleLocation1;
	timerSettingsNext.Triggers.OFF.Location = sampleLocation2;
}

void SyncedPWMADC::TimerCaptureCallback(SyncedPWMADC * obj)
{
	//if (!obj) return; // currently commented out for speed

	if (obj->samplingPin) {
		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE || obj->_DMA_ADC1_ongoing || obj->_DMA_ADC2_ongoing) {
			obj->samplingPin->High();
		}
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

	// Latch timer settings since we now start the next period
	obj->timerSettingsCurrent = obj->timerSettingsNext;

	/* Start/Trigger the ADC1 DMA */
	if (obj->hDMA_ADC1.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC1);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC1, (uint32_t)&obj->hADC1.Instance->DR, (uint32_t)&obj->_ADC1_buffer[0], obj->_samplingNumSamples*obj->timerSettingsCurrent.Triggers.numEnabledTriggers*obj->Channels.ADCnumChannels[0]);
	LL_ADC_REG_StartConversion(obj->hADC1.Instance);
	obj->_DMA_ADC1_ongoing = 1;

	/* Start/Trigger the ADC2 DMA */
	if (obj->hDMA_ADC2.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC2);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC2, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC2, (uint32_t)&obj->hADC2.Instance->DR, (uint32_t)&obj->_ADC2_buffer[0], obj->_samplingNumSamples*obj->timerSettingsCurrent.Triggers.numEnabledTriggers*obj->Channels.ADCnumChannels[1]);
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

	if (obj->samplingPin) {
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
			obj->Samples.Vref.Timestamp = HAL_GetHighResTick();
			if (obj->timerSettingsCurrent.Triggers.ON.Enabled)
				obj->Samples.Vref.ValueON = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc];
			if (obj->timerSettingsCurrent.Triggers.OFF.Enabled)
				obj->Samples.Vref.ValueOFF = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc];
		}

		if (obj->timerSettingsCurrent.Direction) { // Forward
			// Current sense from VSENSE1 since CH1 is LOW at all times
			if (obj->Channels.VSENSE1.ADC) {
				if (obj->Channels.VSENSE1.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				obj->Samples.CurrentSense.Timestamp = HAL_GetHighResTick();
				if (obj->timerSettingsCurrent.Triggers.ON.Enabled)
					obj->Samples.CurrentSense.ValueON = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VSENSE1.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc] / 4096.f;
				if (obj->timerSettingsCurrent.Triggers.OFF.Enabled)
					obj->Samples.CurrentSense.ValueOFF = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VSENSE1.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc] / 4096.f;
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
				obj->Samples.CurrentSense.Timestamp = HAL_GetHighResTick();
				if (obj->timerSettingsCurrent.Triggers.ON.Enabled)
					obj->Samples.CurrentSense.ValueON = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VSENSE3.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc] / 4096.f;
				if (obj->timerSettingsCurrent.Triggers.OFF.Enabled)
					obj->Samples.CurrentSense.ValueOFF = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VSENSE3.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc] / 4096.f;
			}
		}

		if (obj->timerSettingsCurrent.Direction) { // Forward
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
				obj->Samples.Bemf.Timestamp = HAL_GetHighResTick();

				if (obj->timerSettingsCurrent.Triggers.ON.Enabled) {
					float v_bemf = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.BEMF3.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc] / 4096.f;
					obj->Samples.Bemf.ValueON = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				}

				if (obj->timerSettingsCurrent.Triggers.OFF.Enabled) {
					float v_bemf = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.BEMF3.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc] / 4096.f;
					obj->Samples.Bemf.ValueOFF = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				}

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
				obj->Samples.Bemf.Timestamp = HAL_GetHighResTick();

				if (obj->timerSettingsCurrent.Triggers.ON.Enabled) {
					float v_bemf = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.BEMF1.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc] / 4096.f;
					obj->Samples.Bemf.ValueON = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				}

				if (obj->timerSettingsCurrent.Triggers.OFF.Enabled) {
					float v_bemf = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.BEMF1.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc] / 4096.f;
					obj->Samples.Bemf.ValueOFF = (v_bemf - 0.1f) * (2.2f+10.f)/2.2f + 0.1f;
				}
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

			obj->Samples.Vbus.Timestamp = HAL_GetHighResTick();

			if (obj->timerSettingsCurrent.Triggers.ON.Enabled) {
				float v_vbus = obj->Samples.Vref.ValueON * (float)buf[obj->Channels.VBUS.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc] / 4096.f;
				obj->Samples.Vbus.ValueON = v_vbus * (18.f+169.f)/18.f;
			}

			if (obj->timerSettingsCurrent.Triggers.OFF.Enabled) {
				float v_vbus = obj->Samples.Vref.ValueOFF * (float)buf[obj->Channels.VBUS.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc] / 4096.f;
				obj->Samples.Vbus.ValueOFF = v_vbus * (18.f+169.f)/18.f;
			}
		}

		// Latch here instead of in TriggerSample function when we sample in every PWM cycle, since TriggerSample won't be called in that case
		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE) {
			// Latch timer settings since we now start the next period
			obj->timerSettingsCurrent = obj->timerSettingsNext;
		}
	}
}

void SyncedPWMADC::Debug_SetSamplingPin(IO * pin)
{
	if (DEBUG_MODE_ENABLED && pin) {
		samplingPin = pin;
	}
}

SyncedPWMADC::Sample SyncedPWMADC::GetCurrent()
{
	return Samples.CurrentSense;
}

SyncedPWMADC::Sample SyncedPWMADC::GetVin()
{
	return Samples.Vbus;
}

SyncedPWMADC::Sample SyncedPWMADC::GetBemf()
{
	return Samples.Bemf;
}

SyncedPWMADC::SampleSingle SyncedPWMADC::GetPotentiometer()
{
	return SampleSingle();
}
