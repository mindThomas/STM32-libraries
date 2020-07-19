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
		timerSettingsNext.samplingInterval = 1; // sample every PWM period
	} else {
		timerSettingsNext.samplingInterval = 2; // sample every second PWM period (with SAMPLE_IN_EVERY_PWM_CYCLE=false, this is the fastest possible)
	}
	timerSettingsNext.samplingAveragingNumSamples = 1; // capture samples from one PWM period after every interval (hence every PWM period)
	timerSettingsNext.InvalidateSamples = 0;
	timerSettingsNext.BemfAlternateRange = false;
	timerSettingsNext.BemfHighRange = true;
	_DMA_ADC1_ongoing = false;
	_DMA_ADC2_ongoing = false;
	_TimerEnabled = false;
	_SamplingEnabled = false;

#ifdef USE_FREERTOS
	_sampleAvailable = xSemaphoreCreateBinary();
	if (_sampleAvailable == NULL) {
		ERROR("Could not create ADC-PWM Sample available semaphore");
		return;
	}
	vQueueAddToRegistry(_sampleAvailable, "ADC-PWM Sample Available");
#else
	_sampleAvailable = false;
#endif

#ifdef USE_FREERTOS
	_sampleAddedToQueue = xSemaphoreCreateBinary();
	if (_sampleAddedToQueue == NULL) {
		ERROR("Could not create ADC-PWM Sample added to queue semaphore");
		return;
	}
	vQueueAddToRegistry(_sampleAddedToQueue, "ADC-PWM Queued Sample Available");
#endif

#ifdef USE_FREERTOS
	_timerSettingsMutex = xSemaphoreCreateBinary();
	if (_timerSettingsMutex == NULL) {
		ERROR("Could not create ADC-PWM Timer settings mutex");
		return;
	}
	vQueueAddToRegistry(_timerSettingsMutex, "ADC-PWM Timer settings");
	xSemaphoreGive( _timerSettingsMutex ); // give the semaphore the first time
#endif

#ifdef USE_FREERTOS
	_latestSampleMutex = xSemaphoreCreateBinary();
	if (_latestSampleMutex == NULL) {
		ERROR("Could not create ADC-PWM Latest sample mutex");
		return;
	}
	vQueueAddToRegistry(_latestSampleMutex, "ADC-PWM Latest sample");
	xSemaphoreGive( _latestSampleMutex ); // give the semaphore the first time
#endif

#ifdef USE_FREERTOS
#if SAMPLE_QUEUE_SIZE > 0
	SampleQueue = xQueueCreate( SAMPLE_QUEUE_SIZE, sizeof(CombinedSample_t) );
	if (SampleQueue == NULL) {
		ERROR("Could not create Samples queue");
		return;
	}
	vQueueAddToRegistry(SampleQueue, "Samples queue");
#endif
#endif

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
	_operatingMode = mode;

	if (_operatingMode == BRAKE) {
		ADC_ConfigureCurrentSenseSampling();
		Timer_ConfigureBrakeMode();
	}
	else if (_operatingMode == COAST) {
		//ADC_ConfigureBackEMFSampling();
		ADC_ConfigureCoastModeSampling();
		Timer_ConfigureCoastMode(timerSettingsCurrent.Direction);
	}
}

SyncedPWMADC::OperatingMode_t SyncedPWMADC::GetOperatingMode(void)
{
	return _operatingMode;
}

void SyncedPWMADC::SetBemfRange(bool auto_range, bool high_range, bool alternating_range)
{
	// OBS! Only enable auto_range if Bemf is calibrated
	timerSettingsNext.BemfAutoRange = auto_range;
	timerSettingsNext.BemfHighRange = high_range;
	timerSettingsNext.BemfAlternateRange = alternating_range;
}

void SyncedPWMADC::SetPWMFrequency(uint32_t frequency, bool fixed_prescaler)
{
	Timer_Configure(frequency, fixed_prescaler);
	RecomputePredefinedCounts(); // recompute at different frequency
}

/*
void SyncedPWMADC::SetDutyCycle(float duty)
{
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

	timerSettingsNext.DutyCycle = duty;

	if (timerSettingsNext.DutyCycle > _PWM_maxDuty)
		timerSettingsNext.DutyCycle = _PWM_maxDuty;
	else if (timerSettingsNext.DutyCycle < -_PWM_maxDuty)
		timerSettingsNext.DutyCycle = -_PWM_maxDuty;

	if (timerSettingsNext.DutyCycle > 0)
		timerSettingsNext.Direction = true;
	else if (timerSettingsNext.DutyCycle < 0)
		timerSettingsNext.Direction = false;

	// OBS! This function is not complete. Use the functions below where the sample location is specified
	// This function should be extended to automatically chose the current sampling scheme based on a stored parameter such that the sampling scheme can be configured/selected once

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings
}
*/

float SyncedPWMADC::GetCurrentDutyCycle()
{
	return timerSettingsCurrent.DutyCycle;
}

bool SyncedPWMADC::GetCurrentDirection()
{
	return timerSettingsCurrent.Direction;
}

SyncedPWMADC::CombinedSample_t SyncedPWMADC::GetLatestSample()
{
	CombinedSample_t sample;

	xSemaphoreTake( _latestSampleMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change
	sample = _latestSample;
	xSemaphoreGive( _latestSampleMutex ); // unlock settings

	return sample;
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
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

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

	if (timerSettingsNext.Direction) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_operatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.DutyCycleLocation = Duty_Count;
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
	timerSettingsNext.Changed = true;

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	// When we sample in every cycle the DMA is not triggered periodically but only configured and enabled once. So we need to reconfigure the DMA if we changed the number of triggers
	if (SAMPLE_IN_EVERY_PWM_CYCLE && timerSettingsNext.Triggers.numEnabledTriggers != timerSettingsCurrent.Triggers.numEnabledTriggers) {
		RestartSampling();
	}
}

void SyncedPWMADC::SetDutyCycle_EndSampling(float dutyPct)
{
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

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

	if (timerSettingsNext.Direction) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_operatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.DutyCycleLocation = Duty_Count;
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
	timerSettingsNext.Changed = true;

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	// When we sample in every cycle the DMA is not triggered periodically but only configured and enabled once. So we need to reconfigure the DMA if we changed the number of triggers
	if (SAMPLE_IN_EVERY_PWM_CYCLE && timerSettingsNext.Triggers.numEnabledTriggers != timerSettingsCurrent.Triggers.numEnabledTriggers) {
		RestartSampling();
	}
}

void SyncedPWMADC::SetDutyCycle_MiddleSamplingOnce(float dutyPct)
{
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

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
	} else if (timerSettingsNext.DutyCycle < 0.5 && sampleLocation2 > 0) {
		sampleLocation1 = 0; // since the OFF-period is longest, remove sampling from the ON-period
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (timerSettingsNext.Direction) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_operatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.DutyCycleLocation = Duty_Count;
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
	timerSettingsNext.Changed = true;

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	// When we sample in every cycle the DMA is not triggered periodically but only configured and enabled once. So we need to reconfigure the DMA if we changed the number of triggers
	if (SAMPLE_IN_EVERY_PWM_CYCLE && timerSettingsNext.Triggers.numEnabledTriggers != timerSettingsCurrent.Triggers.numEnabledTriggers) {
		RestartSampling();
	}
}

// Modifies only the sampling location
void SyncedPWMADC::SetCustomSamplingLocations(float samplingLocation1, float samplingLocation2)
{
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

	const uint16_t& End_Count = PredefinedCounts.End;
	const uint16_t& Ripple_Count = PredefinedCounts.Ripple;
	const uint16_t& Sample_Count = PredefinedCounts.Sample;
	// Duty cycle is defined as:
	// Duty Cycle = (CCR+1) / (ARR+1)
	// So 100% duty cycle is achieved by setting CCR=ARR
	//const uint16_t Duty_Count = (uint16_t)(roundf( fabsf(timerSettingsNext.DutyCycle)*End_Count )); // note that from testing I have identified that it is more accurate not to subtract -1 here

	// Sanity check
	if (End_Count <= Sample_Count) {
		ERROR("PWM frequency is too fast for ADC sampling at the configured ADC sample time");
	}
	if (End_Count <= Ripple_Count) {
		ERROR("PWM frequency is faster than the expected ripple");
	}

	// Put the sample location as close to the end as possible
	uint16_t sampleLocation1 = (uint16_t)(roundf(samplingLocation1 * End_Count));
	uint16_t sampleLocation2 = (uint16_t)(roundf(samplingLocation2 * End_Count));

	if (sampleLocation1 > sampleLocation2) {
		// Swap the locations
		uint16_t tmp = sampleLocation2;
		sampleLocation2 = sampleLocation1;
		sampleLocation1 = tmp;
	}

	// Check that none of the samples goes beyond the period
	if ((sampleLocation1 + Sample_Count) > End_Count) {
		sampleLocation1 = 0;
	}
	if ((sampleLocation2 + Sample_Count) > End_Count) {
		sampleLocation2 = 0;
	}

	// Ensure that the sample locations are not too close
	if (sampleLocation2 < (sampleLocation1 + Sample_Count)) {
		sampleLocation2 = 0; // not possible - samples are too close
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
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
	timerSettingsNext.Changed = true;

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	// When we sample in every cycle the DMA is not triggered periodically but only configured and enabled once. So we need to reconfigure the DMA if we changed the number of triggers
	if (SAMPLE_IN_EVERY_PWM_CYCLE && timerSettingsNext.Triggers.numEnabledTriggers != timerSettingsCurrent.Triggers.numEnabledTriggers) {
		RestartSampling();
	}
}


void SyncedPWMADC::SetDutyCycle_CustomSampling(float dutyPct, float samplingLocation1, float samplingLocation2)
{
	xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change

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
	uint16_t sampleLocation1 = (uint16_t)(roundf(samplingLocation1 * End_Count));
	uint16_t sampleLocation2 = (uint16_t)(roundf(samplingLocation2 * End_Count));

	if (sampleLocation1 > sampleLocation2) {
		// Swap the locations
		uint16_t tmp = sampleLocation2;
		sampleLocation2 = sampleLocation1;
		sampleLocation1 = tmp;
	}

	// Check that none of the samples goes beyond the period
	if ((sampleLocation1 + Sample_Count) > End_Count) {
		sampleLocation1 = 0;
	}
	if ((sampleLocation2 + Sample_Count) > End_Count) {
		sampleLocation2 = 0;
	}

	// Ensure that the sample locations are not too close
	if (sampleLocation2 < (sampleLocation1 + Sample_Count)) {
		sampleLocation2 = 0; // not possible - samples are too close
	}

	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_5, sampleLocation1);
	__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_6, sampleLocation2);

	if (DEBUG_MODE_ENABLED) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_2, sampleLocation1);
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_4, sampleLocation2);
	}

	if (timerSettingsNext.Direction) {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, Duty_Count); // control the duty cycle with the other side of the motor
	} else {
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_1, Duty_Count); // control the duty cycle with the other side of the motor
		__HAL_TIM_SET_COMPARE(&hTimer, TIM_CHANNEL_3, 0); // set one side of the motor to LOW all the time  (necessary to be able to measure current through Shunt1)
	}

	if (_operatingMode == COAST) {
		Timer_ConfigureCoastMode(timerSettingsNext.Direction);
	}

	timerSettingsNext.DutyCycleLocation = Duty_Count;
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
	timerSettingsNext.Changed = true;

	xSemaphoreGive( _timerSettingsMutex ); // unlock settings

	// When we sample in every cycle the DMA is not triggered periodically but only configured and enabled once. So we need to reconfigure the DMA if we changed the number of triggers
	if (SAMPLE_IN_EVERY_PWM_CYCLE && timerSettingsNext.Triggers.numEnabledTriggers != timerSettingsCurrent.Triggers.numEnabledTriggers) {
		RestartSampling();
	}
}


void SyncedPWMADC::TimerCaptureCallback(SyncedPWMADC * obj)
{
	//if (!obj) return; // currently commented out for speed

	if (obj->samplingPin) {
		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE || obj->_DMA_ADC1_ongoing || obj->_DMA_ADC2_ongoing) {

			if ((__HAL_TIM_GET_FLAG(&obj->hTimer, TIM_FLAG_CC2) != RESET && obj->timerSettingsNext.Triggers.ON.Enabled) ||
				(__HAL_TIM_GET_FLAG(&obj->hTimer, TIM_FLAG_CC4) != RESET && obj->timerSettingsNext.Triggers.OFF.Enabled))
			{
				obj->samplingPin->High();
			}
		}
	}
}

void SyncedPWMADC::LatchTimerSettings(SyncedPWMADC * obj)
{
#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#endif

	if (obj->timerSettingsNext.Changed) {
#ifdef USE_FREERTOS
		if (xSemaphoreTakeFromISR( obj->_timerSettingsMutex, &xHigherPriorityTaskWoken )) {
#endif
			obj->timerSettingsNext.Changed = false;

			// Apply the changed timer settings that has to be latched
			obj->hTimer.Instance->RCR = obj->timerSettingsNext.samplingInterval - 1;

			if (obj->timerSettingsNext.BemfHighRange != obj->timerSettingsCurrent.BemfHighRange) {
				obj->EnableBemfVoltageDivider(obj->timerSettingsNext.BemfHighRange); // enable voltage divider in high-range mode
			}

			obj->timerSettingsCurrent = obj->timerSettingsNext;
#ifdef USE_FREERTOS
			xSemaphoreGiveFromISR( obj->_timerSettingsMutex, &xHigherPriorityTaskWoken );
		}
#endif

#ifdef USE_FREERTOS
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
	}
}

void SyncedPWMADC::TriggerSample(SyncedPWMADC * obj)
{
	if (!obj) return;

	/* Called by the Timer Update interrupt vector to trigger/initiate a DMA sample */
	// Latch timer settings since we now start the next period
	LatchTimerSettings(obj);

	//if (obj->_DMA_ADC1_ongoing || obj->_DMA_ADC2_ongoing) return; // Previous conversion not finished yet
	//if (obj->hDMA_ADC1.State != HAL_DMA_STATE_READY || obj->hDMA_ADC2.State != HAL_DMA_STATE_READY) return;

	/* Start/Trigger the ADC1 DMA */
	if (obj->hDMA_ADC1.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC1);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC1, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC1, (uint32_t)&obj->hADC1.Instance->DR, (uint32_t)&obj->_ADC1_buffer[0], obj->timerSettingsCurrent.samplingAveragingNumSamples*obj->timerSettingsCurrent.Triggers.numEnabledTriggers*obj->Channels.ADCnumChannels[0]);
	LL_ADC_REG_StartConversion(obj->hADC1.Instance);
	obj->_DMA_ADC1_ongoing = 1;

	/* Start/Trigger the ADC2 DMA */
	if (obj->hDMA_ADC2.State != HAL_DMA_STATE_READY) {
		HAL_DMA_Abort(&obj->hDMA_ADC2);
	}
	__HAL_ADC_CLEAR_FLAG(&obj->hADC2, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
	HAL_DMA_Start_IT(&obj->hDMA_ADC2, (uint32_t)&obj->hADC2.Instance->DR, (uint32_t)&obj->_ADC2_buffer[0], obj->timerSettingsCurrent.samplingAveragingNumSamples*obj->timerSettingsCurrent.Triggers.numEnabledTriggers*obj->Channels.ADCnumChannels[1]);
	LL_ADC_REG_StartConversion(obj->hADC2.Instance);
	obj->_DMA_ADC2_ongoing = 1;

	if (!obj->_SamplingEnabled) {
		// Interrupt called for the first time
		obj->_SamplingEnabled = 1;

		/* Enable ADC DMA mode */
		SET_BIT(obj->hADC1.Instance->CFGR, ADC_CFGR_DMAEN);
		SET_BIT(obj->hADC2.Instance->CFGR, ADC_CFGR_DMAEN);

		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE) {
			// When sampling in every sample we only need to enable the DMA once
			// And then we disable the interrupt since it is now running
			__HAL_TIM_DISABLE_IT(&obj->hTimer, TIM_IT_UPDATE);
		}
		return;
	}
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


#pragma GCC push_options
#pragma GCC optimize ("O3")
void SyncedPWMADC::SamplingCompleted(SyncedPWMADC * obj, uint8_t ADC)
{
	if (!obj) return;

	if (!obj->SAMPLE_IN_EVERY_PWM_CYCLE) {
		if (ADC == 1) {
			obj->_DMA_ADC1_ongoing = 0;
			LL_ADC_REG_StopConversion(obj->hADC1.Instance);
		}
		else if (ADC == 2) {
			obj->_DMA_ADC2_ongoing = 0;
			LL_ADC_REG_StopConversion(obj->hADC2.Instance);
		}
		if (obj->samplingPin) {
			obj->samplingPin->Low();
		}
	}

	if (obj->SAMPLE_IN_EVERY_PWM_CYCLE ||
	   (obj->_DMA_ADC1_ongoing == 0 && obj->_DMA_ADC2_ongoing == 0)) {
		uint16_t * buf;
		uint8_t nc;
		uint8_t numTriggers = obj->timerSettingsCurrent.Triggers.numEnabledTriggers;
		uint16_t bufSkip;
		uint32_t timestamp = HAL_GetHighResTick();
#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#endif

		if (obj->samplingPin) {
			obj->samplingPin->High();
		}

		if (obj->timerSettingsCurrent.InvalidateSamples == 0 && obj->timerSettingsNext.InvalidateSamples == 0)
		{
			obj->Samples.Vref.UpdatedON = false;
			obj->Samples.Vref.UpdatedOFF = false;
			obj->Samples.CurrentSense.UpdatedON = false;
			obj->Samples.CurrentSense.UpdatedOFF = false;
			obj->Samples.Bemf.Updated = false;
			obj->Samples.Vbus.UpdatedON = false;
			obj->Samples.Vbus.UpdatedOFF = false;
			obj->Samples.Encoder.Updated = false;

			// Compute Vref voltage if sampled (based on __LL_ADC_CALC_VREFANALOG_VOLTAGE - with 12-bit ADC resolution)
			if (obj->Channels.VREF.ADC) {
				if (obj->Channels.VREF.ADC == 1) {
					buf = obj->_ADC1_buffer;
					nc = obj->Channels.ADCnumChannels[0];
				} else {
					buf = obj->_ADC2_buffer;
					nc = obj->Channels.ADCnumChannels[1];
				}
				bufSkip = nc * numTriggers;
				obj->Samples.Vref.Timestamp = timestamp;
				obj->Samples.Vref.UpdatedON = obj->timerSettingsCurrent.Triggers.ON.Enabled;
				obj->Samples.Vref.UpdatedOFF = obj->timerSettingsCurrent.Triggers.OFF.Enabled;

				if (obj->Samples.Vref.UpdatedON) {
					obj->Samples.Vref.ValueON = 0;
					for (uint16_t i = 0, bufIdx = obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc;
						i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
						i++, bufIdx += bufSkip)
					{
						obj->VrefON[i] = (0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[bufIdx]) / ADC_MAX_VALUE;
						obj->Samples.Vref.ValueON += obj->VrefON[i];
					}
					obj->Samples.Vref.ValueON /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
					//obj->Samples.Vref.ValueON = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc];
				}

				if (obj->Samples.Vref.UpdatedOFF) {
					obj->Samples.Vref.ValueOFF = 0;
					for (uint16_t i = 0, bufIdx = obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
						i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
						i++, bufIdx += bufSkip)
					{
						obj->VrefOFF[i] = (0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[bufIdx]) / ADC_MAX_VALUE;
						obj->Samples.Vref.ValueOFF += obj->VrefOFF[i];
					}
					obj->Samples.Vref.ValueOFF /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
					//obj->Samples.Vref.ValueOFF = 0.001f * (float)((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF) / (float)buf[obj->Channels.VREF.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc];
				}
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
					obj->Samples.CurrentSense.Timestamp = timestamp;
					obj->Samples.CurrentSense.UpdatedON = obj->timerSettingsCurrent.Triggers.ON.Enabled;
					obj->Samples.CurrentSense.UpdatedOFF = (obj->timerSettingsCurrent.Triggers.OFF.Enabled & (obj->_operatingMode == BRAKE)); // Current sense can only be sampled during OFF-period in BRAKE mode

					if (obj->Samples.CurrentSense.UpdatedON) {
						obj->Samples.CurrentSense.ValueON = 0;
						for (uint16_t i = 0, bufIdx = obj->Channels.VSENSE1.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc;
							i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
							i++, bufIdx += bufSkip)
						{
							obj->Samples.CurrentSense.ValueON += obj->VrefON[i] * (float)buf[bufIdx];
						}
						obj->Samples.CurrentSense.ValueON /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
						if (obj->ChannelCalibrations.CurrentSense.Enabled)
							obj->Samples.CurrentSense.ValueON = obj->ChannelCalibrations.CurrentSense.VSENSE1.Scale * (obj->Samples.CurrentSense.ValueON + obj->ChannelCalibrations.CurrentSense.VSENSE1.Offset);
					}
					if (obj->Samples.CurrentSense.UpdatedOFF) {
						obj->Samples.CurrentSense.ValueOFF = 0;
						for (uint16_t i = 0, bufIdx = obj->Channels.VSENSE1.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
							i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
							i++, bufIdx += bufSkip)
						{
							obj->Samples.CurrentSense.ValueOFF += obj->VrefOFF[i] * (float)buf[bufIdx];
						}
						obj->Samples.CurrentSense.ValueOFF /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
						if (obj->ChannelCalibrations.CurrentSense.Enabled)
							obj->Samples.CurrentSense.ValueOFF = obj->ChannelCalibrations.CurrentSense.VSENSE1.Scale * (obj->Samples.CurrentSense.ValueOFF + obj->ChannelCalibrations.CurrentSense.VSENSE1.Offset);
					}
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
					obj->Samples.CurrentSense.Timestamp = timestamp;
					obj->Samples.CurrentSense.UpdatedON = obj->timerSettingsCurrent.Triggers.ON.Enabled;
					obj->Samples.CurrentSense.UpdatedOFF = (obj->timerSettingsCurrent.Triggers.OFF.Enabled & (obj->_operatingMode == BRAKE)); // Current sense can only be sampled during OFF-period in BRAKE mode

					if (obj->Samples.CurrentSense.UpdatedON) {
						obj->Samples.CurrentSense.ValueON = 0;
						for (uint16_t i = 0, bufIdx = obj->Channels.VSENSE3.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc;
							i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
							i++, bufIdx += bufSkip)
						{
							obj->Samples.CurrentSense.ValueON += obj->VrefON[i] * (float)buf[bufIdx];
						}
						obj->Samples.CurrentSense.ValueON /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
						if (obj->ChannelCalibrations.CurrentSense.Enabled) {
							obj->Samples.CurrentSense.ValueON = obj->ChannelCalibrations.CurrentSense.VSENSE3.Scale * (obj->Samples.CurrentSense.ValueON + obj->ChannelCalibrations.CurrentSense.VSENSE3.Offset);

							// Invert value since motor is driven backwards (and current is measured in opposite direction)
							obj->Samples.CurrentSense.ValueON = -obj->Samples.CurrentSense.ValueON;
						}
					}
					if (obj->Samples.CurrentSense.UpdatedOFF) {
						obj->Samples.CurrentSense.ValueOFF = 0;
						for (uint16_t i = 0, bufIdx = obj->Channels.VSENSE3.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
							i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
							i++, bufIdx += bufSkip)
						{
							obj->Samples.CurrentSense.ValueOFF += obj->VrefOFF[i] * (float)buf[bufIdx];
						}
						obj->Samples.CurrentSense.ValueOFF /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
						if (obj->ChannelCalibrations.CurrentSense.Enabled) {
							obj->Samples.CurrentSense.ValueOFF = obj->ChannelCalibrations.CurrentSense.VSENSE3.Scale * (obj->Samples.CurrentSense.ValueOFF + obj->ChannelCalibrations.CurrentSense.VSENSE3.Offset);

							// Invert value since motor is driven backwards (and current is measured in opposite direction)
							obj->Samples.CurrentSense.ValueOFF = -obj->Samples.CurrentSense.ValueOFF;
						}
					}
				}
			}

			if (obj->_operatingMode == COAST) { // Back-EMF can only be sampled in OFF-period of COAST mode
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
						obj->Samples.Bemf.Timestamp = timestamp;
						obj->Samples.Bemf.Updated = obj->timerSettingsCurrent.Triggers.OFF.Enabled;
						if (obj->Samples.Bemf.Updated) {
							obj->Samples.Bemf.Value = 0;
							for (uint16_t i = 0, bufIdx = obj->Channels.BEMF3.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
								i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
								i++, bufIdx += bufSkip)
							{
								obj->Samples.Bemf.Value += obj->VrefOFF[i] * (float)buf[bufIdx];
							}
							obj->Samples.Bemf.Value /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
							if (obj->ChannelCalibrations.Bemf.Enabled) {
								if (obj->timerSettingsCurrent.BemfHighRange)
									obj->Samples.Bemf.Value = obj->ChannelCalibrations.Bemf.BEMF3.HighRange.Scale * (obj->Samples.Bemf.Value + obj->ChannelCalibrations.Bemf.BEMF3.HighRange.Offset);
								else
									obj->Samples.Bemf.Value = obj->ChannelCalibrations.Bemf.BEMF3.LowRange.Scale * (obj->Samples.Bemf.Value + obj->ChannelCalibrations.Bemf.BEMF3.LowRange.Offset);
							}
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
						obj->Samples.Bemf.Timestamp = timestamp;
						obj->Samples.Bemf.Updated = obj->timerSettingsCurrent.Triggers.OFF.Enabled;
						if (obj->Samples.Bemf.Updated) {
							obj->Samples.Bemf.Value = 0;
							for (uint16_t i = 0, bufIdx = obj->Channels.BEMF1.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
								i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
								i++, bufIdx += bufSkip)
							{
								obj->Samples.Bemf.Value += obj->VrefOFF[i] * (float)buf[bufIdx];
							}
							obj->Samples.Bemf.Value /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
							if (obj->ChannelCalibrations.Bemf.Enabled) {
								if (obj->timerSettingsCurrent.BemfHighRange)
									obj->Samples.Bemf.Value = obj->ChannelCalibrations.Bemf.BEMF1.HighRange.Scale * (obj->Samples.Bemf.Value + obj->ChannelCalibrations.Bemf.BEMF1.HighRange.Offset);
								else
									obj->Samples.Bemf.Value = obj->ChannelCalibrations.Bemf.BEMF1.LowRange.Scale * (obj->Samples.Bemf.Value + obj->ChannelCalibrations.Bemf.BEMF1.LowRange.Offset);

								// Invert value since BEMF sense is in backwards direction
								obj->Samples.Bemf.Value = -obj->Samples.Bemf.Value;
							}
						}
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

				obj->Samples.Vbus.Timestamp = timestamp;
				obj->Samples.Vbus.UpdatedON = obj->timerSettingsCurrent.Triggers.ON.Enabled;
				obj->Samples.Vbus.UpdatedOFF = obj->timerSettingsCurrent.Triggers.OFF.Enabled;
				if (obj->Samples.Vbus.UpdatedON) {
					obj->Samples.Vbus.ValueON = 0;
					for (uint16_t i = 0, bufIdx = obj->Channels.VBUS.Index + obj->timerSettingsCurrent.Triggers.ON.Index*nc;
						i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
						i++, bufIdx += bufSkip)
					{
						obj->Samples.Vbus.ValueON += obj->VrefON[i] * (float)buf[bufIdx];
					}
					obj->Samples.Vbus.ValueON /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
					if (obj->ChannelCalibrations.Vbus.Enabled)
						obj->Samples.Vbus.ValueON = obj->ChannelCalibrations.Vbus.VBUS.Scale * (obj->Samples.Vbus.ValueON + obj->ChannelCalibrations.Vbus.VBUS.Offset);
				}

				if (obj->Samples.Vbus.UpdatedOFF) {
					obj->Samples.Vbus.ValueOFF = 0;
					for (uint16_t i = 0, bufIdx = obj->Channels.VBUS.Index + obj->timerSettingsCurrent.Triggers.OFF.Index*nc;
						i < obj->timerSettingsCurrent.samplingAveragingNumSamples;
						i++, bufIdx += bufSkip)
					{
						obj->Samples.Vbus.ValueOFF += obj->VrefOFF[i] * (float)buf[bufIdx];
					}
					obj->Samples.Vbus.ValueOFF /= obj->timerSettingsCurrent.samplingAveragingNumSamples;
					if (obj->ChannelCalibrations.Vbus.Enabled)
						obj->Samples.Vbus.ValueOFF = obj->ChannelCalibrations.Vbus.VBUS.Scale * (obj->Samples.Vbus.ValueOFF + obj->ChannelCalibrations.Vbus.VBUS.Offset);
				}
			}

			// Sample Encoder if associated
			if (obj->_encoder) {
				obj->Samples.Encoder.Timestamp = timestamp;
				obj->Samples.Encoder.Updated = true;
				obj->Samples.Encoder.Value = obj->_encoder->Get();
			}

#ifdef USE_FREERTOS
			CombinedSample_t CombinedSample = {0};
			CombinedSample.Timestamp = timestamp;
			CombinedSample.PWM_Frequency = obj->timerSettingsCurrent.Frequency;
			CombinedSample.TimerMax = obj->timerSettingsCurrent.TimerMax;
			CombinedSample.DutyCycleLocation = obj->timerSettingsCurrent.DutyCycleLocation;
			CombinedSample.TriggerLocationON = obj->timerSettingsCurrent.Triggers.ON.Location;
			CombinedSample.TriggerLocationOFF = obj->timerSettingsCurrent.Triggers.OFF.Location;

			if (obj->Samples.CurrentSense.UpdatedON)
				CombinedSample.Current.ON = obj->Samples.CurrentSense.ValueON;
			if (obj->Samples.CurrentSense.UpdatedOFF)
				CombinedSample.Current.OFF = obj->Samples.CurrentSense.ValueOFF;

			if (obj->Samples.Bemf.Updated) {
				CombinedSample.Bemf = obj->Samples.Bemf.Value;
				CombinedSample.BemfHighRange = obj->timerSettingsCurrent.BemfHighRange;
			}

			if (obj->Samples.Vbus.UpdatedON)
				CombinedSample.VbusON = obj->Samples.Vbus.ValueON;
			if (obj->Samples.Vbus.UpdatedOFF)
				CombinedSample.VbusOFF = obj->Samples.Vbus.ValueOFF;
			if (obj->Samples.Encoder.Updated)
				CombinedSample.Encoder = obj->Samples.Encoder.Value;

			if (uxQueueSpacesAvailableFromISR(obj->SampleQueue) > 0) { // check for space in queue
				xQueueSendFromISR(obj->SampleQueue, (void *)&CombinedSample, &xHigherPriorityTaskWoken);

				if (obj->_sampleAddedToQueue)
					xSemaphoreGiveFromISR( obj->_sampleAddedToQueue, &xHigherPriorityTaskWoken );
			} else {
				obj->_missedSamples++;
			}

			if (obj->_latestSampleMutex) {
				if (xSemaphoreTakeFromISR( obj->_latestSampleMutex, &xHigherPriorityTaskWoken )) {
					obj->_latestSample = CombinedSample;
					xSemaphoreGiveFromISR( obj->_latestSampleMutex, &xHigherPriorityTaskWoken );
				}
			}

			if (obj->_sampleAvailable)
				xSemaphoreGiveFromISR( obj->_sampleAvailable, &xHigherPriorityTaskWoken );
#else
			_sampleAvailable = true;
#endif

			if (obj->timerSettingsCurrent.BemfAlternateRange) {
				obj->timerSettingsCurrent.BemfHighRange = !obj->timerSettingsCurrent.BemfHighRange;
				obj->timerSettingsNext.BemfHighRange = obj->timerSettingsCurrent.BemfHighRange;
				obj->EnableBemfVoltageDivider(obj->timerSettingsCurrent.BemfHighRange); // enable voltage divider in high-range mode
			}
			else if (obj->timerSettingsCurrent.BemfAutoRange) {
				// Make hysteresis for changing from high-to-low and low-to-high range
				if (obj->Samples.Bemf.Updated) {
					if (!obj->timerSettingsCurrent.BemfHighRange && obj->Samples.Bemf.Value >= obj->BEMF_SWITCH_TO_HIGH_RANGE_HYSTERESIS_THRESHOLD) {
						obj->timerSettingsCurrent.BemfHighRange = true;
						obj->timerSettingsNext.BemfHighRange = obj->timerSettingsCurrent.BemfHighRange;
						obj->EnableBemfVoltageDivider(obj->timerSettingsCurrent.BemfHighRange); // enable voltage divider in high-range mode
					}
					else if (obj->timerSettingsCurrent.BemfHighRange && obj->Samples.Bemf.Value <= obj->BEMF_SWITCH_TO_LOW_RANGE_HYSTERESIS_THRESHOLD) {
						obj->timerSettingsCurrent.BemfHighRange = false;
						obj->timerSettingsNext.BemfHighRange = obj->timerSettingsCurrent.BemfHighRange;
						obj->EnableBemfVoltageDivider(obj->timerSettingsCurrent.BemfHighRange); // enable voltage divider in high-range mode
					}
				}
			}
		}
		else {
			// Clear DMA buffer
			memset(obj->_ADC1_buffer, 0, ADC_DMA_HALFWORD_MAX_BUFFER_SIZE);
			memset(obj->_ADC2_buffer, 0, ADC_DMA_HALFWORD_MAX_BUFFER_SIZE);
			if (obj->timerSettingsCurrent.InvalidateSamples)
				obj->timerSettingsCurrent.InvalidateSamples--;
			if (obj->timerSettingsNext.InvalidateSamples)
				obj->timerSettingsNext.InvalidateSamples--;
		}

		if (obj->SAMPLE_IN_EVERY_PWM_CYCLE) {
			// When sampling in every cycle the TriggerSample is not called, so we need to latch the timer settings here
			LatchTimerSettings(obj);
		}

		if (obj->samplingPin) {
			obj->samplingPin->Low();
		}

		obj->interruptComputeTimeTicks = HAL_GetHighResTick() - timestamp;

#ifdef USE_FREERTOS
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#endif
	}
}
#pragma GCC pop_options

void SyncedPWMADC::Debug_SetSamplingPin(IO * pin)
{
	if (DEBUG_MODE_ENABLED && pin) {
		samplingPin = pin;
	}
}

void SyncedPWMADC::AssignEncoder(Encoder * encoder)
{
	if (encoder)
		_encoder = encoder;
}

void SyncedPWMADC::WaitForNewSample()
{
#ifdef USE_FREERTOS
	xSemaphoreTake( _sampleAvailable, ( TickType_t ) portMAX_DELAY );
#else
	while (!_sampleAvailable) {
		osDelay(1);
	}
#endif
}

void SyncedPWMADC::WaitForNewQueuedSample()
{
#ifdef USE_FREERTOS
	xSemaphoreTake( _sampleAddedToQueue, ( TickType_t ) portMAX_DELAY );
#endif
}


SyncedPWMADC::SampleFloat SyncedPWMADC::GetCurrent()
{
	return Samples.CurrentSense;
}

SyncedPWMADC::SampleFloat SyncedPWMADC::GetVin()
{
	return Samples.Vbus;
}

SyncedPWMADC::SampleSingleFloat SyncedPWMADC::GetBemf()
{
	return Samples.Bemf;
}

SyncedPWMADC::SampleSingleFloat SyncedPWMADC::GetPotentiometer()
{
	return SampleSingleFloat();
}

void SyncedPWMADC::DetermineCurrentSenseOffset()
{
	volatile float CurrentSenseOffset1 = 0;
	volatile float CurrentSenseOffset3 = 0;

	bool CalibrationEnabled = ChannelCalibrations.CurrentSense.Enabled;
	ChannelCalibrations.CurrentSense.Enabled = false; // disable calibration-compensation during calibration

	SetOperatingMode(BRAKE);
	SetNumberOfAveragingSamples(1);

	// Sample current sense in Forward direction (VSENSE1)
	{
		xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change
		timerSettingsNext.Direction = true;
		xSemaphoreGive( _timerSettingsMutex ); // unlock settings
		SetDutyCycle_MiddleSamplingOnce(0.0f); // zero duty cycle during current-sense offset capture
		osDelay(10);

		// Wait for the queue to fill up
		uint16_t SampleCount = 0;
		CombinedSample_t sample;
		xQueueReset(SampleQueue);
		while (SampleCount < 100) {
			if (xQueueReceive( SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS) {
				if (sample.Current.OFF != 0) {
					CurrentSenseOffset1 += sample.Current.OFF; // Single trigger sampling configured and since duty cycle is OFF the sample would be put in the OFF field
					SampleCount++;
				}
			}
		}

		CurrentSenseOffset1 /= SampleCount;
	}
	ChannelCalibrations.CurrentSense.VSENSE1.Offset = -CurrentSenseOffset1;

	// Sample current sense in Backward direction (VSENSE3)
	{
		xSemaphoreTake( _timerSettingsMutex, ( TickType_t ) portMAX_DELAY ); // lock settings for change
		timerSettingsNext.Direction = false;
		xSemaphoreGive( _timerSettingsMutex ); // unlock settings
		SetDutyCycle_MiddleSamplingOnce(0.0f); // zero duty cycle during current-sense offset capture
		osDelay(10);

		// Wait for the queue to fill up
		uint16_t SampleCount = 0;
		CombinedSample_t sample;
		xQueueReset(SampleQueue);
		while (SampleCount < 100) {
			if (xQueueReceive( SampleQueue, &sample, ( TickType_t ) portMAX_DELAY ) == pdPASS) {
				if (sample.Current.OFF != 0) {
					CurrentSenseOffset3 += sample.Current.OFF; // Single trigger sampling configured and since duty cycle is OFF the sample would be put in the OFF field
					SampleCount++;
				}
			}
		}
		CurrentSenseOffset3 /= SampleCount;
	}
	ChannelCalibrations.CurrentSense.VSENSE3.Offset = -CurrentSenseOffset3;

	ChannelCalibrations.CurrentSense.Enabled = CalibrationEnabled;
}
