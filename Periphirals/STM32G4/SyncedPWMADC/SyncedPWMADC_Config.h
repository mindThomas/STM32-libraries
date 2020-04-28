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
 
#ifndef PERIPHIRALS_SYNCEDPWMADC_CONFIG_H
#define PERIPHIRALS_SYNCEDPWMADC_CONFIG_H

	private:
		void ConfigureDigitalPins();
		void DeInitDigitalPins();
		void ConfigureAnalogPins();
		void DeInitAnalogPins();
		void ADC_ConfigureCurrentSenseSampling();
		void ADC_ConfigureBackEMFSampling();

		void InitTimer();
		void DeInitTimer();
		void Timer_ConfigureBrakeMode();
		void Timer_ConfigureCoastMode();
		void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState);
		void SetCoastModeTimerConfiguration(bool direction);

		void InitADCs();
		void DeInitADCs();
		uint32_t ADC_ChannelIndexToADCRank(uint8_t chIndex);

		void InitDMAs();
		void DeInitDMAs();

		void InitOpAmps();
		void DeInitOpAmps();

	public:
		void SetSamplingInterval(uint16_t samplingInterval);

		void StartSampling();
		void StopSampling();

		void StartPWM();
		void StopPWM();
		void Timer_Configure(uint32_t frequency, uint32_t maxValue);

		void SetConstantOutput(bool OnOff, bool direction);

	public: // consider to move these into private (but this should be handled with specific interrupt call methods)
		TIM_HandleTypeDef hTimer;
		ADC_HandleTypeDef hADC1;
		ADC_HandleTypeDef hADC2;
		DMA_HandleTypeDef hDMA_ADC1;
		DMA_HandleTypeDef hDMA_ADC2;

	private:
		uint32_t _ADC_Clock;
		uint16_t _ADC_SampleTime_Total_us;

		bool _SamplingEnabled;
		bool _TimerEnabled;

#endif
