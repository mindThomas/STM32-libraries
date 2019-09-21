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
 
#include "IO.h"
#include "stm32h7xx_hal.h"
#include "Priorities.h"
#include "Debug.h"

IO * IO::interruptObjects[16] = {0};

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void EXTI0_IRQHandler(void);
extern "C" __EXPORT void EXTI1_IRQHandler(void);
extern "C" __EXPORT void EXTI2_IRQHandler(void);
extern "C" __EXPORT void EXTI3_IRQHandler(void);
extern "C" __EXPORT void EXTI4_IRQHandler(void);
extern "C" __EXPORT void EXTI9_5_IRQHandler(void);
extern "C" __EXPORT void EXTI15_10_IRQHandler(void);

// Configure as output
IO::IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin) : _InterruptCallback(0), _InterruptCallbackParams(0), _InterruptSemaphore(0), _GPIO(GPIOx), _pin(GPIO_Pin), _isInput(false), _pull()
{
	ConfigurePin(GPIOx, GPIO_Pin, false, false, PULL_NONE);
}

// Configure as input
IO::IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, pull_t pull) : _InterruptCallback(0), _InterruptCallbackParams(0), _InterruptSemaphore(0), _GPIO(GPIOx), _pin(GPIO_Pin), _isInput(true), _pull()
{
	ConfigurePin(GPIOx, GPIO_Pin, true, false, pull);
}

IO::~IO()
{
	if (!_GPIO) return;
	HAL_GPIO_DeInit(_GPIO, _pin);

	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex;
	uint16_t tmp = _pin;
	for (pinIndex = -1; tmp != 0; pinIndex++)
		tmp = tmp >> 1;

	if (interruptObjects[pinIndex]) { // interrupt was configured - so disable it
		interruptObjects[pinIndex] = 0;

		// Enable interrupt
		if (_pin == GPIO_PIN_0)
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		else if (_pin == GPIO_PIN_1)
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		else if (_pin == GPIO_PIN_2)
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		else if (_pin == GPIO_PIN_3)
			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		else if (_pin == GPIO_PIN_4)
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		else if (_pin >= GPIO_PIN_5 && _pin <= GPIO_PIN_9)
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		else if (_pin >= GPIO_PIN_10 && _pin <= GPIO_PIN_15)
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
}

void IO::ConfigurePin(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, bool isInput, bool isOpenDrain, pull_t pull)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// GPIO Ports Clock Enable
	if (GPIOx == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if (GPIOx == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (GPIOx == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if (GPIOx == GPIOD)
		__HAL_RCC_GPIOD_CLK_ENABLE();
	else if (GPIOx == GPIOE)
		__HAL_RCC_GPIOE_CLK_ENABLE();
	else if (GPIOx == GPIOF)
		__HAL_RCC_GPIOF_CLK_ENABLE();
	else if (GPIOx == GPIOG)
		__HAL_RCC_GPIOG_CLK_ENABLE();
	else
	{
		_GPIO = 0;
		return;
	}

	// Configure GPIO pin Output Level
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

	// Configure pin as output or input
	_isInput = isInput;
	_isOpenDrain = isOpenDrain;
	if (isInput)
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	else if (isOpenDrain)
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	else
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

	_pull = pull;
	if (pull == PULL_NONE)
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	else if (pull == PULL_UP)
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	else if (pull == PULL_DOWN)
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	// Configure GPIO
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void IO::ChangeToInput(pull_t pull)
{
	ConfigurePin(_GPIO, _pin, true, false, _pull);
}

void IO::ChangeToOutput(bool state)
{
	ConfigurePin(_GPIO, _pin, false, false, PULL_NONE);
	Set(state);
}

void IO::ChangeToOpenDrain(bool state)
{
	ConfigurePin(_GPIO, _pin, false, true, PULL_NONE);
	Set(state);
}

void IO::RegisterInterrupt(interrupt_trigger_t trigger, SemaphoreHandle_t semaphore)
{
	if (!_GPIO || !_isInput) return;

	_InterruptSemaphore = semaphore;
	ConfigureInterrupt(trigger);
}

void IO::RegisterInterrupt(interrupt_trigger_t trigger, void (*InterruptCallback)(void * params), void * callbackParams)
{
	if (!_GPIO || !_isInput) return;

	_InterruptCallback = InterruptCallback;
	_InterruptCallbackParams = callbackParams;
	ConfigureInterrupt(trigger);
}

void IO::DeregisterInterrupt()
{
	if (!_GPIO || !_isInput || (!_InterruptSemaphore && !_InterruptCallback)) return; // no interrupt configured
	_InterruptSemaphore = 0;
	_InterruptCallback = 0;
	_InterruptCallbackParams = 0;
	DisableInterrupt();
}

void IO::ConfigureInterrupt(interrupt_trigger_t trigger)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (!_GPIO || !_isInput) return;

	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex;
	uint16_t tmp = _pin;
	for (pinIndex = -1; tmp != 0; pinIndex++)
		tmp = tmp >> 1;

	if (interruptObjects[pinIndex] != 0) {
		ERROR("Interrupt vector already used for this pin");
		return;
	}

	interruptObjects[pinIndex] = this;

	// Reconfigure pin to enable interrupt triggering
	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	if (trigger == TRIGGER_RISING)
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	else if (trigger == TRIGGER_FALLING)
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	else if (trigger == TRIGGER_BOTH)
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;

	if (_pull == PULL_NONE)
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	else if (_pull == PULL_UP)
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	else if (_pull == PULL_DOWN)
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(_GPIO, &GPIO_InitStruct);

	// Enable interrupt
	if (_pin == GPIO_PIN_0) {
		HAL_NVIC_SetPriority(EXTI0_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	}
	else if (_pin == GPIO_PIN_1) {
		HAL_NVIC_SetPriority(EXTI1_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	}
	else if (_pin == GPIO_PIN_2) {
		HAL_NVIC_SetPriority(EXTI2_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	}
	else if (_pin == GPIO_PIN_3) {
		HAL_NVIC_SetPriority(EXTI3_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	}
	else if (_pin == GPIO_PIN_4) {
		HAL_NVIC_SetPriority(EXTI4_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	}
	else if (_pin >= GPIO_PIN_5 && _pin <= GPIO_PIN_9) {
		HAL_NVIC_SetPriority(EXTI9_5_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	else if (_pin >= GPIO_PIN_10 && _pin <= GPIO_PIN_15) {
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, IO_INTERRUPT_PRIORITY, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}

void IO::DisableInterrupt()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (!_GPIO || !_isInput) return;

	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex;
	uint16_t tmp = _pin;
	for (pinIndex = -1; tmp != 0; pinIndex++)
		tmp = tmp >> 1;

	interruptObjects[pinIndex] = 0;

	// Reconfigure pin to just input (disable interrupt)
	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	if (_pull == PULL_NONE)
		GPIO_InitStruct.Pull = GPIO_NOPULL;
	else if (_pull == PULL_UP)
		GPIO_InitStruct.Pull = GPIO_PULLUP;
	else if (_pull == PULL_DOWN)
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(_GPIO, &GPIO_InitStruct);
}

void IO::Set(bool state)
{
	if (!_GPIO || _isInput) return;
	if (state)
		_GPIO->BSRRL = _pin;
	else
		_GPIO->BSRRH = _pin;
}

void IO::High()
{
	if (!_GPIO || _isInput) return;
	_GPIO->BSRRL = _pin;
}

void IO::Low()
{
	if (!_GPIO || _isInput) return;
	_GPIO->BSRRH = _pin;
}

void IO::Toggle()
{
	if (!_GPIO || _isInput) return;
	_GPIO->ODR ^= _pin;
}

bool IO::Read()
{
	if (!_GPIO || !_isInput) return false;
	if((_GPIO->IDR & _pin) != (uint32_t)GPIO_PIN_RESET)
		return true;
	else
		return false;
}

void IO::InterruptHandler(IO * io)
{
	if (!io) return;

	if (io->_InterruptSemaphore) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		//xSemaphoreGiveFromISR( timer->callbackSemaphore, &xHigherPriorityTaskWoken );
		xQueueSendFromISR(io->_InterruptSemaphore, NULL, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

	if (io->_InterruptCallback)
		io->_InterruptCallback(io->_InterruptCallbackParams);

	/*if (timer->callbackTaskHandle)
		xTaskResumeFromISR(timer->callbackTaskHandle);*/
}

void EXTI0_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
		IO::InterruptHandler(IO::interruptObjects[0]);
	}
}

void EXTI1_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		IO::InterruptHandler(IO::interruptObjects[1]);
	}
}

void EXTI2_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
		IO::InterruptHandler(IO::interruptObjects[2]);
	}
}

void EXTI3_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
		IO::InterruptHandler(IO::interruptObjects[3]);
	}
}

void EXTI4_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
		IO::InterruptHandler(IO::interruptObjects[4]);
	}
}

void EXTI9_5_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
		IO::InterruptHandler(IO::interruptObjects[5]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
		IO::InterruptHandler(IO::interruptObjects[6]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
		IO::InterruptHandler(IO::interruptObjects[7]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
		IO::InterruptHandler(IO::interruptObjects[8]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
		IO::InterruptHandler(IO::interruptObjects[9]);
	}
}

void EXTI15_10_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
		IO::InterruptHandler(IO::interruptObjects[10]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
		IO::InterruptHandler(IO::interruptObjects[11]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
		IO::InterruptHandler(IO::interruptObjects[12]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
		IO::InterruptHandler(IO::interruptObjects[13]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
		IO::InterruptHandler(IO::interruptObjects[14]);
	}
	else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
		IO::InterruptHandler(IO::interruptObjects[15]);
	}
}
