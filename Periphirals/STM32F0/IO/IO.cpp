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
#include "Debug.h"
#include "Priorities.h"

IO * IO::interruptObjects[16] = {0};

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void EXTI0_1_IRQHandler(void);
extern "C" void EXTI2_3_IRQHandler(void);
extern "C" void EXTI4_15_IRQHandler(void);

// Configure as output
IO::IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin) : _InterruptCallback(0), _InterruptCallbackParams(0),
	#ifdef USE_FREERTOS
		_InterruptSemaphore(0),
	#endif
		_GPIO(GPIOx), _pin(GPIO_Pin), _isInput(false)
{
	ConfigurePin(GPIOx, GPIO_Pin, false, false, PULL_NONE);
}

// Configure as input
IO::IO(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, pull_t pull) : _InterruptCallback(0), _InterruptCallbackParams(0),
	#ifdef USE_FREERTOS
		_InterruptSemaphore(0),
	#endif
		_GPIO(GPIOx), _pin(GPIO_Pin), _isInput(true)
{
	ConfigurePin(GPIOx, GPIO_Pin, true, false, pull);
}

IO::~IO()
{
	if (!_GPIO) return;
	//HAL_GPIO_DeInit(_GPIO, _pin);
	// Reset pin
	LL_GPIO_InitTypeDef GPIO_ResetStruct = {0};
	LL_GPIO_StructInit(&GPIO_ResetStruct);
	GPIO_ResetStruct.Pin = _pin;
	LL_GPIO_Init(_GPIO, &GPIO_ResetStruct);

	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex = getPinIndex();
	if (interruptObjects[pinIndex]) { // interrupt was configured - so disable it
		interruptObjects[pinIndex] = 0;

		// Enable interrupt
		if (_pin == GPIO_PIN_0 || _pin == GPIO_PIN_1)
			NVIC_DisableIRQ(EXTI0_1_IRQn);
		if (_pin == GPIO_PIN_2 || _pin == GPIO_PIN_3)
			NVIC_DisableIRQ(EXTI2_3_IRQn);
		else if (_pin >= GPIO_PIN_4 && _pin <= GPIO_PIN_15)
			NVIC_DisableIRQ(EXTI4_15_IRQn);
	}
}

uint16_t IO::getPinIndex()
{
	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex;
	uint16_t tmp = _pin;
	for (pinIndex = -1; tmp != 0; pinIndex++)
		tmp = tmp >> 1;

	return pinIndex;
}

void IO::ConfigurePin(GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin, bool isInput, bool isOpenDrain, pull_t pull)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	// GPIO Ports Clock Enable
	if (GPIOx == GPIOA)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); // __HAL_RCC_GPIOA_CLK_ENABLE();
#ifdef GPIOB
	else if (GPIOx == GPIOB)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#ifdef GPIOC
	else if (GPIOx == GPIOC)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); // __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
#ifdef GPIOD
	else if (GPIOx == GPIOD)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); // __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
	else if (GPIOx == GPIOE)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE); // __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
	else if (GPIOx == GPIOF)
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF); // __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
	else
	{
		_GPIO = 0;
		return;
	}

	// Configure GPIO pin Output Level
	LL_GPIO_ResetOutputPin(GPIOx, GPIO_Pin);

	// Configure pin as output or input
	_isInput = isInput;
	_isOpenDrain = isOpenDrain;
	if (isInput)
		GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	else
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;

	if (isOpenDrain)
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	else
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

	_pull = pull;
	if (pull == PULL_NONE)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	else if (pull == PULL_UP)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	else if (pull == PULL_DOWN)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;

	// Configure GPIO
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
	LL_GPIO_Init(GPIOx, &GPIO_InitStruct);
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

#ifdef USE_FREERTOS
void IO::RegisterInterrupt(interrupt_trigger_t trigger, SemaphoreHandle_t semaphore)
{
	if (!_GPIO || !_isInput) return;

	_InterruptSemaphore = semaphore;
	ConfigureInterrupt(trigger);
}
#endif

void IO::RegisterInterrupt(interrupt_trigger_t trigger, void (*InterruptCallback)(void * params), void * callbackParams)
{
	if (!_GPIO || !_isInput) return;

	_InterruptCallback = InterruptCallback;
	_InterruptCallbackParams = callbackParams;
	ConfigureInterrupt(trigger);
}

void IO::DeregisterInterrupt()
{
#ifdef USE_FREERTOS
	if (!_GPIO || !_isInput || (!_InterruptSemaphore && !_InterruptCallback)) return; // no interrupt configured
	_InterruptSemaphore = 0;
#else
	if (!_GPIO || !_isInput || !_InterruptCallback) return; // no interrupt configured
#endif
	_InterruptCallback = 0;
	_InterruptCallbackParams = 0;
	DisableInterrupt();
}

void IO::ConfigureInterrupt(interrupt_trigger_t trigger)
{
	LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

	if (!_GPIO || !_isInput) return;

	uint16_t pinIndex = getPinIndex();
	if (interruptObjects[pinIndex] != 0) {
		ERROR("Interrupt vector already used for this pin");
		return;
	}

	interruptObjects[pinIndex] = this;

	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
	uint32_t SYSCFG_EXTI_LINE = ((uint32_t)(4*(pinIndex & 0b0011))) << 16 | ((pinIndex & 0b1100) >> 2); // according to SYSTEM_LL_EC_EXTI_LINE in stm32f0xx_ll_sysytem.h

	if (_GPIO == GPIOA)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, SYSCFG_EXTI_LINE);
#ifdef GPIOB
	else if (_GPIO == GPIOB)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, SYSCFG_EXTI_LINE);
#endif
#ifdef GPIOC
	else if (_GPIO == GPIOC)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, SYSCFG_EXTI_LINE);
#endif
#ifdef GPIOD
	else if (_GPIO == GPIOD)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, SYSCFG_EXTI_LINE);
#endif
#ifdef GPIOE
	else if (_GPIO == GPIOE)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, SYSCFG_EXTI_LINE);
#endif
#ifdef GPIOF
	else if (_GPIO == GPIOF)
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTF, SYSCFG_EXTI_LINE);
#endif

	EXTI_InitStruct.Line_0_31 = _pin; // this is a hack, should have been e.g. LL_EXTI_LINE_13
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	if (trigger == TRIGGER_RISING)
		EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	else if (trigger == TRIGGER_FALLING)
		EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	else if (trigger == TRIGGER_BOTH)
		EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;

	LL_EXTI_Init(&EXTI_InitStruct);

	// Enable interrupt
	if (_pin == GPIO_PIN_0 || _pin == GPIO_PIN_1) {
		NVIC_EnableIRQ(EXTI0_1_IRQn);
		NVIC_SetPriority(EXTI0_1_IRQn, IO_INTERRUPT_PRIORITY);
	}
	else if (_pin == GPIO_PIN_2 || _pin == GPIO_PIN_3) {
		NVIC_EnableIRQ(EXTI2_3_IRQn);
		NVIC_SetPriority(EXTI2_3_IRQn, IO_INTERRUPT_PRIORITY);
	}
	else if (_pin >= GPIO_PIN_4 && _pin <= GPIO_PIN_15) {
		NVIC_EnableIRQ(EXTI4_15_IRQn);
		NVIC_SetPriority(EXTI4_15_IRQn, IO_INTERRUPT_PRIORITY);
	}
}

void IO::DisableInterrupt()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (!_GPIO || !_isInput) return;

	// Calculate pin index by extracting bit index from GPIO_PIN
	uint16_t pinIndex = getPinIndex();
	interruptObjects[pinIndex] = 0;

	// Reconfigure pin to just input (disable interrupt)
	GPIO_InitStruct.Pin = _pin;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;

	if (_pull == PULL_NONE)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	else if (_pull == PULL_UP)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	else if (_pull == PULL_DOWN)
		GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;

	LL_GPIO_Init(_GPIO, &GPIO_InitStruct);
}

void IO::Set(bool state)
{
	if (!_GPIO || _isInput) return;
	if (state)
		_GPIO->BSRR = _pin;
	else
		_GPIO->BRR = _pin;
}

void IO::High()
{
	if (!_GPIO || _isInput) return;
	_GPIO->BSRR = _pin;
}

void IO::Low()
{
	if (!_GPIO || _isInput) return;
	_GPIO->BRR = _pin;
}

void IO::Toggle()
{
	if (!_GPIO || _isInput) return;
	_GPIO->ODR ^= _pin;
}

bool IO::Read()
{
	if (!_GPIO || !_isInput) return false;
	if (_GPIO->IDR & _pin)
		return true;
	else
		return false;
}

void IO::InterruptHandler(IO * io)
{
	if (!io) return;

#ifdef USE_FREERTOS
	if (io->_InterruptSemaphore) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		//xSemaphoreGiveFromISR( timer->callbackSemaphore, &xHigherPriorityTaskWoken );
		xQueueSendFromISR(io->_InterruptSemaphore, NULL, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
#endif

	if (io->_InterruptCallback)
		io->_InterruptCallback(io->_InterruptCallbackParams);

	/*if (timer->callbackTaskHandle)
		xTaskResumeFromISR(timer->callbackTaskHandle);*/
}

void EXTI0_1_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(GPIO_PIN_0) != RESET) {
		LL_EXTI_ClearFlag_0_31(GPIO_PIN_0);
		IO::InterruptHandler(IO::interruptObjects[0]);
	}
	if (LL_EXTI_IsActiveFlag_0_31(GPIO_PIN_1) != RESET) {
		LL_EXTI_ClearFlag_0_31(GPIO_PIN_1);
		IO::InterruptHandler(IO::interruptObjects[1]);
	}
}

void EXTI2_3_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(GPIO_PIN_2) != RESET) {
		LL_EXTI_ClearFlag_0_31(GPIO_PIN_2);
		IO::InterruptHandler(IO::interruptObjects[3]);
	}
	if (LL_EXTI_IsActiveFlag_0_31(GPIO_PIN_3) != RESET) {
		LL_EXTI_ClearFlag_0_31(GPIO_PIN_3);
		IO::InterruptHandler(IO::interruptObjects[3]);
	}
}

void EXTI4_15_IRQHandler(void)
{
	uint32_t pin = GPIO_PIN_4;
	for (uint8_t i = 4; i <= 15; i++) {
		if (LL_EXTI_IsActiveFlag_0_31(pin) != RESET) {
			LL_EXTI_ClearFlag_0_31(pin);
			IO::InterruptHandler(IO::interruptObjects[i]);
		}
		pin <<= 1;
	}
}
