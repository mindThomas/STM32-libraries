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
 
#include "SPI.h"

#include "Priorities.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

SPI::hardware_resource_t * SPI::resSPI2 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void SPI2_IRQHandler(void);

SPI::SPI(port_t port, uint32_t frequency, GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin) : _csPort(GPIOx), _csPin(GPIO_Pin)
{
	InitPeripheral(port, frequency);
	InitChipSelect();
	ConfigurePeripheral();
	_ongoingTransaction = false;
}

SPI::SPI(port_t port, uint32_t frequency)
{
	InitPeripheral(port, frequency);
	if (!_hRes) return;

	// Set default chip selects
	switch (_hRes->port)
	{
		case PORT_SPI2:
			_csPort = GPIOB;
			_csPin = LL_GPIO_PIN_12;
			break;
		default:
			_hRes = 0;
			ERROR("Undefined SPI port");
			return;
	}

	InitChipSelect();
	ConfigurePeripheral();
	_ongoingTransaction = false;
}

SPI::SPI(port_t port) : SPI(port, SPI_DEFAULT_FREQUENCY)
{
}

SPI::~SPI()
{
	if (!_hRes) return;

	_hRes->instances--;
	if (_hRes->instances == 0) { // deinitialize port and periphiral
		DeInitPeripheral();
		DeInitChipSelect();

		// Delete hardware resource
		port_t tmpPort = _hRes->port;
		delete(_hRes);

		switch (tmpPort)
		{
			case PORT_SPI2:
				resSPI2 = 0;
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}
	}
}

void SPI::DeInitPeripheral()
{
	if (!_hRes) return;
	if (_hRes->port == PORT_SPI2) {
		/* Peripheral clock disable */
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);

		/**SPI3 GPIO Configuration
		PC10     ------> SPI3_SCK
		PC11     ------> SPI3_MISO
		PC12     ------> SPI3_MOSI
		*/
		// ToDo: Deinitialize GPIO

		/* SPI3 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI2_IRQn);
	}
}

void SPI::DeInitChipSelect()
{
	if (!_hRes) return;
	if (_csPort) { // general chip select configuration
		// ToDo: Deinitialize Chip select GPIO
		//HAL_GPIO_DeInit(_csPort, _csPin);
	}
	else // hardware chip select
	{
		switch (_hRes->port) {
			case PORT_SPI2: // SPI2_NSS ------> PB12
				// ToDo: Deinitialize Chip select GPIO
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}
	}
}

void SPI::InitPeripheral(port_t port, uint32_t frequency)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool firstTime = false;

	switch (port)
	{
		case PORT_SPI2:
			if (!resSPI2) {
				resSPI2 = new SPI::hardware_resource_t;
				memset(resSPI2, 0, sizeof(SPI::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resSPI2;
			break;
		default:
			ERROR("Undefined SPI port");
			_hRes = 0;
			return;
	}

	if (firstTime) { // first time configuring peripheral
		_hRes->port = port;
		_hRes->frequency = frequency;
		_hRes->configured = false;
		_hRes->instances = 0;
	#ifdef USE_FREERTOS
		_hRes->resourceSemaphore = xSemaphoreCreateBinary();
		if (_hRes->resourceSemaphore == NULL) {
			ERROR("Could not create SPI resource semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->resourceSemaphore, "SPI Resource");
		xSemaphoreGive( _hRes->resourceSemaphore ); // give the semaphore the first time

		_hRes->transmissionFinished = xSemaphoreCreateBinary();
		if (_hRes->transmissionFinished == NULL) {
			ERROR("Could not create SPI transmission semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->transmissionFinished, "SPI Finish");
		xSemaphoreGive( _hRes->transmissionFinished ); // ensure that the semaphore is not taken from the beginning
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // ensure that the semaphore is not taken from the beginning
	#else
		_hRes->transmissionFinished = true;
		_hRes->resourceSemaphore = false;
	#endif

		// Configure pins for SPI and SPI peripheral accordingly
		if (port == PORT_SPI2) {
			/* Peripheral clock enable */
			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

			/**SPI3 GPIO Configuration
			PB13     ------> SPI2_SCK
			PB14     ------> SPI2_MISO
			PB15     ------> SPI2_MOSI
			*/
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
			GPIO_InitStruct.Pin = LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
			GPIO_InitStruct.Speed = LL_GPIO_SPEED_HIGH;
			GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
			GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
			GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
			GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
			LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			/* SPI3 interrupt Init */
			NVIC_SetPriority(SPI2_IRQn, SPI_INTERRUPT_PRIORITY);
			NVIC_EnableIRQ(SPI2_IRQn);
		}
	}

	_hRes->instances++;
}

void SPI::InitChipSelect()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (_csPort) { // general chip select configuration
		// Enable clock
		if (_csPort == GPIOA)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); // __HAL_RCC_GPIOA_CLK_ENABLE();
	#ifdef GPIOB
		else if (_csPort == GPIOB)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); // __HAL_RCC_GPIOB_CLK_ENABLE();
	#endif
	#ifdef GPIOC
		else if (_csPort == GPIOC)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); // __HAL_RCC_GPIOC_CLK_ENABLE();
	#endif
	#ifdef GPIOD
		else if (_csPort == GPIOD)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD); // __HAL_RCC_GPIOD_CLK_ENABLE();
	#endif
	#ifdef GPIOE
		else if (_csPort == GPIOE)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE); // __HAL_RCC_GPIOE_CLK_ENABLE();
	#endif
	#ifdef GPIOF
		else if (_csPort == GPIOF)
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF); // __HAL_RCC_GPIOF_CLK_ENABLE();
	#endif
		else
		{
			_csPort = 0;
			return;
		}

		LL_GPIO_SetOutputPin(_csPort, _csPin); // set pin high after configuration = chip is NOT selected

		GPIO_InitStruct.Pin = _csPin;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		LL_GPIO_Init(_csPort, &GPIO_InitStruct);

	}
	else { // hardware chip select configuration
		if (!_hRes) return;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_0;

		switch (_hRes->port) {
			case PORT_SPI2: // SPI2_NSS ------> PB12
				GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
				LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}
	}
}

void SPI::ConfigurePeripheral()
{
	LL_SPI_InitTypeDef SPI_InitStruct;

	if (!_hRes) return;

	if (!_hRes->configured) { // only configure periphiral once
		switch (_hRes->port) {
			case PORT_SPI2:
				_hRes->instance = SPI2;
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}

		_hRes->txRegSent = true;
		_hRes->txReg = 0;
		_hRes->txBuffer = 0;
		_hRes->rxBuffer = 0;
		_hRes->txBytesToWrite = 0;
		_hRes->rxBytesToRead = 0;
		_hRes->dummyReadBytesToSend = _hRes->rxBytesToRead;

		SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
		SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
		SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
		SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
		SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
		if (_csPort)
			SPI_InitStruct.NSS = LL_SPI_NSS_SOFT; // general chip-select - we have decided not to support hardware chip-select (NSS pin)
		else
			SPI_InitStruct.NSS = LL_SPI_NSS_HARD_OUTPUT;
		SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
		SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
		SPI_InitStruct.CRCPoly = 7;

		LL_RCC_ClocksTypeDef clocks;
		LL_RCC_GetSystemClocksFreq(&clocks);
		uint32_t SPI_Clock = clocks.PCLK1_Frequency; //HAL_RCC_GetPCLK1Freq();
		// Find the prescaler which matches the desired frequency best (rounding down)
		float prescaler = (float)SPI_Clock / _hRes->frequency;
		if (prescaler <= 2)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
		else if (prescaler <= 4)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
		else if (prescaler <= 8)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
		else if (prescaler <= 16)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
		else if (prescaler <= 32)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32;
		else if (prescaler <= 64)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
		else if (prescaler <= 128)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV128;
		else if (prescaler <= 256)
			SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
		else
		{
			ERROR("SPI baud rate too low to be configurable");
			return;
		}

		if (LL_SPI_Init(_hRes->instance, &SPI_InitStruct) != SUCCESS)
		{
			ERROR("Could not initialize SPI port");
			return;
		}

		LL_SPI_SetRxFIFOThreshold(_hRes->instance, LL_SPI_RX_FIFO_TH_QUARTER);

		/* Enable SPI */
		LL_SPI_Enable(_hRes->instance);

#ifdef USE_FREERTOS
		osDelay(10); // wait 10 ms for clock to stabilize
#else
		HAL_Delay(10);
#endif

		_hRes->configured = true;
	}
}

void SPI::ReconfigureFrequency(uint32_t frequency)
{
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
#else
	if (_hRes->resourceSemaphore) return; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

	_hRes->frequency = frequency;
	_hRes->configured = false;
	ConfigurePeripheral();

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif
}

bool SPI::Write(uint8_t reg, uint8_t value)
{
	return Write(reg, &value, 1);
}

bool SPI::Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	return WriteInterrupt(reg, buffer, writeLength);
}

bool SPI::WriteBlocking(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	bool success = false;
	if (!_hRes) return success;
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

	if (_csPort)
		_csPort->BRR = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	uint8_t bytesSent = 0;

	while (bytesSent < writeLength+1) {
		/* Write data in Transmit Data register.
		TXE flag is cleared by writing data in DR register */
		if (bytesSent == 0)
			LL_SPI_TransmitData8(_hRes->instance, reg);
		else
			LL_SPI_TransmitData8(_hRes->instance, (*buffer++));

		bytesSent++;

		/* Check TXE flag value in ISR register */
		while (!LL_SPI_IsActiveFlag_TXE(_hRes->instance));

		/* Wait while busy */
	    while (LL_SPI_IsActiveFlag_BSY(_hRes->instance));
	}

	if (_csPort)
		_csPort->BSRR = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

bool SPI::WriteInterrupt(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	bool success = false;
	if (!_hRes) return success;
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	// Consider to use task notifications instead: https://www.freertos.org/RTOS-task-notifications.html
	// However using notifications can possibly lead to other problems if multiple objects are going to notify the same task simultaneously
	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

#ifndef USE_FREERTOS
	_hRes->transmissionFinished = false;
#endif

	_hRes->txRegSent = false;
	_hRes->txReg = reg;
	_hRes->txBuffer = buffer;
	_hRes->rxBuffer = 0;
	_hRes->txBytesToWrite = writeLength;
	_hRes->rxBytesToRead = 0;
	_hRes->dummyReadBytesToSend = _hRes->rxBytesToRead;

	if (_csPort)
		_csPort->BRR = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	LL_SPI_EnableIT_RXNE(_hRes->instance);
	LL_SPI_EnableIT_TXE(_hRes->instance);
	LL_SPI_EnableIT_ERR(_hRes->instance);

#ifdef USE_FREERTOS
	// Wait for the transmission to finish
	xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
#else
	while (!_hRes->transmissionFinished);
#endif

	if (_csPort)
		_csPort->BSRR = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

#if 0
void SPI::Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	if (!_hRes) return;
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts

	uint8_t * txBuffer = (uint8_t *)pvPortMalloc(writeLength+1);
	uint8_t * rxBuffer = (uint8_t *)pvPortMalloc(writeLength+1);

	if (!txBuffer || !rxBuffer) return;

	txBuffer[0] = reg;
	memcpy(&txBuffer[1], buffer, writeLength);

	if (_csPort)
		_csPort->BSRRH = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	if (HAL_SPI_TransmitReceive_IT(&_hRes->handle, txBuffer, rxBuffer, writeLength+1) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		ERROR("Failed SPI transmission");
	}

	if (_csPort)
		_csPort->BSRRL = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	vPortFree(txBuffer);
	vPortFree(rxBuffer);

	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
}
#endif


uint8_t SPI::Read(uint8_t reg)
{
	uint8_t rx;
	if (Read(reg, &rx, 1))
		return rx;
	else
		return 0;
}

bool SPI::Read(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	return ReadInterrupt(reg, buffer, readLength);
}

bool SPI::ReadBlocking(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	bool success = false;
	if (!_hRes) return success;
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

	if (_csPort)
		_csPort->BRR = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	uint8_t bytesRead = 0;

	while (bytesRead < readLength+1) {
		/* Write data in Transmit Data register.
		TXE flag is cleared by writing data in DR register */
		if (bytesRead == 0)
			LL_SPI_TransmitData8(_hRes->instance, reg);
		else
			LL_SPI_TransmitData8(_hRes->instance, 0xFF); // padding byte for reading

		/* Check TXE flag value in ISR register */
		while (!LL_SPI_IsActiveFlag_TXE(_hRes->instance));

		/* Wait while busy */
	    while (LL_SPI_IsActiveFlag_BSY(_hRes->instance));

	    if (bytesRead == 0) {
	    	/* Byte has now been clocked out, wait for read data to be available */
	    	while (!LL_SPI_IsActiveFlag_RXNE(_hRes->instance));
	    	// Discard data
			LL_SPI_ReceiveData8(_hRes->instance);  // issue new 8 clocks to read data
	    } else {
	    	/* Byte has now been clocked out, wait for read data to be available */
	    	while (!LL_SPI_IsActiveFlag_RXNE(_hRes->instance));
			buffer[bytesRead-1] = LL_SPI_ReceiveData8(_hRes->instance);  // issue new 8 clocks to read data
	    }

		bytesRead++;
	}

	if (_csPort)
		_csPort->BSRR = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

bool SPI::ReadInterrupt(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	bool success = false;
	if (!_hRes) return success;
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	// Consider to use task notifications instead: https://www.freertos.org/RTOS-task-notifications.html
	// However using notifications can possibly lead to other problems if multiple objects are going to notify the same task simultaneously
	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

#ifndef USE_FREERTOS
	_hRes->transmissionFinished = false;
#endif

	_hRes->txRegSent = false;
	_hRes->txReg = reg;
	_hRes->txBuffer = 0;
	_hRes->rxBuffer = buffer;
	_hRes->txBytesToWrite = 0;
	_hRes->rxBytesToRead = readLength;
	_hRes->dummyReadBytesToSend = _hRes->rxBytesToRead;

	if (_csPort)
		_csPort->BRR = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	LL_SPI_EnableIT_RXNE(_hRes->instance);
	LL_SPI_EnableIT_TXE(_hRes->instance);
	LL_SPI_EnableIT_ERR(_hRes->instance);

#ifdef USE_FREERTOS
	// Wait for the transmission to finish
	xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
#else
	while (!_hRes->transmissionFinished);
#endif

	if (_csPort)
		_csPort->BSRR = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

#if 0
void SPI::Read(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	if (!_hRes) return;
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts

	uint8_t * txBuffer = (uint8_t *)pvPortMalloc(readLength+1);
	uint8_t * rxBuffer = (uint8_t *)pvPortMalloc(readLength+1);

	if (!txBuffer || !rxBuffer) return;

	memset(txBuffer, 0, readLength+1);
	txBuffer[0] = reg;

#if 1
	if (_csPort)
		_csPort->BSRRH = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	if (HAL_SPI_TransmitReceive_IT(&_hRes->handle, txBuffer, rxBuffer, readLength+1) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		ERROR("Failed SPI transmission");
	}

	if (_csPort)
		_csPort->BSRRL = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	memcpy(buffer, &rxBuffer[1], readLength);
#else

	if (_csPort)
		_csPort->BSRRH = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	if (HAL_SPI_TransmitReceive_IT(&_hRes->handle, txBuffer, rxBuffer, 1) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		ERROR("Failed SPI transmission");
	}
	/*if (HAL_SPI_Transmit(&_hRes->handle, txBuffer, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		ERROR("Failed SPI transmission");
	}*/

	if (_csPort)
		_csPort->BSRRL = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	// short wait
	for (uint32_t i = 0; i < 0x0040; i++)
		asm("nop");

	if (uxSemaphoreGetCount(_hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state before starting the transmission, since we use the semaphore for flagging the finish transmission event
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // something incorrect happened, as the transmissionFinished semaphore should always be taken before a transmission starts

	if (_csPort)
		_csPort->BSRRH = _csPin; // assert chip select  (LOW)     // HAL_GPIO_WritePin

	memset(txBuffer, 0, readLength);
	if (HAL_SPI_TransmitReceive_IT(&_hRes->handle, txBuffer, rxBuffer, readLength) == HAL_OK)
	{
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
	} else {
		ERROR("Failed SPI transmission");
	}

	if (_csPort)
		_csPort->BSRRL = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin

	memcpy(buffer, rxBuffer, readLength);
#endif

	vPortFree(txBuffer);
	vPortFree(rxBuffer);

	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
}
#endif

#if 0
void SPI::BeginTransaction()
{
	if (!_hRes) return;
	if (_ongoingTransaction) return;
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	if (_csPort)
		_csPort->BSRRH = _csPin; // assert chip select  (LOW)
}

void SPI::EndTransaction()
{
	if (!_hRes) return;
	if (!_ongoingTransaction) return;
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back

	if (_csPort)
		_csPort->BSRRL = _csPin; // deassert chip select  (HIGH)     // HAL_GPIO_WritePin
}

void SPI::TransactionWrite8(uint8_t value)
{
	if (!_hRes) return;
	if (!_ongoingTransaction) return;

	uint8_t tx = value;
	uint8_t rx = 0;
	HAL_SPI_TransmitReceive(&_hRes->handle, &tx, &rx, 1, HAL_MAX_DELAY);
}

void SPI::TransactionWrite16(uint16_t value)
{
	if (!_hRes) return;
	if (!_ongoingTransaction) return;

	uint8_t tx[2] = {(uint8_t)((value>>8) & 0xFF),
					 (uint8_t)((value) & 0xFF)};
	uint8_t rx[2] = {0};
	HAL_SPI_TransmitReceive(&_hRes->handle, tx, rx, 2, HAL_MAX_DELAY);
}

void SPI::TransactionWrite32(uint32_t value)
{
	if (!_hRes) return;
	if (!_ongoingTransaction) return;

	uint8_t tx[4] = {(uint8_t)((value>>24) & 0xFF),
					 (uint8_t)((value>>16) & 0xFF),
					 (uint8_t)((value>>8) & 0xFF),
					 (uint8_t)((value) & 0xFF)};
	uint8_t rx[4] = {0};
	HAL_SPI_TransmitReceive(&_hRes->handle, tx, rx, 4, HAL_MAX_DELAY);
}

uint8_t SPI::TransactionRead()
{
	if (!_hRes) return 0;
	if (!_ongoingTransaction) return 0;

	uint8_t tx = 0;
	uint8_t rx = 0;
	HAL_SPI_TransmitReceive(&_hRes->handle, &tx, &rx, 1, HAL_MAX_DELAY);

	return rx;
}
#endif

void SPI2_IRQHandler(void)
{
	SPI::SPI_Interrupt(SPI::PORT_SPI2);
}

void SPI::SPI_Interrupt(port_t port)
{
	SPI::hardware_resource_t * spi = 0;
	switch (port) {
		case PORT_SPI2:
			spi = SPI::resSPI2;
			break;
		default:
			break;
	}

	if (!spi) {
		ERROR("SPI interrupt for unconfigured port");
		return;
	}

	/* Check BSY flag value in ISR register */
	if(!LL_SPI_IsActiveFlag_BSY(spi->instance))
	{
		if (spi->txRegSent && spi->txBytesToWrite == 0 && spi->rxBytesToRead == 0) {
			/* Disable RXNE  Interrupt             */
			LL_SPI_DisableIT_RXNE(spi->instance);

			/* Disable TXE   Interrupt             */
			LL_SPI_DisableIT_TXE(spi->instance);

			/* Disable ERR   Interrupt             */
			LL_SPI_DisableIT_ERR(spi->instance);

		#ifdef USE_FREERTOS
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR( spi->transmissionFinished, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		#else
			spi->transmissionFinished = true;
		#endif

			return;
		}
	}

	/* Check RXNE flag value in ISR register */
	if(LL_SPI_IsActiveFlag_RXNE(spi->instance))
	{
		/* Read character in Receive Data register.
		RXNE flag is cleared by reading data in RXDR register */
		if (spi->txRegSent && spi->txBytesToWrite == 0 && spi->rxBytesToRead > 0) {
			*spi->rxBuffer = LL_SPI_ReceiveData8(spi->instance);
			spi->rxBuffer++;
			spi->rxBytesToRead--;
		} else {
			LL_SPI_ReceiveData8(spi->instance); // read blindly to clear interrupt
		}
	}
	/* Check TXE flag value in ISR register */
	else if(LL_SPI_IsActiveFlag_TXE(spi->instance))
	{
		/* Write data in Transmit Data register.
		TXE flag is cleared by writing data in DR register */
		if (!spi->txRegSent) {
			LL_SPI_TransmitData8(spi->instance, spi->txReg);
			spi->txRegSent = true;
		} else if (spi->txBytesToWrite > 0) {
			LL_SPI_TransmitData8(spi->instance, (*spi->txBuffer++));
			spi->txBytesToWrite--;
		} else if (spi->txRegSent && spi->txBytesToWrite == 0 && spi->dummyReadBytesToSend > 0) {
			LL_SPI_TransmitData8(spi->instance, 0xFF); // send dummy byte for reading
			spi->dummyReadBytesToSend--;
		}
	}
	/* Check ERROR flag value in ISR register */
	else if(LL_SPI_IsActiveFlag_OVR(spi->instance))
	{
		/* Disable RXNE  Interrupt             */
		LL_SPI_DisableIT_RXNE(spi->instance);

		/* Disable TXE   Interrupt             */
		LL_SPI_DisableIT_TXE(spi->instance);

		/* Disable ERR   Interrupt             */
		LL_SPI_DisableIT_ERR(spi->instance);

		ERROR("Unknown I2C interrupt error");

	#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( spi->transmissionFinished, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	#else
		spi->transmissionFinished = true;
	#endif
	}
}
