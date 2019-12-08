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
#include "stm32h7xx_hal.h"
#include "Priorities.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

SPI::hardware_resource_t * SPI::resSPI3 = 0;
SPI::hardware_resource_t * SPI::resSPI5 = 0;
SPI::hardware_resource_t * SPI::resSPI6 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void SPI3_IRQHandler(void);
extern "C" __EXPORT void SPI5_IRQHandler(void);
extern "C" __EXPORT void SPI6_IRQHandler(void);
extern "C" __EXPORT void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
extern "C" __EXPORT void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);

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
		case PORT_SPI3:
			_csPort = GPIOA;
			_csPin = GPIO_PIN_15;
			break;
		case PORT_SPI5:
			_csPort = GPIOD;
			_csPin = GPIO_PIN_4;
			break;
		case PORT_SPI6:
			_csPort = GPIOG;
			_csPin = GPIO_PIN_8;
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
			case PORT_SPI3:
				resSPI3 = 0;
				break;
			case PORT_SPI5:
				resSPI5 = 0;
				break;
			case PORT_SPI6:
				resSPI6 = 0;
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
	if (_hRes->port == PORT_SPI3) {
		/* Peripheral clock disable */
		__HAL_RCC_SPI3_CLK_DISABLE();

		/**SPI3 GPIO Configuration
		PC10     ------> SPI3_SCK
		PC11     ------> SPI3_MISO
		PC12     ------> SPI3_MOSI
		*/
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

		/* SPI3 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI3_IRQn);
	}
	else if (_hRes->port == PORT_SPI5)
	{
		/* Peripheral clock disable */
		__HAL_RCC_SPI5_CLK_DISABLE();

		/**SPI6 GPIO Configuration
		PG12     ------> SPI6_MISO
		PG13     ------> SPI6_SCK
		PG14     ------> SPI6_MOSI
		*/
		HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

		/* SPI6 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI5_IRQn);
	}
	else if (_hRes->port == PORT_SPI6)
	{
		/* Peripheral clock disable */
		__HAL_RCC_SPI6_CLK_DISABLE();

		/**SPI6 GPIO Configuration
		PG12     ------> SPI6_MISO
		PG13     ------> SPI6_SCK
		PG14     ------> SPI6_MOSI
		*/
		HAL_GPIO_DeInit(GPIOG, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14);

		/* SPI6 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI6_IRQn);
	}
}

void SPI::DeInitChipSelect()
{
	if (!_hRes) return;
	if (_csPort) { // general chip select configuration
		HAL_GPIO_DeInit(_csPort, _csPin);
	}
	else // hardware chip select
	{
		switch (_hRes->port) {
			case PORT_SPI3: // hardware SPI not defined (for now)
				ERROR("Hardware chip-select for SPI3 is not defined");
				return;
			case PORT_SPI5: // hardware SPI not defined (for now)
				ERROR("Hardware chip-select for SPI5 is not defined");
				return;
			case PORT_SPI6: // SPI6_NSS ------> PG8
				HAL_GPIO_DeInit(GPIOG, GPIO_PIN_8);
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}
	}
}

void SPI::InitPeripheral(port_t port, uint32_t frequency)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool firstTime = false;

	switch (port)
	{
		case PORT_SPI3:
			if (!resSPI3) {
				resSPI3 = new SPI::hardware_resource_t;
				memset(resSPI3, 0, sizeof(SPI::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resSPI3;
			break;
		case PORT_SPI5:
			if (!resSPI5) {
				resSPI5 = new SPI::hardware_resource_t;
				memset(resSPI3, 0, sizeof(SPI::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resSPI5;
			break;
		case PORT_SPI6:
			if (!resSPI6) {
				resSPI6 = new SPI::hardware_resource_t;
				memset(resSPI3, 0, sizeof(SPI::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resSPI6;
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

		// Configure pins for SPI and SPI peripheral accordingly
		if (port == PORT_SPI3) {
			__HAL_RCC_SPI3_CLK_ENABLE();
			/**SPI3 GPIO Configuration
			PC10     ------> SPI3_SCK
			PC11     ------> SPI3_MISO
			PC12     ------> SPI3_MOSI
			*/
			__HAL_RCC_GPIOC_CLK_ENABLE();
			GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
			HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

			/* SPI3 interrupt Init */
			HAL_NVIC_SetPriority(SPI3_IRQn, SPI_INTERRUPT_PRIORITY, 0);
			HAL_NVIC_EnableIRQ(SPI3_IRQn);
		}
		else if (port == PORT_SPI5)
		{
			__HAL_RCC_SPI5_CLK_ENABLE();
			/**SPI3 GPIO Configuration
			PF7     ------> SPI5_SCK
			PF8     ------> SPI5_MISO
			PF9     ------> SPI5_MOSI
			*/
			__HAL_RCC_GPIOF_CLK_ENABLE();
			GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
			HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

			/* SPI3 interrupt Init */
			HAL_NVIC_SetPriority(SPI5_IRQn, SPI_INTERRUPT_PRIORITY, 0);
			HAL_NVIC_EnableIRQ(SPI5_IRQn);
		}
		else if (port == PORT_SPI6)
		{
			/* Peripheral clock enable */
			__HAL_RCC_SPI6_CLK_ENABLE();
			/**SPI6 GPIO Configuration
			PG12     ------> SPI6_MISO
			PG13     ------> SPI6_SCK
			PG14     ------> SPI6_MOSI
			*/
			__HAL_RCC_GPIOG_CLK_ENABLE();
			GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
			GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;
			HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

			/* SPI6 interrupt Init */
			HAL_NVIC_SetPriority(SPI6_IRQn, SPI_INTERRUPT_PRIORITY, 0);
			HAL_NVIC_EnableIRQ(SPI6_IRQn);
		}
	}

	_hRes->instances++;
}

void SPI::InitChipSelect()
{
	if (_csPort) { // general chip select configuration
		// Enable clock
		if (_csPort == GPIOA)
			__HAL_RCC_GPIOA_CLK_ENABLE();
		else if (_csPort == GPIOB)
			__HAL_RCC_GPIOB_CLK_ENABLE();
		else if (_csPort == GPIOC)
			__HAL_RCC_GPIOC_CLK_ENABLE();
		else if (_csPort == GPIOD)
			__HAL_RCC_GPIOD_CLK_ENABLE();
		else if (_csPort == GPIOE)
			__HAL_RCC_GPIOE_CLK_ENABLE();
		else if (_csPort == GPIOF)
			__HAL_RCC_GPIOF_CLK_ENABLE();
		else if (_csPort == GPIOG)
			__HAL_RCC_GPIOG_CLK_ENABLE();
		else if (_csPort == GPIOH)
			__HAL_RCC_GPIOH_CLK_ENABLE();

		HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET); // set pin high after configuration = chip is NOT selected

		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = _csPin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(_csPort, &GPIO_InitStruct);

	}
	else { // hardware chip select configuration
		if (!_hRes) return;
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI6;

		switch (_hRes->port) {
			case PORT_SPI3: // hardware SPI not defined (for now)
				ERROR("Hardware chip-select for SPI3 is not defined");
				return;
			case PORT_SPI5: // hardware SPI not defined (for now)
				ERROR("Hardware chip-select for SPI5 is not defined");
				return;
			case PORT_SPI6: // SPI6_NSS ------> PG8
				GPIO_InitStruct.Pin = GPIO_PIN_8;
				HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}
	}
}

void SPI::ConfigurePeripheral()
{
	if (!_hRes) return;
	if (!_hRes->configured) { // only configure periphiral once
		switch (_hRes->port) {
			case PORT_SPI3:
				_hRes->handle.Instance = SPI3;
				break;
			case PORT_SPI5:
				_hRes->handle.Instance = SPI5;
				break;
			case PORT_SPI6:
				_hRes->handle.Instance = SPI6;
				break;
			default:
				ERROR("Undefined SPI port");
				return;
		}

		_hRes->handle.Init.Mode = SPI_MODE_MASTER;
		_hRes->handle.Init.Direction = SPI_DIRECTION_2LINES;
		_hRes->handle.Init.DataSize = SPI_DATASIZE_8BIT;
		_hRes->handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
		_hRes->handle.Init.CLKPhase = SPI_PHASE_2EDGE;
		_hRes->handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
		_hRes->handle.Init.TIMode = SPI_TIMODE_DISABLE;
		_hRes->handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		_hRes->handle.Init.CRCPolynomial = 7;
		_hRes->handle.Init.NSS = SPI_NSS_SOFT;  // general chip-select - we have decided not to support hardware chip-select (NSS pin)
		_hRes->handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		_hRes->handle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
		_hRes->handle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
		_hRes->handle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		_hRes->handle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
		_hRes->handle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
		_hRes->handle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
		_hRes->handle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
		_hRes->handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
		_hRes->handle.Init.IOSwap = SPI_IO_SWAP_DISABLE;

		uint32_t SPI_Clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI123); // assuming all SPI periphirals to be configured with the same clock frequency!!!
		// Find the prescaler which matches the desired frequency best (rounding down)
		float prescaler = (float)SPI_Clock / _hRes->frequency;
		if (prescaler <= 2)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
		else if (prescaler <= 4)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		else if (prescaler <= 8)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
		else if (prescaler <= 16)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
		else if (prescaler <= 32)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
		else if (prescaler <= 64)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
		else if (prescaler <= 128)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
		else if (prescaler <= 256)
			_hRes->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
		else
		{
			ERROR("SPI baud rate too low to be configurable");
			return;
		}

		if (HAL_SPI_Init(&_hRes->handle) != HAL_OK)
		{
			ERROR("Could not initialize SPI port");
			return;
		}

		osDelay(10); // wait 10 ms for clock to stabilize

		_hRes->configured = true;
	}
}

void SPI::ReconfigureFrequency(uint32_t frequency)
{
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	_hRes->frequency = frequency;
	_hRes->configured = false;
	ConfigurePeripheral();

	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
}

void SPI::Write(uint8_t reg, uint8_t value)
{
	Write(reg, &value, 1);
}

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

uint8_t SPI::Read(uint8_t reg)
{
	uint8_t rx;
	Read(reg, &rx, 1);
	return rx;
}

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

void SPI3_IRQHandler(void)
{
	if (SPI::resSPI3)
		HAL_SPI_IRQHandler(&SPI::resSPI3->handle);
}

void SPI5_IRQHandler(void)
{
	if (SPI::resSPI5)
		HAL_SPI_IRQHandler(&SPI::resSPI5->handle);
}

void SPI6_IRQHandler(void)
{
	if (SPI::resSPI6)
		HAL_SPI_IRQHandler(&SPI::resSPI6->handle);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// TRANSFER_COMPLETE
	SPI::hardware_resource_t * spi;
	if (hspi->Instance == SPI3)
		spi = SPI::resSPI3;
	else if (hspi->Instance == SPI5)
		spi = SPI::resSPI5;
	else if (hspi->Instance == SPI6)
		spi = SPI::resSPI6;
	else
		return;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( spi->transmissionFinished, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	// TRANSFER_ERROR
	SPI::hardware_resource_t * spi;
	if (hspi->Instance == SPI3)
		spi = SPI::resSPI3;
	else if (hspi->Instance == SPI5)
		spi = SPI::resSPI5;
	else if (hspi->Instance == SPI6)
		spi = SPI::resSPI6;
	else
		return;

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( spi->transmissionFinished, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
