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
 
#ifndef PERIPHIRALS_SPI_H
#define PERIPHIRALS_SPI_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback

class SPI
{
	private:
		const uint32_t SPI_DEFAULT_FREQUENCY = 1000000;		// 1 MHz

	public:
		typedef enum port_t {
			PORT_UNDEFINED = 0,
			PORT_SPI3,
			PORT_SPI5,
			PORT_SPI6
		} port_t;

	public:
		SPI(port_t port); // use default chip-select and default frequency (or current configured frequency)
		SPI(port_t port, uint32_t frequency); // use default chip-select and configure with frequency if possible
		SPI(port_t port, uint32_t frequency, GPIO_TypeDef * GPIOx, uint32_t GPIO_Pin); // use chosen chip select and configure frequency if possible
		~SPI();

		void InitPeripheral(port_t port, uint32_t frequency);
		void InitChipSelect();
		void DeInitPeripheral();
		void DeInitChipSelect();
		void ConfigurePeripheral();
		void ReconfigureFrequency(uint32_t frequency);

		void Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength);
		void Write(uint8_t reg, uint8_t value);
		void Read(uint8_t reg, uint8_t * buffer, uint8_t readLength);
		uint8_t Read(uint8_t reg);

		void BeginTransaction();
		void EndTransaction();
		void TransactionWrite8(uint8_t value);
		void TransactionWrite16(uint16_t value);
		void TransactionWrite32(uint32_t value);
		uint8_t TransactionRead();

	public:
		typedef struct hardware_resource_t {
			port_t port;
			uint32_t frequency;
			SemaphoreHandle_t resourceSemaphore;
			SemaphoreHandle_t transmissionFinished;
			bool configured;
			uint8_t instances; // how many objects are using this hardware resource
			SPI_HandleTypeDef handle;
		} hardware_resource_t;

		static hardware_resource_t * resSPI3;
		static hardware_resource_t * resSPI5;
		static hardware_resource_t * resSPI6;
	
	private:
		hardware_resource_t * _hRes;
		GPIO_TypeDef * _csPort;
		uint32_t _csPin;
		bool _ongoingTransaction;
};
	
	
#endif
