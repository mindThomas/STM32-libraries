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
 
#ifndef PERIPHIRALS_I2C_H
#define PERIPHIRALS_I2C_H


#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
//#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_i2c.h"

#ifdef USE_FREERTOS
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback
#endif

class I2C
{
	private:
		const uint32_t I2C_DEFAULT_FREQUENCY = 100000;	// 100 kHz

	public:
		typedef enum port_t {
			PORT_UNDEFINED = 0,
			PORT_I2C1
		} port_t;

	public:
		I2C(port_t port, uint8_t devAddr); // use default frequency (or current configured frequency)
		I2C(port_t port, uint8_t devAddr, uint32_t frequency); // configure with frequency if possible
		~I2C();

		void InitPeripheral(port_t port, uint32_t frequency);
		void DeInitPeripheral();
		void ConfigurePeripheral();
		bool Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength);
		bool Write(uint8_t reg, uint8_t value);
		bool Read(uint8_t reg, uint8_t * buffer, uint8_t readLength);
		uint8_t Read(uint8_t reg);

	private:
		bool WriteBlocking(uint8_t reg, uint8_t * buffer, uint8_t writeLength);
		bool WriteInterrupt(uint8_t reg, uint8_t * buffer, uint8_t writeLength);
		bool ReadInterrupt(uint8_t reg, uint8_t * buffer, uint8_t readLength);

	public:
		typedef struct hardware_resource_t {
			port_t port;
			uint32_t frequency;
			I2C_TypeDef * instance;
			bool configured;
			uint8_t instances; // how many objects are using this hardware resource
			// Variables for Interrupt based TX/RX
			uint8_t addr;
			bool txRegSent;
			uint8_t txReg;
			uint8_t * txBuffer;
			uint8_t * rxBuffer;
			uint8_t txBytesToWrite;
			uint8_t rxBytesToRead;
		#ifdef USE_FREERTOS
			SemaphoreHandle_t resourceSemaphore;
			SemaphoreHandle_t transmissionFinished;
		#else
			bool resourceSemaphore; // resource in use?
			bool transmissionFinished;
		#endif
		} hardware_resource_t;

		static hardware_resource_t * resI2C1;

	private:
		hardware_resource_t * _hRes;
		uint8_t _devAddr;
	
	public:
		static void I2C_Interrupt(port_t port);
};
	
	
#endif
