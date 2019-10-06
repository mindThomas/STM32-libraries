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
 
#include "I2C.h"

#include "Priorities.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

I2C::hardware_resource_t * I2C::resI2C1 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void I2C1_IRQHandler(void);

I2C::I2C(port_t port, uint8_t devAddr, uint32_t frequency)
{
	_devAddr = devAddr << 1;
	InitPeripheral(port, frequency);
	ConfigurePeripheral();
}

I2C::I2C(port_t port, uint8_t devAddr)  /* : I2C(port, devAddr, I2C_DEFAULT_FREQUENCY)    // this is apparently not working properly   */
{
	_devAddr = devAddr << 1;
	InitPeripheral(port, I2C_DEFAULT_FREQUENCY);
	ConfigurePeripheral();
}

I2C::~I2C()
{
	if (!_hRes) return;

	_hRes->instances--;
	if (_hRes->instances == 0) { // deinitialize port and periphiral
		DeInitPeripheral();

		// Delete hardware resource
		port_t tmpPort = _hRes->port;
		delete(_hRes);

		switch (tmpPort)
		{
			case PORT_I2C1:
				resI2C1 = 0;
				break;
			default:
				ERROR("Undefined I2C port");
				return;
		}
	}
}

void I2C::DeInitPeripheral()
{
	if (!_hRes) return;
	if (_hRes->port == PORT_I2C1) {
		/* Peripheral clock disable */
		LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_I2C1);

		// ToDo: Deinit GPIO pins

		/* I2C1 interrupt DeInit */
		HAL_NVIC_DisableIRQ(I2C1_IRQn);
	}
}

void I2C::InitPeripheral(port_t port, uint32_t frequency)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	bool firstTime = false;

	switch (port)
	{
		case PORT_I2C1:
			if (!resI2C1) {
				resI2C1 = new I2C::hardware_resource_t;
				memset(resI2C1, 0, sizeof(I2C::hardware_resource_t));
				firstTime = true;
			}
			_hRes = resI2C1;
			break;
		default:
			ERROR("Undefined I2C port");
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
			_hRes = 0;
			ERROR("Could not create I2C resource semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->resourceSemaphore, "I2C Resource");
		xSemaphoreGive( _hRes->resourceSemaphore ); // give the semaphore the first time

		_hRes->transmissionFinished = xSemaphoreCreateBinary();
		if (_hRes->transmissionFinished == NULL) {
			_hRes = 0;
			ERROR("Could not create I2C transmission semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->transmissionFinished, "I2C Finish");
		xSemaphoreGive( _hRes->transmissionFinished ); // ensure that the semaphore is not taken from the beginning
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // ensure that the semaphore is not taken from the beginning
	#else
		_hRes->transmissionFinished = true;
		_hRes->resourceSemaphore = false;
	#endif

		// Configure pins for I2C and I2C peripheral accordingly
		if (port == PORT_I2C1) {
			/* Peripheral clock enable */
			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

			/**I2C1 GPIO Configuration
			PB10     ------> I2C1_SCL
			PB11     ------> I2C1_SDA
			*/
			LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
			GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
			GPIO_InitStruct.Speed = LL_GPIO_SPEED_HIGH;
			GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
			GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
			GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
			GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
			LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			/* I2C1 interrupt Init */
			NVIC_SetPriority(I2C1_IRQn, I2C_INTERRUPT_PRIORITY);
			NVIC_EnableIRQ(I2C1_IRQn);
		}
	}

	_hRes->instances++;
}

void I2C::ConfigurePeripheral()
{
	LL_I2C_InitTypeDef I2C_InitStruct;

	if (!_hRes) return;

	if (!_hRes->configured) { // only configure periphiral once
		switch (_hRes->port) {
			case PORT_I2C1:
				_hRes->instance = I2C1;
				break;
			default:
				_hRes = 0;
				ERROR("Undefined I2C port");
				return;
		}

		/* Reset I2C2 data registers */
	    if (LL_I2C_DeInit(_hRes->instance) != SUCCESS) {
	    	_hRes = 0;
	    	ERROR("I2C Error initializing");
			return;
	    }

	    /* Configure the Master to operate in 7-bit or 10-bit addressing mode */
	    /* Reset Value is LL_I2C_ADDRESSING_MODE_7BIT                         */
	    //LL_I2C_SetMasterAddressingMode(_hRes->instance, LL_I2C_ADDRESSING_MODE_7BIT);

	    /* Enable Peripheral in I2C mode */
	    /* Reset Value is I2C mode */
	    //LL_I2C_SetMode(_hRes->instance, LL_I2C_MODE_I2C);

	    /* Set fields of initialization structure:
	     *  - Peripheral Mode     = Mode I2C
	     *  - Timing Value        = fast Mode @400kHz with I2CCLK = 48 MHz
	     *                          rise time = 100ns,
	     *                          fall time = 10ns
	     *  - Analog Filter       = Enabled
	     *  - Digital Filter      = Disabled
	     *  - Own Address1        = No own address 1
	     *  - Type of Acknowledge = Acknowledge bytes/address received
	     *  - Own Address 1 Size  = default value Slave Address in 7-bit
	    */
	    I2C_InitStruct.PeripheralMode  = LL_I2C_MODE_I2C;
	    I2C_InitStruct.Timing          = 0x00901850; // 400 kHz I2C frequency @ 48 MHz I2C clock  (Timing register value is computed with the STM32CubeMX Tool)
	    I2C_InitStruct.AnalogFilter    = LL_I2C_ANALOGFILTER_ENABLE;
	    I2C_InitStruct.DigitalFilter   = 0x00;
	    I2C_InitStruct.OwnAddress1     = 0x00;
	    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
	    I2C_InitStruct.OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT;

	    /* Initialize I2C instance according to parameters defined in initialization structure. */
	    if (LL_I2C_Init(_hRes->instance, &I2C_InitStruct) != SUCCESS)
	    {
			_hRes = 0;
			ERROR("Could not initialize I2C port");
			return;
	    }

	    _hRes->configured = true;
	}
}

bool I2C::Write(uint8_t reg, uint8_t value)
{
	return Write(reg, &value, 1);
}

bool I2C::Write(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	return WriteInterrupt(reg, buffer, writeLength);
}

bool I2C::WriteBlocking(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
{
	bool success = false;
	if (!_hRes) return success;
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

	uint8_t bytesSent = 0;

	/* Master Generate Start condition for a write request :              */
	/*    - to the Slave with a 7-Bit _devAddr                            */
	/*    - with a auto stop condition generation when transmit all bytes */
	LL_I2C_HandleTransfer(_hRes->instance, _devAddr, LL_I2C_ADDRSLAVE_7BIT, writeLength+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	/* (2) Loop until end of transfer received (STOP flag raised) ***************/

	/* Loop until STOP flag is raised  */
	while(!LL_I2C_IsActiveFlag_STOP(_hRes->instance))
	{
		/* (2.1) Transmit data (TXIS flag raised) *********************************/

		/* Check TXIS flag value in ISR register */
		if(LL_I2C_IsActiveFlag_TXIS(_hRes->instance))
		{
			/* Write data in Transmit Data register.
			TXIS flag is cleared by writing data in TXDR register */
			if (bytesSent == 0)
				LL_I2C_TransmitData8(_hRes->instance, reg);
			else
				LL_I2C_TransmitData8(_hRes->instance, (*buffer++));

			bytesSent++;
			if (bytesSent > writeLength+1) {
				ERROR("I2C Error: Too many bytes sent");
				break;
			}
		}
	}

	/* (3) Clear pending flags, Data consistency are checking into Slave process */

	/* End of I2C_SlaveReceiver_MasterTransmitter Process */
	LL_I2C_ClearFlag_STOP(_hRes->instance);

	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

bool I2C::WriteInterrupt(uint8_t reg, uint8_t * buffer, uint8_t writeLength)
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

	_hRes->addr = _devAddr;
	_hRes->txRegSent = false;
	_hRes->txReg = reg;
	_hRes->txBuffer = buffer;
	_hRes->rxBuffer = 0;
	_hRes->txBytesToWrite = writeLength;
	_hRes->rxBytesToRead = 0;

	/* Master Generate Start condition for a write request :              */
	/*    - to the Slave with a 7-Bit _devAddr                            */
	/*    - with a auto stop condition generation when transmit all bytes */
	LL_I2C_HandleTransfer(_hRes->instance, _devAddr, LL_I2C_ADDRSLAVE_7BIT, writeLength+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	LL_I2C_EnableIT_TX(_hRes->instance);
	//LL_I2C_EnableIT_TC(_hRes->instance);
	//LL_I2C_EnableIT_RX(_hRes->instance);
	LL_I2C_EnableIT_NACK(_hRes->instance);
	LL_I2C_EnableIT_ERR(_hRes->instance);
	LL_I2C_EnableIT_STOP(_hRes->instance);

#ifdef USE_FREERTOS
	// Wait for the transmission to finish
	xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
#else
	while (!_hRes->transmissionFinished);
#endif
	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}


uint8_t I2C::Read(uint8_t reg)
{
	uint8_t rx;
	if (Read(reg, &rx, 1))
		return rx;
	else
		return 0;
}

bool I2C::Read(uint8_t reg, uint8_t * buffer, uint8_t readLength)
{
	return ReadInterrupt(reg, buffer, readLength);
}

bool I2C::ReadInterrupt(uint8_t reg, uint8_t * buffer, uint8_t readLength)
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
	_hRes->transmissionFinished = false;
#endif

	_hRes->addr = _devAddr;
	_hRes->txRegSent = false;
	_hRes->txReg = reg;
	_hRes->txBuffer = 0;
	_hRes->rxBuffer = buffer;
	_hRes->txBytesToWrite = 0;
	_hRes->rxBytesToRead = readLength;

	/* Master Generate Start condition for a write request :              */
	/*    - to the Slave with a 7-Bit _devAddr                            */
	/*    - with a auto stop condition generation when transmit all bytes */
	LL_I2C_HandleTransfer(_hRes->instance, _devAddr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

	LL_I2C_EnableIT_TX(_hRes->instance);
	//LL_I2C_EnableIT_TC(_hRes->instance);
	LL_I2C_EnableIT_RX(_hRes->instance);
	LL_I2C_EnableIT_NACK(_hRes->instance);
	LL_I2C_EnableIT_ERR(_hRes->instance);
	LL_I2C_EnableIT_STOP(_hRes->instance);

#ifdef USE_FREERTOS
	// Wait for the transmission to finish
	xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
#else
	while (!_hRes->transmissionFinished);
#endif
	success = true;

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

void I2C1_IRQHandler(void)
{
	I2C::I2C_Interrupt(I2C::PORT_I2C1);
}

void I2C::I2C_Interrupt(port_t port)
{
	I2C::hardware_resource_t * i2c = 0;
	switch (port) {
		case PORT_I2C1:
			i2c = I2C::resI2C1;
			break;
		default:
			break;
	}

	if (!i2c) {
		ERROR("I2C interrupt for unconfigured port");
		return;
	}

	/* Check TXE flag value in ISR register */
	if (LL_I2C_IsActiveFlag_TXIS(i2c->instance))
	{
		/* Write data in Transmit Data register.
		TXIS flag is cleared by writing data in TXDR register */
		if (!i2c->txRegSent) {
			LL_I2C_TransmitData8(i2c->instance, i2c->txReg);
			i2c->txRegSent = true;
		} else if (i2c->txBytesToWrite > 0) {
			LL_I2C_TransmitData8(i2c->instance, (*i2c->txBuffer++));
			i2c->txBytesToWrite--;
		}
	}
	/* Check RXNE flag value in ISR register */
	else if(LL_I2C_IsActiveFlag_RXNE(i2c->instance))
	{
		/* Read character in Receive Data register.
		RXNE flag is cleared by reading data in RXDR register */
		if (i2c->rxBytesToRead > 0) {
			*i2c->rxBuffer = LL_I2C_ReceiveData8(i2c->instance);
			i2c->rxBuffer++;
			i2c->rxBytesToRead--;
		}
	}
	/* Check STOP flag value in ISR register */
	else if(LL_I2C_IsActiveFlag_STOP(i2c->instance))
	{
		/* End of Transfer */
		LL_I2C_ClearFlag_STOP(i2c->instance);

	    /* Check TXE flag value in ISR register */
	    if(!LL_I2C_IsActiveFlag_TXE(i2c->instance))
	    {
	      /* Flush the TXDR register */
	      LL_I2C_ClearFlag_TXE(i2c->instance);
	    }

		// STOP condition received - determine if a repeated start should be sent
	    if (i2c->rxBytesToRead > 0) {
	    	LL_I2C_HandleTransfer(i2c->instance, i2c->addr, LL_I2C_ADDRSLAVE_7BIT, i2c->rxBytesToRead, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
	    } else {
	    	// Transmission finished
		#ifdef USE_FREERTOS
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR( i2c->transmissionFinished, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		#else
			i2c->transmissionFinished = true;
		#endif
	    }
	}
	/* Check NACK flag value in ISR register */
	else if(LL_I2C_IsActiveFlag_NACK(i2c->instance))
	{
		/* Clear interrupt flag */
		LL_I2C_ClearFlag_NACK(i2c->instance);
	}
	/* Check TXE flag value in ISR register */
	else if(!LL_I2C_IsActiveFlag_TXE(i2c->instance))
	{
		/* Do nothing */
		/* This Flag will be set by hardware when the TXDR register is empty */
		/* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
	}
	else
	{
		/* Call Error function */
		ERROR("Unknown I2C interrupt error");
	}
}
