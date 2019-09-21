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
 
#ifndef PERIPHIRALS_UART_H
#define PERIPHIRALS_UART_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback

#define UART_CALLBACK_PARAMS (void * param, uint8_t * buffer, uint32_t bufLen)

class UART
{
	private:
		const int CALLBACK_THREAD_POP_BUFFER_SIZE = 200;

	public:
		const uint32_t BaudRate;

	public:
		typedef enum port_t {
			PORT_UNDEFINED = 0,
			PORT_UART3,
			PORT_UART4,
			PORT_UART7
		} port_t;

	public:
		UART(port_t port, uint32_t baud); // unbuffered constructor = polling only
		UART(port_t port, uint32_t baud, uint32_t bufferLength); // ring-buffered constructor
		~UART();

		void InitPeripheral();
		void DeInitPeripheral();
		void ConfigurePeripheral();
		void RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS, void * parameter = (void*)0, uint32_t chunkLength = 0); // callback with chunks of available data
		void DeregisterCallback();

		void Write(uint8_t byte);
		uint32_t Write(uint8_t * buffer, uint32_t length);
		uint32_t WriteBlocking(uint8_t * buffer, uint32_t length);
		int16_t Read();
		bool Available();
		uint32_t AvailableLength();
		uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY);
		bool Connected();

	private:
		void TransmitBlocking(uint8_t * buffer, uint32_t bufLen);
		void TransmitBlockingHard(uint8_t * buffer, uint32_t bufLen);

	private:
		port_t _port;
		uint32_t _baud;
		UART_HandleTypeDef _handle;
		uint8_t rxByte;
		uint8_t * _buffer;
		uint32_t _bufferLength;
		uint32_t _bufferWriteIdx;
		uint32_t _bufferReadIdx;
		uint32_t _callbackChunkLength;
		TaskHandle_t _callbackTaskHandle;
		SemaphoreHandle_t _resourceSemaphore;
		SemaphoreHandle_t _transmitByteFinished;
		SemaphoreHandle_t _RXdataAvailable;
		void (*_RXcallback)UART_CALLBACK_PARAMS;
		void * _RXcallbackParameter;

	public:
		static void UART_Interrupt(port_t port);

	private:
		static void UART_IncomingDataInterrupt(UART * uart);
		static void CallbackThread(void * pvParameters);

	private:
		void BufferPush(uint8_t byte);
		uint8_t BufferPop();
		uint32_t BufferContentSize();
		uint32_t BufferPopMax(uint8_t * buffer, uint32_t bufferSize);
		uint8_t * BufferPopN(uint32_t numberOfBytesToPush);

	private:
		static UART * objUART3;
		static UART * objUART4;
		static UART * objUART7;
};
	
#endif
