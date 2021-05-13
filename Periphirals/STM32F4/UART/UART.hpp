/* Copyright (C) 2018- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#pragma once

#include <stm32f4xx_hal.h>

// FreeRTOS for memory allocation (for the buffer) and callback
#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#define UART_CALLBACK_PARAMS (void * param, uint8_t * buffer, uint32_t bufLen)

class UART
{
	private:
		const int CALLBACK_THREAD_POP_BUFFER_SIZE = 200;
		const int DMA_RX_MIN_BUFFER_SIZE = 100;

	public:
		const uint32_t BaudRate;
		const bool DMA_Enabled;

	public:
		typedef enum port_t {
			PORT_UNDEFINED = 0,
			PORT_UART2
		} port_t;

		typedef enum // borrowed from stm32*_hal_uart.h
		{
			UART_ERROR_NONE      = 0x00U,    /*!< No error            */
			UART_ERROR_PE        = 0x01U,    /*!< Parity error        */
			UART_ERROR_NE        = 0x02U,    /*!< Noise error         */
			UART_ERROR_FE        = 0x04U,    /*!< frame error         */
			UART_ERROR_ORE       = 0x08U,    /*!< Overrun error       */
			UART_ERROR_DMA       = 0x10U     /*!< DMA transfer error  */
		} uart_errors_t;

	public:
		UART(port_t port, uint32_t baud, bool DMA_enabled = true); // unbuffered constructor = polling only
		UART(port_t port, uint32_t baud, uint32_t bufferLength, bool DMA_enabled = true); // ring-buffered constructor
		~UART();

		void InitPeripheral();
		void DeInitPeripheral();
		void InitPeripheral_DMA();
		void DeInitPeripheral_DMA();
		void ConfigurePeripheral();
		void ConfigurePeripheral_DMA();

		void RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS, void * parameter = (void*)0, uint32_t chunkLength = 0); // callback with chunks of available data
		void DeregisterCallback();

		void Write(uint8_t byte);
		uint32_t Write(uint8_t * buffer, uint32_t length);
		uint32_t WriteBlocking(uint8_t * buffer, uint32_t length);
		int16_t Read();
		bool Available();
		uint32_t AvailableLength();
#ifdef USE_FREERTOS
		uint32_t WaitForNewData(uint32_t millisecondsToWait = portMAX_DELAY);
#else
		uint32_t WaitForNewData(uint32_t millisecondsToWait = 0);
#endif
		bool Connected();

	private:
		void TransmitBlocking(uint8_t * buffer, uint32_t bufLen);
		void TransmitBlockingDMA(uint8_t * buffer, uint32_t bufLen);
		void StartDMATransfer(uint8_t *pData, uint16_t Size);
		void TransmitBlockingAutoInterrupt(uint8_t * buffer, uint32_t bufLen);
		void TransmitBlockingManualInterrupt(uint8_t * buffer, uint32_t bufLen);
		void TransmitBlockingHardPolling(uint8_t * buffer, uint32_t bufLen);

	private:
		port_t _port;
		UART_HandleTypeDef _handle;
		DMA_HandleTypeDef _rxDMA;
		DMA_HandleTypeDef _txDMA;
		uint8_t _ErrorCode;
		uint8_t rxByte;
		bool _rxByteAvailable;
		// Receive circular buffer
		uint8_t * _buffer;
		uint32_t _bufferLength;
		uint32_t _bufferWriteIdx;
		uint32_t _bufferReadIdx;
		uint32_t _callbackChunkLength;
		uint8_t * _txPointer;
		uint32_t _txRemainingBytes;
		bool _rxAvailableOnIdle;
	#ifdef USE_FREERTOS
		TaskHandle_t _callbackTaskHandle;
		SemaphoreHandle_t _resourceSemaphore;
		SemaphoreHandle_t _transmitFinished;
		SemaphoreHandle_t _RXdataAvailable;
    #else
        bool _transmitFinished;
        bool _RXdataAvailable;
	#endif
		void (*_RXcallback)UART_CALLBACK_PARAMS;
		void * _RXcallbackParameter;

	public:
		static void UART_Interrupt(port_t port);
		static void DMA_RX_Interrupt(UART * uart);
		static void DMA_TX_Interrupt(UART * uart);
		static void DMA_TX_Completed(UART * uart);
		static void DMA_RX_Check(UART * uart);
		static void DMA_Error(UART * uart);
		static void DMATransmitCplt(DMA_HandleTypeDef *hdma);
		static void DMAError(DMA_HandleTypeDef *hdma);

	private:
		static void UART_IncomingDataInterrupt(UART * uart);
	#ifdef USE_FREERTOS
		static void CallbackThread(void * pvParameters);
	#endif

	private:
		void BufferPush(uint8_t byte);
		uint8_t BufferPop();
		uint32_t BufferContentSize();
		uint32_t BufferPopMax(uint8_t * buffer, uint32_t bufferSize);
		uint8_t * BufferPopN(uint32_t numberOfBytesToPush);

	public:
		static UART * objUART2;
};
