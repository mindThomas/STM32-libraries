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
 
#include "UART.h"

#include "Priorities.h"
#include "Debug.h"
#include <string.h> // for memset

#ifndef USE_FREERTOS
#include <malloc.h>
#endif
 
UART * UART::objUART1 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void USART1_IRQHandler(void);

#ifdef USE_FREERTOS
UART::UART(port_t port, uint32_t baud, uint32_t bufferLength) : BaudRate(baud), _port(port), _bufferLength(bufferLength), _bufferWriteIdx(0), _bufferReadIdx(0),
		_callbackTaskHandle(0), _resourceSemaphore(0),
		_callbackChunkLength(0), _txPointer(0), _txRemainingBytes(0),
		_transmitByteFinished(0), _RXdataAvailable(0), _RXcallback(0), _RXcallbackParameter(0)
#else
UART::UART(port_t port, uint32_t baud, uint32_t bufferLength) : BaudRate(baud), _port(port), _bufferLength(bufferLength), _bufferWriteIdx(0), _bufferReadIdx(0),
		_callbackChunkLength(0), _txPointer(0), _txRemainingBytes(0),
		_transmitByteFinished(true), _RXcallback(0), _RXcallbackParameter(0)
#endif
{
	if (_bufferLength > 0) {
		#ifdef USE_FREERTOS
		_buffer = (uint8_t *)pvPortMalloc(_bufferLength);
		#else
		_buffer = (uint8_t *)malloc(_bufferLength);
		#endif
		memset(_buffer, 0, _bufferLength);
	} else {
		_buffer = 0;
	}
	InitPeripheral();
	ConfigurePeripheral();
}

UART::UART(port_t port, uint32_t baud) : UART(port, baud, 0)
{
}

UART::~UART()
{
	DeInitPeripheral();

	if (_buffer) {
		#ifdef USE_FREERTOS
		vPortFree(_buffer);
		#else
		free(_buffer);
		#endif
	}
}

void UART::ConfigurePeripheral()
{
	LL_USART_InitTypeDef USART_InitStruct;

	switch (_port) {
		case PORT_UART1:
			if (objUART1) {
				ERROR("UART1 already in used");
				return;
			}
			_instance = USART1;
			break;
		default:
			ERROR("Undefined UART port");
			return;
	}

	USART_InitStruct.BaudRate            = BaudRate;
	USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;

	if (LL_USART_Init(_instance, &USART_InitStruct) != SUCCESS)
	{
		ERROR("Could not initialize UART port");
		return;
	}

	switch (_port) {
		case PORT_UART1:
			objUART1 = this;
			break;
		default:
			break;
	}

#ifdef USE_FREERTOS
	_resourceSemaphore = xSemaphoreCreateBinary();
	if (_resourceSemaphore == NULL) {
		ERROR("Could not create UART resource semaphore");
		return;
	}
	vQueueAddToRegistry(_resourceSemaphore, "UART Resource");
	xSemaphoreGive( _resourceSemaphore ); // give the semaphore the first time

	// Create binary semaphore for indicating when a single byte has finished transmitting (for flagging to the transmit thread)
	_transmitByteFinished = xSemaphoreCreateBinary();
	if (_transmitByteFinished == NULL) {
		ERROR("Could not create UART transmit finished semaphore");
		return;
	}
	vQueueAddToRegistry(_transmitByteFinished, "UART Finished");
	xSemaphoreGive( _transmitByteFinished ); // give the semaphore the first time

	_RXdataAvailable = xSemaphoreCreateBinary();
	if (_RXdataAvailable == NULL) {
		ERROR("Could not create UART RX available semaphore");
		return;
	}
	vQueueAddToRegistry(_RXdataAvailable, "UART RX Available");
#endif

	/* (5) Enable USART *********************************************************/
	LL_USART_Enable(_instance);

	/* Polling USART initialization */
	while((!(LL_USART_IsActiveFlag_TEACK(_instance))) || (!(LL_USART_IsActiveFlag_REACK(_instance))))
	{
	}

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	LL_USART_EnableIT_ERROR(_instance);

	/* Disable the UART Transmit Complete Interrupt */
	LL_USART_DisableIT_TC(_instance);

	/* Enable the UART Parity Error interupt and Data Register (RX) Not Empty interrupt */
	LL_USART_EnableIT_PE(_instance);
	LL_USART_EnableIT_RXNE(_instance);

	/* Clear OverRun Error Flag */
	LL_USART_ClearFlag_ORE(_instance);
}

void UART::InitPeripheral()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (_port == PORT_UART1)
	{
		/* Peripheral clock enable */
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
		LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		/**UART4 GPIO Configuration
		PB6     ------> UART1_TX
		PB7     ------> UART1_RX
		*/

		GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_LOW;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* UART1 interrupt Init */
		NVIC_SetPriority(USART1_IRQn, UART_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(USART1_IRQn);
	}
}

void UART::DeInitPeripheral()
{
  if(_port == PORT_UART1)
  {
	/* Peripheral clock disable */
	  LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_USART1);

	// ToDo: Deinit GPIO pins

	/* UART1 interrupt DeInit */
	NVIC_DisableIRQ(USART1_IRQn);
  }
}

void UART::RegisterRXcallback(void (*callback)UART_CALLBACK_PARAMS, void * parameter, uint32_t chunkLength)
{
	_RXcallback = callback;
	_RXcallbackParameter = parameter;
	_callbackChunkLength = chunkLength;

	if (_callbackChunkLength > _bufferLength)
		_callbackChunkLength = _bufferLength;

#ifdef USE_FREERTOS
	switch (_port) {
		case PORT_UART1:
			xTaskCreate(UART::CallbackThread, (char *)"UART1 callback", 512, (void*) this, UART_RECEIVER_PRIORITY, &_callbackTaskHandle);
			break;
		default:
			break;
	}
#endif
}

void UART::DeregisterCallback()
{
	_RXcallback = 0;
	_RXcallbackParameter = 0;
	_callbackChunkLength = 0;

#ifdef USE_FREERTOS
	if (_callbackTaskHandle) {
		xTaskResumeFromISR(_callbackTaskHandle);
		_callbackTaskHandle = 0;
	}
#endif
}

void UART::TransmitBlocking(uint8_t * buffer, uint32_t bufLen)
{
	if (!bufLen) return;

	_txPointer = buffer;
	_txRemainingBytes = bufLen;

    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(_instance, *_txPointer++);
    _txRemainingBytes--;

	if (_txRemainingBytes > 0) {
	    /* Enable TXE interrupt */
	    LL_USART_EnableIT_TXE(_instance);
	} else {
	    /* Enable TC interrupt */
	    LL_USART_EnableIT_TC(_instance);
	}

    while (_txRemainingBytes > 0);
}

void UART::TransmitBlockingHardInterrupt(uint8_t * buffer, uint32_t bufLen)
{
	if (!bufLen) return;

	/* Enable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled). */
	//LL_USART_EnableIT_TXE(_instance);
	//LL_USART_DisableIT_TC(_instance);
	//LL_USART_ClearFlag_TC(_instance);

	do {
		#ifdef USE_FREERTOS
		xSemaphoreTake( _transmitByteFinished, ( TickType_t ) portMAX_DELAY ); // block until it has finished sending the byte
		#else
		while (!_transmitByteFinished);
		_transmitByteFinished = false;
		#endif

		// Transmit the data
		LL_USART_TransmitData8(_instance, *buffer++);

		// If last char to be sent, enable TC interrupt
		if (bufLen == 1) {
			LL_USART_EnableIT_TC(_instance);
		} else {
			// Enable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
			LL_USART_EnableIT_TXE(_instance);
		}
	} while (--bufLen > 0);

	/* Disable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled). */
	//LL_USART_DisableIT_TXE(_instance);
}

void UART::TransmitBlockingHardPolling(uint8_t * buffer, uint32_t bufLen)
{
	if (!bufLen) return;

	LL_USART_DisableIT_TXE(_instance);
	LL_USART_DisableIT_TC(_instance);

	do {
		/* Wait for TXE flag to be raised */
		while (!LL_USART_IsActiveFlag_TXE(_instance));

	    /* If last char to be sent, clear TC flag */
	    if (bufLen == 1)
	      LL_USART_ClearFlag_TC(_instance);

		/* Write character in Transmit Data register.
		 * TXE flag is cleared by writing data in TDR register */
	    LL_USART_TransmitData8(_instance, *buffer++);
	} while (--bufLen > 0);

    /* Wait for TC flag to be raised for last char */
    while (!LL_USART_IsActiveFlag_TC(_instance));
}

void UART::Write(uint8_t byte)
{
#ifdef USE_FREERTOS
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	// OBS! This should be replaced with a non-blocking call by making a TX queue and a processing thread in the UART object
	TransmitBlocking(&byte, 1); // transmit with interrupt-based semaphore waiting (blocking only this thread)

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
#else
	TransmitBlocking(&byte, 1);
#endif
}

uint32_t UART::Write(uint8_t * buffer, uint32_t length)
{
#ifdef USE_FREERTOS
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

	TransmitBlocking(buffer, length); // transmit with interrupt-based semaphore waiting (blocking only this thread)

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
#else
	TransmitBlocking(buffer, length);
#endif
	return length;
}

uint32_t UART::WriteBlocking(uint8_t * buffer, uint32_t length)
{
	return Write(buffer, length);
}

int16_t UART::Read()
{
	if (!_bufferLength) return -1; // error, buffer not enabled
	if (BufferContentSize() == 0) return -1; // error, buffer is empty
	return BufferPop();
}

bool UART::Available()
{
	if (!_bufferLength) return false; // error, buffer not enabled
	return (_bufferWriteIdx != _bufferReadIdx);
}

uint32_t UART::AvailableLength()
{
	return BufferContentSize();
}

bool UART::Connected()
{
	return true; // UART is always connected after initializing and configuring the periphiral (port is opened)
}

#if 0
void UART::CallbackThread(void * pvParameters)
{
	UART * uart = (UART *)pvParameters;

	uint8_t * popBuffer = (uint8_t *)pvPortMalloc(uart->CALLBACK_THREAD_POP_BUFFER_SIZE);
	if (!popBuffer) ERROR("Could not create pop-buffer for UART Callback thread");

	while (1) {
		vTaskSuspend(NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

		if (uart->_RXcallback) {
			if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
				uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
			} else { // buffer enabled, hence process buffer content
				while (uart->Available()) {
					if (uart->_callbackChunkLength == 0) { // call callback with available chunks
						uint32_t dataPopped = uart->BufferPopMax(popBuffer, uart->CALLBACK_THREAD_POP_BUFFER_SIZE);
						if (dataPopped) {
							uart->_RXcallback(uart->_RXcallbackParameter, popBuffer, dataPopped);
						}
					}
					else if (uart->_callbackChunkLength == 1) { // call callback with only 1 byte
						uint8_t byte = uart->BufferPop();
						uart->_RXcallback(uart->_RXcallbackParameter, &byte, 1);
					}
					else if (uart->BufferContentSize() >= uart->_callbackChunkLength) { // call callback with given chunk size, only when available
						uint8_t * chunkBuffer = uart->BufferPopN(uart->_callbackChunkLength);
						if (chunkBuffer) {
							uart->_RXcallback(uart->_RXcallbackParameter, chunkBuffer, uart->_callbackChunkLength);
							vPortFree(chunkBuffer);
						}
					}
					else{
						break; // exit while and sleep thread until next receive event
					}
				}
			}
		}
		else {
			break; // exit while loop to stop current task, since the RX callback is no longer registered
		}
	}

	vTaskDelete(NULL); // delete/stop this current task
}
#endif

void UART::UART_IncomingDataInterrupt(UART * uart)
{
#ifdef USE_FREERTOS
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#endif

	// Only call this function is RXNE interrupt flag is set
	if (!uart) {
		ERROR("UART interrupt for unconfigured port");
		return;
	}

	uart->rxByte = LL_USART_ReceiveData8(uart->_instance);
	/* Clear RXNE interrupt flag */
	//__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // should already have been cleared by reading

	uart->BufferPush(uart->rxByte); // push into local buffer

#ifdef USE_FREERTOS
	if (uart->_RXcallback && uart->_callbackTaskHandle)
		xTaskResumeFromISR(uart->_callbackTaskHandle);

  if (uart->_RXdataAvailable)
	  xSemaphoreGiveFromISR( uart->_RXdataAvailable, &xHigherPriorityTaskWoken );

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
	if (uart->_RXcallback) {
		if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
			uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
		}
	}
#endif
}

#if 0
uint32_t UART::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
	return xSemaphoreTake( _RXdataAvailable, ( TickType_t ) xTicksToWait );
}
#endif

void USART1_IRQHandler(void)
{
	UART::UART_Interrupt(UART::PORT_UART1);
}

void UART::UART_Interrupt(port_t port)
{
	UART * uart = 0;
	switch (port) {
		case PORT_UART1:
			uart = objUART1;
			break;
		default:
			break;
	}

	if (!uart) {
		ERROR("UART interrupt for unconfigured port");
		return;
	}

	uint32_t isrflags   = READ_REG(uart->_instance->ISR);
	uint32_t cr1its     = READ_REG(uart->_instance->CR1);
	uint32_t cr3its;
	uint32_t errorflags;

	/* If no error occurs */
	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if (errorflags == RESET)
	{
		/* UART in mode Receiver ---------------------------------------------------*/
		if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
		{
			UART_IncomingDataInterrupt(uart);
			return;
		}
	}

	/* If some errors occur */
	cr3its = READ_REG(uart->_instance->CR3);
	if(   (errorflags != RESET)
		&& (   ((cr3its & USART_CR3_EIE) != RESET)
		     || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)) )
	{
		/* UART parity error interrupt occurred -------------------------------------*/
		if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
		{
			//__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_PEF);
			LL_USART_ClearFlag_PE(uart->_instance);
			uart->_ErrorCode |= UART_ERROR_PE;
		}

		/* UART frame error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
			//__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_FEF);
			LL_USART_ClearFlag_FE(uart->_instance);
			uart->_ErrorCode |= UART_ERROR_FE;
		}

		/* UART noise error interrupt occurred --------------------------------------*/
		if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
		{
			//__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_NEF);
			LL_USART_ClearFlag_NE(uart->_instance);
			uart->_ErrorCode |= UART_ERROR_NE;
		}

		/* UART Over-Run interrupt occurred -----------------------------------------*/
	    if(((isrflags & USART_ISR_ORE) != RESET) &&
	       (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
		{
			//__HAL_UART_CLEAR_IT(&uart->_handle, UART_CLEAR_OREF);
			LL_USART_ClearFlag_ORE(uart->_instance);
			uart->_ErrorCode |= UART_ERROR_ORE;
		}

		/* Call UART Error Call back function if need be --------------------------*/
		if(uart->_ErrorCode != UART_ERROR_NONE)
		{
			/* UART in mode Receiver ---------------------------------------------------*/
			if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
			{
				UART_IncomingDataInterrupt(uart);
			}

			/* If Overrun error occurs, or if any error occurs in DMA mode reception,
			consider error as blocking */
			if (((uart->_ErrorCode & UART_ERROR_ORE) != RESET) ||
			(HAL_IS_BIT_SET(uart->_instance->CR3, USART_CR3_DMAR)))
			{
				/* Blocking error : transfer is aborted
				Set the UART state ready to be able to start again the process,
				Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
				//UART_EndRxTransfer(huart);

				/* Disable the UART DMA Rx request if enabled */
				if (HAL_IS_BIT_SET(uart->_instance->CR3, USART_CR3_DMAR))
				{
					CLEAR_BIT(uart->_instance->CR3, USART_CR3_DMAR);

					/* Abort the UART DMA Rx channel */
					// ToDo: Implement DMA abort (if DMA ends up being used)
				}
				else
				{
					/* Call user error callback */
					//HAL_UART_ErrorCallback(huart);
				}
			}
			else
			{
				/* Non Blocking error : transfer could go on.
				Error is notified to user through user error callback */
				//HAL_UART_ErrorCallback(huart);
				uart->_ErrorCode = UART_ERROR_NONE;
			}
		}
		return;

	} /* End if some error occurs */

	/* UART wakeup from Stop mode interrupt occurred ---------------------------*/
	if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
	{
		LL_USART_ClearFlag_WKUP(uart->_instance);
		return;
	}

	/* UART in mode Transmitter ------------------------------------------------*/
	if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
	{
		if (uart->_txRemainingBytes > 0) { // interrupt based transmission
			if (uart->_txRemainingBytes == 1) {
			    /* Disable TXE interrupt */
			    LL_USART_DisableIT_TXE(uart->_instance);

			    /* Enable TC interrupt */
			    LL_USART_EnableIT_TC(uart->_instance);
			}

			// Transmit the next byte from the buffer
			LL_USART_TransmitData8(uart->_instance, *uart->_txPointer++);
			uart->_txRemainingBytes--;
		} else { // polling based transmission
			/* Disable the Transmit Data Register Empty interrupt */
			LL_USART_DisableIT_TXE(uart->_instance);

	#ifdef USE_FREERTOS
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR( uart->_transmitByteFinished, &xHigherPriorityTaskWoken );
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	#else
			uart->_transmitByteFinished = true;
	#endif
		}
		return;
	}

	/* UART in mode Transmitter (transmission end) -----------------------------*/
	if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		LL_USART_ClearFlag_TC(uart->_instance);
		LL_USART_DisableIT_TC(uart->_instance);

#ifdef USE_FREERTOS
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( uart->_transmitByteFinished, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
		uart->_transmitByteFinished = true;
#endif
		return;
	}
}

void UART::BufferPush(uint8_t byte)
{
	if (!_bufferLength) return; // error, buffer not enabled
	if (BufferContentSize() >= _bufferLength) return; // error, buffer is full

	_buffer[_bufferWriteIdx] = byte;
	_bufferWriteIdx++;
	if (_bufferWriteIdx == _bufferLength) _bufferWriteIdx = 0;
}

uint8_t UART::BufferPop()
{
	if (!_bufferLength) return 0; // error, buffer not enabled
	if (BufferContentSize() == 0) return 0; // error, buffer is empty

	uint8_t byte = _buffer[_bufferReadIdx];
	_bufferReadIdx++;
	if (_bufferReadIdx == _bufferLength) _bufferReadIdx = 0;

	return byte;
}

uint32_t UART::BufferContentSize()
{
	if (!_bufferLength) return 0; // error, buffer not enabled
	uint32_t length = (_bufferWriteIdx - _bufferReadIdx) % _bufferLength;

	return length;
}

// Pop as many bytes as possible
uint32_t UART::BufferPopMax(uint8_t * buffer, uint32_t bufferSize)
{
	uint32_t poppedBytes = 0;
	if (!buffer) return 0;

	while (Available() && poppedBytes < bufferSize) {
		buffer[poppedBytes] = BufferPop();
		poppedBytes++;
	}

	return poppedBytes;
}

uint8_t * UART::BufferPopN(uint32_t numberOfBytesToPop)
{
	uint8_t * popBuffer = 0;
	if (numberOfBytesToPop > BufferContentSize()) return 0; // error, not enough content in buffer
#ifdef USE_FREERTOS
	popBuffer = (uint8_t *)pvPortMalloc(numberOfBytesToPop);
#else
	popBuffer = (uint8_t *)malloc(numberOfBytesToPop);
#endif

	if (popBuffer) {
		for (uint32_t i = 0; i < numberOfBytesToPop; i++) {
			popBuffer[i] = BufferPop();
		}
	}

	return popBuffer;
}
