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
 
#include <CircularBuffer.hpp>
#include <string.h> // for memset

#ifndef USE_FREERTOS
#include <malloc.h>
#endif
 
template<typename T>
CircularBuffer<T>::CircularBuffer(uint32_t size) : _bufferSize(size)
{
	#ifdef USE_FREERTOS
	_buffer = (T *)pvPortMalloc(size * sizeof(T));
	#else
	_buffer = (T *)malloc(size * sizeof(T));
	#endif
	
	if (!_buffer) {
		_bufferSize = 0; // error, memory not available
		return;
	}
	
	#ifdef USE_FREERTOS
	_bufferSemaphore = xSemaphoreCreateBinary();
	if (_bufferSemaphore == NULL) {
		ERROR("Could not create Circular buffer semaphore");
		return;
	}
	vQueueAddToRegistry(_bufferSemaphore, "CircularBuffer semaphore");
	xSemaphoreGive( _bufferSemaphore ); // give the semaphore the first time
	#endif
}

template<typename T>
CircularBuffer<T>::~CircularBuffer()
{
	if (_buffer) {
		#ifdef USE_FREERTOS
		vPortFree(_buffer);
		#else
		free(_buffer);
		#endif
	}
}

template<typename T>
void CircularBuffer<T>::Push(T packet)
{
	if (!_buffer) return; // error, buffer memory not available
	if (AvailablePackets() >= _bufferSize) return; // error, buffer is full

	#ifdef USE_FREERTOS
	xSemaphoreTake( _bufferSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
	#endif
	
	_buffer[_bufferWriteIdx] = packet;
	_bufferWriteIdx++;
	if (_bufferWriteIdx == _bufferSize) _bufferWriteIdx = 0;
	
	#ifdef USE_FREERTOS
	xSemaphoreGive( _bufferSemaphore ); // give hardware resource back
	#endif
}

template<typename T>
void CircularBuffer<T>::PushFromInterrupt(T packet)
{
	if (!_buffer) return; // error, buffer memory not available
	if (AvailablePackets() >= _bufferSize) return; // error, buffer is full

	#ifdef USE_FREERTOS
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreTakeFromISR( _bufferSemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	#endif

	_buffer[_bufferWriteIdx] = packet;
	_bufferWriteIdx++;
	if (_bufferWriteIdx == _bufferSize) _bufferWriteIdx = 0;

	#ifdef USE_FREERTOS
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( _bufferSemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	#endif
}

template<typename T>
T CircularBuffer<T>::Pop()
{	
	if (AvailablePackets() == 0) return 0; // error, buffer is empty

	#ifdef USE_FREERTOS
	xSemaphoreTake( _bufferSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
	#endif	
	
	T packet = _buffer[_bufferReadIdx];
	_bufferReadIdx++;
	if (_bufferReadIdx == _bufferSize) _bufferReadIdx = 0;

	#ifdef USE_FREERTOS
	xSemaphoreGive( _bufferSemaphore ); // give hardware resource back
	#endif
	
	return packet;
}


// Pop as many bytes as possible
template<typename T>
uint32_t CircularBuffer<T>::PopAll(T * buffer, uint32_t bufferSize)
{
	uint32_t poppedBytes = 0;
	if (!buffer) return 0;

	while (Available() && poppedBytes < bufferSize) {
		buffer[poppedBytes] = Pop();
		poppedBytes++;
	}

	return poppedBytes;
}

template<typename T>
T * CircularBuffer<T>::PopN(uint32_t numberOfBytesToPop)
{
	T * popBuffer = 0;
	if (numberOfBytesToPop > AvailablePackets()) return 0; // error, not enough content in buffer
#ifdef USE_FREERTOS
	popBuffer = (T *)pvPortMalloc(numberOfBytesToPop * sizeof(T));
#else
	popBuffer = (T *)malloc(numberOfBytesToPop * sizeof(T));
#endif

	if (popBuffer) {
		for (uint32_t i = 0; i < numberOfBytesToPop; i++) {
			popBuffer[i] = Pop();
		}
	}

	return popBuffer;
}

template<typename T>
uint32_t CircularBuffer<T>::AvailablePackets()
{	
	if (!_buffer) return 0;
	uint32_t length = (_bufferWriteIdx - _bufferReadIdx) % _bufferSize;

	return length;
}

template<typename T>
bool CircularBuffer<T>::Available()
{
	return (_bufferWriteIdx != _bufferReadIdx);
}
