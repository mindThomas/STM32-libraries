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
 
#ifndef MISC_CIRCULARBUFFER_H
#define MISC_CIRCULARBUFFER_H

#include "Debug.h" // for error messages
#include <string.h> // for memset

#ifdef USE_FREERTOS
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback
#else
#include <malloc.h>
#endif

template<typename T>
class CircularBuffer
{
	public:
		CircularBuffer(uint32_t size, bool allowOverrun = false) : _buffer(0), _bufferSize(size), _bufferWriteIdx(0), _bufferReadIdx(0), _allowOverrun(allowOverrun), _overrunDetected(false)
		{
			if (!size)
				return;

			#ifdef USE_FREERTOS
			_buffer = (T *)pvPortMalloc(size * sizeof(T));
			#else
			_buffer = (T *)malloc(size * sizeof(T));
			#endif

			if (!_buffer)
				return;

			memset(_buffer, 0, size * sizeof(T));

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

		~CircularBuffer()
		{
			if (_buffer) {
				#ifdef USE_FREERTOS
				vPortFree(_buffer);
				#else
				free(_buffer);
				#endif
			}

			#ifdef USE_FREERTOS
			vQueueUnregisterQueue(_bufferSemaphore);
			vSemaphoreDelete(_bufferSemaphore);
			#endif
		}

		void Push(T packet)
		{
			if (!_buffer) return; // error, buffer memory not available
			if (!_allowOverrun && AvailablePackets() >= _bufferSize) return; // error, buffer is full

			#ifdef USE_FREERTOS
			xSemaphoreTake( _bufferSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
			#endif

			_buffer[_bufferWriteIdx] = packet;
			/*_bufferWriteIdx++;
			if (_bufferWriteIdx == _bufferSize) _bufferWriteIdx = 0;*/
			_bufferWriteIdx = (_bufferWriteIdx + 1) % _bufferSize;
			if (!_allowOverrun && _bufferWriteIdx == _bufferReadIdx) {
				if (_overrunDetected) { // overrun detected in previous write, so now we need to discard an old sample by increase the reading pointer
					_bufferReadIdx = (_bufferReadIdx + 1) % _bufferSize;
				} else {
					_overrunDetected = true;
				}
			}

			#ifdef USE_FREERTOS
			xSemaphoreGive( _bufferSemaphore ); // give hardware resource back
			#endif
		}

		void PushFromInterrupt(T packet)
		{
			if (!_buffer) return; // error, buffer memory not available
			if (AvailablePackets() >= _bufferSize) return; // error, buffer is full

			_buffer[_bufferWriteIdx] = packet;
			/*_bufferWriteIdx++;
			if (_bufferWriteIdx == _bufferSize) _bufferWriteIdx = 0;*/
			_bufferWriteIdx = (_bufferWriteIdx + 1) % _bufferSize;
			if (_bufferWriteIdx == _bufferReadIdx) {
				if (_overrunDetected) { // overrun detected in previous write, so now we need to discard an old sample by increase the reading pointer
					_bufferReadIdx = (_bufferReadIdx + 1) % _bufferSize;
				} else {
					_overrunDetected = true;
				}
			}
		}

		T Pop()
		{
			if (AvailablePackets() == 0) return T(); // error, buffer is empty

			#ifdef USE_FREERTOS
			xSemaphoreTake( _bufferSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
			#endif

			T packet = _buffer[_bufferReadIdx];
			/*_bufferReadIdx++;
			if (_bufferReadIdx == _bufferSize) _bufferReadIdx = 0;*/
			_bufferReadIdx = (_bufferReadIdx + 1) % _bufferSize;
			_overrunDetected = false;

			#ifdef USE_FREERTOS
			xSemaphoreGive( _bufferSemaphore ); // give hardware resource back
			#endif

			return packet;
		}

		// Pop as many bytes as possible
		uint32_t PopAll(T * buffer, uint32_t bufferSize)
		{
			uint32_t poppedPackets = 0;
			if (!buffer) return 0;

			while (Available() && poppedPackets < bufferSize) {
				buffer[poppedPackets] = Pop();
				poppedPackets++;
			}

			return poppedPackets;
		}

		T * PopN(uint32_t numberOfPacketsToPop)
		{
			T * popBuffer = 0;
			if (numberOfPacketsToPop > AvailablePackets()) return 0; // error, not enough content in buffer
		#ifdef USE_FREERTOS
			popBuffer = (T *)pvPortMalloc(numberOfPacketsToPop * sizeof(T));
		#else
			popBuffer = (T *)malloc(numberOfPacketsToPop * sizeof(T));
		#endif

			if (popBuffer) {
				for (uint32_t i = 0; i < numberOfPacketsToPop; i++) {
					popBuffer[i] = Pop();
				}
			}

			return popBuffer;
		}

		uint32_t AvailablePackets()
		{
			if (!_buffer) return 0;
			uint32_t length = (_bufferWriteIdx - _bufferReadIdx) % _bufferSize;

			return length;
		}

		bool Available()
		{
			return (_bufferWriteIdx != _bufferReadIdx);
		}
		
		uint32_t FreeSpace()
		{
			if (!_buffer) return 0;
			return (_bufferSize - AvailablePackets());
		}

		// Only truly useful when overrun is allowed
		T Front()
		{
			// Return the most recently pushed element
			if (_bufferWriteIdx == 0)
				return _buffer[_bufferSize-1];
			else
				return _buffer[_bufferWriteIdx-1];
		}

		T Back()
		{
			// Return the oldest pushed element
			return _buffer[_bufferWriteIdx];
		}


	private:
		const uint32_t _bufferSize;
		T * _buffer;
		uint32_t _bufferWriteIdx;
		uint32_t _bufferReadIdx;
		bool _allowOverrun;
		bool _overrunDetected;
		
	#ifdef USE_FREERTOS
		SemaphoreHandle_t _bufferSemaphore;
	#endif
};
	
	
#endif
