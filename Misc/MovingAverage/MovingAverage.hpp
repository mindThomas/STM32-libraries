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
 
#ifndef MISC_MOVINGAVERAGE_H
#define MISC_MOVINGAVERAGE_H

#include "stm32h7xx_hal.h" // for uint16_t etc
#include "Debug.h"
#include "cmsis_os.h" // for memory allocation (for the buffer)
#include "string.h" // for memset

class MovingAverage
{
		public:
		MovingAverage(const uint16_t n)
		{
			_n = n;
			_yPrev = 0;
			_inputBufferIdx = 0;

			/* Construct a simple circular buffer for storing past inputs */
			_inputBuffer = (float *)pvPortMalloc(n*sizeof(float));
			memset((uint8_t *)_inputBuffer, 0, n*sizeof(float)); // clear buffer
		}
		
		~MovingAverage()
		{
			if (_inputBuffer) {			
				vPortFree(_inputBuffer); // clear buffer memory
			}
		}

		float Filter(float input)
		{
			if (!_inputBuffer) return 0; // buffer error
			
			// Moving average filter defined as:
			// y[k] = 1/n * (u[k] + u[k-1] + u[k-2] + ... + u[k-(n-1)]
			// Which can also be computed iteratively
			// y[k] = y[k-1] + 1/n * (u[k] - u[k-n])
			float input_k_n = _inputBuffer[_inputBufferIdx]; // get n-step old input sample
			float y = _yPrev + (1.0f / _n) * (input - input_k_n);
			
			// Store input into old input circular buffer
			_inputBuffer[_inputBufferIdx] = input;
			_inputBufferIdx++;
			if (_inputBufferIdx == _n) _inputBufferIdx = 0;
			
			// Store previous output
			_yPrev = y;
			
			return y;
		}

		private:
			uint16_t _n; // moving horizon length
			float * _inputBuffer; // store previous inputs: u[k-1], ..., u[k-n]
			uint16_t _inputBufferIdx;
			float _yPrev;
};
	
	
#endif
