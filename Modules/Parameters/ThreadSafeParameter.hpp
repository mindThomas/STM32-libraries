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
 
#ifndef MODULES_THREADSAFEPARAMETER_H
#define MODULES_THREADSAFEPARAMETER_H

#include "cmsis_os.h" // for semaphore

template <class T>
class ThreadSafeParameter
{	
	public:
		ThreadSafeParameter()
		{
			_semaphore = xSemaphoreCreateBinary();
			if (_semaphore)
				xSemaphoreGive( _semaphore ); // give the semaphore the first time, to ensure that it can be accessed
			_value = 0;
		};
		ThreadSafeParameter(T init)
		{
			_semaphore = xSemaphoreCreateBinary();
			if (_semaphore)
				xSemaphoreGive( _semaphore ); // give the semaphore the first time, to ensure that it can be accessed
			_value = init;
		};
		//ThreadSafeParameter(ThreadSafeParameter& init) {_value = init;}; // copy constructor
		~ThreadSafeParameter()
		{
			if (_semaphore)
				vSemaphoreDelete(_semaphore);
		};
	
		// Get Overloading using cast operator
		// a = param;
		operator T()
		{
			Lock();
			T tmp = _value;
			Unlock();
			return tmp;
		};
		
		// Set Overloading of = operator
		// param = a;
		ThreadSafeParameter& operator= (const T& value)
		{
			/*// self-assignment guard
			if (this == &fraction)
				return *this;*/

			Lock();
			_value = value;
			Unlock();

			// return the existing object so we can chain this operator
			return *this;
		};

		// prefix ++
		ThreadSafeParameter& operator++()
	    {
			Lock();
			_value++;
			Unlock();
			return *this;
	    };

		// postfix ++
	    // You want to make the ++ operator work like the standard operators
	    // The simple way to do this is to implement postfix in terms of prefix.
	    //
	    ThreadSafeParameter operator++(int)
	    {
			Lock();
			T old = _value;
			_value++;                // Now use the prefix version (same as function above) to do the work
			Unlock();

			ThreadSafeParameter result(old);   // make a copy for result
			return result;          // return the copy (the old) value.
	    };

		// prefix --
		ThreadSafeParameter& operator--()
	    {
			Lock();
			_value--;
			Unlock();
			return *this;
	    };

		// postfix --
	    // You want to make the -- operator work like the standard operators
	    // The simple way to do this is to implement postfix in terms of prefix.
	    //
	    ThreadSafeParameter operator--(int)
	    {
			Lock();
			T old = _value;
			_value--;                // Now use the prefix version (same as function above) to do the work
			Unlock();

			ThreadSafeParameter result(old);   // make a copy for result
			return result;          // return the copy (the old) value.
	    };

	    ThreadSafeParameter& operator+= (const T& value)
		{
			Lock();
			_value += value;
			Unlock();
			return *this;
		};

	    ThreadSafeParameter& operator-= (const T& value)
		{
			Lock();
			_value -= value;
			Unlock();
			return *this;
		};

	    ThreadSafeParameter& operator*= (const T& value)
		{
			Lock();
			_value *= value;
			Unlock();
			return *this;
		}

	    ThreadSafeParameter& operator/= (const T& value)
		{
			Lock();
			_value /= value;
			Unlock();
			return *this;
		};

	private:
		T _value;
		SemaphoreHandle_t _semaphore;

		void Lock()
		{
			if (_semaphore)
				xSemaphoreTake( _semaphore, ( TickType_t ) portMAX_DELAY );
		};

		void Unlock()
		{
			if (_semaphore)
				xSemaphoreGive( _semaphore );
		};
};
	
#endif
