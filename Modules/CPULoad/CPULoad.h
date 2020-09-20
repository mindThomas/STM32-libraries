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
 
#ifndef MODULES_CPULOAD_H
#define MODULES_CPULOAD_H

#include "cmsis_os.h" // for processing task

#include "LSPC.hpp"

class CPULoad
{
	private:
		const uint32_t CPULOAD_THREAD_STACK = 256;

	public:
		CPULoad(LSPC& lspc, uint32_t cpuLoadTaskPriority);
		~CPULoad();

	private:
		TaskHandle_t _cpuLoadTaskHandle;

		LSPC& _lspc;

	private:
		static void CPULoadThread(void * pvParameters);
		
};
	
	
#endif
