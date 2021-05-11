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
 
#pragma once

// FreeRTOS include for processing task
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#else
#error "CPU Load should be used with FreeRTOS"
#endif

#include <LSPC/LSPC.hpp>

#include <map>

class CPULoad
{
	private:
		const uint32_t CPULOAD_THREAD_STACK = 256;

	public:
		CPULoad(LSPC& lspc, uint32_t cpuLoadTaskPriority);
		~CPULoad();

    private:
        void TaskRunTimeStats(char * pcWriteBuffer);
        void MemoryStats(char * pcWriteBuffer);

	private:
		TaskHandle_t cpuLoadTaskHandle_;

		LSPC& lspc_;

		std::map<TaskHandle_t, uint32_t> prevTaskRunTime;
        uint32_t prevTotalRunTime{0};

	private:
		static void CPULoadThread(void * pvParameters);
        static char * WriteTaskNameToBuffer(char * pcBuffer, const char * pcTaskName);
		
};