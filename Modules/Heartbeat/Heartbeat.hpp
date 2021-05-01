/* Copyright (C) 2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
 
#ifndef MODULES_HEARTBEAT_H
#define MODULES_HEARTBEAT_H

#include "cmsis_os.h" // for processing task

#include "LSPC.hpp"

class Heartbeat
{
	private:
		const uint32_t HEARTBEAT_THREAD_STACK = 256;

	public:
		Heartbeat(uint16_t heartbeatFrequency, LSPC& lspc, uint32_t heartbeatTaskPriority);
		~Heartbeat();

	private:
        uint16_t _heartbeatFrequency;
		TaskHandle_t _heartbeatTaskHandle;

		LSPC& _lspc;

	private:
		static void HeartbeatThread(void * pvParameters);
		
};
	
	
#endif
