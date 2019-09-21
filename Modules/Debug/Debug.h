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
 
#ifndef MODULES_DEBUG_H
#define MODULES_DEBUG_H

#ifdef __cplusplus // for C++ usage

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <string>

#include "cmsis_os.h" // for semaphore
#include "Priorities.h"

#define DEBUG(msg)	Debug::Message("DEBUG: ", __PRETTY_FUNCTION__, msg)
#define ERROR(msg)	Debug::Error("ERROR: ", __PRETTY_FUNCTION__, msg)

#define MAX_DEBUG_TEXT_LENGTH	210 // LSPC_MAXIMUM_PACKAGE_LENGTH

class Debug
{
	private:
		const int THREAD_STACK_SIZE = 256; // notice that this much stack is apparently necessary to avoid issues
		const uint32_t THREAD_PRIORITY = DEBUG_MESSAGE_PRIORITY;

	public:
		Debug(void * com);
		~Debug();
	
		static void Message(const char * type, const char * functionName, const char * msg);
		static void Message(std::string type, const char * functionName, std::string msg);
		static void Message(const char * functionName, const char * msg);
		static void Message(const char * functionName, std::string msg);
		static void Message(const char * msg);
		static void Message(std::string msg);
		static void print(const char * msg);
		static void printf( const char *msgFmt, ... );
		static void Error(const char * type, const char * functionName, const char * msg);
		static void Pulse();

	private:
		static void PackageGeneratorThread(void * pvParameters);

	private:
		void * com_; // LSPC object pointer
		SemaphoreHandle_t mutex_;
		TaskHandle_t _TaskHandle;
		void * debugPulsePin_;

		char messageBuffer_[MAX_DEBUG_TEXT_LENGTH];
		uint16_t currentBufferLocation_;


	public:
		static Debug * debugHandle;

};

#else  // for C usage

void Error_Handler(void);
void Debug_print(const char * msg);
void Debug_Pulse();

#endif
	
	
#endif
