/* Copyright (C) 2018-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h" // for semaphore
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

#if defined(DEBUG_USE_UART) || defined(DEBUG_USE_LSPC)
#define DEBUG_PRINT_ENABLED
#endif

#if defined(DEBUG_USE_PRINTF) && defined(DEBUG_PRINT_ENABLED)
#define DEBUG_PRINTF_ENABLED
#endif

#include <Priorities.h>

#define STRINGIFY_(x) #x  // turn the exact input argument string (code string) into a C string (const char *)
#define STRINGIFY(x) STRINGIFY_(x)  // turn the input variable content into a C string (const char *)
#define TOSTRING(x) std::string(STRINGIFY(x))
#define CONCAT_VARIABLES_(x, y) x##y
#define CONCAT_VARIABLES(x, y) CONCAT_VARIABLES_(x, y)

#define DEBUG_STRRCHR(str, sep) strrchr(str, sep)
#define FILE_BASENAME(file) DEBUG_STRRCHR("/" file, '/') + 1 // 1 indexed instead of 0 indexed

#ifdef DEBUG_USE_VERBOSE
#define DEBUG(msg) Debug::DebugMessage(FILE_BASENAME(__FILE__), ":" STRINGIFY(__LINE__), msg)
#define ERROR(msg) Debug::Error(FILE_BASENAME(__FILE__), ":" STRINGIFY(__LINE__), "ERROR: " msg)
#else
#define DEBUG(msg) Debug::Message("[Debug] ", __PRETTY_FUNCTION__, msg)
#define ERROR(msg) Debug::Error("[Error] ", __PRETTY_FUNCTION__, msg)
#endif

#define MAX_DEBUG_TEXT_LENGTH	210 // LSPC_MAXIMUM_PACKAGE_LENGTH

class Debug
{
	private:
		const int THREAD_STACK_SIZE = 64;
		const uint32_t THREAD_PRIORITY = DEBUG_MESSAGE_PRIORITY;

	public:
		Debug();
		~Debug();

#ifdef DEBUG_PRINT_ENABLED
		static void AssignDebugCOM(void * com);
#endif

        static void DebugMessage(const char * filePath, const char * lineNumber, const char * msg);

		static void Message(const char * type, const char * functionName, const char * msg);
		static void Message(std::string type, const char * functionName, std::string msg);
		static void Message(const char * type, const char * msg);
		static void Message(const char * type, std::string msg);
		static void Message(const char * msg);
		static void Message(std::string msg);
		static void print(const char * msg);

#ifdef DEBUG_PRINTF_ENABLED
		static void printf( const char *msgFmt, ... );
#endif
        static void Error(const char * filePath, const char * lineNumber, const char * msg);
        static void Error(const char * type, const char * msg);
		static void SetDebugPin(void * pin);
		static void Pulse();
		static void Toggle();

#ifdef USE_FREERTOS
	private:
		static void PackageGeneratorThread(void * pvParameters);
#endif

	private:
		void * com_{0}; // Interface object pointer
	#ifdef USE_FREERTOS
		SemaphoreHandle_t mutex_;
	#endif
		void * debugPulsePin_{0}; // of class IO

	#ifdef DEBUG_PRINT_ENABLED
    #ifdef DEBUG_USE_LSPC
		char messageBuffer_[MAX_DEBUG_TEXT_LENGTH];
		uint16_t currentBufferLocation_;

        SemaphoreHandle_t packagedDataToSend_;
        TaskHandle_t _TaskHandle;
    #endif
    #endif

};

#else  // for C usage

void Error_Handler(void);
void Debug_print(const char * msg);
void Debug_Pulse();

#endif
	
	
#endif
