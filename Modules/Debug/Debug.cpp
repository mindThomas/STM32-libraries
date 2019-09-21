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
 
#include "Debug.h"
#include "cmsis_os.h"
#include "LSPC.hpp"
#include "IO.h"

Debug * Debug::debugHandle = 0;

// Necessary to export for compiler such that the Error_Handler function can be called by C code
extern "C" __EXPORT void Error_Handler(void);
extern "C" __EXPORT void Debug_print(const char * msg);
extern "C" __EXPORT void Debug_Pulse();

Debug::Debug(void * com) : com_(com)
{
	if (debugHandle) {
		ERROR("Debug object already created");
		return;
	}

	if (!com) {
		ERROR("LSPC object does not exist");
		return;
	}

	mutex_ = xSemaphoreCreateBinary();
	if (mutex_ == NULL) {
		ERROR("Could not create Debug mutex");
		return;
	}
	vQueueAddToRegistry(mutex_, "Debug mutex");
	xSemaphoreGive( mutex_ ); // give the semaphore the first time

	debugPulsePin_ = new IO(GPIOE, GPIO_PIN_6);
	((IO*)debugPulsePin_)->Set(false);

	currentBufferLocation_ = 0;
	memset(messageBuffer_, 0, MAX_DEBUG_TEXT_LENGTH);
	xTaskCreate( Debug::PackageGeneratorThread, (char *)"Debug transmitter", THREAD_STACK_SIZE, (void*) this, THREAD_PRIORITY, &_TaskHandle);

	debugHandle = this;
}

Debug::~Debug()
{
	debugHandle = 0;
}


void Debug::PackageGeneratorThread(void * pvParameters)
{
	Debug * debug = (Debug *)pvParameters;

	while (1)
	{
		osDelay(1);
		xSemaphoreTake( debug->mutex_, ( TickType_t ) portMAX_DELAY ); // take debug mutex
		if (debug->currentBufferLocation_ > 0) {
			((LSPC*)debug->com_)->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t *)debug->messageBuffer_, debug->currentBufferLocation_);
			debug->currentBufferLocation_ = 0;
		}
		xSemaphoreGive( debug->mutex_ ); // give hardware resource back
	}
}

void Debug::Message(const char * msg)
{
	if (!debugHandle) return;
	if (!debugHandle->com_) return;
	if (!((LSPC*)debugHandle->com_)->Connected()) return;

	xSemaphoreTake( debugHandle->mutex_, ( TickType_t ) portMAX_DELAY ); // take debug mutex

	uint16_t stringLength = strlen(msg);
	if (stringLength > MAX_DEBUG_TEXT_LENGTH) { // message is too long to fit in one package
		// Send current buffered package now and clear buffer
		((LSPC*)debugHandle->com_)->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t *)debugHandle->messageBuffer_, debugHandle->currentBufferLocation_);
		debugHandle->currentBufferLocation_ = 0;

		uint8_t * msgPtr = (uint8_t *)msg;
		while (stringLength > 0) { // split the message up in seperate packages
			uint16_t sendLength = stringLength;
			if (sendLength > MAX_DEBUG_TEXT_LENGTH) sendLength = MAX_DEBUG_TEXT_LENGTH;
			((LSPC*)debugHandle->com_)->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t *)msgPtr, sendLength);
			msgPtr += sendLength;
			stringLength -= sendLength;
		}
	} else { // package can fit in one package
		if (stringLength > (MAX_DEBUG_TEXT_LENGTH-debugHandle->currentBufferLocation_)) {// stringLength = (MAX_DEBUG_TEXT_LENGTH-debugHandle->currentBufferLocation_); // "cut away" any parts above the maximum string length
			// Send package now and clear buffer
			((LSPC*)debugHandle->com_)->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t *)debugHandle->messageBuffer_, debugHandle->currentBufferLocation_);
			debugHandle->currentBufferLocation_ = 0;
		}

		memcpy(&debugHandle->messageBuffer_[debugHandle->currentBufferLocation_], msg, stringLength);
		debugHandle->currentBufferLocation_ += stringLength;
	}
	xSemaphoreGive( debugHandle->mutex_ ); // give hardware resource back
}

void Debug::Message(std::string msg)
{
	Message(msg.c_str());
	Message("\n");
}

void Debug::Message(const char * functionName, const char * msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg);
	Message("\n");
}

void Debug::Message(const char * functionName, std::string msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg.c_str());
	Message("\n");
}

void Debug::Message(const char * type, const char * functionName, const char * msg)
{
	Message(type);
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg);
	Message("\n");
}

void Debug::Message(std::string type, const char * functionName, std::string msg)
{
	Message("[");
	Message(functionName);
	Message("] ");
	Message(msg.c_str());
	Message("\n");
}

void Debug::print(const char * msg)
{
	Message(msg);
}

void Debug::printf( const char *msgFmt, ... )
{
	va_list args;

	if (!debugHandle) return;
	if (!debugHandle->com_) return;
	if (!((LSPC*)debugHandle->com_)->Connected()) return;

	va_start( args,  msgFmt );

	char * strBuf = (char *) pvPortMalloc(MAX_DEBUG_TEXT_LENGTH);
	if (!strBuf) return;

	vsnprintf( strBuf, MAX_DEBUG_TEXT_LENGTH, msgFmt, args );

	Message(strBuf);

	vPortFree(strBuf);

	va_end( args );
}

void Debug::Error(const char * type, const char * functionName, const char * msg)
{
	// At errors do not continue current task/thread but print instead the error message repeatedly
	while (1)
	{
		Debug::Message(type, functionName, msg);
		osDelay(500);
	}
}

void Debug::Pulse()
{
	if (!debugHandle) return;
	if (!debugHandle->debugPulsePin_) return;

	((IO*)debugHandle->debugPulsePin_)->High();
	osDelay(50);
	((IO*)debugHandle->debugPulsePin_)->Low();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	Debug::Error("ERROR: ", "Error_Handler", "Global ");
}

void Debug_print(const char * msg)
{
	Debug::print(msg);
}

void Debug_Pulse()
{
	Debug::Pulse();
}
