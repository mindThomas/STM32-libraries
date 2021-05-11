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

#include <Debug/Debug.h>

#include <IO/IO.hpp>

#ifdef USE_FREERTOS_CMSIS
#define delay(x) osDelay(x)
#elif defined(USE_FREERTOS)
#define delay(x) vTaskDelay(x)
#else
void HAL_Delay(uint32_t Delay); // forward declaration
#define delay(x) HAL_Delay(x)
#endif

#ifdef DEBUG_USE_UART
#include <UART/UART.hpp>
#elif defined(DEBUG_USE_LSPC)
#include <LSPC/LSPC.hpp>
#endif

bool  Debug::handleCreated = false;
Debug Debug::debugHandle;

// Necessary to export for compiler such that the Error_Handler function can be called by C code
extern "C" void Error_Handler(void);
extern "C" void Debug_print(const char* msg);
extern "C" void Debug_Pulse();

Debug::Debug()
    : com_(0)
    , debugPulsePin_(0)
{
    if (handleCreated) {
        ERROR("Debug object already created");
        return;
    }

    handleCreated = true;
}

Debug::~Debug() {}

#ifdef DEBUG_PRINT_ENABLED
void Debug::AssignDebugCOM(void* com)
{
    debugHandle.com_ = com;
    if (!com) {
        ERROR("COM object does not exist");
        return;
    }
    debugHandle.currentBufferLocation_ = 0;
    memset(debugHandle.messageBuffer_, 0, MAX_DEBUG_TEXT_LENGTH);
#ifdef USE_FREERTOS
    debugHandle.mutex_ = xSemaphoreCreateBinary();
    if (debugHandle.mutex_ == NULL) {
        ERROR("Could not create Debug mutex");
        return;
    }
    vQueueAddToRegistry(debugHandle.mutex_, "Debug mutex");
    xSemaphoreGive(debugHandle.mutex_); // give the semaphore the first time

#ifdef DEBUG_USE_LSPC
    xTaskCreate(Debug::PackageGeneratorThread, (char*)"Debug transmitter", debugHandle.THREAD_STACK_SIZE,
                (void*)&debugHandle, debugHandle.THREAD_PRIORITY, &debugHandle._TaskHandle);
#endif
#endif
}
#endif

#ifdef DEBUG_PRINT_ENABLED
#ifdef USE_FREERTOS
#ifdef DEBUG_USE_LSPC
void Debug::PackageGeneratorThread(void* pvParameters)
{
    Debug* debug = (Debug*)pvParameters;

    while (1) {
        delay(1);
        xSemaphoreTake(debug->mutex_, (TickType_t)portMAX_DELAY); // take debug mutex
        if (debug->currentBufferLocation_ > 0) {
            ((LSPC*)debug->com_)
              ->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t*)debug->messageBuffer_,
                              debug->currentBufferLocation_);
            debug->currentBufferLocation_ = 0;
        }
        xSemaphoreGive(debug->mutex_); // give hardware resource back
    }
}
#endif
#endif
#endif

void Debug::Message(const char* msg)
{
#ifdef DEBUG_PRINT_ENABLED
    if (!debugHandle.com_)
        return;
#ifndef DEBUG_USE_LSPC
#ifdef USE_FREERTOS
    xSemaphoreTake(debugHandle.mutex_, (TickType_t)portMAX_DELAY); // take debug mutex
#endif

    uint8_t* msgPtr       = (uint8_t*)msg;
    uint16_t stringLength = strlen(msg);
    if (!((UART*)debugHandle.com_)->Write(msgPtr, stringLength))
        return;

#ifdef USE_FREERTOS
    xSemaphoreGive(debugHandle.mutex_); // give hardware resource back
#endif
#else
    if (!((LSPC*)debugHandle.com_)->Connected())
        return;

#ifdef USE_FREERTOS
    xSemaphoreTake(debugHandle.mutex_, (TickType_t)portMAX_DELAY); // take debug mutex
#endif

    uint16_t stringLength = strlen(msg);
    if (stringLength > MAX_DEBUG_TEXT_LENGTH) { // message is too long to fit in one package
        // Send current buffered package now and clear buffer
        ((LSPC*)debugHandle.com_)
          ->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t*)debugHandle.messageBuffer_,
                          debugHandle.currentBufferLocation_);
        debugHandle.currentBufferLocation_ = 0;

        uint8_t* msgPtr = (uint8_t*)msg;
        while (stringLength > 0) { // split the message up in seperate packages
            uint16_t sendLength = stringLength;
            if (sendLength > MAX_DEBUG_TEXT_LENGTH)
                sendLength = MAX_DEBUG_TEXT_LENGTH;
            ((LSPC*)debugHandle.com_)->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t*)msgPtr, sendLength);
            msgPtr += sendLength;
            stringLength -= sendLength;
        }
    } else { // package can fit in one package
        if (stringLength >
            (MAX_DEBUG_TEXT_LENGTH -
             debugHandle
               .currentBufferLocation_)) { // stringLength = (MAX_DEBUG_TEXT_LENGTH-debugHandle.currentBufferLocation_);
                                           // // "cut away" any parts above the maximum string length
            // Send package now and clear buffer
            ((LSPC*)debugHandle.com_)
              ->TransmitAsync(lspc::MessageTypesToPC::Debug, (const uint8_t*)debugHandle.messageBuffer_,
                              debugHandle.currentBufferLocation_);
            debugHandle.currentBufferLocation_ = 0;
        }

        memcpy(&debugHandle.messageBuffer_[debugHandle.currentBufferLocation_], msg, stringLength);
        debugHandle.currentBufferLocation_ += stringLength;
    }
#ifdef USE_FREERTOS
    xSemaphoreGive(debugHandle.mutex_); // give hardware resource back
#endif
#endif
#endif
}

void Debug::Message(std::string msg)
{
    Message(msg.c_str());
    Message("\n");
}

void Debug::Message(const char* type, const char* msg)
{
    Message(type);
    Message(msg);
    Message("\n");
}

void Debug::Message(const char* type, std::string msg)
{
    Message(type);
    Message(msg.c_str());
    Message("\n");
}

void Debug::Message(const char* type, const char* functionName, const char* msg)
{
    Message(type);
    Message("[");
    Message(functionName);
    Message("] ");
    Message(msg);
    Message("\n");
}

void Debug::Message(std::string type, const char* functionName, std::string msg)
{
    Message(type.c_str());
    Message("[");
    Message(functionName);
    Message("] ");
    Message(msg.c_str());
    Message("\n");
}

void Debug::DebugMessage(const char * filePath, const char * lineNumber, const char * msg)
{
    Message("[");
    Message(filePath);
    Message(lineNumber);
    Message("] ");
    Message(msg);
    Message("\n");
}

void Debug::print(const char* msg)
{
    Message(msg);
}

#ifdef DEBUG_PRINTF_ENABLED
void Debug::printf(const char* msgFmt, ...)
{
    va_list args;

    if (!debugHandle.com_)
        return;
#ifdef DEBUG_USE_LSPC
    if (!((LSPC*)debugHandle.com_)->Connected())
        return;
#endif

    va_start(args, msgFmt);

    char* strBuf = (char*)pvPortMalloc(MAX_DEBUG_TEXT_LENGTH);
    if (!strBuf)
        return;

    vsnprintf(strBuf, MAX_DEBUG_TEXT_LENGTH, msgFmt, args);

    Message(strBuf);

    vPortFree(strBuf);

    va_end(args);
}
#endif


void Debug::Error(const char * filePath, const char * lineNumber, const char * msg)
{
    // At errors do not continue current task/thread but print instead the error message repeatedly
    __asm__("BKPT");
    while (1) {
        Debug::DebugMessage(filePath, lineNumber, msg);
        delay(500);
    }
}

void Debug::Error(const char* type, const char* msg)
{
    // At errors do not continue current task/thread but print instead the error message repeatedly
    __asm__("BKPT");
    while (1) {
        Debug::Message(type, msg);
        delay(500);
    }
}

void Debug::SetDebugPin(void* pin)
{
    if (debugHandle.debugPulsePin_)
        return; // pin already set
    debugHandle.debugPulsePin_ = pin;
}

void Debug::Pulse()
{
    if (!debugHandle.debugPulsePin_)
        return;
    ((IO*)debugHandle.debugPulsePin_)->High();
    delay(50);
    ((IO*)debugHandle.debugPulsePin_)->Low();
}

void Debug::Toggle()
{
    if (!debugHandle.debugPulsePin_)
        return;
    ((IO*)debugHandle.debugPulsePin_)->Toggle();
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    Debug::Error("ERROR: ", "Error_Handler", "Global ");
}

void Debug_print(const char* msg)
{
    Debug::print(msg);
}

void Debug_Pulse()
{
    Debug::Pulse();
}
