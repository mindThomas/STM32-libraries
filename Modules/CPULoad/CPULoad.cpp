/* Copyright (C) 2019-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "CPULoad.hpp"
#include "cmsis_os.h" // for processing task

#include <Debug/Debug.h>

CPULoad::CPULoad(LSPC& lspc, uint32_t cpuLoadTaskPriority)
    : _lspc(lspc)
    , _cpuLoadTaskHandle(0)
{
    xTaskCreate(CPULoad::CPULoadThread, (char*)"CPU Load", CPULOAD_THREAD_STACK, (void*)this, cpuLoadTaskPriority,
                &_cpuLoadTaskHandle);
}

CPULoad::~CPULoad()
{
    if (_cpuLoadTaskHandle)
        vTaskDelete(_cpuLoadTaskHandle); // stop task
}

void CPULoad::CPULoadThread(void* pvParameters)
{
    CPULoad* obj  = (CPULoad*)pvParameters;
    LSPC&    lspc = obj->_lspc;

    /* Send CPU load every second */
    char* pcWriteBuffer = (char*)pvPortMalloc(1024);
    while (1) {
        vTaskGetRunTimeStats(pcWriteBuffer);
        char* endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
        *endPtr++    = '\n';
        *endPtr++    = '\n';
        *endPtr++    = 0;

        // Split into multiple packages and send
        uint16_t txIdx           = 0;
        uint16_t remainingLength = strlen(pcWriteBuffer);
        uint16_t txLength;

        while (remainingLength > 0) {
            txLength = remainingLength;
            if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
                txLength = LSPC_MAXIMUM_PACKAGE_LENGTH - 1;
                while (pcWriteBuffer[txIdx + txLength] != '\n' && txLength > 0)
                    txLength--; // find and include line-break (if possible)
                if (txLength == 0)
                    txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
                else
                    txLength++;
            }
            lspc.TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t*)&pcWriteBuffer[txIdx], txLength);

            txIdx += txLength;
            remainingLength -= txLength;
        }
        osDelay(1000);
    }
}
