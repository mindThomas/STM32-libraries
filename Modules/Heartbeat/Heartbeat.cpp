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

#include "Heartbeat.hpp"
#include "cmsis_os.h" // for processing task

#include <Debug/Debug.h>

Heartbeat::Heartbeat(uint16_t heartbeatFrequency, LSPC& lspc, uint32_t heartbeatTaskPriority)
    : _heartbeatFrequency(heartbeatFrequency)
    , _heartbeatTaskHandle(0)
    , _lspc(lspc)
{
    xTaskCreate(Heartbeat::HeartbeatThread, (char*)"Heartbeat", HEARTBEAT_THREAD_STACK, (void*)this,
                heartbeatTaskPriority, &_heartbeatTaskHandle);
}

Heartbeat::~Heartbeat()
{
    if (_heartbeatTaskHandle)
        vTaskDelete(_heartbeatTaskHandle); // stop task
}

void Heartbeat::HeartbeatThread(void* pvParameters)
{
    Heartbeat*                          obj  = (Heartbeat*)pvParameters;
    LSPC&                               lspc = obj->_lspc;
    lspc::MessageTypesToPC::Heartbeat_t msg;

    /* Controller loop time / sample rate */
    TickType_t loopWaitTicks = configTICK_RATE_HZ / obj->_heartbeatFrequency;

    /* Send heartbeat at configured frequency */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, loopWaitTicks);

        msg.timestamp_ms               = (xTaskGetTickCount() * 1000) / configTICK_RATE_HZ;
        msg.expected_next_timestamp_ms = ((msg.timestamp_ms + loopWaitTicks) * 1000) / configTICK_RATE_HZ;

        lspc.TransmitAsync(lspc::MessageTypesToPC::Heartbeat, (uint8_t*)&msg, sizeof(msg));
    }
}
