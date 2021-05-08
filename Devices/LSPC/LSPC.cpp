/* Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include "LSPC.hpp"

#include "Packet.hpp"
#include "Serializable.hpp"
#include "SocketBase.hpp"
#include <MessageTypes.h>

#include <string.h> // for memcpy

#ifdef STM32_LSPC_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) ((void)0U); // not implemented
#endif

// FreeRTOS for task creation
#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#endif

namespace lspc {

template<class COM>
Socket<COM>::Socket(COM* com, uint32_t processingTaskPriority, uint32_t transmitterTaskPriority)
    : com(com)
    , _processingTaskHandle(0)
    , _transmitterTaskHandle(0)
{
    _TXqueue = xQueueCreate(LSPC_ASYNCHRONOUS_QUEUE_LENGTH, sizeof(LSPC_Async_Package_t));
    if (_TXqueue == NULL) {
        ERROR("Could not create asynchronous LSPC TX queue");
        return;
    }
    vQueueAddToRegistry(_TXqueue, "LSPC TX");

    xTaskCreate(Socket::ProcessingThread, (char*)"LSPC processing", LSPC_RX_PROCESSING_THREAD_STACK_SIZE,
                (void*)this, processingTaskPriority, &_processingTaskHandle);
    xTaskCreate(Socket::TransmitterThread, (char*)"LSPC transmitter", LSPC_TX_TRANSMITTER_THREAD_STACK_SIZE,
                (void*)this, transmitterTaskPriority, &_transmitterTaskHandle);
}

template<class COM>
bool Socket<COM>::send(uint8_t type, const std::vector<uint8_t>& payload)
{
    Packet outPacket(type, payload);

    // Send it
    if (outPacket.encodedDataSize() == com->Write(outPacket.encodedDataPtr(), outPacket.encodedDataSize()))
        return true;
    else
        return false;
}

template<class COM>
void Socket<COM>::processSerial()
{
    int16_t readChar = 0;
    while (com->Available()) {
        readChar = com->Read();
        if (readChar >= 0)
            processIncomingByte(readChar);
    }
    return;
}

template<class COM>
bool Socket<COM>::TransmitAsync(uint8_t type, const uint8_t* payload, uint16_t payloadLength)
{
    LSPC_Async_Package_t package;
    if (payloadLength > LSPC_MAXIMUM_PACKAGE_LENGTH)
        return false; // payload size is too big
    if (uxQueueSpacesAvailable(_TXqueue) == 0) {
        return false; // no space in queue
    }

    if (xPortGetFreeHeapSize() <= 3 * payloadLength)
        return false; // not enough space for payload

    package.type       = type;
    package.payloadPtr = new std::vector<uint8_t>(payloadLength);
    if (!package.payloadPtr)
        return false;
    memcpy(package.payloadPtr->data(), payload, payloadLength);
    if (xQueueSend(_TXqueue, (void*)&package, (TickType_t)0) != pdTRUE) {
        delete (package.payloadPtr); // could not add package to queue, probably because it is full
        return false;
    }

    return true;
}

template<class COM>
bool Socket<COM>::Connected(void) {
    return com->Connected();
}

template<class COM>
void Socket<COM>::ProcessingThread(void* pvParameters)
{
    Socket<COM>* lspc = (Socket<COM>*)pvParameters;

    lspc->incoming_data.reserve(LSPC_MAXIMUM_PACKAGE_LENGTH);

    // LSPC incoming data processing loop
    while (1) {
        if (lspc->com->WaitForNewData(portMAX_DELAY))
            lspc->processSerial();
    }
}

template<class COM>
void Socket<COM>::TransmitterThread(void* pvParameters)
{
    Socket<COM>*         lspc = (Socket<COM>*)pvParameters;
    LSPC_Async_Package_t package;

    while (1) {
        if (lspc->Connected()) {
            // LSPC outgoing (transmission) data loop
            while (lspc->Connected()) {
                if (xQueueReceive(lspc->_TXqueue, &package, (TickType_t)portMAX_DELAY) == pdPASS) {
                    // Send it if possible
                    Packet* outPacket = new Packet(package.type, *package.payloadPtr);
                    if (!outPacket)
                        continue;

                    if (outPacket->encodedDataSize() ==
                        lspc->com->WriteBlocking(outPacket->encodedDataPtr(), outPacket->encodedDataSize())) {
                        delete (package.payloadPtr); // clear memory used for payload data
                    } else {                         // if not, re-add it to the queue
                        // xQueueSend(lspc->_TXqueue, (void *)&package, (TickType_t) 1); // re-add it to the queue
                        // is probably not a good idea
                        delete (package.payloadPtr); // clear memory used for payload data
                    }
                    delete (outPacket);
                }
            }
        }
        osDelay(100);
    }
}

} // namespace lspc

// Template class instantiation based on enabled interface
#if defined(STM32_LSPC_USE_USB)
template class lspc::Socket<UART>;
#elif defined(STM32_LSPC_USE_UART)
template class lspc::Socket<UART>;
#endif
