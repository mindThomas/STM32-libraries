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

#pragma once

#include "SocketBase.hpp"

#include <MessageTypes.h>

// FreeRTOS for task creation
#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#else
#error "LSPC requires FreeRTOS"
#endif

#if defined(LSPC_USE_USB)
//#include <USBCDC/USBCDC.hpp>
#include <UART/UART.hpp>
#elif defined(LSPC_USE_UART)
#include <UART/UART.hpp>
#else
#error "LSPC interface undefined"
#endif

#include <vector>

#include <mallocTracker.hpp>

#define LSPC_MAXIMUM_PACKAGE_LENGTH 250 // limited by LSPC protocol!
#define LSPC_RX_PROCESSING_THREAD_STACK_SIZE 256
#define LSPC_TX_TRANSMITTER_THREAD_STACK_SIZE (LSPC_MAXIMUM_PACKAGE_LENGTH)

#ifdef LSPC_TX_QUEUE_LENGTH
#define LSPC_ASYNCHRONOUS_QUEUE_LENGTH LSPC_TX_QUEUE_LENGTH
#else
#define LSPC_ASYNCHRONOUS_QUEUE_LENGTH 50 // maximum 50 asynchronous packages in queue by default
#endif

namespace lspc {

typedef struct LSPC_Async_Package_t
{
    uint8_t               type;
    std::vector<uint8_t>* payloadPtr;
} LSPC_Async_Package_t;

template<class COM>
class Socket : public SocketBase
{
public:
    Socket(COM* com, uint32_t processingTaskPriority, uint32_t transmitterTaskPriority);

private:
    using SocketBase::send;

    // Send a package with lspc
    //
    // @brief Sends a packaged buffer over the USB serial link.
    //
    // @param type The message type. This is user specific; any type between 1-255.
    // @param payload A vector with the serialized payload to be sent.
    //
    // @return True if the packet was sent.
    bool send(uint8_t type, const std::vector<uint8_t>& payload) override;

    // Process incoming data on serial link
    //
    // @brief Reads the serial buffer and dispatches the received payload to the
    // relevant message handling callback function.
    void processSerial();

public:
    bool TransmitAsync(uint8_t type, const uint8_t* payload, uint16_t payloadLength);

    bool Connected(void);

private:
    static void ProcessingThread(void* pvParameters);
    static void TransmitterThread(void* pvParameters);

public:
    COM* com;

private:
    TaskHandle_t  _processingTaskHandle;
    TaskHandle_t  _transmitterTaskHandle;
    QueueHandle_t _TXqueue;
};

} // namespace lspc

#if defined(LSPC_USE_USB)
using LSPC = lspc::Socket<UART>; // define whether to use USB or UART
#elif defined(LSPC_USE_UART)
using LSPC = lspc::Socket<UART>; // define whether to use USB or UART
#endif
