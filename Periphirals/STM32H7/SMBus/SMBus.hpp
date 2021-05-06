/* Copyright (C) 2018- Thomas Jespersen, TKJ Electronics. All rights reserved.
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

#include <stm32h7xx_hal.h>

// FreeRTOS for memory allocation (for the buffer) and callback
#ifdef USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

class SMBus
{
private:
    const uint32_t SMBUS_DEFAULT_FREQUENCY = 100000; // 100 kHz
    const uint8_t  SMBUS_MASTER_ADDRESS    = 38;

public:
    typedef enum port_t
    {
        PORT_UNDEFINED = 0,
        PORT_I2C2,
        PORT_I2C4
    } port_t;

public:
    SMBus(port_t port, uint8_t devAddr);                     // use default frequency (or current configured frequency)
    SMBus(port_t port, uint8_t devAddr, uint32_t frequency); // configure with frequency if possible
    ~SMBus();
    void    InitPeripheral(port_t port, uint32_t frequency);
    void    DeInitPeripheral();
    void    ConfigurePeripheral();
    void    Write(uint8_t reg, uint8_t* buffer, uint8_t writeLength);
    void    Write(uint8_t reg, uint8_t value);
    void    Read(uint8_t reg, uint8_t* buffer, uint8_t readLength);
    uint8_t Read(uint8_t reg);

public:
    typedef struct hardware_resource_t
    {
        port_t   port;
        uint32_t frequency;
#ifdef USE_FREERTOS
        SemaphoreHandle_t resourceSemaphore;
        SemaphoreHandle_t transmissionFinished;
#else
        bool transmissionFinished;
#endif
        bool                configured;
        uint8_t             instances; // how many objects are using this hardware resource
        SMBUS_HandleTypeDef handle;
    } hardware_resource_t;

    static hardware_resource_t* resI2C2;
    static hardware_resource_t* resI2C4;

private:
    hardware_resource_t* _hRes;
    uint8_t              _devAddr;

public:
    static void TransmissionCompleteCallback(SMBUS_HandleTypeDef* hsmbus);
};
