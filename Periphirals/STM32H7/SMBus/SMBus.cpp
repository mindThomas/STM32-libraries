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

#include "SMBus.hpp"

#include "Priorities.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // for memset

#ifdef STM32H7_SMBUS_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) ((void)0U); // not implemented
#define DEBUG(msg)             // not implemented
#endif

SMBus::hardware_resource_t* SMBus::resI2C2 = 0;
SMBus::hardware_resource_t* SMBus::resI2C4 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void I2C2_EV_IRQHandler(void);
extern "C" void I2C2_ER_IRQHandler(void);
extern "C" void I2C4_EV_IRQHandler(void);
extern "C" void I2C4_ER_IRQHandler(void);
extern "C" void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef* hsmbus);
extern "C" void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef* hsmbus);
extern "C" void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef* hsmbus);

SMBus::SMBus(port_t port, uint8_t devAddr, uint32_t frequency)
{
    _devAddr = devAddr << 1;
    InitPeripheral(port, frequency);
    ConfigurePeripheral();
}

SMBus::SMBus(port_t port, uint8_t devAddr)
    : SMBus(port, devAddr, SMBUS_DEFAULT_FREQUENCY)
{}

SMBus::~SMBus()
{
    if (!_hRes)
        return;

    _hRes->instances--;
    if (_hRes->instances == 0) { // deinitialize port and periphiral
        DeInitPeripheral();

        // Delete hardware resource
        port_t tmpPort = _hRes->port;
        delete (_hRes);

        switch (tmpPort) {
            case PORT_I2C2:
                resI2C2 = 0;
                break;
            case PORT_I2C4:
                resI2C4 = 0;
                break;
            default:
                ERROR("Undefined SMBus port");
                return;
        }
    }
}

void SMBus::DeInitPeripheral()
{
    if (!_hRes)
        return;
    if (_hRes->port == PORT_I2C2) {
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PF0     ------> I2C2_SDA
        PF1     ------> I2C2_SCL
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1);

        /* I2C1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    } else if (_hRes->port == PORT_I2C4) {
        /* Peripheral clock disable */
        __HAL_RCC_I2C4_CLK_DISABLE();

        /**I2C4 GPIO Configuration
        PF14     ------> I2C4_SCL
        PF15     ------> I2C4_SDA
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_14 | GPIO_PIN_15);

        /* I2C3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C4_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C4_EV_IRQn);
    }
}

void SMBus::InitPeripheral(port_t port, uint32_t frequency)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    bool             firstTime       = false;

    switch (port) {
        case PORT_I2C2:
            if (!resI2C2) {
                resI2C2 = new SMBus::hardware_resource_t;
                memset(resI2C2, 0, sizeof(SMBus::hardware_resource_t));
                firstTime = true;
            }
            _hRes = resI2C2;
            break;
        case PORT_I2C4:
            if (!resI2C4) {
                resI2C4 = new SMBus::hardware_resource_t;
                memset(resI2C4, 0, sizeof(SMBus::hardware_resource_t));
                firstTime = true;
            }
            _hRes = resI2C4;
            break;
        default:
            ERROR("Undefined SMBus port");
            _hRes = 0;
            return;
    }

    if (firstTime) { // first time configuring peripheral
        _hRes->port       = port;
        _hRes->frequency  = frequency;
        _hRes->configured = false;
        _hRes->instances  = 0;

#ifdef USE_FREERTOS
        _hRes->resourceSemaphore = xSemaphoreCreateBinary();
        if (_hRes->resourceSemaphore == NULL) {
            _hRes = 0;
            ERROR("Could not create SMBus resource semaphore");
            return;
        }
        vQueueAddToRegistry(_hRes->resourceSemaphore, "SMBus Resource");
        xSemaphoreGive(_hRes->resourceSemaphore); // give the semaphore the first time

        _hRes->transmissionFinished = xSemaphoreCreateBinary();
        if (_hRes->transmissionFinished == NULL) {
            _hRes = 0;
            ERROR("Could not create I2C transmission semaphore");
            return;
        }
        vQueueAddToRegistry(_hRes->transmissionFinished, "SMBus Finish");
        xSemaphoreGive(_hRes->transmissionFinished); // ensure that the semaphore is not taken from the beginning
        xSemaphoreTake(_hRes->transmissionFinished,
                       (TickType_t)portMAX_DELAY); // ensure that the semaphore is not taken from the beginning
#endif

        // Configure pins for I2C and I2C peripheral accordingly
        if (port == PORT_I2C2) {
            __HAL_RCC_GPIOF_CLK_ENABLE();
            /**I2C2 GPIO Configuration
            PF0     ------> I2C2_SDA
            PF1     ------> I2C2_SCL
            */
            GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

            /* Peripheral clock enable */
            __HAL_RCC_I2C2_CLK_ENABLE();

            /* NVIC for I2C1 */
            HAL_NVIC_SetPriority(I2C2_ER_IRQn, SMBUS_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
            HAL_NVIC_SetPriority(I2C2_EV_IRQn, SMBUS_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        } else if (port == PORT_I2C4) {
            __HAL_RCC_GPIOF_CLK_ENABLE();
            /**I2C4 GPIO Configuration
            PF14     ------> I2C4_SCL
            PF15     ------> I2C4_SDA
            */
            GPIO_InitStruct.Pin       = GPIO_PIN_14 | GPIO_PIN_15;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

            /* Peripheral clock enable */
            __HAL_RCC_I2C4_CLK_ENABLE();

            /* NVIC for I2C3 */
            HAL_NVIC_SetPriority(I2C4_ER_IRQn, SMBUS_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C4_ER_IRQn);
            HAL_NVIC_SetPriority(I2C4_EV_IRQn, SMBUS_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C4_EV_IRQn);
        }
    }

    _hRes->instances++;
}

void SMBus::ConfigurePeripheral()
{
    if (!_hRes)
        return;
    if (!_hRes->configured) { // only configure periphiral once
        switch (_hRes->port) {
            case PORT_I2C2:
                _hRes->handle.Instance = I2C2;
                break;
            case PORT_I2C4:
                _hRes->handle.Instance = I2C4;
                break;
            default:
                _hRes = 0;
                ERROR("Undefined SMBus port");
                return;
        }

        _hRes->handle.Init.OwnAddress1          = SMBUS_MASTER_ADDRESS;
        _hRes->handle.Init.AnalogFilter         = SMBUS_ANALOGFILTER_ENABLE;
        _hRes->handle.Init.AddressingMode       = SMBUS_ADDRESSINGMODE_7BIT;
        _hRes->handle.Init.DualAddressMode      = SMBUS_DUALADDRESS_DISABLE;
        _hRes->handle.Init.OwnAddress2          = 0;
        _hRes->handle.Init.OwnAddress2Masks     = SMBUS_OA2_NOMASK;
        _hRes->handle.Init.GeneralCallMode      = SMBUS_GENERALCALL_DISABLE;
        _hRes->handle.Init.NoStretchMode        = SMBUS_NOSTRETCH_DISABLE;
        _hRes->handle.Init.PacketErrorCheckMode = SMBUS_PEC_DISABLE;
        _hRes->handle.Init.PeripheralMode       = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
        _hRes->handle.Init.SMBusTimeout         = 0x000084C4;

        //_hRes->handle.Init.Timing = 0x10C0ECFF;
        /* Determine I2C timing configuration  (see reference manual page 1015)
            tI2CCLK = 1/100Mhz = 10 ns

            PRESC = 0001 = 1
            tPRESC = (PRESC+1)*tI2CCLK = 20 ns

            SCLDEL = 1100 = 12
            tSCLDEL = (SCLDEL+1)*tPRESC = 260 ns     (Data setup time)

            SDADEL = 0000 = 0
            tSDADEL = (SDADEL+1)*tPRESC = 20 ns      (Data hold time)

            SCLH = 11101100 = 236
            tSCLH = (SCLH+1)*tPRESC = 4740 ns        (SCL high period)

            SCLL = 11111111 = 255
            tSCLL = (SCLL+1)*tPRESC = 5120 ns        (SCL low period)

            Period = 4740 + 5120 = 9860 ns
            I2C frequency = 101419,88 Hz
        */

        uint32_t I2C_Clock = HAL_RCC_GetPCLK1Freq(); // assuming all I2C periphirals to be configured with the same
                                                     // clock frequency running at the same frequency as APB1
        uint32_t timingParam      = 0x10C00000;      // set prescaler, data setup and data hold time as listed above
        uint8_t  PRESC            = (timingParam & 0xF0000000) >> 28;
        uint8_t  SCLDEL           = (timingParam & 0x00F00000) >> 20;
        uint8_t  SDADEL           = (timingParam & 0x000F0000) >> 16;
        uint16_t period           = ((I2C_Clock / (PRESC + 1)) / _hRes->frequency) / 2;
        uint16_t periodHigh       = period - (SCLDEL + 1) - (SDADEL + 1);
        uint16_t periodLow        = period;
        _hRes->handle.Init.Timing = timingParam | (periodHigh << 8) | periodLow;

        if (HAL_SMBUS_Init(&_hRes->handle) != HAL_OK) {
            _hRes = 0;
            ERROR("Could not initialize SMBus port");
            return;
        }
    }
}

void SMBus::Write(uint8_t reg, uint8_t value)
{
    Write(reg, &value, 1);
}

void SMBus::Write(uint8_t reg, uint8_t* buffer, uint8_t writeLength)
{
    if (!_hRes)
        return;
#ifdef USE_FREERTOS
    xSemaphoreTake(_hRes->resourceSemaphore, (TickType_t)portMAX_DELAY); // take hardware resource

    if (uxSemaphoreGetCount(
          _hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state
                                        // before starting the transmission, since we use the semaphore for flagging the
                                        // finish transmission event
        xSemaphoreTake(_hRes->transmissionFinished,
                       (TickType_t)portMAX_DELAY); // something incorrect happened, as the transmissionFinished
                                                   // semaphore should always be taken before a transmission starts
#else
    _hRes->transmissionFinished = false;
#endif

#ifdef USE_FREERTOS
    uint8_t* txBuffer = (uint8_t*)pvPortMalloc(writeLength + 1);
#else
    uint8_t* txBuffer           = (uint8_t*)malloc(writeLength + 1);
#endif

    if (!txBuffer)
        return;

    txBuffer[0] = reg;
    memcpy(&txBuffer[1], buffer, writeLength);

    uint32_t XferOptions = SMBUS_OTHER_FRAME_NO_PEC;

    if (HAL_SMBUS_Master_Transmit_IT(&_hRes->handle, (uint16_t)_devAddr, buffer, writeLength, XferOptions) == HAL_OK) {
// Wait for the transmission to finish
#ifdef USE_FREERTOS
        xSemaphoreTake(_hRes->transmissionFinished, (TickType_t)portMAX_DELAY);
#else
        while (!_hRes->transmissionFinished)
            ;
#endif
    } else {
        DEBUG("Failed SMBus transmission");
    }

    // uint32_t errCode = HAL_SMBUS_GetError(&_hRes->handle);

#ifdef USE_FREERTOS
    vPortFree(txBuffer);
#else
    free(txBuffer);
#endif

#ifdef USE_FREERTOS
    xSemaphoreGive(_hRes->resourceSemaphore); // give hardware resource back
#endif
}

uint8_t SMBus::Read(uint8_t reg)
{
    uint8_t rx;
    Read(reg, &rx, 1);
    return rx;
}

void SMBus::Read(uint8_t reg, uint8_t* buffer, uint8_t readLength)
{
    if (!_hRes)
        return;
#ifdef USE_FREERTOS
    xSemaphoreTake(_hRes->resourceSemaphore, (TickType_t)portMAX_DELAY); // take hardware resource

    if (uxSemaphoreGetCount(
          _hRes->transmissionFinished)) // semaphore is available to be taken - which it should not be at this state
                                        // before starting the transmission, since we use the semaphore for flagging the
                                        // finish transmission event
        xSemaphoreTake(_hRes->transmissionFinished,
                       (TickType_t)portMAX_DELAY); // something incorrect happened, as the transmissionFinished
                                                   // semaphore should always be taken before a transmission starts
#else
    _hRes->transmissionFinished = false;
#endif

    uint32_t XferOptions = SMBUS_OTHER_FRAME_NO_PEC;

    // Write register address
    if (HAL_SMBUS_Master_Transmit_IT(&_hRes->handle, (uint16_t)_devAddr, &reg, 1, XferOptions) == HAL_OK) {
        // Wait for the transmission to finish
#ifdef USE_FREERTOS
        xSemaphoreTake(_hRes->transmissionFinished, (TickType_t)portMAX_DELAY);
#else
        while (!_hRes->transmissionFinished)
            ;
#endif
    } else {
        DEBUG("Failed SMBus transmission");
    }

    // uint32_t errCode = HAL_SMBUS_GetError(&_hRes->handle);

    // Read data requested from register address
    if (HAL_SMBUS_Master_Receive_IT(&_hRes->handle, (uint16_t)_devAddr, buffer, readLength, XferOptions) == HAL_OK) {
        // Wait for the transmission to finish
#ifdef USE_FREERTOS
        xSemaphoreTake(_hRes->transmissionFinished, (TickType_t)portMAX_DELAY);
#else
        while (!_hRes->transmissionFinished)
            ;
#endif
    } else {
        DEBUG("Failed SMBus transmission");
    }

    // uint32_t errCode2 = HAL_SMBUS_GetError(&_hRes->handle);

#ifdef USE_FREERTOS
    xSemaphoreGive(_hRes->resourceSemaphore); // give hardware resource back
#endif
}

void I2C2_EV_IRQHandler(void)
{
    if (SMBus::resI2C2)
        HAL_SMBUS_EV_IRQHandler(&SMBus::resI2C2->handle);
}
void I2C2_ER_IRQHandler(void)
{
    if (SMBus::resI2C2)
        HAL_SMBUS_ER_IRQHandler(&SMBus::resI2C4->handle);
}

void I2C4_EV_IRQHandler(void)
{
    if (SMBus::resI2C2)
        HAL_SMBUS_EV_IRQHandler(&SMBus::resI2C2->handle);
}
void I2C4_ER_IRQHandler(void)
{
    if (SMBus::resI2C4)
        HAL_SMBUS_ER_IRQHandler(&SMBus::resI2C4->handle);
}

void SMBus::TransmissionCompleteCallback(SMBUS_HandleTypeDef* hsmbus)
{
    // Tx Transfer completed
    SMBus::hardware_resource_t* smbus;
    if (hsmbus->Instance == I2C2)
        smbus = SMBus::resI2C2;
    else if (hsmbus->Instance == I2C4)
        smbus = SMBus::resI2C4;
    else
        return;

#ifdef USE_FREERTOS
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(smbus->transmissionFinished, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef* hsmbus)
{
    // Tx Transfer completed
    SMBus::TransmissionCompleteCallback(hsmbus);
}
void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef* hsmbus)
{
    // Rx Transfer completed
    SMBus::TransmissionCompleteCallback(hsmbus);
}
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef* hsmbus)
{
    SMBus::TransmissionCompleteCallback(hsmbus);
}
