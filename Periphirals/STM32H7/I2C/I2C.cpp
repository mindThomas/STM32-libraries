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

#include "I2C.hpp"

#include <Priorities.h>
#include <math.h>
#include <string.h> // for memset

#ifdef STM32H7_I2C_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) while(1) { __asm("bkpt #0"); }; // break on error and halt if debugging
#define DEBUG(msg)             // not implemented
#endif

I2C::hardware_resource_t* I2C::resI2C1 = 0;
I2C::hardware_resource_t* I2C::resI2C2 = 0;
I2C::hardware_resource_t* I2C::resI2C3 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void I2C1_EV_IRQHandler(void);
extern "C" void I2C1_ER_IRQHandler(void);
extern "C" void I2C2_EV_IRQHandler(void);
extern "C" void I2C2_ER_IRQHandler(void);
extern "C" void I2C3_EV_IRQHandler(void);
extern "C" void I2C3_ER_IRQHandler(void);
extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* I2cHandle);
extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* I2cHandle);
extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* I2cHandle);

I2C::I2C(port_t port, uint8_t devAddr, uint32_t frequency)
{
    _devAddr = devAddr << 1;
    InitPeripheral(port, frequency);
    ConfigurePeripheral();
}

I2C::I2C(
  port_t  port,
  uint8_t devAddr) /* : I2C(port, devAddr, I2C_DEFAULT_FREQUENCY)    // this is apparently not working properly   */
{
    _devAddr = devAddr << 1;
    InitPeripheral(port, I2C_DEFAULT_FREQUENCY);
    ConfigurePeripheral();
}

I2C::~I2C()
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
            case PORT_I2C1:
                resI2C1 = 0;
                break;
            case PORT_I2C2:
                resI2C2 = 0;
                break;
            case PORT_I2C3:
                resI2C3 = 0;
                break;
            default:
                ERROR("Undefined I2C port");
                return;
        }
    }
}

void I2C::DeInitPeripheral()
{
    if (!_hRes)
        return;
    if (_hRes->port == PORT_I2C1) {
        /* Peripheral clock disable */
        __HAL_RCC_I2C1_CLK_DISABLE();

        /**I2C1 GPIO Configuration
        PB8     ------> I2C1_SCL
        PB9     ------> I2C1_SDA
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);

        /* I2C1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    } else if (_hRes->port == PORT_I2C2) {
        /* Peripheral clock disable */
        __HAL_RCC_I2C2_CLK_DISABLE();

        /**I2C2 GPIO Configuration
        PF1     ------> I2C2_SCL
        PF0     ------> I2C2_SDA
        */
        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0 | GPIO_PIN_1);

        /* I2C2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    } else if (_hRes->port == PORT_I2C3) {
        /* Peripheral clock disable */
        __HAL_RCC_I2C3_CLK_DISABLE();

        /**I2C3 GPIO Configuration
        PC9     ------> I2C3_SDA
        PA8     ------> I2C3_SCL
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

        /* I2C3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    }
}

void I2C::InitPeripheral(port_t port, uint32_t frequency)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    bool             firstTime       = false;

    switch (port) {
        case PORT_I2C1:
            if (!resI2C1) {
                resI2C1 = new I2C::hardware_resource_t;
                memset(resI2C1, 0, sizeof(I2C::hardware_resource_t));
                firstTime = true;
            }
            _hRes = resI2C1;
            break;
        case PORT_I2C2:
            if (!resI2C2) {
                resI2C2 = new I2C::hardware_resource_t;
                memset(resI2C2, 0, sizeof(I2C::hardware_resource_t));
                firstTime = true;
            }
            _hRes = resI2C2;
            break;
        case PORT_I2C3:
            if (!resI2C3) {
                resI2C3 = new I2C::hardware_resource_t;
                memset(resI2C3, 0, sizeof(I2C::hardware_resource_t));
                firstTime = true;
            }
            _hRes = resI2C3;
            break;
        default:
            ERROR("Undefined I2C port");
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
            ERROR("Could not create I2C resource semaphore");
            return;
        }
        vQueueAddToRegistry(_hRes->resourceSemaphore, "I2C Resource");
        xSemaphoreGive(_hRes->resourceSemaphore); // give the semaphore the first time

        _hRes->transmissionFinished = xSemaphoreCreateBinary();
        if (_hRes->transmissionFinished == NULL) {
            _hRes = 0;
            ERROR("Could not create I2C transmission semaphore");
            return;
        }

        vQueueAddToRegistry(_hRes->transmissionFinished, "I2C Finish");
        xSemaphoreGive(_hRes->transmissionFinished); // ensure that the semaphore is not taken from the beginning
        xSemaphoreTake(_hRes->transmissionFinished,
                       (TickType_t)portMAX_DELAY); // ensure that the semaphore is not taken from the beginning
#endif

        // Configure pins for I2C and I2C peripheral accordingly
        if (port == PORT_I2C1) {
            __HAL_RCC_GPIOB_CLK_ENABLE();
            /**I2C1 GPIO Configuration
            PB8     ------> I2C1_SCL
            PB9     ------> I2C1_SDA
            */
            GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

            /* Peripheral clock enable */
            __HAL_RCC_I2C1_CLK_ENABLE();

            /* NVIC for I2C1 */
            HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
            HAL_NVIC_SetPriority(I2C1_EV_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        } else if (port == PORT_I2C2) {
            __HAL_RCC_GPIOF_CLK_ENABLE();
            /**I2C1 GPIO Configuration
            PF1     ------> I2C2_SCL
            PF0     ------> I2C2_SDA
            */
            GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
            HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

            /* Peripheral clock enable */
            __HAL_RCC_I2C2_CLK_ENABLE();

            /* NVIC for I2C2 */
            HAL_NVIC_SetPriority(I2C2_ER_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
            HAL_NVIC_SetPriority(I2C2_EV_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        } else if (port == PORT_I2C3) {
            __HAL_RCC_GPIOC_CLK_ENABLE();
            __HAL_RCC_GPIOA_CLK_ENABLE();
            /**I2C3 GPIO Configuration
            PC9     ------> I2C3_SDA
            PA8     ------> I2C3_SCL
            */
            GPIO_InitStruct.Pin       = GPIO_PIN_9;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

            GPIO_InitStruct.Pin       = GPIO_PIN_8;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            /* Peripheral clock enable */
            __HAL_RCC_I2C3_CLK_ENABLE();

            /* NVIC for I2C3 */
            HAL_NVIC_SetPriority(I2C3_ER_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
            HAL_NVIC_SetPriority(I2C3_EV_IRQn, I2C_INTERRUPT_PRIORITY, 0);
            HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
        }
    }

    _hRes->instances++;
}

void I2C::ConfigurePeripheral()
{
    if (!_hRes)
        return;
    if (!_hRes->configured) { // only configure periphiral once
        switch (_hRes->port) {
            case PORT_I2C1:
                _hRes->handle.Instance = I2C1;
                break;
            case PORT_I2C2:
                _hRes->handle.Instance = I2C2;
                break;
            case PORT_I2C3:
                _hRes->handle.Instance = I2C3;
                break;
            default:
                _hRes = 0;
                ERROR("Undefined I2C port");
                return;
        }

        _hRes->handle.Init.OwnAddress1      = 0;
        _hRes->handle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
        _hRes->handle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
        _hRes->handle.Init.OwnAddress2      = 0;
        _hRes->handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
        _hRes->handle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
        _hRes->handle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

        //_hRes->handle.Init.Timing = 0x10C0ECFF;
        /* Determine I2C timing configuration  (see reference manual page 1015)
            tI2CCLK = 1/100Mhz = 10 ns

            PRESC = 0001 = 1
            tPRESC = (PRESC+1)*tI2CCLK = 20 ns

            SCLDEL = 1100 = 12
            tSCLDEL = (SCLDEL+1)*tPRESC = 260 ns     (Data setup time)

            SDADEL = 0000 = 0
            tSDADEL = (SDADEL)*tPRESC = 0 ns         (Data hold time)

            SCLH = 11101100 = 236
            tSCLH = (SCLH+1)*tPRESC = 4740 ns        (SCL high period)

            SCLL = 11111111 = 255
            tSCLL = (SCLL+1)*tPRESC = 5120 ns        (SCL low period)

            Period = 4740 + 5120 = 9860 ns
            I2C frequency = 101419,88 Hz
        */

        // See chapter 30.4.9 and table 181 in Reference manual for recommended values
        // fI2CCLK = 100 Mhz ??
        // tI2CCLK = 10 ns ??
        // tPRESC = (PRESC+1)*tI2CCLK
        // tSYNC1 + tSYNC2 = minimum value of 4 x tI2CCLK
        // tSCL = tSCLL + tSCLH + tSYNC1 + tSYNC2
        // tSDADEL recommended = 500 ns
        // tSCLDEL recommended = 1250 ns
        // tSCLDEL = (SCLDEL+1)*tPRESC    (Data setup time)
        // tSDADEL = (SDADEL)*tPRESC      (Data hold time)
        // tSCLH = (SCLH+1)*tPRESC        (SCL high period)
        // tSCLL = (SCLL+1)*tPRESC        (SCL low period)

        volatile uint32_t tSDADELns;
        volatile uint32_t tSCLDELns;
        volatile uint8_t  SYNC_PERIODS_upperbound;

        if (_hRes->frequency > 350000) { // fast mode I2C, eg. f = 400 KHz
            tSDADELns               = 375;
            tSCLDELns               = 500;
            SYNC_PERIODS_upperbound = 8;
        } else { // standard mode I2C
            tSDADELns               = 500;
            tSCLDELns               = 1250;
            SYNC_PERIODS_upperbound = 4;
        }

        volatile uint32_t I2C_Clock =
          HAL_RCC_GetPCLK1Freq(); // assuming all I2C periphirals to be configured with the same clock frequency running
                                  // at the same frequency as APB1
        volatile uint32_t tI2CCLKns = 1000000000 / I2C_Clock;

        volatile uint32_t tSCLns = 1000000000 / _hRes->frequency;

        volatile uint8_t  PRESC;
        volatile uint32_t tPRESCns;
        volatile uint32_t tSYNCns;
        volatile uint32_t tSCLLns;
        volatile uint32_t tSCLHns;
        volatile uint8_t  SCLL;
        volatile uint8_t  SCLH;
        volatile uint8_t  SCLDEL;
        volatile uint8_t  SDADEL;

        for (PRESC = 0; PRESC < 0x0F; PRESC++) {
            tPRESCns = (PRESC + 1) * tI2CCLKns;
            tSYNCns  = SYNC_PERIODS_upperbound * tPRESCns;

            tSCLLns = tSCLns / 2;
            tSCLHns = tSCLns - tSCLLns - tSYNCns;

            SCLL   = tSCLLns / tPRESCns - 1; // tSCLL = (SCLL+1)*tPRESC   --->   SCLL = tSCLL/tPRESC - 1
            SCLH   = tSCLHns / tPRESCns - 1;
            SCLDEL = tSCLDELns / tPRESCns - 1;
            SDADEL = tSDADELns / tPRESCns;

            if (SCLDEL < 0x10 && SDADEL < 0x10)
                break;
        }

        /*volatile uint32_t timingParam = 0x10C00000; // set prescaler, data setup and data hold time as listed above
        volatile uint8_t PRESC =  (timingParam & 0xF0000000) >> 28;
        volatile uint8_t SCLDEL = (timingParam & 0x00F00000) >> 20;
        volatile uint8_t SDADEL = (timingParam & 0x000F0000) >> 16;
        volatile uint16_t period = ((I2C_Clock/(PRESC+1)) / _hRes->frequency) / 2;
        volatile uint16_t periodHigh = period - (SCLDEL+1) - (SDADEL+1);
        volatile uint16_t periodLow = period;*/

        volatile uint32_t timingParam = ((uint32_t)(PRESC & 0x0F) << 28) | ((uint32_t)(SCLDEL & 0x0F) << 20) |
                                        ((uint32_t)(SDADEL & 0x0F) << 16) | ((uint32_t)SCLH << 8) | (uint32_t)SCLL;

        _hRes->handle.Init.Timing = timingParam;

        if (HAL_I2C_Init(&_hRes->handle) != HAL_OK) {
            _hRes = 0;
            ERROR("Could not initialize I2C port");
            return;
        }

        if (HAL_I2CEx_ConfigAnalogFilter(&_hRes->handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
            _hRes = 0;
            ERROR("Could not initialize analog filter for I2C port");
            return;
        }

        if (HAL_I2CEx_ConfigAnalogFilter(&_hRes->handle, 0) != HAL_OK) {
            _hRes = 0;
            ERROR("Could not initialize digital filters for I2C port");
            return;
        }

        _hRes->configured = true;
    }
}

bool I2C::Write(uint8_t reg, uint8_t value)
{
    return Write(reg, &value, 1);
}

bool I2C::Write(uint8_t reg, uint8_t* buffer, uint8_t writeLength)
{
    bool success = false;
    if (!_hRes)
        return success;
#ifdef USE_FREERTOS
    xSemaphoreTake(_hRes->resourceSemaphore, (TickType_t)portMAX_DELAY); // take hardware resource

    // Consider to use task notifications instead: https://www.freertos.org/RTOS-task-notifications.html
    // However using notifications can possibly lead to other problems if multiple objects are going to notify the same
    // task simultaneously
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
    /*uint8_t * txBuffer = (uint8_t *)pvPortMalloc(writeLength+1);

    if (!txBuffer) return;

    txBuffer[0] = reg;
    memcpy(&txBuffer[1], buffer, writeLength);*/

    if (HAL_I2C_Mem_Write_IT(&_hRes->handle, (uint16_t)_devAddr, reg, I2C_MEMADD_SIZE_8BIT, buffer, writeLength) ==
        HAL_OK) {
        // Wait for the transmission to finish
#ifdef USE_FREERTOS
        xSemaphoreTake(_hRes->transmissionFinished, (TickType_t)portMAX_DELAY);
#else
        while (!_hRes->transmissionFinished)
            ;
#endif
        success = true;
    } else {
        DEBUG("Failed I2C transmission");
    }

    // uint32_t errCode = HAL_I2C_GetError(&_hRes->handle);

    // vPortFree(txBuffer);

#ifdef USE_FREERTOS
    xSemaphoreGive(_hRes->resourceSemaphore); // give hardware resource back
#endif

    return success;
}

uint8_t I2C::Read(uint8_t reg)
{
    uint8_t rx;
    if (Read(reg, &rx, 1))
        return rx;
    else
        return 0;
}

bool I2C::Read(uint8_t reg, uint8_t* buffer, uint8_t readLength)
{
    bool success = false;
    if (!_hRes)
        return success;
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

    /*uint8_t * txBuffer = (uint8_t *)pvPortMalloc(readLength+1);
    uint8_t * rxBuffer = (uint8_t *)pvPortMalloc(readLength+1);

    if (!txBuffer || !rxBuffer) return;

    memset(txBuffer, 0, readLength+1);
    txBuffer[0] = reg;*/

    if (HAL_I2C_Mem_Read_IT(&_hRes->handle, (uint16_t)_devAddr, reg, I2C_MEMADD_SIZE_8BIT, buffer, readLength) ==
        HAL_OK) {
        // Wait for the transmission to finish
#ifdef USE_FREERTOS
        xSemaphoreTake(_hRes->transmissionFinished, (TickType_t)portMAX_DELAY);
#else
        while (!_hRes->transmissionFinished)
            ;
#endif
        success = true;
    } else {
        DEBUG("Failed I2C transmission");
    }

    // uint32_t errCode = HAL_I2C_GetError(&_hRes->handle);

    /*memcpy(buffer, &rxBuffer[1], readLength);

    vPortFree(txBuffer);
    vPortFree(rxBuffer);*/

#ifdef USE_FREERTOS
    xSemaphoreGive(_hRes->resourceSemaphore); // give hardware resource back
#endif
    return success;
}

void I2C1_EV_IRQHandler(void)
{
    if (I2C::resI2C1)
        HAL_I2C_EV_IRQHandler(&I2C::resI2C1->handle);
}
void I2C1_ER_IRQHandler(void)
{
    if (I2C::resI2C1)
        HAL_I2C_ER_IRQHandler(&I2C::resI2C1->handle);
}

void I2C2_EV_IRQHandler(void)
{
    if (I2C::resI2C2)
        HAL_I2C_EV_IRQHandler(&I2C::resI2C2->handle);
}
void I2C2_ER_IRQHandler(void)
{
    if (I2C::resI2C2)
        HAL_I2C_ER_IRQHandler(&I2C::resI2C2->handle);
}

void I2C3_EV_IRQHandler(void)
{
    if (I2C::resI2C3)
        HAL_I2C_EV_IRQHandler(&I2C::resI2C3->handle);
}
void I2C3_ER_IRQHandler(void)
{
    if (I2C::resI2C3)
        HAL_I2C_ER_IRQHandler(&I2C::resI2C3->handle);
}

void I2C::TransmissionCompleteCallback(I2C_HandleTypeDef* I2cHandle)
{
    // Tx Transfer completed
    I2C::hardware_resource_t* i2c;
    if (I2cHandle->Instance == I2C1)
        i2c = I2C::resI2C1;
    else if (I2cHandle->Instance == I2C2)
        i2c = I2C::resI2C2;
    else if (I2cHandle->Instance == I2C3)
        i2c = I2C::resI2C3;
    else
        return;

#ifdef USE_FREERTOS
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c->transmissionFinished, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#endif
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* I2cHandle)
{
    // Tx Transfer completed
    I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* I2cHandle)
{
    // Rx Transfer completed
    I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* I2cHandle)
{
    // Tx Transfer completed
    I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* I2cHandle)
{
    // Rx Transfer completed
    I2C::TransmissionCompleteCallback(I2cHandle);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* I2cHandle)
{
    I2C::TransmissionCompleteCallback(I2cHandle);
}
