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

#include "UART.hpp"

#include <Priorities.h>
#include <string.h> // for memset

// HAL libraries
#include <stm32f4xx_hal_uart.h>
#include <stm32f4xx_hal_dma.h>

#ifdef STM32F4_UART_USE_DEBUG
#include <Debug/Debug.h>
#else
#define ERROR(msg) ((void)0U); // not implemented
#endif

#ifndef USE_FREERTOS
#include <malloc.h>
#endif

UART* UART::objUART2 = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" void USART2_IRQHandler(void);
extern "C" void DMA1_Stream5_IRQHandler(void);
extern "C" void DMA1_Stream6_IRQHandler(void);
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* UartHandle);
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle);
extern "C" void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* UartHandle);

#ifdef USE_FREERTOS
UART::UART(port_t port, uint32_t baud, uint32_t bufferLength, bool DMA_enabled)
    : BaudRate(baud)
    , DMA_Enabled(DMA_enabled)
    , _port(port)
    , _rxByteAvailable(false)
    , _rxAvailableOnIdle(false)
    , _bufferLength(bufferLength)
    , _bufferWriteIdx(0)
    , _bufferReadIdx(0)
#ifdef USE_FREETOS
    , _callbackTaskHandle(0)
    , _resourceSemaphore(0)
#endif
    , _callbackChunkLength(0)
    , _txPointer(0)
    , _txRemainingBytes(0)
    , _transmitFinished(0)
    , _RXdataAvailable(0)
    , _RXcallback(0)
    , _RXcallbackParameter(0)
#else
UART::UART(port_t port, uint32_t baud, uint32_t bufferLength, bool DMA_enabled)
    : BaudRate(baud)
    , DMA_Enabled(DMA_enabled)
    , _port(port)
    , _rxByteAvailable(false)
    , _rxAvailableOnIdle(false)
    , _bufferLength(bufferLength)
    , _bufferWriteIdx(0)
    , _bufferReadIdx(0)
    , _callbackChunkLength(0)
    , _txPointer(0)
    , _txRemainingBytes(0)
    , _transmitFinished(true)
    , _RXcallback(0)
    , _RXcallbackParameter(0)
#endif
{
    if (DMA_enabled && bufferLength == 0) {
        _bufferLength = DMA_RX_MIN_BUFFER_SIZE;
    }
    if (_bufferLength > 0) {
#ifdef USE_FREERTOS
        _buffer = (uint8_t*)pvPortMalloc(_bufferLength);
#else
        _buffer = (uint8_t*)malloc(_bufferLength);
#endif
        if (_buffer == NULL) {
            ERROR("Could not create UART buffer");
            return;
        }
        memset(_buffer, 0, _bufferLength);
    } else {
        _buffer = 0;
    }
    InitPeripheral();
    ConfigurePeripheral();
}

UART::UART(port_t port, uint32_t baud, bool DMA_enabled)
    : UART(port, baud, 0, DMA_enabled)
{}

UART::~UART()
{
    DeInitPeripheral();

    if (_buffer) {
#ifdef USE_FREERTOS
        vPortFree(_buffer);
#else
        free(_buffer);
#endif
    }
}

void UART::ConfigurePeripheral()
{
    switch (_port) {
        case PORT_UART2:
            if (objUART2) {
                ERROR("UART2 already in used");
                return;
            }
            _handle.Instance = USART2;
            objUART2         = this;
            break;
        default:
            ERROR("Undefined UART port");
            return;
    }

#ifdef USE_FREERTOS
    _resourceSemaphore = xSemaphoreCreateBinary();
    if (_resourceSemaphore == NULL) {
        ERROR("Could not create UART resource semaphore");
        return;
    }
    vQueueAddToRegistry(_resourceSemaphore, "UART Resource");
    xSemaphoreGive(_resourceSemaphore); // give the semaphore the first time

    // Create binary semaphore for indicating when a single byte has finished transmitting (for flagging to the transmit
    // thread)
    _transmitFinished = xSemaphoreCreateBinary();
    if (_transmitFinished == NULL) {
        ERROR("Could not create UART transmit finished semaphore");
        return;
    }
    vQueueAddToRegistry(_transmitFinished, "UART Finished");
    xSemaphoreGive(_transmitFinished); // give the semaphore the first time

    _RXdataAvailable = xSemaphoreCreateBinary();
    if (_RXdataAvailable == NULL) {
        ERROR("Could not create UART RX available semaphore");
        return;
    }
    vQueueAddToRegistry(_RXdataAvailable, "UART RX Available");
#else
    _transmitFinished = true;
#endif

    _handle.Init.BaudRate               = BaudRate;
    _handle.Init.WordLength             = UART_WORDLENGTH_8B;
    _handle.Init.StopBits               = UART_STOPBITS_1;
    _handle.Init.Parity                 = UART_PARITY_NONE;
    _handle.Init.Mode                   = UART_MODE_TX_RX;
    _handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    _handle.Init.OverSampling           = UART_OVERSAMPLING_16;

    __HAL_UART_DISABLE_IT(&_handle,  UART_IT_CTS | UART_IT_LBD | UART_IT_TXE | UART_IT_TC | UART_IT_RXNE | UART_IT_IDLE | UART_IT_PE | UART_IT_ERR);
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    if (HAL_UART_Init(&_handle) != HAL_OK) {
        ERROR("Could not initialize UART port");
        return;
    }

    if (!DMA_Enabled) {
        /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        __HAL_UART_ENABLE_IT(&_handle, UART_IT_ERR);

        /* Disable the UART Transmit Complete Interrupt */
        __HAL_UART_DISABLE_IT(&_handle, UART_IT_TC);

        /* Enable the UART Parity Error interupt and Data Register (RX) Not Empty interrupt (since FIFO mode is
         * disabled) */
        __HAL_UART_ENABLE_IT(&_handle, UART_IT_PE);
        __HAL_UART_ENABLE_IT(&_handle, UART_IT_RXNE);
    } else {
        ConfigurePeripheral_DMA();
    }

    // Enable IDLE line detection
    __HAL_UART_ENABLE_IT(&_handle, UART_IT_IDLE);
    _rxAvailableOnIdle = true;

    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void UART::ConfigurePeripheral_DMA()
{
    /* USART2 DMA Init */
    /* USART2_RX Init */
    _rxDMA.Instance                 = DMA1_Stream5;
    _rxDMA.Init.Channel             = DMA_CHANNEL_4;
    _rxDMA.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    _rxDMA.Init.PeriphInc           = DMA_PINC_DISABLE;
    _rxDMA.Init.MemInc              = DMA_MINC_ENABLE;
    _rxDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    _rxDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    _rxDMA.Init.Mode                = DMA_CIRCULAR;
    _rxDMA.Init.Priority            = DMA_PRIORITY_LOW;
    _rxDMA.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    _rxDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    _rxDMA.Init.MemBurst            = DMA_MBURST_INC4;
    _rxDMA.Init.PeriphBurst         = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&_rxDMA) != HAL_OK) {
        ERROR("Could not initialize UART RX DMA");
        return;
    }

    __HAL_LINKDMA(&_handle, hdmarx, _rxDMA);

    /* USART2_TX Init */
    _txDMA.Instance                 = DMA1_Stream6;
    _txDMA.Init.Channel             = DMA_CHANNEL_4;
    _txDMA.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    _txDMA.Init.PeriphInc           = DMA_PINC_DISABLE;
    _txDMA.Init.MemInc              = DMA_MINC_ENABLE;
    _txDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    _txDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    _txDMA.Init.Mode                = DMA_NORMAL;
    _txDMA.Init.Priority            = DMA_PRIORITY_LOW;
    _txDMA.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    _txDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    _txDMA.Init.MemBurst            = DMA_MBURST_INC4;
    _txDMA.Init.PeriphBurst         = DMA_PBURST_INC4;
    if (HAL_DMA_Init(&_txDMA) != HAL_OK) {
        ERROR("Could not initialize UART TX DMA");
        return;
    }

    __HAL_LINKDMA(&_handle, hdmatx, _txDMA);

    // Start DMA reception
    HAL_UART_Receive_DMA(&_handle, _buffer, _bufferLength);
}

void UART::InitPeripheral()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (_port == PORT_UART2) {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART1 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, UART_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }

    if (DMA_Enabled) {
        InitPeripheral_DMA();
    }
}

void UART::DeInitPeripheral()
{
    if (_port == PORT_UART2) {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        // ToDo: Deinit GPIO pins
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);

        /* UART1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }

    if (DMA_Enabled) {
        DeInitPeripheral_DMA();
    }
}

void UART::InitPeripheral_DMA()
{
    if (_port == PORT_UART2) {
        /* DMA controller clock enable */
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* DMA interrupt init */
        /* DMA1_Channel5_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, UART_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

        /* DMA1_Channel6_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, UART_INTERRUPT_PRIORITY, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    }
}

void UART::DeInitPeripheral_DMA()
{
    if (_port == PORT_UART2) {
        HAL_DMA_DeInit(_handle.hdmarx);
        HAL_DMA_DeInit(_handle.hdmatx);
    }
}

void UART::RegisterRXcallback(void(*callback) UART_CALLBACK_PARAMS, void* parameter, uint32_t chunkLength)
{
    _RXcallback          = callback;
    _RXcallbackParameter = parameter;
    _callbackChunkLength = chunkLength;

    if (_callbackChunkLength > _bufferLength) {
        _callbackChunkLength = _bufferLength;
    }

#ifdef USE_FREERTOS
    switch (_port) {
        case PORT_UART2:
            xTaskCreate(UART::CallbackThread, (char*)"UART2 callback", 512, (void*)this, UART_RECEIVER_PRIORITY,
                        &_callbackTaskHandle);
            break;
        default:
            break;
    }
#endif
}

void UART::DeregisterCallback()
{
    _RXcallback          = 0;
    _RXcallbackParameter = 0;
    _callbackChunkLength = 0;

#ifdef USE_FREERTOS
    if (_callbackTaskHandle) {
        xTaskResumeFromISR(_callbackTaskHandle);
        _callbackTaskHandle = 0;
    }
#endif
}

void UART::TransmitBlocking(uint8_t* buffer, uint32_t bufLen)
{
    if (DMA_Enabled) {
        TransmitBlockingDMA(buffer, bufLen);
    } else {
        TransmitBlockingAutoInterrupt(buffer, bufLen);
    }
}

void UART::TransmitBlockingDMA(uint8_t* buffer, uint32_t bufLen)
{
    if (!bufLen)
        return;

#ifdef USE_FREERTOS
    xSemaphoreTake(_transmitFinished, (TickType_t)0); // assuming that transmission is already finished
#else
    _transmitFinished = false; // assuming that transmission is already finished
#endif

    StartDMATransfer(buffer, bufLen);

    // Wait for transmission to finish
#ifdef USE_FREERTOS
    xSemaphoreTake(_transmitFinished, (TickType_t)portMAX_DELAY); // block until it has finished sending the byte
#else
    while (!_transmitFinished)
        ;
    _transmitFinished = false;
#endif
}

// Slight modification to the "HAL_UART_Transmit_DMA" method from "stm32g4xx_hal_uart.c" to point to a different DMA
// Transmit Completed handler
void UART::StartDMATransfer(uint8_t* pData, uint16_t Size)
{
    if ((pData == NULL) || (Size == 0U)) {
        return; // HAL_ERROR;
    }

    _handle.pTxBuffPtr  = pData;
    _handle.TxXferSize  = Size;
    _handle.TxXferCount = Size;

    _handle.ErrorCode = HAL_UART_ERROR_NONE;
    _handle.gState    = HAL_UART_STATE_BUSY_TX;

    if (_handle.hdmatx != NULL) {
        /* Set the UART DMA transfer complete callback */
        _handle.hdmatx->XferCpltCallback = UART::DMATransmitCplt;

        /* Set the UART DMA Half transfer complete callback */
        _handle.hdmatx->XferHalfCpltCallback = NULL;

        /* Set the DMA error callback */
        _handle.hdmatx->XferErrorCallback = UART::DMAError;

        /* Set the DMA abort callback */
        _handle.hdmatx->XferAbortCallback = NULL;

        /* Enable the UART transmit DMA channel */
        if (HAL_DMA_Start_IT(_handle.hdmatx, (uint32_t)_handle.pTxBuffPtr, (uint32_t)&_handle.Instance->DR, Size) !=
            HAL_OK) {
            /* Set error code to DMA */
            _handle.ErrorCode = HAL_UART_ERROR_DMA;

            return;
        }
    }

    /* Clear the TC flag in the ICR register */
    __HAL_UART_CLEAR_FLAG(&_handle, UART_FLAG_TC);

    /* Enable the DMA transfer for transmit request by setting the DMAT bit
    in the UART CR3 register */
    SET_BIT(_handle.Instance->CR3, USART_CR3_DMAT);
}

void UART::TransmitBlockingAutoInterrupt(uint8_t* buffer, uint32_t bufLen)
{
    if (!bufLen)
        return;

    _txPointer        = buffer;
    _txRemainingBytes = bufLen;
#ifdef USE_FREERTOS
    xSemaphoreTake(_transmitFinished, (TickType_t)0); // assuming that transmission is already finished
#else
    _transmitFinished = false; // assuming that transmission is already finished
#endif

    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    _txRemainingBytes--;
    _handle.Instance->DR = *_txPointer++;
    if (_txRemainingBytes > 0) {
        /* Enable TXE interrupt */
        __HAL_UART_ENABLE_IT(&_handle, UART_IT_TXE);
    } else {
        /* Enable TC interrupt */
        __HAL_UART_ENABLE_IT(&_handle, UART_IT_TC);
    }

#ifdef USE_FREERTOS
    xSemaphoreTake(_transmitFinished, (TickType_t)portMAX_DELAY); // block until it has finished sending the byte
#else
    while (!_transmitFinished)
        ;
    _transmitFinished = false;
#endif
}

void UART::TransmitBlockingManualInterrupt(uint8_t* buffer, uint32_t bufLen)
{
    if (!bufLen)
        return;

        /* Enable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled). */
        // LL_USART_EnableIT_TXE(_instance);
        // LL_USART_DisableIT_TC(_instance);
        // LL_USART_ClearFlag_TC(_instance);

#ifdef USE_FREERTOS
    xSemaphoreTake(_transmitFinished, (TickType_t)0); // assuming that transmission is already finished
#else
    _transmitFinished = false; // assuming that transmission is already finished
#endif

    do {
        // Transmit the data
        _handle.Instance->DR = *buffer++;

        // If last char to be sent, enable TC interrupt
        if (bufLen == 1) {
            __HAL_UART_ENABLE_IT(&_handle, UART_IT_TC);
        } else {
            // Enable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled).
            __HAL_UART_ENABLE_IT(&_handle, UART_IT_TXE);
        }

#ifdef USE_FREERTOS
        xSemaphoreTake(_transmitFinished, (TickType_t)portMAX_DELAY); // block until it has finished sending the byte
#else
        while (!_transmitFinished)
            ;
        _transmitFinished = false;
#endif
    } while (--bufLen > 0);

    /* Disable the Transmit Data Register Empty interrupt (if FIFO mode is Disabled). */
    // LL_USART_DisableIT_TXE(_instance);
}

void UART::TransmitBlockingHardPolling(uint8_t* buffer, uint32_t bufLen)
{
    if (!bufLen)
        return;

    __HAL_UART_DISABLE_IT(&_handle, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&_handle, UART_IT_TC);

    do {
        /* Wait for TXE flag to be raised */
        while (__HAL_UART_GET_FLAG(&_handle, UART_FLAG_TXE) == RESET)
            ;

        /* If last char to be sent, clear TC flag */
        if (bufLen == 1)
            __HAL_UART_CLEAR_FLAG(&_handle, UART_FLAG_TC);

        /* Write character in Transmit Data register.
         * TXE flag is cleared by writing data in TDR register */
        _handle.Instance->DR = *buffer++;
    } while (--bufLen > 0);

    /* Wait for TC flag to be raised for last char */
    while (__HAL_UART_GET_FLAG(&_handle, UART_FLAG_TC) == RESET)
        ;
}

void UART::Write(uint8_t byte)
{
#ifdef USE_FREERTOS
    xSemaphoreTake(_resourceSemaphore, (TickType_t)portMAX_DELAY); // take hardware resource

    // OBS! This should be replaced with a non-blocking call by making a TX queue and a processing thread in the UART
    // object
    TransmitBlocking(&byte, 1); // transmit with interrupt-based semaphore waiting (blocking only this thread)

    xSemaphoreGive(_resourceSemaphore); // give hardware resource back
#else
    TransmitBlocking(&byte, 1);
#endif
}

uint32_t UART::Write(uint8_t* buffer, uint32_t length)
{
#ifdef USE_FREERTOS
    xSemaphoreTake(_resourceSemaphore, (TickType_t)portMAX_DELAY); // take hardware resource

    TransmitBlocking(buffer, length); // transmit with interrupt-based semaphore waiting (blocking only this thread)

    xSemaphoreGive(_resourceSemaphore); // give hardware resource back
#else
    TransmitBlocking(buffer, length);
#endif
    return length;
}

uint32_t UART::WriteBlocking(uint8_t* buffer, uint32_t length)
{
    return Write(buffer, length);
}

int16_t UART::Read()
{
    if (!_bufferLength) {
        if (_rxByteAvailable) {
            int16_t ret      = rxByte;
            _rxByteAvailable = false;
            return ret;
        } else {
            return -1; // error, buffer not enabled
        }
    }
    if (BufferContentSize() == 0)
        return -1; // error, buffer is empty
    return BufferPop();
}

bool UART::Connected()
{
    return true; // UART is always connected after initializing and configuring the periphiral (port is opened)
}

#ifdef USE_FREERTOS
void UART::CallbackThread(void* pvParameters)
{
    UART* uart = (UART*)pvParameters;

    uint8_t* popBuffer = (uint8_t*)pvPortMalloc(uart->CALLBACK_THREAD_POP_BUFFER_SIZE);
    if (!popBuffer) {
        ERROR("Could not create pop-buffer for UART Callback thread");
        uart->_callbackTaskHandle = 0;
        vTaskDelete(NULL); // delete/stop this current task
    }

    while (uart->_callbackTaskHandle) {
        vTaskSuspend(
          NULL); // suspend current thread - this could also be replaced by semaphore-based waiting (flagging)

        if (uart->_RXcallback) {
            if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
                uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
            } else { // buffer enabled, hence process buffer content
                while (uart->Available()) {
                    if (uart->_callbackChunkLength == 0) { // call callback with available chunks
                        uint32_t dataPopped = uart->BufferPopMax(popBuffer, uart->CALLBACK_THREAD_POP_BUFFER_SIZE);
                        if (dataPopped) {
                            uart->_RXcallback(uart->_RXcallbackParameter, popBuffer, dataPopped);
                        }
                    } else if (uart->_callbackChunkLength == 1) { // call callback with only 1 byte
                        uint8_t byte = uart->BufferPop();
                        uart->_RXcallback(uart->_RXcallbackParameter, &byte, 1);
                    } else if (uart->BufferContentSize() >=
                               uart->_callbackChunkLength) { // call callback with given chunk size, only when available
                        uint8_t* chunkBuffer = uart->BufferPopN(uart->_callbackChunkLength);
                        if (chunkBuffer) {
                            uart->_RXcallback(uart->_RXcallbackParameter, chunkBuffer, uart->_callbackChunkLength);
                            vPortFree(chunkBuffer);
                        }
                    } else {
                        break; // exit while and sleep thread until next receive event
                    }
                }
            }
        } else {
            break; // exit while loop to stop current task, since the RX callback is no longer registered
        }
    }

    vTaskDelete(NULL); // delete/stop this current task
}
#endif

void UART::UART_IncomingDataInterrupt(UART* uart)
{
#ifdef USE_FREERTOS
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
#endif

    // Only call this function is RXNE interrupt flag is set
    if (!uart) {
        ERROR("UART interrupt for unconfigured port");
        return;
    }

    uint16_t uhdata = (uint16_t)READ_REG(uart->_handle.Instance->DR);
    uart->BufferPush((uint8_t)(uhdata & 0x00FF)); // push into local buffer
    /* Clear RXNE interrupt flag */
    //__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST); // should already have been cleared by reading

    if (!uart->_rxAvailableOnIdle) {
#ifdef USE_FREERTOS
        if (uart->_RXcallback && uart->_callbackTaskHandle) {
            portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(uart->_callbackTaskHandle);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        if (uart->_RXdataAvailable)
            xSemaphoreGiveFromISR(uart->_RXdataAvailable, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        if (uart->_RXcallback) {
            if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
                uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
            }
        }
#endif
    }
}

uint32_t UART::WaitForNewData(uint32_t millisecondsToWait) // blocking call
{
#ifdef USE_FREERTOS
    if (Available()) {
        xSemaphoreTake(_RXdataAvailable, (TickType_t)0);
        return pdTRUE;
    }
    return xSemaphoreTake(_RXdataAvailable, (TickType_t)millisecondsToWait);
#else
    if (millisecondsToWait == 0) {
        while (!Available())
            ;
    } else {
        uint32_t timeout = millisecondsToWait;
        while (timeout-- > 0) {
            if (Available()) {
                return true;
            }
            HAL_Delay(1);
        }
        return false;
    }
#endif
}

void UART::DMA_TX_Completed(UART* uart)
{
    /* Disable the DMA transfer for transmit request by resetting the DMAT bit
       in the UART CR3 register */
    CLEAR_BIT(uart->_handle.Instance->CR3, USART_CR3_DMAT);

    /* Enable the UART Transmit Complete Interrupt */
    SET_BIT(uart->_handle.Instance->CR1, USART_CR1_TCIE);

#ifdef USE_FREERTOS
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart->_transmitFinished, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
    uart->_transmitFinished = true;
#endif
}

void UART::DMA_Error(UART* uart)
{
    UART_HandleTypeDef* huart = &uart->_handle;

    const HAL_UART_StateTypeDef gstate  = huart->gState;
    const HAL_UART_StateTypeDef rxstate = huart->RxState;

    /* Stop UART DMA Tx request if ongoing */
    if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT)) && (gstate == HAL_UART_STATE_BUSY_TX)) {
        huart->TxXferCount = 0U;
        // UART_EndTxTransfer(huart);

        /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_STATE_READY;
    }

    /* Stop UART DMA Rx request if ongoing */
    if ((HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) && (rxstate == HAL_UART_STATE_BUSY_RX)) {
        huart->RxXferCount = 0U;
        // UART_EndRxTransfer(huart);

        /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        /* At end of Rx process, restore huart->RxState to Ready */
        huart->RxState = HAL_UART_STATE_READY;
    }

    huart->ErrorCode |= HAL_UART_ERROR_DMA;
}

void UART::DMA_RX_Check(UART* uart)
{
    if (!uart) {
        ERROR("UART interrupt for unconfigured port");
        return;
    }

    // ToDo: OBS! This is not yet handling overrun/overwriting non-read data
    uart->_bufferWriteIdx = uart->_bufferLength - __HAL_DMA_GET_COUNTER(uart->_handle.hdmarx);

    if (!uart->_rxAvailableOnIdle) {
#ifdef USE_FREERTOS
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

        if (uart->Available()) {
            if (uart->_RXcallback && uart->_callbackTaskHandle) {
                portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(uart->_callbackTaskHandle);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }

            if (uart->_RXdataAvailable)
                xSemaphoreGiveFromISR(uart->_RXdataAvailable, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        if (uart->_RXcallback) {
            if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
                uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
            }
        }
#endif
    }
}

void USART2_IRQHandler(void)
{
    UART::UART_Interrupt(UART::PORT_UART2);
}

void UART::UART_Interrupt(port_t port)
{
    UART* uart = 0;
    switch (port) {
        case PORT_UART2:
            uart = objUART2;
            break;
        default:
            break;
    }

    if (!uart) {
        ERROR("UART interrupt for unconfigured port");
        return;
    }

    uint32_t isrflags = READ_REG(uart->_handle.Instance->SR);
    uint32_t cr1its   = READ_REG(uart->_handle.Instance->CR1);
    uint32_t cr3its   = READ_REG(uart->_handle.Instance->CR3);

    uint32_t errorflags;
    uint32_t dmarequest = 0x00U;

    /* If no error occurs */
    errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    if (errorflags == RESET) {
        /* UART in mode Receiver ---------------------------------------------------*/
        if (((isrflags & USART_SR_RXNE) != 0U) &&
            ((cr1its & USART_CR1_RXNEIE) != 0U)) {
            UART_IncomingDataInterrupt(uart);
            return;
        }
    }

    /* If some errors occur */
    if ((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
    {
	    /* UART parity error interrupt occurred ----------------------------------*/
	    if (((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
	    {
	    	uart->_handle.ErrorCode |= HAL_UART_ERROR_PE;
	    }

	    /* UART noise error interrupt occurred -----------------------------------*/
	    if (((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
	    {
	    	uart->_handle.ErrorCode |= HAL_UART_ERROR_NE;
	    }

	    /* UART frame error interrupt occurred -----------------------------------*/
	    if (((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
	    	uart->_handle.ErrorCode |= HAL_UART_ERROR_FE;
	    }

	    /* UART Over-Run interrupt occurred --------------------------------------*/
	    if (((isrflags & USART_SR_ORE) != RESET) && (((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET)))
	    {
	    	uart->_handle.ErrorCode |= HAL_UART_ERROR_ORE;
	    }

        /* Call UART Error Call back function if need be --------------------------*/
        if (uart->_ErrorCode != UART_ERROR_NONE) {
            /* UART in mode Receiver ---------------------------------------------------*/
        	if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        	{
                UART_IncomingDataInterrupt(uart);
            }

            /* If Overrun error occurs, or if any error occurs in DMA mode reception,
               consider error as blocking */
            dmarequest = HAL_IS_BIT_SET(uart->_handle.Instance->CR3, USART_CR3_DMAR);
            if (((uart->_handle.ErrorCode & HAL_UART_ERROR_ORE) != RESET) || dmarequest)
            {
                /* Blocking error : transfer is aborted
                Set the UART state ready to be able to start again the process,
                Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
                // UART_EndRxTransfer(huart);

                /* Disable the UART DMA Rx request if enabled */
                if (HAL_IS_BIT_SET(uart->_handle.Instance->CR3, USART_CR3_DMAR)) {
                    CLEAR_BIT(uart->_handle.Instance->CR3, USART_CR3_DMAR);

                    /* Abort the UART DMA Rx channel */
                    // ToDo: Implement DMA abort (if DMA ends up being used)
                    // No, we should never come here since UART Error Interrupts are currently not enabled during DMA
                } else {
                    /* Call user error callback */
                    // HAL_UART_ErrorCallback(huart);
                }
            } else {
                /* Non Blocking error : transfer could go on.
                Error is notified to user through user error callback */
                // HAL_UART_ErrorCallback(huart);
                uart->_ErrorCode = UART_ERROR_NONE;
            }
        }
        return;
    } /* End if some error occurs */

    /* UART in mode Transmitter ------------------------------------------------*/
    if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)) {
    	__HAL_UART_CLEAR_FLAG(&uart->_handle, UART_FLAG_TXE);
        if (uart->_txRemainingBytes > 0) { // interrupt based transmission
            if (uart->_txRemainingBytes == 1) {
                /* Disable TXE interrupt */
                __HAL_UART_DISABLE_IT(&uart->_handle, UART_IT_TXE);

                /* Enable TC interrupt */
                __HAL_UART_ENABLE_IT(&uart->_handle, UART_IT_TC);
            }

            // Transmit the next byte from the buffer
            uart->_handle.Instance->DR = *uart->_txPointer++;
            uart->_txRemainingBytes--;
        } else { // polling based transmission
            /* Disable the Transmit Data Register Empty interrupt */
            __HAL_UART_DISABLE_IT(&uart->_handle, UART_IT_TXE);

#ifdef USE_FREERTOS
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(uart->_transmitFinished, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
            uart->_transmitFinished = true;
#endif
        }
        return;
    }

    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)) {
    	__HAL_UART_CLEAR_FLAG(&uart->_handle, UART_FLAG_TC);
        __HAL_UART_DISABLE_IT(&uart->_handle, UART_IT_TC);

#ifdef USE_FREERTOS
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(uart->_transmitFinished, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
        uart->_transmitFinished = true;
#endif
        return;
    }

    /* Check for IDLE line interrupt */
    if (((isrflags & USART_SR_IDLE) != 0U) && ((cr1its & USART_CR1_IDLEIE) != 0U)) {
    	//__HAL_UART_CLEAR_FLAG(&uart->_handle, UART_FLAG_IDLE);
    	// Has to use clear macro (see https://community.st.com/s/question/0D50X00009XkXxk/stm32f205-usart-idle-interrupt-fires-for-no-reason)
    	__HAL_UART_CLEAR_IDLEFLAG(&uart->_handle);
        // ToDo: If DMA is enabled, then check for new data here
        if (uart->DMA_Enabled) {
            UART::DMA_RX_Check(uart);
        }
        if (uart->_rxAvailableOnIdle) {
#ifdef USE_FREERTOS
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

            if (uart->Available()) {
                if (uart->_RXcallback && uart->_callbackTaskHandle) {
                    portBASE_TYPE xHigherPriorityTaskWoken = xTaskResumeFromISR(uart->_callbackTaskHandle);
                    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                }

                if (uart->_RXdataAvailable)
                    xSemaphoreGiveFromISR(uart->_RXdataAvailable, &xHigherPriorityTaskWoken);
            }
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#else
            if (uart->_RXcallback) {
                if (!uart->_bufferLength) { // buffer not enabled - use raw byte reading
                    uart->_RXcallback(uart->_RXcallbackParameter, &uart->rxByte, 1);
                }
            }
#endif
        }
        return;
    }
}

void UART::DMA_RX_Interrupt(UART* uart)
{
    if (!uart->DMA_Enabled) {
        ERROR("DMA not enabled for this UART");
        return;
    }

    HAL_DMA_IRQHandler(uart->_handle.hdmarx);
}

void UART::DMA_TX_Interrupt(UART* uart)
{
    if (!uart->DMA_Enabled) {
        ERROR("DMA not enabled for this UART");
        return;
    }

    HAL_DMA_IRQHandler(uart->_handle.hdmatx);
}

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void)
{
    if (!UART::objUART2) {
        ERROR("UART DMA interrupt for unconfigured port");
        return;
    }

    UART::DMA_RX_Interrupt(UART::objUART2);
}

/**
 * @brief This function handles DMA1 channel4 global interrupt.
 */
void DMA1_Stream6_IRQHandler(void)
{
    if (!UART::objUART2) {
        ERROR("UART DMA interrupt for unconfigured port");
        return;
    }

    UART::DMA_TX_Interrupt(UART::objUART2);
}

void UART::DMATransmitCplt(DMA_HandleTypeDef* hdma)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    if (huart->Instance == USART2 && UART::objUART2) {
        UART::DMA_TX_Completed(UART::objUART2);
    } else {
        ERROR("UART interrupt for unconfigured port");
        return;
    }
}

void UART::DMAError(DMA_HandleTypeDef* hdma)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    if (huart->Instance == USART2 && UART::objUART2) {
        UART::DMA_Error(UART::objUART2);
    } else {
        ERROR("UART interrupt for unconfigured port");
        return;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* UartHandle)
{
    if (UartHandle->Instance == USART2 && UART::objUART2) {
        UART::DMA_TX_Completed(UART::objUART2);
    } else {
        ERROR("UART interrupt for unconfigured port");
        return;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* UartHandle)
{
    if (UartHandle->Instance == USART2 && UART::objUART2) {
        UART::DMA_RX_Check(UART::objUART2);
    } else {
        ERROR("UART interrupt for unconfigured port");
        return;
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* UartHandle)
{
    if (UartHandle->Instance == USART2 && UART::objUART2) {
        UART::DMA_RX_Check(UART::objUART2);
    } else {
        ERROR("UART interrupt for unconfigured port");
        return;
    }
}

bool UART::Available()
{
    if (!_bufferLength)
        return _rxByteAvailable; // error, buffer not enabled
    return (_bufferWriteIdx != _bufferReadIdx);
}

uint32_t UART::AvailableLength()
{
    return BufferContentSize();
}

uint32_t UART::BufferContentSize()
{
    if (!_bufferLength)
        return _rxByteAvailable; // error, buffer not enabled
    uint32_t length = (uint32_t)(((int32_t)_bufferWriteIdx - (int32_t)_bufferReadIdx) % (int32_t)_bufferLength);

    return length;
}

void UART::BufferPush(uint8_t byte)
{
    if (DMA_Enabled)
        return; // Buffer push can not be used in DMA mode
    if (!_bufferLength) {
        rxByte           = byte;
        _rxByteAvailable = true;
        return; // error, buffer not enabled
    }
    if (BufferContentSize() >= _bufferLength)
        return; // error, buffer is full

    _buffer[_bufferWriteIdx] = byte;
    _bufferWriteIdx++;
    if (_bufferWriteIdx == _bufferLength)
        _bufferWriteIdx = 0;
}

uint8_t UART::BufferPop()
{
    if (!_bufferLength) {
        if (_rxByteAvailable) {
            _rxByteAvailable = false;
            return rxByte;
        }
        return 0; // error, buffer not enabled
    }
    if (BufferContentSize() == 0)
        return 0; // error, buffer is empty

    uint8_t byte = _buffer[_bufferReadIdx];
    _bufferReadIdx++;
    if (_bufferReadIdx == _bufferLength)
        _bufferReadIdx = 0;

    return byte;
}

// Pop as many bytes as possible
uint32_t UART::BufferPopMax(uint8_t* buffer, uint32_t bufferSize)
{
    uint32_t poppedBytes = 0;
    if (!buffer)
        return 0;

    while (Available() && poppedBytes < bufferSize) {
        buffer[poppedBytes] = BufferPop();
        poppedBytes++;
    }

    return poppedBytes;
}

uint8_t* UART::BufferPopN(uint32_t numberOfBytesToPop)
{
    uint8_t* popBuffer = 0;
    if (numberOfBytesToPop > BufferContentSize())
        return 0; // error, not enough content in buffer
#ifdef USE_FREERTOS
    popBuffer = (uint8_t*)pvPortMalloc(numberOfBytesToPop);
#else
    popBuffer = (uint8_t*)malloc(numberOfBytesToPop);
#endif

    if (popBuffer) {
        for (uint32_t i = 0; i < numberOfBytesToPop; i++) {
            popBuffer[i] = BufferPop();
        }
    }

    return popBuffer;
}
