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
 
#include "CANBus.h"
#include "stm32g4xx_hal_timebase_tim.h" // for HAL_GetHighResTick()

#include "Priorities.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

CANBus::hardware_resource_t * CANBus::resCAN = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void FDCAN1_IT0_IRQHandler(void);
extern "C" __EXPORT void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
extern "C" __EXPORT void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes);

CANBus::CANBus()
{
	InitPeripheral();
	ConfigurePeripheral();
}

CANBus::~CANBus()
{
	if (!_hRes) return;

	_hRes->instances--;
	if (_hRes->instances == 0) { // deinitialize port and periphiral
		DeInitPeripheral();

		// Delete hardware resource
		delete(_hRes);
		resCAN = 0;
	}
}

void CANBus::DeInitPeripheral()
{
	if (!_hRes) return;
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PB9     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* FDCAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
}

void CANBus::InitPeripheral()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool firstTime = false;

	if (!resCAN) {
		resCAN = new CANBus::hardware_resource_t;
		memset(resCAN, 0, sizeof(CANBus::hardware_resource_t));
		resCAN->handle.Instance = FDCAN1;
		firstTime = true;
	}
	_hRes = resCAN;

	if (firstTime) { // first time configuring peripheral
		_hRes->configured = false;
		_hRes->instances = 0;
		
	#ifdef USE_FREERTOS
		_hRes->resourceSemaphore = xSemaphoreCreateBinary();
		if (_hRes->resourceSemaphore == NULL) {
			_hRes = 0;
			ERROR("Could not create CAN resource semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->resourceSemaphore, "CAN Resource");
		xSemaphoreGive( _hRes->resourceSemaphore ); // give the semaphore the first time

		_hRes->transmissionFinished = xSemaphoreCreateBinary();
		if (_hRes->transmissionFinished == NULL) {
			_hRes = 0;
			ERROR("Could not create CAN transmission semaphore");
			return;
		}
		vQueueAddToRegistry(_hRes->transmissionFinished, "CAN Finish");
		xSemaphoreGive( _hRes->transmissionFinished ); // ensure that the semaphore is not taken from the beginning
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY ); // ensure that the semaphore is not taken from the beginning

		_hRes->RXqueue = xQueueCreate( CAN_RX_QUEUE_LENGTH, sizeof(package_t) );
		if (_hRes->RXqueue == NULL) {
			ERROR("Could not create CAN RX queue");
			return;
		}
		vQueueAddToRegistry(_hRes->RXqueue, "CAN RX");

		xTaskCreate(CANBus::ProcessingThread, (char *)"CAN receiver", CAN_RX_PROCESSING_THREAD_STACK_SIZE, (void*) _hRes, CAN_RECEIVER_PRIORITY, &_hRes->RXprocessingTaskHandle);
	#else
		_hRes->transmissionFinished = true;
		_hRes->resourceSemaphore = false;
	#endif
		
		// Configure pins for CAN and CAN peripheral accordingly
	    /* Peripheral clock enable */
	    __HAL_RCC_FDCAN_CLK_ENABLE();

	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	    /**FDCAN1 GPIO Configuration
	    PA11     ------> FDCAN1_RX
	    PB9     ------> FDCAN1_TX
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_11;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /* FDCAN1 interrupt Init */
	    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, CAN_INTERRUPT_PRIORITY, 0);
	    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	}

	_hRes->instances++;
}

void CANBus::ConfigurePeripheral()
{
	if (!_hRes) return;
	if (_hRes->configured) return; // only configure periphiral once

	// Configure CAN to 100 kbit/s   (see CAN Timing.xmcd)
	// Assumes a 168 MHz CAN peripheral clock
	_hRes->handle.Init.ClockDivider = FDCAN_CLOCK_DIV28;
	_hRes->handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	_hRes->handle.Init.Mode = FDCAN_MODE_NORMAL;
	_hRes->handle.Init.AutoRetransmission = DISABLE;
	_hRes->handle.Init.TransmitPause = DISABLE;
	_hRes->handle.Init.ProtocolException = DISABLE;
	_hRes->handle.Init.NominalPrescaler = 4;
	_hRes->handle.Init.NominalSyncJumpWidth = 1;
	_hRes->handle.Init.NominalTimeSeg1 = 7;
	_hRes->handle.Init.NominalTimeSeg2 = 7;
	_hRes->handle.Init.DataPrescaler = 4;
	_hRes->handle.Init.DataSyncJumpWidth = 1;
	_hRes->handle.Init.DataTimeSeg1 = 7;
	_hRes->handle.Init.DataTimeSeg2 = 7;
	_hRes->handle.Init.StdFiltersNbr = 0;
	_hRes->handle.Init.ExtFiltersNbr = 0;
	_hRes->handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	if (HAL_FDCAN_Init(&_hRes->handle) != HAL_OK)
	{
		/* Initialization Error */
		ERROR("Could not initialize CAN bus");
	}

	/* Configure reception filter to Rx FIFO 0 on both FDCAN instances */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;
	if (HAL_FDCAN_ConfigFilter(&_hRes->handle, &sFilterConfig) != HAL_OK)
	{
		ERROR("Error configuring CAN bus filter");
	}

	/* Configure global filter on both FDCAN instances:
	Filter all remote frames with STD and EXT ID
	Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&_hRes->handle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		ERROR("Error configuring CAN bus filter");
	}

	/* Activate Rx FIFO 0 new message notification */
	if (HAL_FDCAN_ActivateNotification(&_hRes->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		ERROR("Failed activating CAN RX/TX notification");
	}
	if (HAL_FDCAN_ConfigInterruptLines(&_hRes->handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK)
	{
		ERROR("Failed configuring CAN RX/TX notification");
	}

	/* Activate Tx Completed notification */
	if (HAL_FDCAN_ActivateNotification(&_hRes->handle, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
	{
		ERROR("Failed activating CAN RX/TX notification");
	}
	if (HAL_FDCAN_ConfigInterruptLines(&_hRes->handle, FDCAN_IT_TX_COMPLETE, FDCAN_INTERRUPT_LINE0) != HAL_OK)
	{
		ERROR("Failed configuring CAN RX/TX notification");
	}



	/* Configure and enable Tx Delay Compensation, required for BRS mode.
	TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
	TdcFilter default recommended value: 0 */
	/*if (HAL_FDCAN_ConfigTxDelayCompensation(&_hRes->handle, 5, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(&_hRes->handle) != HAL_OK)
	{
		Error_Handler();
	}*/

	/* Start the FDCAN module on both FDCAN instances */
	if (HAL_FDCAN_Start(&_hRes->handle) != HAL_OK)
	{
		ERROR("Error starting CAN periphiral");
	}

	_hRes->configured = true;
}

bool CANBus::Transmit(uint32_t ID, uint8_t * Payload, uint8_t payloadLength)
{
	bool success = false;
	if (!_hRes) return success;
	
#ifdef USE_FREERTOS
	xSemaphoreTake( _hRes->resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource
#else
	if (_hRes->resourceSemaphore) return success; // resource already in use
	_hRes->resourceSemaphore = true; // put resource in use
#endif

	// Check that we can push a message to the CAN Mailbox, otherwise wait
	// Make sure that CAN bus is terminated, otherwise the TX mailboxes will fill up and the task will hang here!
	if (HAL_FDCAN_GetTxFifoFreeLevel(&_hRes->handle) == 0) {
#ifdef USE_FREERTOS
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) TX_TIMEOUT );
#else
		uint16_t timeout = TX_TIMEOUT;
		while (!_hRes->transmissionFinished && timeout > 0) {
			HAL_Delay(1);
			timeout--;
		}
#endif
	} else {
#ifdef USE_FREERTOS
		// Reset transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) 0 );
#else
		_hRes->transmissionFinished = false;
#endif
	}

	if (payloadLength > 8) return success; // error, payload length too long
	if (ID > 0x7FF) return success; // error, identifier out of bounds (only 11 bits available)

	// Prepare CAN message
	_hRes->TxHeader.Identifier = ID;
	_hRes->TxHeader.IdType = FDCAN_STANDARD_ID;
	_hRes->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	_hRes->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	_hRes->TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	_hRes->TxHeader.FDFormat = FDCAN_FD_CAN;
	_hRes->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	_hRes->TxHeader.MessageMarker = 0;
	_hRes->TxHeader.DataLength = ConvertToPackageLength(payloadLength);
	
    /* Set the data to be transmitted */
	uint8_t TxData[8];
	memset(TxData, 0, 8);
	memcpy(TxData, Payload, payloadLength);

    /* Start the Transmission process */
	HAL_StatusTypeDef errCode = HAL_FDCAN_AddMessageToTxFifoQ(&_hRes->handle, &_hRes->TxHeader, TxData);
    if (errCode == HAL_OK)
    {
    	success = true;
    }

#ifdef USE_FREERTOS
	xSemaphoreGive( _hRes->resourceSemaphore ); // give hardware resource back
#else
	_hRes->resourceSemaphore = false; // finished using resource
#endif

	return success;
}

bool CANBus::registerCallback(uint16_t ID, void (*handler)(void * param, const package_t& package), void * parameter)
{
	if (_hRes->callbackHandlers[ID].handler)
		return false; // callback already registered - this ensures that we can not overwrite an existing registered callback

	callback_t callback;
	callback.handler = handler;
	callback.param = parameter;

	_hRes->callbackHandlers[ID] = callback;

	return true;
}

bool CANBus::unregisterCallback(uint16_t ID)
{
	if (_hRes->callbackHandlers[ID].handler)
		return false; // callback not registered already registered - this ensures that we can not overwrite an existing registered callback

	_hRes->callbackHandlers.erase(_hRes->callbackHandlers.find(ID)); // remove/unregister the callback

	return true;
}

#ifdef USE_FREERTOS
void CANBus::ProcessingThread(void * pvParameters)
{
	hardware_resource_t * can = (hardware_resource_t *)pvParameters;
	package_t package;

	// CAN incoming data processing loop
	while (1)
	{
		if ( xQueueReceive( can->RXqueue, &package, ( TickType_t ) portMAX_DELAY ) == pdPASS ) {
			std::unordered_map<uint16_t, callback_t>::iterator it = can->callbackHandlers.find(package.ID);
			if (it != can->callbackHandlers.end()) {
				if (it->second.handler)
					it->second.handler(it->second.param, package);
			}
		}
	}
}
#endif

uint8_t CANBus::GetPackageLength(uint32_t DLC)
{
	switch (DLC)
	{
		case FDCAN_DLC_BYTES_0: return 0;
		case FDCAN_DLC_BYTES_1: return 1;
		case FDCAN_DLC_BYTES_2: return 2;
		case FDCAN_DLC_BYTES_3: return 3;
		case FDCAN_DLC_BYTES_4: return 4;
		case FDCAN_DLC_BYTES_5: return 5;
		case FDCAN_DLC_BYTES_6: return 6;
		case FDCAN_DLC_BYTES_7: return 7;
		case FDCAN_DLC_BYTES_8: return 8;
		case FDCAN_DLC_BYTES_12: return 12;
		case FDCAN_DLC_BYTES_16: return 16;
		case FDCAN_DLC_BYTES_20: return 20;
		case FDCAN_DLC_BYTES_24: return 24;
		case FDCAN_DLC_BYTES_32: return 32;
		case FDCAN_DLC_BYTES_48: return 48;
		case FDCAN_DLC_BYTES_64: return 64;
		default: return 0;
	}
}

uint32_t CANBus::ConvertToPackageLength(uint8_t payloadLength)
{
	if (payloadLength <= 8) {
		return ((uint32_t)payloadLength) * 0x00010000U;
	}
	else {
		switch (payloadLength) {
			case 12: return FDCAN_DLC_BYTES_12;
			case 16: return FDCAN_DLC_BYTES_16;
			case 20: return FDCAN_DLC_BYTES_20;
			case 24: return FDCAN_DLC_BYTES_24;
			case 32: return FDCAN_DLC_BYTES_32;
			case 48: return FDCAN_DLC_BYTES_48;
			case 64: return FDCAN_DLC_BYTES_64;
			default: return 0;
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if (hfdcan->Instance != FDCAN1 && CANBus::resCAN) return; // error/unconfigured
	CANBus::hardware_resource_t * can = CANBus::resCAN;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Get RX message */
	uint8_t data[64];
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can->RxHeader, data) == HAL_OK)
	{
		if (can->RxHeader.IdType == FDCAN_STANDARD_ID) {
			// Convert received frame into CAN package type
			can->RxPackage.ID = can->RxHeader.Identifier;
			can->RxPackage.DataLength = CANBus::GetPackageLength(can->RxHeader.DataLength);
			can->RxPackage.Timestamp = HAL_GetHighResTick(); // could also be can->RxHeader.Timestamp
			memcpy(can->RxPackage.Data, data, can->RxPackage.DataLength);

		#ifdef USE_FREERTOS
			if (can->RXqueue)
				  xQueueSendFromISR(can->RXqueue, (void *)&can->RxPackage, &xHigherPriorityTaskWoken);
		#endif
		}
	} else {
		/* Reception Error */
		ERROR("Reception error");
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	if (hfdcan->Instance == FDCAN1 && CANBus::resCAN)
		CANBus::TXFinishedInterrupt(CANBus::resCAN);
}

void CANBus::TXFinishedInterrupt(CANBus::hardware_resource_t * can)
{
	if (!can) return;

	// Transmission finished
#ifdef USE_FREERTOS
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( can->transmissionFinished, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
	can->transmissionFinished = true;
#endif
}

void FDCAN1_IT0_IRQHandler(void)
{
	if (CANBus::resCAN)
		HAL_FDCAN_IRQHandler(&CANBus::resCAN->handle);
}
