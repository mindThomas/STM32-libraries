/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
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
#include "stm32f0xx_hal_timebase_tim.h" // for HAL_GetHighResTick()

#include "Priorities.h"
#include "Debug.h"
#include <math.h>
#include <string.h> // for memset

CANBus::hardware_resource_t * CANBus::resCAN = 0;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void CEC_CAN_IRQHandler(void);
extern "C" __EXPORT void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
extern "C" __EXPORT void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
extern "C" __EXPORT void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
extern "C" __EXPORT void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);

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
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_CAN);

	// ToDo: Deinit GPIO pins

	/* CAN interrupt DeInit */
	HAL_NVIC_DisableIRQ(CEC_CAN_IRQn);
}

void CANBus::InitPeripheral()
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	bool firstTime = false;

	if (!resCAN) {
		resCAN = new CANBus::hardware_resource_t;
		memset(resCAN, 0, sizeof(CANBus::hardware_resource_t));
		resCAN->handle.Instance = CAN;
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
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_CAN);

		/**CAN GPIO Configuration
		PB8     ------> CAN_RX
		PB9     ------> CAN_TX
		*/
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		GPIO_InitStruct.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
		GPIO_InitStruct.Speed = LL_GPIO_SPEED_HIGH;
		GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
		GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
		GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
		GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* I2C1 interrupt Init */
		NVIC_SetPriority(CEC_CAN_IRQn, CAN_INTERRUPT_PRIORITY);
		NVIC_EnableIRQ(CEC_CAN_IRQn);
	}

	_hRes->instances++;
}

void CANBus::ConfigurePeripheral()
{
	CAN_FilterTypeDef  sFilterConfig;

	if (!_hRes) return;
	if (_hRes->configured) return; // only configure periphiral once

	/*##-1- Configure the CAN peripheral #######################################*/
	_hRes->handle.Init.TimeTriggeredMode = DISABLE;
	_hRes->handle.Init.AutoBusOff = DISABLE;
	_hRes->handle.Init.AutoWakeUp = DISABLE;
	_hRes->handle.Init.AutoRetransmission = ENABLE;
	_hRes->handle.Init.ReceiveFifoLocked = DISABLE;
	_hRes->handle.Init.TransmitFifoPriority = DISABLE;
	_hRes->handle.Init.Mode = CAN_MODE_NORMAL;
	_hRes->handle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	_hRes->handle.Init.TimeSeg1 = CAN_BS1_5TQ;
	_hRes->handle.Init.TimeSeg2 = CAN_BS2_6TQ;
	_hRes->handle.Init.Prescaler = 4;

	if (HAL_CAN_Init(&_hRes->handle) != HAL_OK)
	{
		/* Initialization Error */
		ERROR("Could not initialize CAN bus");
	}

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&_hRes->handle, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		ERROR("Error configuring CAN bus filter");
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&_hRes->handle) != HAL_OK)
	{
		/* Start Error */
		ERROR("Error starting CAN periphiral");
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&_hRes->handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		ERROR("Failed activating CAN RX notification");
	}

	/*##-5- Activate CAN TX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&_hRes->handle, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		ERROR("Failed activating CAN TX notification");
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
	if (HAL_CAN_GetTxMailboxesFreeLevel(&_hRes->handle) == 0) {
#ifdef USE_FREERTOS
		// Wait for the transmission to finish
		xSemaphoreTake( _hRes->transmissionFinished, ( TickType_t ) portMAX_DELAY );
#else
		while (!_hRes->transmissionFinished);
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
	_hRes->TxHeader.StdId = ID;
	_hRes->TxHeader.ExtId = 0; // not using extended format
	_hRes->TxHeader.RTR = CAN_RTR_DATA;
	_hRes->TxHeader.IDE = CAN_ID_STD;
	_hRes->TxHeader.DLC = payloadLength; // length of the frame that will be transmitted
	_hRes->TxHeader.TransmitGlobalTime = DISABLE;
	
    /* Set the data to be transmitted */
	uint8_t TxData[8];
	memset(TxData, 0, 8);
	memcpy(TxData, Payload, payloadLength);

    /* Start the Transmission process */
	HAL_StatusTypeDef errCode = HAL_CAN_AddTxMessage(&_hRes->handle, &_hRes->TxHeader, TxData, &_hRes->TxMailbox);
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

bool CANBus::registerCallback(uint16_t ID, void (*handler)(void * param, const package_t&), void * parameter)
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
			callback_t handler = can->callbackHandlers[package.ID];
			if (handler.handler)
				handler.handler(handler.param, package);
		}
	}
}
#endif

void CEC_CAN_IRQHandler(void)
{
	if (CANBus::resCAN)
		HAL_CAN_IRQHandler(&CANBus::resCAN->handle);
}

/**
  * @brief  Transmission  complete callback in non blocking mode
  * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
/**
  * @brief  Rx Fifo 0 message pending callback
  * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan->Instance != CAN && CANBus::resCAN) return; // error/unconfigured
	CANBus::hardware_resource_t * can = CANBus::resCAN;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can->RxHeader, can->RxPackage.Data) == HAL_OK)
	{
		if (can->RxHeader.IDE == CAN_ID_STD) {
			// Convert received frame into CAN package type
			can->RxPackage.ID = can->RxHeader.StdId;
			can->RxPackage.DataLength = can->RxHeader.DLC;
			can->RxPackage.Timestamp = HAL_GetHighResTick(); // could also be can->RxHeader.Timestamp

		#ifdef USE_FREERTOS
			if (can->RXqueue)
				  xQueueSendFromISR(can->RXqueue, (void *)&can->RxPackage, &xHigherPriorityTaskWoken);
		#endif
		}
	} else {
		/* Reception Error */
		Error_Handler();
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANBus::TXFinishedInterrupt(CANBus::resCAN);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CANBus::TXFinishedInterrupt(CANBus::resCAN);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
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
