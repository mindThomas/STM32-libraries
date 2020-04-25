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
 
#ifndef PERIPHIRALS_CANBUS_H
#define PERIPHIRALS_CANBUS_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

//#include <algorithm>
//#include <array>
//#include <cstddef>
//#include <cstdint>
#include <unordered_map>
//#include <stdexcept>
//#include <vector>

#ifdef USE_FREERTOS
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback
#endif

class CANBus
{
	private:
		const int CAN_RX_PROCESSING_THREAD_STACK_SIZE = 128;
		const int CAN_RX_QUEUE_LENGTH = 3;
		const uint16_t TX_TIMEOUT = 500; // milliseconds

	public:
		typedef struct package_t {
			uint32_t Timestamp;
			uint16_t ID;
			uint8_t DataLength;
			uint8_t Data[8];
		} package_t;

		// Map of callback functions to handle the incoming messages
		typedef struct callback_t {
			void (*handler)(void * param, const package_t&){0};
			void * param{0};
		} callback_t;

	public:
		CANBus();
		~CANBus();

		void InitPeripheral();
		void DeInitPeripheral();
		void ConfigurePeripheral();

		bool Transmit(uint32_t ID, uint8_t * Payload, uint8_t payloadLength);

		bool registerCallback(uint16_t ID, void (*handler)(void * param, const package_t& package), void * parameter = 0);
		bool unregisterCallback(uint16_t ID);

	public:
		typedef struct hardware_resource_t {
	#ifdef USE_FREERTOS
			SemaphoreHandle_t resourceSemaphore;
			SemaphoreHandle_t transmissionFinished;
			TaskHandle_t RXprocessingTaskHandle;
			QueueHandle_t RXqueue;
	#else
			bool resourceSemaphore;
			bool transmissionFinished;
	#endif
			bool configured;
			uint8_t instances; // how many objects are using this hardware resource
			FDCAN_HandleTypeDef handle;
			FDCAN_TxHeaderTypeDef   TxHeader;
			FDCAN_RxHeaderTypeDef   RxHeader;
			package_t			  RxPackage;
			uint32_t              TxMailbox;
			std::unordered_map<uint16_t, callback_t> callbackHandlers; // map from ID to callback function
		} hardware_resource_t;

		static hardware_resource_t * resCAN;

	private:
		hardware_resource_t * _hRes;

	public:
		static void ProcessingThread(void * pvParameters);
		static void TXFinishedInterrupt(hardware_resource_t * can);

		static uint8_t GetPackageLength(uint32_t DLC);
		static uint32_t ConvertToPackageLength(uint8_t payloadLength);
};
	
	
#endif
