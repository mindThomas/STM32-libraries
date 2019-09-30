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
 
#ifndef PERIPHIRALS_CANBUS_H
#define PERIPHIRALS_CANBUS_H

#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_exti.h"
//#include "stm32f0xx_ll_dma.h"

#ifdef USE_FREERTOS
#include "cmsis_os.h" // for memory allocation (for the buffer) and callback
#endif

class CANBus
{
	public:
		CANBus();
		~CANBus();

		void InitPeripheral();
		void DeInitPeripheral();
		void ConfigurePeripheral();

		bool Transmit(uint32_t ID, uint8_t * Payload, uint8_t payloadLength);

	public:
		typedef struct hardware_resource_t {
	#ifdef USE_FREERTOS
			SemaphoreHandle_t resourceSemaphore;
	#endif
			bool configured;
			uint8_t instances; // how many objects are using this hardware resource
			CAN_HandleTypeDef handle;
			CAN_TxHeaderTypeDef   TxHeader;
			CAN_RxHeaderTypeDef   RxHeader;
			uint8_t               RxData[8];
			uint32_t              TxMailbox;
		} hardware_resource_t;

		static hardware_resource_t * resCAN;

	private:
		hardware_resource_t * _hRes;

	public:
};
	
	
#endif
