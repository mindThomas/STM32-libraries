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

#include "stm32h7xx_hal.h"

#include "Priorities.h"

// FreeRTOS for USB processing task
#ifdef USE_FREERTOS_CMSIS
#include <cmsis_os.h>
#elif defined(USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#endif

#include "usbd_conf.h"
#include "STM32_USB_Device_Library/Core/Inc/usbd_core.h"
#include "usbd_desc.h"
#include "STM32_USB_Device_Library/Class/CDC/Inc/usbd_cdc.h"
#include "usbd_cdc_if.h"


class USBCDC
{
	private:
		const int USBCDC_TX_PROCESSING_THREAD_STACK_SIZE = 256;
		const int USBCDC_TX_QUEUE_LENGTH = 10;
		const int USBCDC_RX_QUEUE_LENGTH = 10;

	public:
#ifdef USE_FREERTOS
		USBCDC(uint32_t transmitterTaskPriority = USBCDC_TRANSMITTER_PRIORITY);
#else
        USBCDC();
#endif
		~USBCDC();
#ifdef USE_FREERTOS
		bool GetPackage(USB_CDC_Package_t * packageBuffer);
#endif

#ifdef USE_FREERTOS
		void Write(uint8_t byte);
		uint32_t Write(uint8_t * buffer, uint32_t length);
#endif
		uint32_t WriteBlocking(uint8_t * buffer, uint32_t length);

#ifdef USE_FREERTOS // reading without FreeRTOS is not implemented yet
		int16_t Read();
		bool Available();
		uint32_t WaitForNewData(uint32_t xTicksToWait = portMAX_DELAY);
#endif
		bool Connected();

	private:
#ifdef USE_FREERTOS
    TaskHandle_t _processingTaskHandle;
#endif
		USB_CDC_Package_t _tmpPackageForRead;
		uint8_t _readIndex;
#ifdef USE_FREERTOS
		SemaphoreHandle_t _TXfinishedSemaphore;
		SemaphoreHandle_t _RXdataAvailable;
		QueueHandle_t _TXqueue;
		QueueHandle_t _RXqueue;
		SemaphoreHandle_t _resourceSemaphore;
#endif
		bool _connected;

#ifdef USE_FREERTOS
	private:
		static void TransmitterThread(void * pvParameters);
#endif

	public:
		static USBCDC * usbHandle;
		static USBD_HandleTypeDef hUsbDeviceFS;
		static PCD_HandleTypeDef hpcd_USB_OTG_FS;
};