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
 
#include "USBCDC.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h" // for USB processing task
#include "Debug.h"

#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

USBCDC * USBCDC::usbHandle = 0;
USBD_HandleTypeDef USBCDC::hUsbDeviceFS;
PCD_HandleTypeDef USBCDC::hpcd_USB_OTG_FS;

// Necessary to export for compiler to generate code to be called by interrupt vector
extern "C" __EXPORT void OTG_FS_IRQHandler(void);

USBCDC::USBCDC(uint32_t transmitterTaskPriority) : _processingTaskHandle(0), _TXfinishedSemaphore(0), _RXdataAvailable(0), _TXqueue(0), _RXqueue(0), _connected(false)
{
	if (usbHandle) {
		ERROR("USB object already created");
		return;
	}

	_tmpPackageForRead.length = 0;
	_readIndex = 0;

	_resourceSemaphore = xSemaphoreCreateBinary();
	if (_resourceSemaphore == NULL) {
		ERROR("Could not create USBCDC resource semaphore");
		return;
	}
	vQueueAddToRegistry(_resourceSemaphore, "USBCDC Resource");

	_TXqueue = xQueueCreate( USBCDC_TX_QUEUE_LENGTH, sizeof(USB_CDC_Package_t) );
	if (_TXqueue == NULL) {
		ERROR("Could not create USBCDC TX queue");
		return;
	}
	vQueueAddToRegistry(_TXqueue, "USB TX");
	CDC_RegisterReceiveQueue(_TXqueue);

	_RXqueue = xQueueCreate( USBCDC_RX_QUEUE_LENGTH, sizeof(USB_CDC_Package_t) );
	if (_RXqueue == NULL) {
		ERROR("Could not create USBCDC RX queue");
		return;
	}
	vQueueAddToRegistry(_RXqueue, "USB RX");
	CDC_RegisterReceiveQueue(_RXqueue);

	_RXdataAvailable = xSemaphoreCreateBinary();
	if (_RXdataAvailable == NULL) {
		ERROR("Could not create USBCDC RX available semaphore");
		return;
	}
	vQueueAddToRegistry(_RXdataAvailable, "USB RX Available");
	CDC_RegisterRXsemaphore(_RXdataAvailable);

	_TXfinishedSemaphore = xSemaphoreCreateBinary();
	if (_TXfinishedSemaphore == NULL) {
		ERROR("Could not create USBCDC TX semaphore");
		return;
	}
	vQueueAddToRegistry(_TXfinishedSemaphore, "USB TX Finished");
	USBD_CDC_SetTXfinishedSemaphore(_TXfinishedSemaphore);

	/* Init USB Device Library, add supported class and start the library. */
	USBD_LL_SetPCD(&USBCDC::hpcd_USB_OTG_FS);
	USBD_Init(&USBCDC::hUsbDeviceFS, &FS_Desc, DEVICE_FS);
	USBD_RegisterClass(&USBCDC::hUsbDeviceFS, &USBD_CDC);
	CDC_RegisterUsbDeviceObject(&USBCDC::hUsbDeviceFS);
	USBD_CDC_RegisterInterface(&USBCDC::hUsbDeviceFS, &USBD_Interface_fops_FS);
	USBD_Start(&USBCDC::hUsbDeviceFS);

	xTaskCreate(USBCDC::TransmitterThread, (char *)"USB transmitter", USBCDC_TX_PROCESSING_THREAD_STACK_SIZE, (void*) this, transmitterTaskPriority, &_processingTaskHandle);

	usbHandle = this;
}

USBCDC::~USBCDC()
{
	if (_processingTaskHandle)
		vTaskDelete(_processingTaskHandle); // stop task

	if (_resourceSemaphore) {
		vQueueUnregisterQueue(_resourceSemaphore);
		vSemaphoreDelete(_resourceSemaphore);
	}

	if (_RXdataAvailable) {
		vQueueUnregisterQueue(_RXdataAvailable);
		vSemaphoreDelete(_RXdataAvailable);
	}

	if (_TXfinishedSemaphore) {
		vQueueUnregisterQueue(_TXfinishedSemaphore);
		vSemaphoreDelete(_TXfinishedSemaphore);
	}

	if (_TXfinishedSemaphore) {
		vQueueUnregisterQueue(_TXfinishedSemaphore);
		vSemaphoreDelete(_TXfinishedSemaphore);
	}

	if (_TXqueue) {
		vQueueUnregisterQueue(_TXqueue);
		vQueueDelete(_TXqueue);
	}

	if (_RXqueue) {
		vQueueUnregisterQueue(_RXqueue);
		vQueueDelete(_RXqueue);
	}

	USBD_Stop(&USBCDC::hUsbDeviceFS);
	usbHandle = NULL;
}

bool USBCDC::GetPackage(USB_CDC_Package_t * packageBuffer)
{
	if (!packageBuffer) return false;

	if ( xQueueReceive( _RXqueue, packageBuffer, ( TickType_t ) 0 ) == pdPASS )
		return true;
	else
		return false;
}

void USBCDC::Write(uint8_t byte)
{
	USB_CDC_Package_t package;

	if (xSemaphoreTake( _resourceSemaphore, ( TickType_t ) 1 ) != pdTRUE ) return; // take hardware resource - wait maximum 1 ms on this
	package.data[0] = byte;
	package.length = 1;
	xQueueSend(_TXqueue, (void *)&package, (TickType_t) 1);
	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
}

uint32_t USBCDC::Write(uint8_t * buffer, uint32_t length)
{
	USB_CDC_Package_t package;

	uint32_t txLength = length;
	uint8_t packageLength;

	//if (xSemaphoreTake( _resourceSemaphore, ( TickType_t ) 1 ) != pdTRUE ) return 0; // take hardware resource - wait max 1 ms

	// Split buffer data into packages
	while (txLength > 0) {
		if (txLength > USB_PACKAGE_MAX_SIZE)
			packageLength = USB_PACKAGE_MAX_SIZE;
		else
			packageLength = txLength;

		memcpy(package.data, buffer, packageLength);
		package.length = packageLength;

		xQueueSend(_TXqueue, (void *)&package, (TickType_t) 0);

		buffer += packageLength;
		txLength -= packageLength;
	}

	//xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

	return length;
}

uint32_t USBCDC::WriteBlocking(uint8_t * buffer, uint32_t length)
{
	if (!_connected) return 0;

	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY); // take hardware resource

	if (CDC_IsConnected()) {
		if (CDC_Transmit_FS_ThreadBlocking(buffer, length) != USBD_OK) {
			xSemaphoreGive( _resourceSemaphore ); // give hardware resource back
			return 0; // error, could not transmit package
		}
	}

	xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

	return length;
}

int16_t USBCDC::Read()
{
	uint8_t returnValue;

	if (_readIndex == _tmpPackageForRead.length) { // load in new package for reading (if possible)
		_tmpPackageForRead.length = 0;
		_readIndex = 0;
		if ( xQueueReceive( _RXqueue, &_tmpPackageForRead, ( TickType_t ) 1 ) != pdPASS ) {
			return -1; // no new package
		}
	}

	returnValue = _tmpPackageForRead.data[_readIndex];
	_readIndex++;

	return returnValue;
}

bool USBCDC::Available()
{
	if (_readIndex != _tmpPackageForRead.length || uxQueueMessagesWaiting(_RXqueue) > 0)
		return true;
	else
		return false;
}

uint32_t USBCDC::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
	return xSemaphoreTake( _RXdataAvailable, ( TickType_t ) xTicksToWait );
}

bool USBCDC::Connected()
{
	return _connected;
}

void USBCDC::TransmitterThread(void * pvParameters)
{
	USB_CDC_Package_t package;
	USBCDC * usb = (USBCDC *)pvParameters;

	xSemaphoreGive( usb->_resourceSemaphore ); // give the semaphore the first time

	while (1) {
		usb->_connected = false;
		xSemaphoreTake( usb->_resourceSemaphore, ( TickType_t ) portMAX_DELAY); // take hardware resource

		while (!CDC_IsConnected()) {
			xQueueReset(usb->_TXqueue);
			osDelay(1);
		}

		// Wait for the USB connection to be ready
		// Send initial zero package - wait for communication channel to be opened
		memset(package.data, 0, USB_PACKAGE_MAX_SIZE);
		while (CDC_Transmit_FS(package.data, USB_PACKAGE_MAX_SIZE) != USBD_OK) {
			xQueueReset(usb->_TXqueue);
			osDelay(1);
		}

		xSemaphoreGive( usb->_resourceSemaphore ); // give hardware resource back

		usb->_connected = true;

		// Transmit processing loop
		memset(package.data, 0, USB_PACKAGE_MAX_SIZE);
		while (CDC_IsConnected()) {
			if ( xQueueReceive( usb->_TXqueue, &package, ( TickType_t ) 100 ) == pdPASS ) { // check USB connection every 100 ms
				xSemaphoreTake( usb->_resourceSemaphore, ( TickType_t ) portMAX_DELAY); // take hardware resource
				if (CDC_Transmit_FS_ThreadBlocking(package.data, package.length) != USBD_OK) {
					xSemaphoreGive( usb->_resourceSemaphore ); // give hardware resource back
					break; // disconnected or other problem
				}
				xSemaphoreGive( usb->_resourceSemaphore ); // give hardware resource back
			}
		}
	}
}

void OTG_FS_IRQHandler(void)
{
	HAL_PCD_IRQHandler(&USBCDC::hpcd_USB_OTG_FS);
}
