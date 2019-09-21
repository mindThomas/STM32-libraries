/*!
 * \file
 * \copyright Copyright (C) Xsens Technologies B.V., 2015.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy
 * of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef __XBUSMESSAGE_H
#define __XBUSMESSAGE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Xbus message IDs. */
enum XsMessageId
{
	XMID_Undefined			= 0xFF,
	XMID_Wakeup             = 0x3E,
	XMID_WakeupAck          = 0x3F,
	XMID_ReqDid             = 0x00,
	XMID_DeviceId           = 0x01,
	XMID_GotoConfig         = 0x30,
	XMID_GotoConfigAck      = 0x31,
	XMID_GotoMeasurement    = 0x10,
	XMID_GotoMeasurementAck = 0x11,
	XMID_MtData2            = 0x36,
	XMID_ReqOutputConfig    = 0xC0,
	XMID_SetOutputConfig    = 0xC0,
	XMID_OutputConfig       = 0xC1,
	XMID_Reset              = 0x40,
	XMID_ResetAck           = 0x41,
	XMID_Error              = 0x42
};

/*! \brief Xbus data message type IDs. */
enum XsDataIdentifier
{
	XDI_PacketCounter  = 0x1020,
	XDI_SampleTimeFine = 0x1060,
	XDI_Quaternion     = 0x2010, // estimated orientation
	XDI_DeltaV         = 0x4010, // estimated acceleration
	XDI_Acceleration   = 0x4020, // calibrated accelerometer values
	XDI_RateOfTurn     = 0x8020, // calibrated gyro values
	XDI_DeltaQ         = 0x8030, // estimated quaternion derivative
	XDI_RawAccGyrMagTemp = 0xA010, // raw sensors
	XDI_MagneticField  = 0xC020,
	XDI_StatusWord     = 0xE020
};

/*!
 * \brief Low level format to use when formating Xbus messages for transmission.
 */
enum XbusLowLevelFormat
{
	/*! \brief Format for use with I2C interface. */
	XLLF_I2c,
	/*! \brief Format for use with SPI interface. */
	XLLF_Spi,
	/*! \brief Format for use with UART interface. */
	XLLF_Uart
};

/*!
 * \brief An Xbus message structure with optional payload.
 */
struct XbusMessage
{
	/*! \brief The message ID of the message. */
	enum XsMessageId mid;
	/*!
	 * \brief The length of the payload.
	 *
	 * \note The meaning of the length is message dependent. For example,
	 * for XMID_OutputConfig messages it is the number of OutputConfiguration
	 * elements in the configuration array.
	 */
	uint16_t length;
	/*! \brief Pointer to the payload data. */
	void* data;
};

/*!
 * \brief Output configuration structure.
 */
struct OutputConfiguration
{
	/*! \brief Data type of the output. */
	enum XsDataIdentifier dtype;
	/*!
	 * \brief The output frequency in Hz, or 65535 if the value should be
	 * included in every data message.
	 */
	uint16_t freq;
};

size_t XbusMessage_format(uint8_t* raw, struct XbusMessage const* message, enum XbusLowLevelFormat format);
bool XbusMessage_getDataItem(void* item, enum XsDataIdentifier id, struct XbusMessage const* message);
char const* XbusMessage_dataDescription(enum XsDataIdentifier id);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSMESSAGE_H
