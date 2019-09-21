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

#include "xbusparser.h"
#include "xbusdef.h"
#include "xbusutility.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "Debug.h"

/*! \brief XbusParser states. */
enum XbusParserState
{
	XBPS_Preamble,          /*!< \brief Looking for preamble. */
	XBPS_BusId,             /*!< \brief Waiting for bus ID. */
	XBPS_MessageId,         /*!< \brief Waiting for message ID. */
	XBPS_Length,            /*!< \brief Waiting for length. */
	XBPS_ExtendedLengthMsb, /*!< \brief Waiting for extended length MSB*/
	XBPS_ExtendedLengthLsb, /*!< \brief Waiting for extended length LSB*/
	XBPS_Payload,           /*!< \brief Reading payload. */
	XBPS_Checksum           /*!< \brief Waiting for checksum. */
};

/*!
 * \brief Xbus Parser state structure.
 */
struct XbusParser
{
	/*! \brief Callbacks for memory management, and message handling. */
	struct XbusParserCallback callbacks;
	/*! \brief Storage for the current message being received. */
	struct XbusMessage currentMessage;
	/*! \brief The number of bytes of payload received for the current message. */
	uint16_t payloadReceived;
	/*! \brief The calculated checksum for the current message. */
	uint8_t checksum;
	/*! \brief The state of the parser. */
	enum XbusParserState state;
};

/*!
 * \brief Get the amount of memory needed for the XbusParser structure.
 */
size_t XbusParser_mem(void)
{
	return sizeof(struct XbusParser);
}

/*!
 * \brief Create a new XbusParser object.
 * \param callback Pointer to callback structure containing callback functions
 * for memory management and handling received messages.
 * \returns Pointer the new XbusParser structure.
 *
 * Uses malloc to allocate the memory required for the parser.
 */
struct XbusParser* XbusParser_create(struct XbusParserCallback const* callback)
{
	void* mem = pvPortMalloc(XbusParser_mem());
	if (mem)
	{
		return XbusParser_init(mem, callback);
	}
	return NULL;
}

/*!
 * \brief Frees an XbusParser structure allocated by XbusParser_create().
 */
void XbusParser_destroy(struct XbusParser* parser)
{
	vPortFree(parser);
}

/*!
 * \brief Initializes an XbusParser in the passed memory location.
 * \param parserMem Pointer to memory to use for storing parser state. Should
 * be at least as big as the value returned by XbusParser_mem().
 * \param callback Pointer to callback structure containing callback functions
 * for memory management and handling received messages.
 * \returns Initialized XbusParser structure.
 */
struct XbusParser* XbusParser_init(void* parserMem, struct XbusParserCallback const* callback)
{
	struct XbusParser* parser = (struct XbusParser*)parserMem;
	parser->state = XBPS_Preamble;
	parser->callbacks.allocateBuffer = callback->allocateBuffer;
	parser->callbacks.deallocateBuffer = callback->deallocateBuffer;
	parser->callbacks.handleMessage = callback->handleMessage;
	parser->callbacks.parameter = callback->parameter;
	return parser;
}

/*!
 * \brief Parse an XMID_DeviceId message to extract the device ID value.

 * Replaces the raw Xbus message data with the device ID.
 */
static void parseDeviceId(struct XbusParser* parser, uint8_t const* rawData)
{
	uint32_t* deviceId = parser->callbacks.allocateBuffer(sizeof(uint32_t));
	if (deviceId)
	{
		XbusUtility_readU32(deviceId, rawData);
		parser->currentMessage.data = deviceId;
		parser->currentMessage.length = 1;
	}
	else
	{
		parser->currentMessage.data = NULL;
	}
}

/*!
 * \brief Parse an XMID_OutputConfig message.
 *
 * Replaces the raw Xbus message data with an array of OutputConfiguration
 * structures.
 */
static void parseOutputConfig(struct XbusParser* parser, uint8_t const* rawData)
{
	uint8_t fields = parser->currentMessage.length / 4;
	struct OutputConfiguration* conf = parser->callbacks.allocateBuffer(fields * sizeof(struct OutputConfiguration));
	if (conf)
	{
		parser->currentMessage.data = conf;
		parser->currentMessage.length = fields;

		for (int i = 0; i < fields; ++i)
		{
			rawData = XbusUtility_readU16((uint16_t*)&conf->dtype, rawData);
			rawData = XbusUtility_readU16(&conf->freq, rawData);
			++conf;
		}
	}
	else
	{
		parser->currentMessage.data = NULL;
	}
}

/*!
 * \brief Converts raw Xbus payload data to native structures if possible.
 *
 * Raw data payloads are converted to native data structures and the
 * message data pointer is changed to point to the native structure.
 * The raw data is automatically deallocated.
 */
static void parseMessagePayload(struct XbusParser* parser)
{
	uint8_t const* const rawData = parser->currentMessage.data;
	switch (parser->currentMessage.mid)
	{
		default:
			// Leave parsing and memory management to user code
			return;

		case XMID_DeviceId:
			parseDeviceId(parser, rawData);
			break;

		case XMID_OutputConfig:
			parseOutputConfig(parser, rawData);
			break;
	}

	if (rawData)
		parser->callbacks.deallocateBuffer(rawData);
}

/*!
 * \brief Prepare for receiving a message payload.
 *
 * Requests a memory area to store the received data to using the
 * registered callbacks.
 */
void prepareForPayload(struct XbusParser* parser)
{
	parser->payloadReceived = 0;
	parser->currentMessage.data = parser->callbacks.allocateBuffer(parser->currentMessage.length);
}

/*!
 * \brief Parse a byte of data from a motion tracker.
 *
 * When a complete message is received the user will be notified by a call
 * to the handleMessage() callback function.
 */
void XbusParser_parseByte(struct XbusParser* parser, const uint8_t byte)
{
	switch (parser->state)
	{
		case XBPS_Preamble:
			if (byte == XBUS_PREAMBLE)
			{
				parser->checksum = 0;
				parser->state = XBPS_BusId;
			}
			break;

		case XBPS_BusId:
			parser->checksum += byte;
			parser->state = XBPS_MessageId;
			break;

		case XBPS_MessageId:
			parser->checksum += byte;
			parser->currentMessage.mid = (enum XsMessageId)byte;
			parser->state = XBPS_Length;
			break;

		case XBPS_Length:
			parser->checksum += byte;
			if (byte == XBUS_NO_PAYLOAD)
			{
				parser->currentMessage.length = byte;
				parser->currentMessage.data = NULL;
				parser->state = XBPS_Checksum;
			}
			else if (byte < XBUS_EXTENDED_LENGTH)
			{
				parser->currentMessage.length = byte;
				prepareForPayload(parser);
				parser->state = XBPS_Payload;
			}
			else
			{
				parser->state = XBPS_ExtendedLengthMsb;
			}
			break;

		case XBPS_ExtendedLengthMsb:
			parser->checksum += byte;
			parser->currentMessage.length = ((uint16_t)byte) << 8;
			parser->state = XBPS_ExtendedLengthLsb;
			break;

		case XBPS_ExtendedLengthLsb:
			parser->checksum += byte;
			parser->currentMessage.length |= byte;
			prepareForPayload(parser);
			parser->state = XBPS_Payload;
			break;

		case XBPS_Payload:
			parser->checksum += byte;
			if (parser->currentMessage.data)
			{
				((uint8_t*)parser->currentMessage.data)[parser->payloadReceived] = byte;
			}
			if (++parser->payloadReceived == parser->currentMessage.length)
			{
				parser->state = XBPS_Checksum;
			}
			break;

		case XBPS_Checksum:
			parser->checksum += byte;
			if ((parser->checksum == 0) &&
					((parser->currentMessage.length == 0) ||
					 parser->currentMessage.data))
			{
				parseMessagePayload(parser);
				parser->callbacks.handleMessage(parser->callbacks.parameter, &parser->currentMessage);
			}
			else if (parser->currentMessage.data)
			{
				parser->callbacks.deallocateBuffer(parser->currentMessage.data);
			}
			parser->state = XBPS_Preamble;
			break;
	}
}

/*!
 * \brief Parse a buffer of data received from a motion tracker.
 */
void XbusParser_parseBuffer(struct XbusParser* parser, uint8_t const* buf, size_t bufSize)
{
	for (size_t i = 0; i < bufSize; ++i)
	{
		XbusParser_parseByte(parser, buf[i]);
	}
}

