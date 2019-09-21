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

#ifndef __XBUSPARSER_H
#define __XBUSPARSER_H

#include <stddef.h>
#include <stdint.h>
#include "xbusmessage.h"

#ifdef __cplusplus
extern "C" {
#endif

struct XbusParser;

/*!
 * \brief Callback function structure for use with the XbusParser.
 */
struct XbusParserCallback
{
	/*!
	 * \brief Allocate a buffer for message reception.
	 * \param bufSize The size of the buffer to allocate.
	 * \returns Pointer to buffer to use for message reception, or NULL if
	 * a buffer cannot be allocated.
	 *
	 * \note It is the resposibility of the user to deallocate the message
	 * data buffers pointed to by XbusMessage structures passed to the
	 * handleMessage() callback function.
	 */
	void* (*allocateBuffer)(size_t bufSize);

	/*!
	 * \brief Deallocate a buffer that was previously allocated by a call to
	 * allocateBuffer.
	 */
	void (*deallocateBuffer)(void const* buffer);

	/*!
	 * \brief Handle a received message.
	 *
	 * \note If the passed XbusMessage structure has a non-null data pointer
	 * then it is the responsibility of the user to free this once handling
	 * of the message is complete.
	 */
	void (*handleMessage)(void * param, struct XbusMessage const* message);
	void * parameter;
};

size_t XbusParser_mem(void);
struct XbusParser* XbusParser_create(struct XbusParserCallback const* callback);
void XbusParser_destroy(struct XbusParser* parser);
struct XbusParser* XbusParser_init(void* parserMem, struct XbusParserCallback const* callback);

void XbusParser_parseByte(struct XbusParser* parser, uint8_t byte);
void XbusParser_parseBuffer(struct XbusParser* parser, uint8_t const* buf, size_t bufSize);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSPARSER_H
