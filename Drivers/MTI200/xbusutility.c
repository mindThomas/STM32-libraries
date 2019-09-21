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

#include "xbusutility.h"

/*!
 * \name Xbus message reading utility functions.
 * Xbus messages use big-endian representations for multibyte data.
 * These functions help in reading data from an Xbus message and converting
 * it to a native type.
 *
 * The functions are intended to scan over a message as follows:
 * \code
 * void readValues(XbusMessage const* message)
 * {
 *     uint16_t v1;
 *     uint8_t v2;
 *     uint32_t v3;
 *     uint8_t* dptr = message->data;
 *     dptr = XbusUtility_readU16(&v1, dptr);
 *     dptr = XbusUtility_readU8(&v2, dptr);
 *     dptr = XbusUtility_readU32(&v3, dptr);
 * }
 * \endcode
 * \{
 */
/*!
 * \brief Read a uint8_t value from an Xbus message.
 */
uint8_t const* XbusUtility_readU8(uint8_t* out, uint8_t const* in)
{
	*out = *in;
	return ++in;
}

/*! \brief Read a uint16_t value from an Xbus message. */
uint8_t const* XbusUtility_readU16(uint16_t* out, uint8_t const* in)
{
	*out = (in[0] << 8) | in[1];
	return in + sizeof(uint16_t);
}

/*! \brief Read a uint32_t value from an Xbus message. */
uint8_t const* XbusUtility_readU32(uint32_t* out, uint8_t const* in)
{
	*out = (in[0] << 24) | (in[1] << 16) | (in[2] << 8) | in[3];
	return in + sizeof(uint32_t);
}
/*! \} */

/*!
 * \name Xbus message writing utility functions.
 * These functions aid in writing native values to big-endian xbus message
 * payloads. See corresponding reading functions for further details.
 * \{
 */
/*! \brief Write a uint8_t value to an Xbus message. */
uint8_t* XbusUtility_writeU8(uint8_t* out, uint8_t in)
{
	*out++ = in;
	return out;
}

/*! \brief Write a uint16_t value to an Xbus message. */
uint8_t* XbusUtility_writeU16(uint8_t* out, uint16_t in)
{
	*out++ = (in >> 8) & 0xFF;
	*out++ = in & 0xFF;
	return out;
}

/*! \brief Write a uint32_t value to an Xbus message. */
uint8_t* XbusUtility_writeU32(uint8_t* out, uint32_t in)
{
	*out++ = (in >> 24) & 0xFF;
	*out++ = (in >> 16) & 0xFF;
	*out++ = (in >> 8) & 0xFF;
	*out++ = in & 0xFF;
	return out;
}
/*! \}  */
