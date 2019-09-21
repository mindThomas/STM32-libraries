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

#ifndef __XBUSUTILITY_H
#define __XBUSUTILITY_H

#include <stdint.h>

uint8_t const* XbusUtility_readU8(uint8_t* out, uint8_t const* in);
uint8_t const* XbusUtility_readU16(uint16_t* out, uint8_t const* in);
uint8_t const* XbusUtility_readU32(uint32_t* out, uint8_t const* in);

uint8_t* XbusUtility_writeU8(uint8_t* out, uint8_t in);
uint8_t* XbusUtility_writeU16(uint8_t* out, uint16_t in);
uint8_t* XbusUtility_writeU32(uint8_t* out, uint32_t in);

#endif // __XBUSUTILITY_H
