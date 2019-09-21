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

#ifndef __XSDEVICEID_H
#define __XSDEVICEID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool XsDeviceId_isMtMk4_X(uint32_t deviceId);

enum DeviceFunction
{
	/*! \brief Inertial Measurement Unit. */
	DF_IMU  = 1,
	/*! \brief Vertical Reference Unit. */
	DF_VRU  = 2,
	/*! \brief Attitude Heading Reference System. */
	DF_AHRS = 3
};

/*!
 * \brief Get the function of the MTi device.
 */
static inline enum DeviceFunction XsDeviceId_getFunction(uint32_t deviceId)
{
	return (enum DeviceFunction)((deviceId >> 24) & 0xF);
}

char const* XsDeviceId_functionDescription(enum DeviceFunction function);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XSDEVICEID_H
