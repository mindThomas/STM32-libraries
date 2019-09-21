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

#include "xsdeviceid.h"

/*!
 * \brief Return true if device ID corresponds to a MTi-1 series device.
 */
bool XsDeviceId_isMtMk4_X(uint32_t deviceId)
{
	uint8_t deviceSeries = (deviceId >> 20) & 0xF;
	return ((deviceSeries == 0x8) || (deviceSeries == 0xC));
}

/*!
 * \brief Get a string describing the function of the MTi device.
 */
char const* XsDeviceId_functionDescription(enum DeviceFunction function)
{
	switch (function)
	{
		case DF_IMU:
			return "Inertial Measurement Unit";

		case DF_VRU:
			return "Vertical Reference Unit";

		case DF_AHRS:
			return "Attitude Heading Reference System";
	}

	return "Unknown device function";
}

