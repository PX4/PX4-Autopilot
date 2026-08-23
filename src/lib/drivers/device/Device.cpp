/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Device.hpp"

#include <px4_platform_common/i2c.h>
#include <px4_platform_common/spi.h>

namespace device
{

bool device_is_external(uint32_t device_id)
{
	Device::DeviceId id{};
	id.devid = device_id;

	switch (id.devid_s.bus_type) {
	case Device::DeviceBusType_I2C:
#if defined(CONFIG_I2C)
		return px4_i2c_bus_external(id.devid_s.bus);
#else
		return true;
#endif // CONFIG_I2C

	case Device::DeviceBusType_SPI:
#if defined(CONFIG_SPI)
		return px4_spi_bus_external(id.devid_s.bus);
#else
		return true;
#endif // CONFIG_SPI

	case Device::DeviceBusType_SIMULATION:
		return false;

	case Device::DeviceBusType_UAVCAN:
	case Device::DeviceBusType_SERIAL:
	case Device::DeviceBusType_MAVLINK:
	case Device::DeviceBusType_UNKNOWN:
		return true;
	}

	return true;
}

} // namespace device
