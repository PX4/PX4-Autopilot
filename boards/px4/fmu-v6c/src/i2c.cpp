/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <px4_arch/i2c_hw_description.h>

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/i2c.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(1),
	initI2CBusExternal(2),
	initI2CBusExternal(4),
};

bool px4_i2c_device_external(const uint32_t device_id)
{
	{
		// The internal baro and mag on Pixhawk 6C are on an external
		// bus. On rev 0, the bus is actually exposed externally, on
		// rev 1+, it is properly internal, however, still marked as
		// external for compatibility.

		// device_id: 4028193
		device::Device::DeviceId device_id_baro{};
		device_id_baro.devid_s.bus_type = device::Device::DeviceBusType_I2C;
		device_id_baro.devid_s.bus = 4;
		device_id_baro.devid_s.address = 0x77;
		device_id_baro.devid_s.devtype = DRV_BARO_DEVTYPE_MS5611;

		if (device_id_baro.devid == device_id) {
			return false;
		}

		// device_id: 396321
		device::Device::DeviceId device_id_mag{};
		device_id_mag.devid_s.bus_type = device::Device::DeviceBusType_I2C;
		device_id_mag.devid_s.bus = 4;
		device_id_mag.devid_s.address = 0xc;
		device_id_mag.devid_s.devtype = DRV_MAG_DEVTYPE_IST8310;

		if (device_id_mag.devid == device_id) {
			return false;
		}
	}

	device::Device::DeviceId dev_id{};
	dev_id.devid = device_id;
	return px4_i2c_bus_external(dev_id.devid_s.bus);
}
