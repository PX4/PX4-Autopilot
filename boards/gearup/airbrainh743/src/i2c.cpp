/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

/*
 * I2C bus configuration for AirBrainH743
 *
 * I2C1: PB6 (SCL), PB7 (SDA)
 *       Devices: DPS310 baro @ 0x76, IIS2MDC compass @ 0x1E
 *       This single bus is also broken out externally, so it is marked
 *       external to allow scanning for external compasses. The onboard baro
 *       and mag are forced back to internal in px4_i2c_device_external().
 * I2C4: External bus - PD12 (SCL), PD13 (SDA)
 */

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(1),
	initI2CBusExternal(4),
};

bool px4_i2c_device_external(const uint32_t device_id)
{
	// The onboard baro and mag are on I2C1, which is marked external because
	// the same bus is also broken out externally. Keep these known onboard
	// devices classified as internal.

	// DPS310 baro @ 0x76 on I2C1
	device::Device::DeviceId device_id_baro{};
	device_id_baro.devid_s.bus_type = device::Device::DeviceBusType_I2C;
	device_id_baro.devid_s.bus = 1;
	device_id_baro.devid_s.address = 0x76;
	device_id_baro.devid_s.devtype = DRV_BARO_DEVTYPE_DPS310;

	if (device_id_baro.devid == device_id) {
		return false;
	}

	// IIS2MDC mag @ 0x1E on I2C1
	device::Device::DeviceId device_id_mag{};
	device_id_mag.devid_s.bus_type = device::Device::DeviceBusType_I2C;
	device_id_mag.devid_s.bus = 1;
	device_id_mag.devid_s.address = 0x1E;
	device_id_mag.devid_s.devtype = DRV_MAG_DEVTYPE_IIS2MDC;

	if (device_id_mag.devid == device_id) {
		return false;
	}

	device::Device::DeviceId dev_id{};
	dev_id.devid = device_id;
	return px4_i2c_bus_external(dev_id.devid_s.bus);
}
