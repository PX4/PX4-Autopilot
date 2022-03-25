/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/conversion/rotation.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

#if defined(CONFIG_I2C)
# include <px4_platform_common/i2c.h>
#endif // CONFIG_I2C

#if defined(CONFIG_SPI)
# include <px4_platform_common/spi.h>
#endif // CONFIG_SPI

using math::radians;
using matrix::Eulerf;
using matrix::Dcmf;
using matrix::Vector3f;

namespace sensor
{
namespace utilities
{

Eulerf GetSensorLevelAdjustment()
{
	float x_offset = 0.f;
	float y_offset = 0.f;
	float z_offset = 0.f;
	param_get(param_find("SENS_BOARD_X_OFF"), &x_offset);
	param_get(param_find("SENS_BOARD_Y_OFF"), &y_offset);
	param_get(param_find("SENS_BOARD_Z_OFF"), &z_offset);

	return Eulerf{radians(x_offset), radians(y_offset), radians(z_offset)};
}

enum Rotation GetBoardRotation()
{
	// get transformation matrix from sensor/board to body frame
	int32_t board_rot = -1;
	param_get(param_find("SENS_BOARD_ROT"), &board_rot);

	if (board_rot >= 0 && board_rot <= Rotation::ROTATION_MAX) {
		return static_cast<enum Rotation>(board_rot);

	} else {
		PX4_ERR("invalid SENS_BOARD_ROT: %" PRId32, board_rot);
	}

	return Rotation::ROTATION_NONE;
}

Dcmf GetBoardRotationMatrix()
{
	return get_rot_matrix(GetBoardRotation());
}

bool DeviceExternal(uint32_t device_id)
{
	bool external = true;

	// decode device id to determine if external
	union device::Device::DeviceId id {};
	id.devid = device_id;

	const device::Device::DeviceBusType bus_type = id.devid_s.bus_type;

	switch (bus_type) {
	case device::Device::DeviceBusType_I2C:
#if defined(CONFIG_I2C)
		external = px4_i2c_bus_external(id.devid_s.bus);
#endif // CONFIG_I2C
		break;

	case device::Device::DeviceBusType_SPI:
#if defined(CONFIG_SPI)
		external = px4_spi_bus_external(id.devid_s.bus);
#endif // CONFIG_SPI
		break;

	case device::Device::DeviceBusType_UAVCAN:
		external = true;
		break;

	case device::Device::DeviceBusType_SIMULATION:
		external = false;
		break;

	case device::Device::DeviceBusType_SERIAL:
		external = true;
		break;

	case device::Device::DeviceBusType_MAVLINK:
		external = true;
		break;

	case device::Device::DeviceBusType_UNKNOWN:
		external = true;
		break;
	}

	return external;
}

} // namespace utilities
} // namespace sensor
