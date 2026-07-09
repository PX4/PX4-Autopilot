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

#pragma once

#include <lib/matrix/matrix/Quaternion.hpp>
#include <uORB/topics/gimbal_device_set_attitude.h>

namespace gz_gimbal
{

inline matrix::Quatf attitudeSetpointInVehicleFrame(const matrix::Quatf &vehicle_attitude,
		const matrix::Quatf &attitude_setpoint, const uint16_t device_flags)
{
	const bool yaw_in_vehicle_frame = device_flags
					  & gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
	const bool yaw_in_earth_frame = device_flags
					& gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME;
	const bool legacy_yaw_lock = !yaw_in_vehicle_frame && !yaw_in_earth_frame
				     && (device_flags & gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_LOCK);

	if ((yaw_in_earth_frame && !yaw_in_vehicle_frame) || legacy_yaw_lock) {
		return vehicle_attitude.inversed() * attitude_setpoint;
	}

	return attitude_setpoint;
}

} // namespace gz_gimbal
