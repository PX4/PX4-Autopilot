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

#include <gtest/gtest.h>

#include <lib/matrix/matrix/math.hpp>

#include "GZGimbalSetpoint.hpp"

using matrix::Eulerf;
using matrix::Quatf;

TEST(GZGimbalSetpoint, ConvertsEarthFrameToVehicleFrame)
{
	const Quatf vehicle_attitude(Eulerf(0.f, 0.f, M_PI_2_F));
	const Quatf earth_frame_setpoint(Eulerf(0.f, 0.f, 0.f));
	const Quatf vehicle_frame_setpoint = gz_gimbal::attitudeSetpointInVehicleFrame(
			vehicle_attitude, earth_frame_setpoint,
			gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME);

	EXPECT_NEAR(Eulerf(vehicle_frame_setpoint).psi(), -M_PI_2_F, 1e-5f);
}

TEST(GZGimbalSetpoint, PreservesVehicleFrame)
{
	const Quatf vehicle_attitude(Eulerf(0.f, 0.f, M_PI_2_F));
	const Quatf vehicle_frame_input(Eulerf(0.f, 0.f, 0.2f));
	const Quatf vehicle_frame_setpoint = gz_gimbal::attitudeSetpointInVehicleFrame(
			vehicle_attitude, vehicle_frame_input,
			gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME);

	EXPECT_NEAR(Eulerf(vehicle_frame_setpoint).psi(), 0.2f, 1e-5f);
}

TEST(GZGimbalSetpoint, ConvertsLegacyYawLockToVehicleFrame)
{
	const Quatf vehicle_attitude(Eulerf(0.f, 0.f, M_PI_2_F));
	const Quatf earth_frame_setpoint(Eulerf(0.f, 0.f, 0.f));
	const Quatf vehicle_frame_setpoint = gz_gimbal::attitudeSetpointInVehicleFrame(
			vehicle_attitude, earth_frame_setpoint,
			gimbal_device_set_attitude_s::GIMBAL_DEVICE_FLAGS_YAW_LOCK);

	EXPECT_NEAR(Eulerf(vehicle_frame_setpoint).psi(), -M_PI_2_F, 1e-5f);
}

TEST(GZGimbalSetpoint, PreservesLegacyYawFollow)
{
	const Quatf vehicle_attitude(Eulerf(0.f, 0.f, M_PI_2_F));
	const Quatf vehicle_frame_input(Eulerf(0.f, 0.f, 0.2f));
	const Quatf vehicle_frame_setpoint = gz_gimbal::attitudeSetpointInVehicleFrame(
			vehicle_attitude, vehicle_frame_input, 0);

	EXPECT_NEAR(Eulerf(vehicle_frame_setpoint).psi(), 0.2f, 1e-5f);
}
