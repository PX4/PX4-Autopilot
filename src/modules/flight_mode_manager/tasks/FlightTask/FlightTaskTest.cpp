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

#include <gtest/gtest.h>
#include "FlightTask.hpp"

TEST(FlightTaskTest, AvailableInputsFollowValidityFlags)
{
	vehicle_local_position_s local_position{};
	local_position.timestamp = 1'000'000;
	local_position.heading = NAN;
	const hrt_abstime now = local_position.timestamp + 100'000;

	EXPECT_EQ(FlightTask::availableInputs(local_position, false, false, now), FlightTask::LocalPosition);

	local_position.xy_valid = true;
	EXPECT_EQ(FlightTask::availableInputs(local_position, false, false, now),
		  FlightTask::LocalPosition | FlightTask::PositionXY);

	local_position.v_xy_valid = true;
	local_position.z_valid = true;
	local_position.v_z_valid = true;
	local_position.heading = 0.f;
	EXPECT_EQ(FlightTask::availableInputs(local_position, false, false, now),
		  FlightTask::LocalPosition | FlightTask::PositionXY | FlightTask::VelocityXY | FlightTask::PositionZ
		  | FlightTask::VelocityZ | FlightTask::Heading);

	EXPECT_EQ(FlightTask::availableInputs(local_position, true, false, now),
		  FlightTask::LocalPosition | FlightTask::PositionXY | FlightTask::VelocityXY | FlightTask::PositionZ
		  | FlightTask::VelocityZ | FlightTask::Heading | FlightTask::HomeAltitude);

	EXPECT_EQ(FlightTask::availableInputs(local_position, true, true, now), 0xff);
}

TEST(FlightTaskTest, StaleLocalPositionProvidesNoLocalInputs)
{
	vehicle_local_position_s local_position{};
	local_position.timestamp = 1'000'000;
	local_position.xy_valid = true;
	local_position.v_xy_valid = true;
	local_position.z_valid = true;
	local_position.v_z_valid = true;
	local_position.heading = 0.f;

	EXPECT_EQ(FlightTask::availableInputs(local_position, true, true, local_position.timestamp + 499'999), 0xff);
	EXPECT_EQ(FlightTask::availableInputs(local_position, true, true, local_position.timestamp + 500'000),
		  FlightTask::ManualControl | FlightTask::HomeAltitude);
}
