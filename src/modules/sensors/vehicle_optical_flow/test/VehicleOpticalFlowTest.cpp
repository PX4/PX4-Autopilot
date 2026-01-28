/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * Test for VehicleOpticalFlow
 */

#include <gtest/gtest.h>
#include "../VehicleOpticalFlow.hpp"
#include <uORB/uORBManager.hpp>
#include <uORB/topics/distance_sensor.h>


distance_sensor_s createDistanceSensorMessage(uint16_t orientation)
{
	distance_sensor_s message;
	message.timestamp = hrt_absolute_time();
	message.min_distance = 1.f;
	message.max_distance = 100.f;
	message.current_distance = 1.1f;

	message.variance = 0.1f;
	message.signal_quality = 100;
	message.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	message.orientation = orientation;
	message.h_fov = math::radians(50.f);
	message.v_fov = math::radians(30.f);
	return message;

}

class VehicleOpticalFlowTest : public ::testing::Test
{
public:

	class VehicleOpticalFlowTestable  : public  sensors::VehicleOpticalFlow
	{
	public:
		void UpdateDistanceSensorPublic()
		{
			VehicleOpticalFlow::UpdateDistanceSensor();
		}
		bool IsDistanceSensorSelected()
		{
			return _distance_sensor_selected >= 0;

		}
	};

	void SetUp() override
	{
		uORB::Manager::initialize();

	}
	void TearDown() override
	{
		uORB::Manager::terminate();
	}
};


TEST_F(VehicleOpticalFlowTest, CameraFacingDown)
{
	// GIVEN: message with sensor camera facing down
	distance_sensor_s message = createDistanceSensorMessage(distance_sensor_s::ROTATION_DOWNWARD_FACING);
	orb_advertise(ORB_ID(distance_sensor), &message);

	// WHEN: update distance sensor
	VehicleOpticalFlowTest::VehicleOpticalFlowTestable testable;
	testable.UpdateDistanceSensorPublic();

	// THEN: sensor selected
	EXPECT_TRUE(testable.IsDistanceSensorSelected());
}

TEST_F(VehicleOpticalFlowTest, CameraFacingForward)
{
	// GIVEN: message with sensor camera facing forward
	distance_sensor_s message = createDistanceSensorMessage(distance_sensor_s::ROTATION_FORWARD_FACING);
	orb_advertise(ORB_ID(distance_sensor), &message);

	// WHEN: update distance sensor
	VehicleOpticalFlowTest::VehicleOpticalFlowTestable testable;
	testable.UpdateDistanceSensorPublic();

	// THEN: sensor is not selected
	EXPECT_FALSE(testable.IsDistanceSensorSelected());
}
