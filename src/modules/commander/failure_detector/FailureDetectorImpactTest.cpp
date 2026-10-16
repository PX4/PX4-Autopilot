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

#include "FailureDetector.hpp"

#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_land_detected.h>

// to run: make tests TESTFILTER=FailureDetectorImpactTest

using namespace time_literals;

// Expose the parameter refresh of the failure detector to the test
class TestFailureDetector : public FailureDetector
{
public:
	using FailureDetector::FailureDetector;
	using ModuleParams::updateParams;
};

class FailureDetectorImpactTest : public ::testing::Test
{
public:
	static constexpr uint32_t kAccelDeviceId = 1234;

	void SetUp() override
	{
		param_control_autosave(false);

		// FD_IMPACT_THR = 80 m/s^2, FD_IMPACT_T = 0s
		setImpactThreshold(80.f);
		setCrashTime(0.f);

		sensor_selection_s selection{};
		selection.timestamp = hrt_absolute_time();
		selection.accel_device_id = kAccelDeviceId;
		_sensor_selection_pub.publish(selection);
	}

	void setImpactThreshold(float threshold)
	{
		param_t param = param_handle(px4::params::FD_IMPACT_THR);
		param_set(param, &threshold);
		_fd.updateParams();
	}

	void setCrashTime(float time_s)
	{
		param_t param = param_handle(px4::params::FD_IMPACT_T);
		param_set(param, &time_s);
		_fd.updateParams();
	}

	// Publish the IMU status and land detector state, then run the detector
	void update(float impact_metric, bool armed = true, bool landed = false, bool moving = false)
	{
		vehicle_imu_status_s imu_status{};
		imu_status.timestamp = hrt_absolute_time();
		imu_status.accel_device_id = kAccelDeviceId;
		imu_status.accel_impact_metric = impact_metric;
		_imu_status_pub.publish(imu_status);

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = landed;
		land_detected.vertical_movement = moving;
		land_detected.horizontal_movement = moving;
		land_detected.rotational_movement = moving;
		_land_detected_pub.publish(land_detected);

		vehicle_status_s status{};
		status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		status.arming_state = armed ? vehicle_status_s::ARMING_STATE_ARMED : vehicle_status_s::ARMING_STATE_DISARMED;

		vehicle_control_mode_s control_mode{};

		_fd.update(status, control_mode);
	}

	bool impact() const { return _fd.getStatus().flags.impact; }

	bool crash() const { return _fd.getStatus().flags.crash; }

private:
	TestFailureDetector _fd{nullptr};

	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_imu_status_s> _imu_status_pub{ORB_ID(vehicle_imu_status)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
};

TEST_F(FailureDetectorImpactTest, NoTriggerWhenDisabled)
{
	setImpactThreshold(0.f);

	update(150.f);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, NoTriggerBelowThreshold)
{
	update(50.f);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, NoTriggerWhenLanded)
{
	update(150.f, true, true);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, NoTriggerWhenDisarmed)
{
	update(150.f, false);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, ImpactLatchedUntilDisarm)
{
	// impact while flying, the vehicle keeps moving afterwards (e.g. hit a branch and recovered)
	update(150.f, true, false, true);
	EXPECT_TRUE(impact());
	EXPECT_FALSE(crash());

	// metric back to normal: impact stays latched, no crash while moving
	update(10.f, true, false, true);
	EXPECT_TRUE(impact());
	EXPECT_FALSE(crash());

	// disarm resets
	update(10.f, false);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, CrashWhenNotMovingAfterImpact)
{
	update(150.f);
	EXPECT_TRUE(impact());
	EXPECT_TRUE(crash()); // FD_IMPACT_T = 0: immediately

	// crash stays latched even if movement is seen again
	update(10.f, true, false, true);
	EXPECT_TRUE(crash());

	// until disarm
	update(10.f, false);
	EXPECT_FALSE(impact());
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, NoCrashWhenLandedAfterImpact)
{
	// hard landing: impact followed by the land detector declaring landed
	update(150.f, true, false, true);
	EXPECT_TRUE(impact());

	update(10.f, true, true, false);
	EXPECT_FALSE(crash());
}

TEST_F(FailureDetectorImpactTest, CrashRequiresNoMovementForConfiguredTime)
{
	setCrashTime(0.2f);

	update(150.f);
	EXPECT_TRUE(impact());
	EXPECT_FALSE(crash());

	px4_usleep(100000);
	update(10.f);
	EXPECT_FALSE(crash());

	px4_usleep(150000);
	update(10.f);
	EXPECT_TRUE(crash());
}

TEST_F(FailureDetectorImpactTest, MovementResetsCrashTimer)
{
	setCrashTime(0.2f);

	update(150.f);
	EXPECT_TRUE(impact());

	px4_usleep(150000);
	update(10.f, true, false, true); // movement resets the timer
	EXPECT_FALSE(crash());

	update(10.f); // timer restarts here
	EXPECT_FALSE(crash());

	px4_usleep(100000);
	update(10.f);
	EXPECT_FALSE(crash()); // only 100ms without movement

	px4_usleep(150000);
	update(10.f);
	EXPECT_TRUE(crash()); // 250ms without movement
}
