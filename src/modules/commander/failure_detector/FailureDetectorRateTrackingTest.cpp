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

#include <memory>
#include <unistd.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

// to run: make tests TESTFILTER=FailureDetectorRateTrackingTest

using namespace time_literals;
using matrix::Vector3f;

static const float E = math::radians(100.f); // above the threshold
static const float e = math::radians(30.f); // below the threshold
static constexpr hrt_abstime kSustained = 300_ms; // longer than the trigger time

class FailureDetectorRateTrackingTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		configure(60.f, 0.1f);
	}

	// Set FD_RATE_ERR [deg/s] and FD_RATE_ERR_T [s], and create a detector that uses them
	void configure(float threshold, float ttri)
	{
		param_set(param_handle(px4::params::FD_RATE_ERR), &threshold);
		param_set(param_handle(px4::params::FD_RATE_ERR_T), &ttri);
		_fd = std::make_unique<FailureDetector>(nullptr);
	}

	// Publish setpoint and measured rates [rad/s], run the detector, return the rate flag.
	bool update(const Vector3f &rates_sp, const Vector3f &rates,
		    uint8_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
		    bool attitude_enabled = true, bool publish_rates = true, bool publish_setpoint = true)
	{
		if (publish_setpoint) {
			vehicle_rates_setpoint_s sp{};
			sp.timestamp = hrt_absolute_time();
			sp.roll = rates_sp(0);
			sp.pitch = rates_sp(1);
			sp.yaw = rates_sp(2);
			_rates_sp_pub.publish(sp);
		}

		if (publish_rates) {
			vehicle_angular_velocity_s angular_velocity{};
			angular_velocity.timestamp = hrt_absolute_time();
			rates.copyTo(angular_velocity.xyz);
			_angular_velocity_pub.publish(angular_velocity);
		}

		vehicle_land_detected_s land_detected{};
		land_detected.timestamp = hrt_absolute_time();
		land_detected.landed = _landed;
		_land_detected_pub.publish(land_detected);

		vehicle_status_s status{};
		status.vehicle_type = vehicle_type;
		status.in_transition_mode = _in_transition;

		vehicle_control_mode_s control_mode{};
		control_mode.flag_control_attitude_enabled = attitude_enabled;

		_fd->update(status, control_mode);

		_ever_triggered |= _fd->getStatus().flags.rate;

		return _fd->getStatus().flags.rate;
	}

	// Run update() every 10 ms for the given duration, return the last flag state.
	bool updateFor(hrt_abstime duration, const Vector3f &rates_sp, const Vector3f &rates,
		       uint8_t vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
		       bool attitude_enabled = true, bool publish_rates = true, bool publish_setpoint = true)
	{
		const hrt_abstime start = hrt_absolute_time();
		bool flag = false;

		do {
			flag = update(rates_sp, rates, vehicle_type, attitude_enabled, publish_rates, publish_setpoint);
			usleep(10000);
		} while (hrt_elapsed_time(&start) < duration);

		return flag;
	}

	bool _landed{false};
	bool _in_transition{false};
	bool _ever_triggered{false}; // the flag was set after any update

private:
	std::unique_ptr<FailureDetector> _fd;

	uORB::Publication<vehicle_rates_setpoint_s> _rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_land_detected_s> _land_detected_pub{ORB_ID(vehicle_land_detected)};
};

TEST_F(FailureDetectorRateTrackingTest, NoTriggerWhenDisabled)
{
	configure(0.f, 0.1f);
	EXPECT_FALSE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerBelowThreshold)
{
	EXPECT_FALSE(updateFor(kSustained, {e, e, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerOnShortSpike)
{
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}));
	EXPECT_FALSE(updateFor(kSustained, {}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, TriggerOnSustainedRollError)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, TriggerOnSustainedPitchError)
{
	EXPECT_TRUE(updateFor(kSustained, {0.f, E, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, TriggerOnMeasuredRateWithoutSetpointChange)
{
	// e.g. tumbling on the ground while the setpoint asks for no rotation
	EXPECT_TRUE(updateFor(kSustained, {}, {0.f, -E, 0.f}));
}

TEST_F(FailureDetectorRateTrackingTest, TriggerOnCombinedError)
{
	// 45 deg/s on each axis is below the threshold per axis, but 63.6 deg/s combined
	const float combined = math::radians(45.f);
	EXPECT_TRUE(updateFor(kSustained, {combined, combined, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, YawErrorIgnored)
{
	EXPECT_FALSE(updateFor(kSustained, {0.f, 0.f, E}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerInFixedWing)
{
	EXPECT_FALSE(updateFor(kSustained, {E, 0.f, 0.f}, {}, vehicle_status_s::VEHICLE_TYPE_FIXED_WING));
}

TEST_F(FailureDetectorRateTrackingTest, ClearsWhenErrorGone)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
	EXPECT_FALSE(updateFor(kSustained, {}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, ClearsWhenAttitudeControlDisabled)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, false));
	// the trigger time starts over when attitude control is enabled again
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, ClearsOnSwitchToFixedWing)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}, vehicle_status_s::VEHICLE_TYPE_FIXED_WING));
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, SpikesDoNotAccumulate)
{
	for (int i = 0; i < 5; i++) {
		EXPECT_FALSE(updateFor(30_ms, {E, 0.f, 0.f}, {}));
		EXPECT_FALSE(updateFor(30_ms, {}, {}));
	}
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerWithNanSetpoint)
{
	EXPECT_FALSE(updateFor(kSustained, {NAN, NAN, NAN}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerWithoutAngularVelocity)
{
	EXPECT_FALSE(updateFor(kSustained, {E, 0.f, 0.f}, {}, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true, false));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerWithStaleSetpoint)
{
	// the setpoint is published once and then goes stale while the rates keep coming
	update({E, 0.f, 0.f}, {});
	updateFor(800_ms, {}, {}, vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, true, true, false);
	EXPECT_FALSE(_ever_triggered);
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerWhenLanded)
{
	// e.g. stick input while armed on the ground: the vehicle is not expected to rotate
	_landed = true;
	updateFor(kSustained, {E, 0.f, 0.f}, {});
	EXPECT_FALSE(_ever_triggered);
}

TEST_F(FailureDetectorRateTrackingTest, ClearsWhenLanded)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
	_landed = true;
	EXPECT_FALSE(update({E, 0.f, 0.f}, {}));
}

TEST_F(FailureDetectorRateTrackingTest, NoTriggerInVtolTransition)
{
	// the rate setpoint comes from the fixed-wing controller during a transition
	_in_transition = true;
	updateFor(kSustained, {E, 0.f, 0.f}, {});
	EXPECT_FALSE(_ever_triggered);
}

TEST_F(FailureDetectorRateTrackingTest, ClearsOnNanSetpoint)
{
	EXPECT_TRUE(updateFor(kSustained, {E, 0.f, 0.f}, {}));
	EXPECT_FALSE(update({NAN, 0.f, 0.f}, {}));
}
