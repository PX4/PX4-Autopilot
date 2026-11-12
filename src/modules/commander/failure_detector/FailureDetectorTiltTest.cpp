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

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

// to run: make tests TESTFILTER=FailureDetectorTilt

using namespace time_literals;
using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

static constexpr int kMaxTiltErrorDeg = 60;

// Tilt by angle around a horizontal axis, direction 0° tilts around the x-axis (roll), 90° around the y-axis (pitch)
static Quatf tilt(float angle_deg, float direction_deg = 0.f)
{
	const float direction = math::radians(direction_deg);
	return Quatf(AxisAnglef(Vector3f(cosf(direction), sinf(direction), 0.f), math::radians(angle_deg)));
}

// Tailsitter attitude in the multicopter body frame from the attitude in the fixed-wing frame
static Quatf tailsitterMcFromFw(const Quatf &q_fw)
{
	return q_fw * Quatf(Eulerf(0.f, M_PI_2_F, 0.f)).inversed();
}

// The fixture is the detector's parameter parent, like Commander in flight
class FailureDetectorTiltTest : public ::testing::Test, public ModuleParams
{
public:
	FailureDetectorTiltTest() : ModuleParams(nullptr) {}

	void SetUp() override
	{
		param_control_autosave(false);
		setParams(kMaxTiltErrorDeg, 0.1f);
		_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	}

	void setParams(int32_t max_tilt_error_deg, float trigger_time_s)
	{
		param_set(param_handle(px4::params::FD_FAIL_R), &max_tilt_error_deg);
		param_set(param_handle(px4::params::FD_FAIL_R_TTRI), &trigger_time_s);
		updateParams();
	}

	// Publish attitude and setpoint, then run the detector. Returns the tilt flag state.
	// setpoint_age is the age of the attitude setpoint relative to the attitude sample.
	bool update(const Quatf &q, const Quatf &q_sp = Quatf(), hrt_abstime setpoint_age = 0, bool attitude_control = true)
	{
		// The trigger time runs on the attitude timestamp
		_timestamp += 10_ms;

		vehicle_attitude_setpoint_s attitude_setpoint{};
		attitude_setpoint.timestamp = _timestamp - setpoint_age;
		q_sp.copyTo(attitude_setpoint.q_d);
		_attitude_setpoint_pub.publish(attitude_setpoint);

		vehicle_attitude_s attitude{};
		attitude.timestamp = _timestamp;
		q.copyTo(attitude.q);
		_attitude_pub.publish(attitude);

		vehicle_control_mode_s control_mode{};
		control_mode.flag_control_attitude_enabled = attitude_control;

		_fd.update(_status, control_mode);
		return _fd.getStatus().flags.tilt;
	}

	// Keep the condition for longer than the 0.1s trigger time
	bool updateAfterTriggerTime(const Quatf &q, const Quatf &q_sp = Quatf(), hrt_abstime setpoint_age = 0)
	{
		update(q, q_sp, setpoint_age);
		advance(150_ms);
		return update(q, q_sp, setpoint_age);
	}

	void advance(hrt_abstime dt) { _timestamp += dt; }

	void resetDetector() { update(Quatf(), Quatf(), 0, false); }

	vehicle_status_s _status{};

private:
	FailureDetector _fd{this};
	hrt_abstime _timestamp{10_s};

	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
};

TEST_F(FailureDetectorTiltTest, NoTriggerWhenLevel)
{
	EXPECT_FALSE(updateAfterTriggerTime(Quatf()));
}

TEST_F(FailureDetectorTiltTest, ThresholdInAllDirections)
{
	for (float direction_deg = 0.f; direction_deg < 360.f; direction_deg += 45.f) {
		SCOPED_TRACE(direction_deg);
		resetDetector();
		EXPECT_FALSE(updateAfterTriggerTime(tilt(kMaxTiltErrorDeg - 1.f, direction_deg)));
		resetDetector();
		EXPECT_TRUE(updateAfterTriggerTime(tilt(kMaxTiltErrorDeg + 1.f, direction_deg)));
	}
}

TEST_F(FailureDetectorTiltTest, CombinedRollAndPitch)
{
	// 45° roll and 45° pitch are each below the threshold but the tilt is ~60.0°
	EXPECT_FALSE(updateAfterTriggerTime(Quatf(Eulerf(math::radians(44.f), math::radians(44.f), 0.f))));
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(Quatf(Eulerf(math::radians(46.f), math::radians(46.f), 0.f))));
}

TEST_F(FailureDetectorTiltTest, TriggerUpsideDown)
{
	EXPECT_TRUE(updateAfterTriggerTime(tilt(180.f)));
}

TEST_F(FailureDetectorTiltTest, YawErrorIgnored)
{
	EXPECT_FALSE(updateAfterTriggerTime(Quatf(Eulerf(0.f, 0.f, M_PI_F)), Quatf()));
	EXPECT_FALSE(updateAfterTriggerTime(Quatf(Eulerf(math::radians(30.f), 0.f, M_PI_F)),
					    Quatf(Eulerf(math::radians(30.f), 0.f, 0.f))));
}

TEST_F(FailureDetectorTiltTest, ErrorRelativeToSetpoint)
{
	// Following a setpoint tilted beyond the threshold is fine
	EXPECT_FALSE(updateAfterTriggerTime(tilt(80.f), tilt(80.f)));
	EXPECT_FALSE(updateAfterTriggerTime(tilt(130.f), tilt(80.f)));
	EXPECT_FALSE(updateAfterTriggerTime(tilt(80.f + kMaxTiltErrorDeg - 1.f), tilt(80.f)));
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(tilt(80.f + kMaxTiltErrorDeg + 1.f), tilt(80.f)));

	// Staying level while a tilt is commanded is an error as well
	resetDetector();
	EXPECT_FALSE(updateAfterTriggerTime(Quatf(), tilt(kMaxTiltErrorDeg - 1.f, 90.f)));
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(Quatf(), tilt(kMaxTiltErrorDeg + 1.f, 90.f)));

	// Same tilt angle in a different direction: 50° roll vs 50° pitch is a ~65.6° error
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(tilt(50.f, 0.f), tilt(50.f, 90.f)));
}

TEST_F(FailureDetectorTiltTest, StaleSetpointFallsBackToLevel)
{
	EXPECT_FALSE(updateAfterTriggerTime(tilt(80.f), tilt(80.f), 400_ms));
	EXPECT_TRUE(updateAfterTriggerTime(tilt(80.f), tilt(80.f), 600_ms));
	resetDetector();
	EXPECT_FALSE(updateAfterTriggerTime(tilt(kMaxTiltErrorDeg - 1.f), tilt(80.f), 600_ms));
}

TEST_F(FailureDetectorTiltTest, NoTriggerWhenDisabled)
{
	setParams(-1, 0.1f);
	EXPECT_FALSE(updateAfterTriggerTime(tilt(180.f)));

	// 0 also disables for backwards compatibility
	setParams(0, 0.1f);
	EXPECT_FALSE(updateAfterTriggerTime(tilt(180.f)));
}

TEST_F(FailureDetectorTiltTest, TriggerOnlyAfterTriggerTime)
{
	// Each update advances the time by 10ms
	setParams(kMaxTiltErrorDeg, 0.5f);
	EXPECT_FALSE(update(tilt(90.f))); // t = 0
	advance(480_ms);
	EXPECT_FALSE(update(tilt(90.f))); // t = 490ms
	EXPECT_TRUE(update(tilt(90.f)));  // t = 500ms
}

TEST_F(FailureDetectorTiltTest, TriggerTimeRestartsWhenErrorRecovers)
{
	setParams(kMaxTiltErrorDeg, 0.5f);
	EXPECT_FALSE(update(tilt(90.f)));                   // t = 0
	advance(470_ms);
	EXPECT_FALSE(update(tilt(90.f)));                   // t = 480ms
	EXPECT_FALSE(update(tilt(kMaxTiltErrorDeg - 1.f))); // t = 490ms, short recovery
	EXPECT_FALSE(update(tilt(90.f)));                   // t = 500ms, trigger time restarts
	advance(480_ms);
	EXPECT_FALSE(update(tilt(90.f)));                   // t = 990ms
	EXPECT_TRUE(update(tilt(90.f)));                    // t = 1000ms
}

TEST_F(FailureDetectorTiltTest, ClearsImmediatelyWhenErrorRecovers)
{
	EXPECT_TRUE(updateAfterTriggerTime(tilt(90.f)));
	EXPECT_FALSE(update(tilt(kMaxTiltErrorDeg - 1.f)));
}

TEST_F(FailureDetectorTiltTest, ResetWithoutAttitudeControl)
{
	setParams(kMaxTiltErrorDeg, 0.5f);
	EXPECT_FALSE(update(tilt(90.f)));                    // t = 0
	advance(470_ms);
	EXPECT_FALSE(update(tilt(90.f)));                    // t = 480ms
	EXPECT_FALSE(update(tilt(90.f), Quatf(), 0, false)); // t = 490ms, e.g. Acro, pending trigger is dropped
	EXPECT_FALSE(update(tilt(90.f)));                    // t = 500ms, trigger time restarts
	advance(480_ms);
	EXPECT_FALSE(update(tilt(90.f)));                    // t = 990ms
	EXPECT_TRUE(update(tilt(90.f)));                     // t = 1000ms
	EXPECT_FALSE(update(tilt(90.f), Quatf(), 0, false)); // flag clears when attitude control is off
}

TEST_F(FailureDetectorTiltTest, TailsitterFixedWingSetpointInFixedWingFrame)
{
	_status.is_vtol = true;
	_status.is_vtol_tailsitter = true;
	_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	// Level fixed-wing flight is pitched by 90° in the multicopter frame
	EXPECT_FALSE(updateAfterTriggerTime(tailsitterMcFromFw(Quatf()), Quatf()));
	EXPECT_FALSE(updateAfterTriggerTime(tailsitterMcFromFw(tilt(kMaxTiltErrorDeg - 1.f)), Quatf()));
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(tailsitterMcFromFw(tilt(kMaxTiltErrorDeg + 1.f)), Quatf()));
	resetDetector();
	EXPECT_TRUE(updateAfterTriggerTime(tailsitterMcFromFw(tilt(kMaxTiltErrorDeg + 1.f, 90.f)), Quatf()));
}

TEST_F(FailureDetectorTiltTest, TailsitterTransitionSetpointInMulticopterFrame)
{
	_status.is_vtol = true;
	_status.is_vtol_tailsitter = true;
	_status.in_transition_mode = true;
	const Quatf q_transition = tailsitterMcFromFw(tilt(20.f, 90.f)); // 70° from hover towards fixed-wing

	for (const uint8_t vehicle_type : {vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, vehicle_status_s::VEHICLE_TYPE_FIXED_WING}) {
		SCOPED_TRACE(static_cast<int>(vehicle_type));
		_status.vehicle_type = vehicle_type;
		resetDetector();
		EXPECT_FALSE(updateAfterTriggerTime(q_transition, q_transition));
		EXPECT_TRUE(updateAfterTriggerTime(Quatf(), q_transition));
	}
}
