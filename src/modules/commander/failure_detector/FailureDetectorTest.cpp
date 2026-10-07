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
 * @file FailureDetectorTest.cpp
 *
 * Comprehensive GTest functional tests and MC/DC verification for Commander's FailureDetector.
 * Authored by Member 1 for SE3002 SQE Assignment #02.
 */

#include <gtest/gtest.h>
#include "FailureDetector.hpp"

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_imu_status.h>

using namespace time_literals;

class FailureDetectorTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Set baseline default parameters
		setParamInt("FD_FAIL_P", 60);
		setParamInt("FD_FAIL_R", 60);
		setParamFloat("FD_FAIL_P_TTRI", 0.05f); // 50ms trigger for fast tests
		setParamFloat("FD_FAIL_R_TTRI", 0.05f);
		setParamInt("FD_EXT_ATS_EN", 1);
		setParamInt("FD_EXT_ATS_TRIG", 1900);
		setParamInt("FD_ESCS_EN", 1);
		setParamInt("FD_IMB_PROP_THR", 0); // disabled by default
		setParamInt("FD_ACT_EN", 1);
		setParamFloat("FD_ACT_MOT_THR", 0.5f);
		setParamFloat("FD_ACT_MOT_C2T", 10.0f);
		setParamInt("FD_ACT_MOT_TOUT", 50); // 50ms timeout for tests
	}

	void setParamInt(const char *name, int32_t val)
	{
		param_t p = param_find(name);

		if (p != PARAM_INVALID) {
			param_set(p, &val);
		}
	}

	void setParamFloat(const char *name, float val)
	{
		param_t p = param_find(name);

		if (p != PARAM_INVALID) {
			param_set(p, &val);
		}
	}

protected:
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};
	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_imu_status_s> _imu_status_pub{ORB_ID(vehicle_imu_status)};
};

class FailureDetectorTestable : public FailureDetector
{
public:
	FailureDetectorTestable(ModuleParams *parent = nullptr) : FailureDetector(parent) {}
	using ModuleParams::updateParams;
};

/* =========================================================================
 * DECISION A-D1: ESC Telemetry Timeout Detection (FailureDetector.cpp:288)
 * Expression: esc_was_valid && esc_timed_out && !esc_timeout_currently_flagged
 * Form: A && B && C
 * ========================================================================= */

// TC-A-01: (T, T, T) -> Decision = True (Timeout detected and flagged)
TEST_F(FailureDetectorTest, EscTimeout_MCDC_Pair_AllTrue)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Step 1: Establish valid ESC telemetry (esc_was_valid = True)
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 2.5f; // Positive current marks ESC valid
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	EXPECT_FALSE(detector.getStatusFlags().motor);

	// Step 2: Telemetry stops for >300ms (esc_timed_out = True, !flagged = True)
	esc.timestamp = now;
	esc.esc[0].timestamp = now - 350_ms; // 350ms old -> timed out
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Condition (T, T, T) evaluates to True
	EXPECT_TRUE(detector.getStatusFlags().motor);
	EXPECT_NE(detector.getMotorFailures() & (1 << 0), 0);
}

// TC-A-02: (F, T, T) -> Decision = False (Shows Condition A independence, Pair: TC-A-01/02)
TEST_F(FailureDetectorTest, EscTimeout_MCDC_Pair_A_False_NotValid)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// ESC telemetry arrives with stale timestamp but has NEVER reported current > 0
	// Thus esc_was_valid = False
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 0.0f; // Never validated!
	esc.esc[0].timestamp = now - 400_ms;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Outcome: (F && T && T) -> False
	EXPECT_FALSE(detector.getStatusFlags().motor);
	EXPECT_EQ(detector.getMotorFailures() & (1 << 0), 0);
}

// TC-A-03: (T, F, T) -> Decision = False (Shows Condition B independence, Pair: TC-A-01/03)
TEST_F(FailureDetectorTest, EscTimeout_MCDC_Pair_B_False_NotTimedOut)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Step 1: Validate ESC
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 2.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Step 2: Fresh telemetry arrives (<300ms, esc_timed_out = False)
	esc.esc[0].timestamp = now - 50_ms;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Outcome: (T && F && T) -> False
	EXPECT_FALSE(detector.getStatusFlags().motor);
	EXPECT_EQ(detector.getMotorFailures() & (1 << 0), 0);
}

// TC-A-04: (T, T, F) -> Decision = False (Shows Condition C independence, Pair: TC-A-01/04)
TEST_F(FailureDetectorTest, EscTimeout_MCDC_Pair_C_False_AlreadyFlagged)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Step 1: Validate ESC
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 2.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Step 2: Flag timeout first time (TC-A-01 path)
	esc.esc[0].timestamp = now - 400_ms;
	_esc_status_pub.publish(esc);
	bool changed1 = detector.update(vehicle_status, control_mode);
	EXPECT_TRUE(changed1);
	EXPECT_TRUE(detector.getStatusFlags().motor);

	// Step 3: Send same stale packet again. Now timeout is ALREADY flagged (!esc_timeout_currently_flagged = False)
	_esc_status_pub.publish(esc);
	bool changed2 = detector.update(vehicle_status, control_mode);

	// Outcome: (T && T && F) -> False (no new flag mutation)
	EXPECT_FALSE(changed2);
	EXPECT_TRUE(detector.getStatusFlags().motor);
}

/* =========================================================================
 * DECISION A-D2: Motor In-Flight Undercurrent Detection (FailureDetector.cpp:313)
 * Expression: throttle_above_threshold && current_too_low && !esc_timed_out
 * Form: A && B && C
 * ========================================================================= */

// TC-A-05: (T, T, T) -> Decision = True (Undercurrent detected, timer starts)
TEST_F(FailureDetectorTest, MotorUndercurrent_MCDC_Pair_AllTrue)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Validate ESC current telemetry support
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 1.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);

	// Commanded throttle = 0.8 (threshold = 0.5 -> throttle_above_threshold = True)
	actuator_motors_s motors{};
	motors.control[0] = 0.8f;
	_actuator_motors_pub.publish(motors);

	// Current = 0.5A (expected = 0.8 * 10.0 = 8.0A -> current_too_low = True)
	// Fresh telemetry -> !esc_timed_out = True
	esc.esc[0].esc_current = 0.5f;
	_esc_status_pub.publish(esc);

	detector.update(vehicle_status, control_mode);

	// Sleep past the 50ms undercurrent time threshold (FD_ACT_MOT_TOUT = 50ms)
	usleep(60000);
	esc.timestamp = hrt_absolute_time();
	esc.esc[0].timestamp = hrt_absolute_time();
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Outcome: Motor undercurrent failure triggered
	EXPECT_TRUE(detector.getStatusFlags().motor);
}

// TC-A-06: (F, T, T) -> Decision = False (Shows Condition A independence, Pair: TC-A-05/06)
TEST_F(FailureDetectorTest, MotorUndercurrent_MCDC_Pair_A_False_LowThrottle)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Validate ESC telemetry
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 1.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);

	// Throttle = 0.2 (< 0.5 -> throttle_above_threshold = False)
	actuator_motors_s motors{};
	motors.control[0] = 0.2f;
	_actuator_motors_pub.publish(motors);

	// Current = 0.1A (< 0.2 * 10 = 2.0A -> current_too_low would be True)
	esc.esc[0].esc_current = 0.1f;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Outcome: (F && T && T) -> False
	EXPECT_FALSE(detector.getStatusFlags().motor);
}

// TC-A-07: (T, F, T) -> Decision = False (Shows Condition B independence, Pair: TC-A-05/07)
TEST_F(FailureDetectorTest, MotorUndercurrent_MCDC_Pair_B_False_NormalCurrent)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Throttle = 0.8 (> 0.5 -> throttle_above_threshold = True)
	actuator_motors_s motors{};
	motors.control[0] = 0.8f;
	_actuator_motors_pub.publish(motors);

	// Current = 15.0A (expected 8.0A -> current_too_low = False)
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 15.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Outcome: (T && F && T) -> False
	EXPECT_FALSE(detector.getStatusFlags().motor);
}

// TC-A-08: (T, T, F) -> Decision = False (Shows Condition C independence, Pair: TC-A-05/08)
TEST_F(FailureDetectorTest, MotorUndercurrent_MCDC_Pair_C_False_TimedOut)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Validate ESC telemetry
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 2.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);

	// Throttle = 0.8 (True), Current = 0.5A (True)
	actuator_motors_s motors{};
	motors.control[0] = 0.8f;
	_actuator_motors_pub.publish(motors);

	// Telemetry is timed out (> 300ms old -> !esc_timed_out = False)
	esc.esc[0].esc_current = 0.5f;
	esc.esc[0].timestamp = now - 400_ms;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Undercurrent branch evaluates to (T && T && F) -> False: timer never started
	// Fresh telemetry resumes after stale period: clears timeout flag, undercurrent was never flagged
	now = hrt_absolute_time();
	esc.timestamp = now;
	esc.esc[0].timestamp = now;
	esc.esc[0].esc_current = 2.0f;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	EXPECT_FALSE(detector.getStatusFlags().motor);
}

/* =========================================================================
 * DECISION A-D3: External ATS Parachute Trigger (FailureDetector.cpp:147)
 * Expression: (pulse_width >= trigger_threshold) && (pulse_width < 3_ms)
 * Form: A && B
 * ========================================================================= */

// TC-A-09: (T, T) -> Decision = True (ATS pulse valid and above trigger)
TEST_F(FailureDetectorTest, AtsTrigger_MCDC_Pair_AllTrue)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Send trigger pulse: 1950us (>= 1900 is True, < 3000 is True)
	pwm_input_s pwm{};
	pwm.pulse_width = 1950;
	_pwm_input_pub.publish(pwm);

	// Update past 100ms hysteresis window
	for (int i = 0; i < 6; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(25000);
		_pwm_input_pub.publish(pwm);
	}

	EXPECT_TRUE(detector.getStatusFlags().ext);
}

// TC-A-10: (F, T) -> Decision = False (Shows Condition A independence, Pair: TC-A-09/10)
TEST_F(FailureDetectorTest, AtsTrigger_MCDC_Pair_A_False_BelowThreshold)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Pulse = 1500us (< 1900 is False, < 3000 is True)
	pwm_input_s pwm{};
	pwm.pulse_width = 1500;
	_pwm_input_pub.publish(pwm);

	for (int i = 0; i < 6; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(25000);
		_pwm_input_pub.publish(pwm);
	}

	// Outcome: (F && T) -> False
	EXPECT_FALSE(detector.getStatusFlags().ext);
}

// TC-A-11: (T, F) -> Decision = False (Shows Condition B independence, Pair: TC-A-09/11)
TEST_F(FailureDetectorTest, AtsTrigger_MCDC_Pair_B_False_OutOfRange)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Pulse = 3500us (>= 1900 is True, < 3000 is False)
	pwm_input_s pwm{};
	pwm.pulse_width = 3500;
	_pwm_input_pub.publish(pwm);

	for (int i = 0; i < 6; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(25000);
		_pwm_input_pub.publish(pwm);
	}

	// Outcome: (T && F) -> False
	EXPECT_FALSE(detector.getStatusFlags().ext);
}

/* =========================================================================
 * DECISION A-D4: Excessive Vehicle Roll Angle (FailureDetector.cpp:123)
 * Expression: (max_roll > FLT_EPSILON) && (fabsf(roll) > max_roll)
 * Form: A && B
 * ========================================================================= */

// TC-A-12: (T, T) -> Decision = True (Roll limit exceeded)
TEST_F(FailureDetectorTest, AttitudeRoll_MCDC_Pair_AllTrue)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Roll angle = 75 degrees (limit = 60 degrees -> fabsf(roll) > max_roll = True)
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(math::radians(75.f), 0.f, 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	// Update past 50ms hysteresis
	for (int i = 0; i < 5; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(20000);
		_attitude_pub.publish(att);
	}

	EXPECT_TRUE(detector.getStatusFlags().roll);
}

// TC-A-13: (F, T) -> Decision = False (Shows Condition A independence, Pair: TC-A-12/13)
TEST_F(FailureDetectorTest, AttitudeRoll_MCDC_Pair_A_False_CheckDisabled)
{
	// Disable roll check by setting threshold to 0
	setParamInt("FD_FAIL_R", 0);

	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Roll angle = 75 degrees
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(math::radians(75.f), 0.f, 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	for (int i = 0; i < 5; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(20000);
		_attitude_pub.publish(att);
	}

	// Outcome: (F && T) -> False (check disabled)
	EXPECT_FALSE(detector.getStatusFlags().roll);
}

// TC-A-14: (T, F) -> Decision = False (Shows Condition B independence, Pair: TC-A-12/14)
TEST_F(FailureDetectorTest, AttitudeRoll_MCDC_Pair_B_False_WithinLimit)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Roll angle = 30 degrees (limit = 60 degrees -> fabsf(roll) > max_roll = False)
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(math::radians(30.f), 0.f, 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	for (int i = 0; i < 5; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(20000);
		_attitude_pub.publish(att);
	}

	// Outcome: (T && F) -> False
	EXPECT_FALSE(detector.getStatusFlags().roll);
}

/* =========================================================================
 * ADDITIONAL STRUCTURAL & BRANCH COVERAGE TESTS
 * ========================================================================= */

// TC-A-15: Pitch threshold violation check
TEST_F(FailureDetectorTest, AttitudePitch_Failure)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Pitch angle = 70 degrees (limit = 60 degrees)
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(0.f, math::radians(70.f), 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	for (int i = 0; i < 5; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(20000);
		_attitude_pub.publish(att);
	}

	EXPECT_TRUE(detector.getStatusFlags().pitch);
}

// TC-A-16: Tailsitter transition disables attitude failure; Fixed-Wing rotates frame
TEST_F(FailureDetectorTest, AttitudeTailsitter_TransitionAndFixedWing)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.is_vtol_tailsitter = true;
	vehicle_status.in_transition_mode = true;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Extreme roll during tailsitter transition should be zeroed
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(math::radians(85.f), 0.f, 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	detector.update(vehicle_status, control_mode);
	EXPECT_FALSE(detector.getStatusFlags().roll);

	// In Fixed-Wing mode, tailsitter attitude is rotated 90 degrees around pitch
	vehicle_status.in_transition_mode = false;
	vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	// In tailsitter FW level flight, body pitch is 90 deg. After 90 deg rotation, level pitch becomes 0 deg.
	const matrix::Eulerf euler_fw(0.f, math::radians(90.f), 0.f);
	matrix::Quatf q_fw(euler_fw);
	q_fw.copyTo(att.q);
	_attitude_pub.publish(att);

	detector.update(vehicle_status, control_mode);
	EXPECT_FALSE(detector.getStatusFlags().pitch);
	EXPECT_FALSE(detector.getStatusFlags().roll);
}

// TC-A-17: Disarm state transition resets motor failure masks and timers
TEST_F(FailureDetectorTest, DisarmStateTransition_ResetsMasks)
{
	setParamInt("FD_ESCS_EN", 1);

	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Trigger motor timeout failure while armed
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 3.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	esc.esc[0].timestamp = now - 400_ms;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);
	EXPECT_TRUE(detector.getStatusFlags().motor);

	// Transition to Disarmed
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_DISARMED;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Motor flag and all status bits are cleanly reset upon disarm
	EXPECT_FALSE(detector.getStatusFlags().motor);
	EXPECT_FALSE(detector.getStatusFlags().arm_escs);
	EXPECT_EQ(detector.getStatus().value, 0);
	EXPECT_EQ(detector.getMotorStopMask(), 0);
}

// TC-A-18: ESC armed bitmask failure & reported error counter
TEST_F(FailureDetectorTest, EscHardwareFailures_Armed)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// 2 ESCs connected, but only 1 reported armed (bitmask mismatch)
	esc_status_s esc{};
	esc.timestamp = hrt_absolute_time();
	esc.esc_count = 2;
	esc.esc_armed_flags = 0b01; // ESC 2 not armed
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[1].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + 1;
	esc.esc[0].failures = 0;
	esc.esc[1].failures = 2; // ESC reports internal failure counter > 0
	_esc_status_pub.publish(esc);

	// Update past 300ms ESC hysteresis
	for (int i = 0; i < 8; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(45000);
		esc.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(esc);
	}

	EXPECT_TRUE(detector.getStatusFlags().arm_escs);
}

// TC-A-19: Attitude control disabled immediately clears attitude flags
TEST_F(FailureDetectorTest, AttitudeControlDisabled_ClearsFlags)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Trigger roll failure
	vehicle_attitude_s att{};
	const matrix::Eulerf euler(math::radians(75.f), 0.f, 0.f);
	matrix::Quatf q(euler);
	q.copyTo(att.q);
	_attitude_pub.publish(att);

	for (int i = 0; i < 5; i++) {
		detector.update(vehicle_status, control_mode);
		usleep(20000);
		_attitude_pub.publish(att);
	}
	EXPECT_TRUE(detector.getStatusFlags().roll);

	// Disable attitude control mode
	control_mode.flag_control_attitude_enabled = false;
	detector.update(vehicle_status, control_mode);

	// Flags should be immediately cleared
	EXPECT_FALSE(detector.getStatusFlags().roll);
	EXPECT_FALSE(detector.getStatusFlags().pitch);
	EXPECT_FALSE(detector.getStatusFlags().ext);
}

// TC-A-20: ESC Telemetry Recovery clears timeout flag (D20)
TEST_F(FailureDetectorTest, TelemetryRecovery_ClearsTimeoutFlag)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Step 1: Validate ESC
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 2.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Step 2: Trigger timeout
	esc.esc[0].timestamp = now - 400_ms;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);
	EXPECT_TRUE(detector.getStatusFlags().motor);

	// Step 3: Fresh telemetry resumes (!esc_timed_out && timeout_flagged)
	esc.timestamp = now;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Timeout flag should be cleared
	EXPECT_FALSE(detector.getStatusFlags().motor);
}

// TC-A-21: Imbalanced Propeller Detection (FailureDetector.cpp:188-244)
TEST_F(FailureDetectorTest, ImbalancedPropeller_Detection)
{
	setParamInt("FD_IMB_PROP_THR", 1);

	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	const uint32_t accel_id = 2147483;
	sensor_selection_s selection{};
	selection.accel_device_id = accel_id;
	_sensor_selection_pub.publish(selection);

	vehicle_imu_status_s imu{};
	imu.accel_device_id = accel_id;
	imu.timestamp = hrt_absolute_time();
	// High vibration on X & Y, low on Z:
	// metric = (std_x + std_y)/2 - std_z = (10 + 10)/2 - 1 = 9.0 > 1.0 threshold
	imu.var_accel[0] = 100.0f; // std = 10.0
	imu.var_accel[1] = 100.0f; // std = 10.0
	imu.var_accel[2] = 1.0f;   // std = 1.0
	_imu_status_pub.publish(imu);

	for (int i = 0; i < 25; i++) {
		usleep(20000);
		imu.timestamp = hrt_absolute_time();
		_imu_status_pub.publish(imu);
		detector.update(vehicle_status, control_mode);
	}

	EXPECT_TRUE(detector.getStatusFlags().imbalanced_prop);
	EXPECT_GT(detector.getImbalancedPropMetric(), 1.0f);
}

// TC-A-22: Actuator Function Index Out of Bounds (FailureDetector.cpp:274)
TEST_F(FailureDetectorTest, ActuatorFunction_OutOfBounds_Ignored)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	esc_status_s esc{};
	esc.timestamp = hrt_absolute_time();
	esc.esc_count = 1;
	// Function index exceeding actuator_motors_s::NUM_CONTROLS
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1 + actuator_motors_s::NUM_CONTROLS + 10;
	esc.esc[0].esc_current = 2.0f;
	_esc_status_pub.publish(esc);

	detector.update(vehicle_status, control_mode);

	// Out of bounds actuator is cleanly ignored, no failures set
	EXPECT_EQ(detector.getMotorFailures(), 0);
	EXPECT_FALSE(detector.getStatusFlags().motor);
}

// TC-A-23: Motor Undercurrent Reset Timer on Recovery (FailureDetector.cpp:320)
TEST_F(FailureDetectorTest, MotorUndercurrent_ResetTimerOnRecovery)
{
	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	hrt_abstime now = hrt_absolute_time();

	// Step 1: Establish valid current
	esc_status_s esc{};
	esc.timestamp = now;
	esc.esc_count = 1;
	esc.esc[0].actuator_function = actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1;
	esc.esc[0].esc_current = 1.0f;
	esc.esc[0].timestamp = now;
	_esc_status_pub.publish(esc);

	actuator_motors_s motors{};
	motors.control[0] = 0.8f;
	_actuator_motors_pub.publish(motors);

	// Start undercurrent (0.5A < 0.8 * 10 = 8.0A)
	esc.esc[0].esc_current = 0.5f;
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Step 2: Current recovers before the 50ms timeout fires
	usleep(10000); // 10ms < 50ms timeout
	esc.timestamp = hrt_absolute_time();
	esc.esc[0].timestamp = esc.timestamp;
	esc.esc[0].esc_current = 10.0f; // Recovered current above 8.0A
	_esc_status_pub.publish(esc);
	detector.update(vehicle_status, control_mode);

	// Timer resets, no failure flagged
	EXPECT_FALSE(detector.getStatusFlags().motor);
	EXPECT_EQ(detector.getMotorFailures(), 0);
}

// TC-A-24: Imbalanced Propeller Multi-Instance Fallback (FailureDetector.cpp:205-218)
TEST_F(FailureDetectorTest, ImbalancedPropeller_MismatchedDeviceId_InstanceSearch)
{
	setParamInt("FD_IMB_PROP_THR", 5);

	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Selected accelerometer ID is 11111
	sensor_selection_s selection{};
	selection.accel_device_id = 11111;
	_sensor_selection_pub.publish(selection);

	// Multi-instance publication:
	// Instance 0 has mismatched ID 22222
	// Instance 1 has matching ID 11111
	uORB::PublicationMulti<vehicle_imu_status_s> imu_multi_pub0{ORB_ID(vehicle_imu_status)};
	uORB::PublicationMulti<vehicle_imu_status_s> imu_multi_pub1{ORB_ID(vehicle_imu_status)};

	vehicle_imu_status_s imu0{};
	imu0.accel_device_id = 22222;
	imu0.timestamp = hrt_absolute_time();
	imu_multi_pub0.publish(imu0);

	vehicle_imu_status_s imu1{};
	imu1.accel_device_id = 11111;
	imu1.timestamp = hrt_absolute_time();
	imu_multi_pub1.publish(imu1);

	detector.update(vehicle_status, control_mode);

	// Instance search succeeds and finds matching instance 1
	EXPECT_FALSE(detector.getStatusFlags().imbalanced_prop);
}

// TC-A-25: Imbalanced Propeller Selected ID Matches No Instance (FailureDetector.cpp:205-210)
TEST_F(FailureDetectorTest, ImbalancedPropeller_MismatchedDeviceId_NoInstanceMatch)
{
	setParamInt("FD_IMB_PROP_THR", 5);

	FailureDetectorTestable detector;
	detector.updateParams();

	vehicle_status_s vehicle_status{};
	vehicle_status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;

	vehicle_control_mode_s control_mode{};
	control_mode.flag_control_attitude_enabled = true;

	// Selected accelerometer ID is 99999 (does not match any registered instance)
	sensor_selection_s selection{};
	selection.accel_device_id = 99999;
	_sensor_selection_pub.publish(selection);

	vehicle_imu_status_s imu{};
	imu.accel_device_id = 22222;
	imu.timestamp = hrt_absolute_time();
	_imu_status_pub.publish(imu);

	detector.update(vehicle_status, control_mode);

	// ChangeInstance(i) returns false for nonexistent instances, executing continue at line 209
	EXPECT_FALSE(detector.getStatusFlags().imbalanced_prop);
}


