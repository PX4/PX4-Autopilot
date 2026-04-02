/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "framework.h"
#include <uORB/topics/vehicle_status.h>
#include "../ModeUtil/mode_requirements.hpp"

// to run: make tests TESTFILTER=failsafe_test

using namespace time_literals;

class FailsafeTester : public FailsafeBase
{
public:
	FailsafeTester(ModuleParams *parent) : FailsafeBase(parent) {}

protected:

	void checkStateAndMode(const hrt_abstime &time_us, const State &state,
			       const failsafe_flags_s &status_flags) override
	{
		CHECK_FAILSAFE(status_flags, manual_control_signal_lost, ActionOptions(Action::RTL).clearOn(ClearCondition::OnModeChangeOrDisarm));
		CHECK_FAILSAFE(status_flags, gcs_connection_lost, Action::Descend);

		if (state.user_intended_mode == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) {
			CHECK_FAILSAFE(status_flags, mission_failure, Action::Descend);
		}

		CHECK_FAILSAFE(status_flags, wind_limit_exceeded, ActionOptions(Action::RTL).allowUserTakeover(UserTakeoverAllowed::Never));
		CHECK_FAILSAFE(status_flags, battery_low_remaining_time, ActionOptions(Action::RTL).causedBy(Cause::RemainingFlightTimeLow));
		CHECK_FAILSAFE(status_flags, offboard_control_signal_lost, ActionOptions(Action::Hold));

		CHECK_FAILSAFE(status_flags, navigator_failure, ActionOptions(Action::Warn));
		CHECK_FAILSAFE(status_flags, fd_imbalanced_prop, ActionOptions(Action::None));

		// Geofence breach mapped to Land (models GF_ACTION=Land_mode, cannotBeDeferred).
		// Used by land_overrides_active_rtl_without_delay to verify A1 + clearDelayIfNeeded.
		CHECK_FAILSAFE(status_flags, geofence_breached, ActionOptions(Action::Land).cannotBeDeferred());

		_last_state_test = checkFailsafe(_caller_id_test, _last_state_test, status_flags.fd_motor_failure
						 && status_flags.fd_critical_failure, ActionOptions(Action::Terminate).cannotBeDeferred());
	}

	Action checkModeFallback(const failsafe_flags_s &status_flags, uint8_t user_intended_mode) const override
	{
		return Action::None;
	}

private:
	const int _caller_id_test{genCallerId()};
	bool _last_state_test{false};
};

/**
 * FailsafeTesterWithFallbackCapture extends FailsafeTester to intercept
 * notifyA3Fallback() calls without touching real uORB/MAVLink plumbing.
 * Each call increments fallback_count and records the last desired/actual pair.
 */
class FailsafeTesterWithFallbackCapture : public FailsafeTester
{
public:
	FailsafeTesterWithFallbackCapture(ModuleParams *parent) : FailsafeTester(parent) {}

	int fallback_count{0};
	FailsafeBase::Action last_desired{FailsafeBase::Action::None};
	FailsafeBase::Action last_actual{FailsafeBase::Action::None};

protected:
	void notifyA3Fallback(FailsafeBase::Action desired, FailsafeBase::Action actual) override
	{
		++fallback_count;
		last_desired = desired;
		last_actual  = actual;
		// Do NOT call the base: avoids real events::send/mavlink in unit tests.
	}
};

class FailsafeTest : public ::testing::Test

{
public:

	void SetUp() override
	{
		param_control_autosave(false);
		param_t param = param_handle(px4::params::COM_FAIL_ACT_T);

		float hold_delay = 5.f;
		param_set(param, &hold_delay);
	}

};


TEST_F(FailsafeTest, general)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;
	bool stick_override_request = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);

	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// manual control lost -> Hold, then RTL
	time += 10_ms;
	failsafe_flags.manual_control_signal_lost = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	time += 6_s;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// DL link lost -> Descend
	time += 10_ms;
	failsafe_flags.gcs_connection_lost = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);

	// DL link regained -> RTL (manual control still lost)
	time += 10_ms;
	failsafe_flags.gcs_connection_lost = false;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Manual control lost cleared -> keep RTL
	time += 10_ms;
	failsafe_flags.manual_control_signal_lost = false;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Mode change -> clear failsafe
	time += 10_ms;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
}

TEST_F(FailsafeTest, takeover)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;
	bool stick_override_request = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);

	// Mission failure -> no failsafe
	time += 10_ms;
	failsafe_flags.mission_failure = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Change to mission -> Hold, then Descend
	time += 10_ms;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	// Try stick movement during hold -> must be denied
	time += 3_s;
	stick_override_request = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	stick_override_request = false;
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	// Now Descend
	time += 3_s;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);

	// Move sticks -> user takeover active
	time += 10_ms;
	stick_override_request = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
	ASSERT_TRUE(failsafe.userTakeoverActive());
	stick_override_request = false;

	// Now the failsafe must clear as we switched mode
	time += 10_ms;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_FALSE(failsafe.userTakeoverActive());
}

TEST_F(FailsafeTest, takeover_denied)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;
	bool stick_override_request = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);

	// Wind limit exceeded -> RTL w/o delay and denying takeover
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);
	// Try takeover (mode switch + stick movements)
	time += 10_ms;
	stick_override_request = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	stick_override_request = false;
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Test termination
	failsafe_flags.fd_motor_failure = true;
	failsafe_flags.fd_critical_failure = true;
	time += 10_ms;
	stick_override_request = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	stick_override_request = false;
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Terminate);

	// Termination is final
	failsafe_flags.wind_limit_exceeded = false;
	failsafe_flags.fd_motor_failure = false;
	failsafe_flags.fd_critical_failure = false;
	state.armed = false;
	time += 10_ms;
	stick_override_request = true;
	updated_user_intented_mode = failsafe.update(time, state, false, stick_override_request, failsafe_flags);
	stick_override_request = false;
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Terminate);
}

TEST_F(FailsafeTest, can_takeover_degraded_failsafe)
{
	FailsafeTester failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;
	failsafe_flags_s failsafe_flags{};
	mode_util::getModeRequirements(state.vehicle_type, failsafe_flags); // Load mode requirements to degrade without valid position estimate
	bool user_intended_mode_updated = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);

	// Battery time low -> Hold for the delay
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);

	// Delay over -> RTL
	time += 5_s;
	failsafe_flags.battery_low_remaining_time = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Global position gets invalid -> Land
	time += 10_ms;
	failsafe_flags.global_position_invalid = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);

	// User wants takeover -> Altitude mode + Warning
	time += 10_ms;
	user_intended_mode_updated = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
	ASSERT_TRUE(failsafe.userTakeoverActive());
}

TEST_F(FailsafeTest, no_delay_for_warn)
{
	// Ensure there is no Hold/delay when the current action is Warn
	FailsafeTester failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_MANUAL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;
	failsafe_flags_s failsafe_flags{};
	bool user_intended_mode_updated = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);

	// Navigator failure -> Warn
	time += 10_ms;
	failsafe_flags.navigator_failure = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);

	// Imbalanced props -> Warn (no delay)
	time += 5_s;
	failsafe_flags.fd_imbalanced_prop = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
}

TEST_F(FailsafeTest, no_immediate_takeover_when_failsafe_on_mode_switch)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;
	bool user_intended_mode_updated = false;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);

	// Switch to offboard but no offboard signal -> No immediate user takeover flagged but rather Hold
	time += 10_ms;
	user_intended_mode_updated = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;
	failsafe_flags.offboard_control_signal_lost = true;
	updated_user_intented_mode = failsafe.update(time, state, user_intended_mode_updated, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	ASSERT_FALSE(failsafe.userTakeoverActive());
}

TEST_F(FailsafeTest, defer)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);

	const int defer_timeout_s = 10;
	failsafe.deferFailsafes(true, defer_timeout_s);
	ASSERT_TRUE(failsafe.getDeferFailsafes());
	ASSERT_FALSE(failsafe.failsafeDeferred());
	// GCS connection lost -> deferred
	time += 10_ms;
	failsafe_flags.gcs_connection_lost = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.getDeferFailsafes());
	ASSERT_TRUE(failsafe.failsafeDeferred());

	// Wait a bit, still deferred
	time += 5_s;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.getDeferFailsafes());
	ASSERT_TRUE(failsafe.failsafeDeferred());

	// Wait a bit, don't defer anymore -> failsafe triggered (hold)
	time += 1_s;
	failsafe.deferFailsafes(false, 0);
	ASSERT_FALSE(failsafe.getDeferFailsafes());
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	ASSERT_FALSE(failsafe.getDeferFailsafes());
	ASSERT_FALSE(failsafe.failsafeDeferred());

	// Cannot enable while in failsafe
	ASSERT_FALSE(failsafe.deferFailsafes(true, 0));

	// Still in hold
	time += 4_s;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);

	// Now changed to descend
	time += 4_s;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);

	// Clear failsafe
	failsafe_flags.gcs_connection_lost = false;
	time += 10_ms;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);


	// Defer and trigger timeout
	failsafe.deferFailsafes(true, defer_timeout_s);
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.failsafeDeferred());
	time += 1_s * defer_timeout_s + 10_ms;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);
	ASSERT_FALSE(failsafe.failsafeDeferred());
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = false;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Defer and clear failsafe -> no action
	failsafe.deferFailsafes(true, defer_timeout_s);
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.failsafeDeferred());
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = false;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_FALSE(failsafe.failsafeDeferred());
	failsafe.deferFailsafes(false, defer_timeout_s);
	time += 10_ms;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Defer and trigger non-deferrable failsafe
	failsafe.deferFailsafes(true, defer_timeout_s);
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.failsafeDeferred());
	time += 10_ms;
	failsafe_flags.fd_motor_failure = true;
	failsafe_flags.fd_critical_failure = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Terminate);
	ASSERT_FALSE(failsafe.failsafeDeferred());
}

TEST_F(FailsafeTest, defer_and_clear)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 3847124342;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);

	failsafe.deferFailsafes(true, -1);
	ASSERT_TRUE(failsafe.getDeferFailsafes());
	ASSERT_FALSE(failsafe.failsafeDeferred());
	// Manual control lost -> deferred
	time += 10_ms;
	failsafe_flags.manual_control_signal_lost = true;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_TRUE(failsafe.failsafeDeferred());

	// Clear flag (the failsafe action only clears on mode switch, but we still expect it to clear as it's being deferred)
	failsafe_flags.manual_control_signal_lost = false;
	time += 5_s;
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_FALSE(failsafe.failsafeDeferred());

	// Wait a bit, don't defer anymore -> no failsafe triggered
	time += 1_s;
	failsafe.deferFailsafes(false, 0);
	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_FALSE(failsafe.getDeferFailsafes());
	ASSERT_FALSE(failsafe.failsafeDeferred());
}

TEST_F(FailsafeTest, skip_failsafe)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Manual control lost while in RTL -> stay in RTL and only warn
	failsafe_flags.manual_control_signal_lost = true;

	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_user_intented_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
}

TEST_F(FailsafeTest, user_termination)
{
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	// User intended termination -> failsafe termination
	uint8_t updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	EXPECT_EQ(updated_user_intented_mode, state.user_intended_mode);
	EXPECT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Terminate);

	// Links lost during termination -> stay in termination
	failsafe_flags.gcs_connection_lost = true;
	failsafe_flags.manual_control_signal_lost = true;

	updated_user_intented_mode = failsafe.update(time, state, false, false, failsafe_flags);
	EXPECT_EQ(updated_user_intented_mode, state.user_intended_mode);
	EXPECT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Terminate);
}

TEST_F(FailsafeTest, conflict_higher_priority_wins)
{
	// Reproduces: a battery-critical failsafe (→RTL) and a higher-priority condition
	// (→Descend) become active simultaneously.  Verifies that the highest-severity action
	// is always selected (priority axiom A1/A5) regardless of which condition arrived first.
	//
	// In the real Failsafe class this maps to, e.g., battery-CRITICAL (→RTL, sev 6)
	// overlapping with a geofence breach whose GF_ACTION resolves to a Land/Descend.
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	// No failsafes active -> None
	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Battery remaining time low -> RTL (priority 6) after the Hold delay
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold); // waiting in Hold

	// Delay expires -> RTL
	time += 6_s;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Higher-priority condition becomes active (GCS lost -> Descend, priority 8)
	// Both battery-RTL (6) and Descend (8) are now active simultaneously.
	// Descend must dominate (axiom A1).
	time += 10_ms;
	failsafe_flags.gcs_connection_lost = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);

	// Higher-priority condition clears -> falls back to lower-priority battery RTL
	time += 10_ms;
	failsafe_flags.gcs_connection_lost = false;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);
}

TEST_F(FailsafeTest, conflict_geofence_breach_during_rtl)
{
	// Reproduces: a geofence breach (→RTL) occurring while the vehicle is already
	// executing RTL due to a previous failsafe.  Verifies that the duplicate RTL
	// condition does not restart the Hold/delay timer or oscillate the mode (axiom A4).
	//
	// In the real Failsafe class: vehicle is in AUTO_RTL (battery-critical failsafe),
	// then GF_ACTION=Return triggers.  Since we are already in RTL the framework must
	// downgrade the second RTL to Warn and let the ongoing RTL continue.
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	// Simulate: vehicle already executing RTL (Commander has set user_intended_mode)
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	// First RTL-level condition active (e.g. RC lost during mission -> RTL)
	failsafe_flags.manual_control_signal_lost = true;
	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	// Already in RTL: skip-RTL UX logic applies -> only Warn, no new Hold delay
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);

	// Second RTL-level condition becomes active (simulates geofence breach -> RTL)
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = true; // also maps to RTL in FailsafeTester
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	// Still Warn: both conditions are RTL-level and we are already in RTL (axiom A4)
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
	ASSERT_FALSE(failsafe.userTakeoverActive());

	// Second condition clears -> still Warn (first RC-lost condition still active)
	time += 10_ms;
	failsafe_flags.wind_limit_exceeded = false;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Warn);
}

TEST_F(FailsafeTest, rtl_fallback_to_land_when_home_invalid)
{
	// Reproduces: a battery-critical failsafe selects RTL, but the home position is
	// unavailable (GPS lost, home never set, etc.).  Verifies that the framework
	// automatically falls through to Land (priority axiom A3) so the vehicle descends
	// in place rather than being stuck or attempting RTL with no valid destination.
	FailsafeTester failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	failsafe_flags_s failsafe_flags{};
	// Load real per-mode requirements so home_position_invalid causes modeCanRun(RTL)
	// to return false, exercising the RTL→Land fallback path.
	mode_util::getModeRequirements(state.vehicle_type, failsafe_flags);

	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Trigger RTL-priority failsafe while home position is unavailable
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	failsafe_flags.home_position_invalid = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	// Hold first (COM_FAIL_ACT_T delay) – Hold (loiter) does not need home position
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);

	// Delay expires -> RTL cannot run (home invalid) -> framework falls to Land
	time += 6_s;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);
}

TEST_F(FailsafeTest, land_overrides_active_rtl_without_delay)
{
	// Reproduces: simultaneous geofence-Land (sev 7) and battery-RTL (sev 6) while the
	// vehicle is already executing RTL.  Verifies three properties:
	//
	//   A1: Land (sev 7) dominates RTL (sev 6) – the higher-severity action always wins.
	//
	//   A4 negative: the RTL-skip UX logic only applies when selected_action == RTL;
	//       it does NOT suppress a Land-severity condition, so Land is selected immediately.
	//
	//   clearDelayIfNeeded(): because _selected_action = RTL > Hold at the moment the
	//       geofence-Land action is registered, clearDelayIfNeeded() zeroes any newly
	//       set delay on that very cycle – no Hold interlude occurs.
	//
	// In the real Failsafe class this maps to GF_ACTION=Land_mode (→Land, sev 7) firing
	// while a battery-CRITICAL (→RTL, sev 6) is already being executed.
	FailsafeTester failsafe(nullptr);

	failsafe_flags_s failsafe_flags{};
	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	// No failsafes active
	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Battery remaining time low -> RTL-priority condition; Hold delay starts
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);

	// Delay expires -> RTL is now the active failsafe (_selected_action = RTL)
	time += 6_s;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// Geofence breach (→Land, sev 7) triggers while RTL (sev 6) is already active.
	// checkFailsafe() would normally set _current_delay = 5 s for the new action, but
	// clearDelayIfNeeded() immediately zeroes it because _selected_action = RTL > Hold.
	// Land > RTL, so A4 skip does not apply – Land is selected on this same cycle.
	time += 10_ms;
	failsafe_flags.geofence_breached = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);

	// Stable for subsequent cycles – no oscillation back to RTL or Hold
	time += 10_ms;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);

	// Geofence clears (clear_condition=WhenConditionClears) -> Land action removed;
	// only battery RTL remains – RTL resumes (A5: each subsystem is independent)
	time += 10_ms;
	failsafe_flags.geofence_breached = false;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);
}

TEST_F(FailsafeTest, rtl_fallback_to_descend_when_position_fully_lost)
{
	// Reproduces: navigation degrades in two steps while RTL is active:
	//   Step 1 – global_position_invalid: RTL cannot run → falls back to Land (A3).
	//   Step 2 – local_position_invalid_relaxed (also lost): Land cannot run → falls
	//            back to Descend (A3 second level).
	//
	// In both steps clearDelayIfNeeded() prevents a new Hold interlude because:
	//   • _selected_action > Hold already (RTL in step 1, Land in step 2).
	//   • AUTO_LOITER also requires global position, so Hold becomes unavailable
	//     when GPS is lost, providing a second guard against spurious delays.
	//
	// This models the scenario where a vehicle loses GPS (global position) and then
	// EKF velocity/position estimates also degrade (relaxed local position) during
	// an active emergency RTL – the deterministic fallback ensures the vehicle
	// always descends rather than entering an undefined state.
	FailsafeTester failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	failsafe_flags_s failsafe_flags{};
	// Load real per-mode requirements so position flags correctly block specific modes.
	mode_util::getModeRequirements(state.vehicle_type, failsafe_flags);

	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);

	// Battery failsafe triggers; global position still valid -> Hold delay
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);

	// Delay expires -> RTL active
	time += 6_s;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);

	// GPS signal lost mid-transition: global position invalid.
	// RTL requires global position -> cannot run.
	// Land does NOT require global position -> Land selected (A3 step 1, immediate).
	time += 10_ms;
	failsafe_flags.global_position_invalid = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);

	// EKF velocity estimate also degrades: relaxed local position invalid.
	// Land requires local_position_relaxed -> cannot run.
	// Descend only requires angular velocity + attitude (both valid) -> Descend
	// selected (A3 step 2, immediate – no Hold interlude).
	time += 10_ms;
	failsafe_flags.local_position_invalid_relaxed = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);

	// Descend is stable (no oscillation)
	time += 10_ms;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);
}

TEST_F(FailsafeTest, a3_fallback_events_emitted_when_position_fully_lost)
{
	// Verifies that notifyA3Fallback() – the hook that emits the operator-facing
	// "commander_failsafe_a3_fallback" event – is called with the correct arguments
	// for each step of the two-step GPS-lost-then-local-relaxed-lost degradation:
	//
	//   Step 1 (GPS lost):               RTL unavailable → fell back to Land
	//     → notifyA3Fallback(RTL, Land)  called exactly once
	//
	//   Step 2 (local relaxed also lost): RTL and Land unavailable → Descend
	//     → notifyA3Fallback(RTL, Descend) called exactly once
	//
	//   Stable cycles (flags unchanged): no further event emissions
	//
	// The test uses FailsafeTesterWithFallbackCapture which overrides
	// notifyA3Fallback() to record calls without sending real events/MAVLink.
	FailsafeTesterWithFallbackCapture failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed = true;
	state.user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time = 5_s;

	failsafe_flags_s failsafe_flags{};
	mode_util::getModeRequirements(state.vehicle_type, failsafe_flags);

	// ── Baseline: no failsafes ────────────────────────────────────────────────
	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::None);
	ASSERT_EQ(failsafe.fallback_count, 0);

	// ── Battery failsafe triggers; GPS still valid → Hold (delay) ────────────
	time += 10_ms;
	failsafe_flags.battery_low_remaining_time = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Hold);
	ASSERT_EQ(failsafe.fallback_count, 0); // no A3 fallback during Hold phase

	// ── Delay expires → RTL selected; GPS still valid ────────────────────────
	time += 6_s;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::RTL);
	ASSERT_EQ(failsafe.fallback_count, 0); // RTL can run, no fallback needed

	// ── Step 1: GPS lost mid-transition → RTL cannot run → Land ──────────────
	// A3 fallback: desired=RTL, actual=Land.
	// notifyA3Fallback(RTL, Land) must fire exactly once on this cycle.
	time += 10_ms;
	failsafe_flags.global_position_invalid = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);
	ASSERT_EQ(failsafe.fallback_count, 1);
	ASSERT_EQ(failsafe.last_desired, FailsafeBase::Action::RTL);
	ASSERT_EQ(failsafe.last_actual,  FailsafeBase::Action::Land);

	// ── Step 2: Relaxed local position also lost → Land cannot run → Descend ─
	// A3 fallback: desired=RTL (still the pool value), actual=Descend.
	// notifyA3Fallback(RTL, Descend) must fire exactly once more.
	time += 10_ms;
	failsafe_flags.local_position_invalid_relaxed = true;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);
	ASSERT_EQ(failsafe.fallback_count, 2);
	ASSERT_EQ(failsafe.last_desired, FailsafeBase::Action::RTL);
	ASSERT_EQ(failsafe.last_actual,  FailsafeBase::Action::Descend);

	// ── Stable: no further events on subsequent cycles (same flags) ───────────
	time += 10_ms;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Descend);
	ASSERT_EQ(failsafe.fallback_count, 2); // no new event: action unchanged
}

TEST_F(FailsafeTest, a3_fallback_event_fires_only_once_in_steady_state)
{
	// Verifies the anti-spam guard in update():
	//
	//   if (action_state.a3_desired_action != Action::None
	//       && action_state.action > _selected_action) { notifyA3Fallback(...); }
	//
	// Scenario: battery failsafe (→RTL) with GPS already invalid on the very
	// first update.  clearDelayIfNeeded() zeroes _current_delay because
	// !modeCanRun(LOITER) (AUTO_LOITER also requires global_position), so there
	// is no Hold interlude — Land is selected immediately on cycle 1.
	//
	// Cycle 1: _selected_action == None → action_state.action (Land) > None → fires once.
	// Cycle 2: _selected_action == Land → Land > Land is false → does NOT fire.
	// Cycle 3: same as cycle 2 → count stays frozen at 1.
	FailsafeTesterWithFallbackCapture failsafe(nullptr);

	FailsafeBase::State state{};
	state.armed                = true;
	state.user_intended_mode   = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
	state.vehicle_type         = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	hrt_abstime time           = 5_s;

	failsafe_flags_s failsafe_flags{};
	mode_util::getModeRequirements(state.vehicle_type, failsafe_flags);

	// Degraded state from the very first cycle:
	// - battery_low_remaining_time triggers RTL in checkStateAndMode()
	// - global_position_invalid blocks both AUTO_RTL and AUTO_LOITER
	//   → clearDelayIfNeeded() clears any delay; A3 falls through to Land
	failsafe_flags.battery_low_remaining_time = true;
	failsafe_flags.global_position_invalid    = true;

	// ── Cycle 1: first transition (RTL desired → Land selected via A3) ────────
	uint8_t updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);
	ASSERT_EQ(failsafe.fallback_count, 1);           // fired exactly once
	ASSERT_EQ(failsafe.last_desired, FailsafeBase::Action::RTL);
	ASSERT_EQ(failsafe.last_actual,  FailsafeBase::Action::Land);

	// ── Cycle 2: identical flags, steady state ────────────────────────────────
	// _selected_action == Land == action_state.action → guard false → no fire.
	time += 10_ms;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);
	ASSERT_EQ(failsafe.fallback_count, 1);           // must not increment

	// ── Cycle 3: identical flags, steady state ────────────────────────────────
	// Confirms the count stays frozen across arbitrarily many repeat updates.
	time += 10_ms;
	updated_mode = failsafe.update(time, state, false, false, failsafe_flags);
	ASSERT_EQ(updated_mode, state.user_intended_mode);
	ASSERT_EQ(failsafe.selectedAction(), FailsafeBase::Action::Land);
	ASSERT_EQ(failsafe.fallback_count, 1);           // must not increment
}
