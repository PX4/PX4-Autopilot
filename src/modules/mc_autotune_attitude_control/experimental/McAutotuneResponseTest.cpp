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
#include <hrt_work.h>
#include <sys/boardctl.h>
#include <cerrno>

#include "mc_autotune_attitude_control.hpp"



class McAutotuneAttitudeControlTest : public ::testing::Test
{
public:
	static void SetUpTestSuite() { hrt_work_queue_init(); }

protected:
	void SetUp() override
	{
		param_reset_all();
		const float timeout = 20.f;
		param_set_no_notification(param_find("MC_AT_TIMEOUT"), &timeout);
		_autotune.updateParams();
	}

	using State = McAutotuneAttitudeControl::state;
	McAutotuneAttitudeControl _autotune;
	static constexpr hrt_abstime start_time = 1_s;

	void setState(State state, bool armed = true)
	{
		_autotune._state = state;
		_autotune._state_start_time = start_time;
		_autotune._start_flight_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		_autotune._nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		_autotune._armed = armed;
	}

	State state() const { return _autotune._state; }
	void update(hrt_abstime elapsed) { _autotune.updateStateMachine(start_time + elapsed); }
	void land(bool armed)
	{
		_autotune._nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		_autotune._armed = armed;
	}

	void setCandidateGains()
	{
		_autotune._rate_k = matrix::Vector3f(.12f, .13f, .18f);
		_autotune._rate_i = matrix::Vector3f(1.f, 1.f, 1.f);
		_autotune._rate_d = matrix::Vector3f(.02f, .02f, 0.f);
		_autotune._att_p = matrix::Vector3f(6.f, 6.f, 5.f);
	}

	float rollP() const
	{
		float value = NAN;
		param_get(param_find("MC_ROLLRATE_P"), &value);
		return value;
	}

	void applyInAir() { _autotune.backupAndSaveGainsToParams(); }
	void setInvalidGain(int group, int axis, float value)
	{
		matrix::Vector3f *gains[] = {&_autotune._rate_k, &_autotune._rate_i, &_autotune._rate_d, &_autotune._att_p};
		(*gains[group])(axis) = value;
	}
	bool gainsGood() const { return _autotune.areGainsGood(); }

	void finishWithoutSamples()
	{
		_autotune._state = State::fail;
		_autotune._state_start_time = hrt_absolute_time() - 3_s;
		_autotune.Run();
	}

	void loseResponseStream()
	{
		parameter_update_s update{};
		_autotune._parameter_update_sub.copy(&update);
		setState(State::roll);
		_autotune._experiment_active = true;
		_autotune._excitation_active = true;
		_autotune._response_time = hrt_absolute_time() - 1_s;
		_autotune.Run();
	}

};

TEST_F(McAutotuneAttitudeControlTest, AppliesGainsAfterLandingWithoutReportingFailure)
{
	const float original = rollP();
	setCandidateGains();
	setState(State::wait_for_disarm);
	land(true);
	update(30_s);
	EXPECT_EQ(state(), State::wait_for_disarm);
	EXPECT_FLOAT_EQ(rollP(), original);
	land(false);
	update(31_s);
	EXPECT_EQ(state(), State::complete);
	EXPECT_FLOAT_EQ(rollP(), .12f);
	update(32_s);
	EXPECT_EQ(state(), State::complete);
	update(34_s);
	EXPECT_EQ(state(), State::idle);
}

TEST_F(McAutotuneAttitudeControlTest, ModeChangeDuringInAirTestRestoresPreviousGains)
{
	const float original = rollP();
	setCandidateGains();
	applyInAir();
	ASSERT_FLOAT_EQ(rollP(), .12f);
	setState(State::test);
	land(true);
	update(500_ms);
	EXPECT_EQ(state(), State::fail);
	EXPECT_FLOAT_EQ(rollP(), original);
}

TEST_F(McAutotuneAttitudeControlTest, RejectsNonFiniteGainOnEveryAxis)
{
	setCandidateGains();
	ASSERT_TRUE(gainsGood());

	for (int group = 0; group < 4; ++group) {
		for (int axis = 0; axis < 3; ++axis) {
			for (float value : {NAN, INFINITY, -INFINITY}) {
				setCandidateGains();
				setInvalidGain(group, axis, value);
				EXPECT_FALSE(gainsGood()) << "gain group " << group << ", axis " << axis;
			}
		}
	}
}

TEST_F(McAutotuneAttitudeControlTest, ModeChangeDuringIdentificationStillAborts)
{
	const float original = rollP();
	setCandidateGains();
	setState(State::roll);
	land(true);
	update(500_ms);
	EXPECT_EQ(state(), State::fail);
	EXPECT_FLOAT_EQ(rollP(), original);
	update(3_s);
	EXPECT_EQ(state(), State::idle);
}

TEST_F(McAutotuneAttitudeControlTest, IdentificationTimeoutDoesNotApplyGains)
{
	const float original = rollP();
	setCandidateGains();
	setState(State::yaw);
	update(21_s);
	EXPECT_EQ(state(), State::fail);
	EXPECT_FLOAT_EQ(rollP(), original);
}

TEST_F(McAutotuneAttitudeControlTest, MissingGainsCannotPassVerification)
{
	const float original = rollP();
	setState(State::verification);
	update(100_ms);
	EXPECT_EQ(state(), State::fail);
	EXPECT_FLOAT_EQ(rollP(), original);
}

TEST_F(McAutotuneAttitudeControlTest, SuccessfulInAirTestIsNotRevertedByLaterModeChange)
{
	setCandidateGains();
	applyInAir();
	setState(State::test);
	update(5_s);
	EXPECT_EQ(state(), State::complete);
	land(true);
	update(6_s);
	EXPECT_EQ(state(), State::complete);
	EXPECT_FLOAT_EQ(rollP(), .12f);
}


TEST_F(McAutotuneAttitudeControlTest, ReportsIdleWithoutControlSamplesSoAnotherTuneCanStart)
{
	uORB::Subscription status_sub{ORB_ID(autotune_attitude_control_status)};
	finishWithoutSamples();
	autotune_attitude_control_status_s status{};
	ASSERT_TRUE(status_sub.copy(&status));
	EXPECT_EQ(status.state, autotune_attitude_control_status_s::STATE_IDLE);
	EXPECT_EQ(state(), State::idle);
}

TEST_F(McAutotuneAttitudeControlTest, MissingResponseStopsExcitationAndPublishesFailure)
{
	uORB::Subscription status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription excitation_sub{ORB_ID(autotune_excitation)};
	loseResponseStream();
	autotune_attitude_control_status_s status{};
	ASSERT_TRUE(status_sub.copy(&status));
	EXPECT_EQ(status.state, autotune_attitude_control_status_s::STATE_FAIL);
	autotune_excitation_s excitation{};
	ASSERT_TRUE(excitation_sub.copy(&excitation));
	EXPECT_EQ(excitation.timestamp, 0u);
	EXPECT_FLOAT_EQ(matrix::Vector3f(excitation.torque).norm(), 0.f);
}
