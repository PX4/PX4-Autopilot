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
#include <cmath>

#include "ExternalSetpoint.hpp"

#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/external_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

// to run: make tests TESTFILTER=ExternalSetpoint

static constexpr uint8_t kPosctl = vehicle_status_s::NAVIGATION_STATE_POSCTL;
static constexpr uint8_t kMission = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;
static constexpr uint8_t kOffboard = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

class TestExternalSetpoint : public ExternalSetpoint
{
public:
	TestExternalSetpoint() : ExternalSetpoint(nullptr) {}
	void paramsChanged() { ModuleParams::updateParams(); }
};

class ExternalSetpointTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}

	/** Enable the feature and push the change into the object under test. */
	void enable(TestExternalSetpoint &es, bool enabled = true)
	{
		int32_t v = enabled ? 1 : 0;
		param_set(param_find("EXT_SP_EN"), &v);
		es.paramsChanged();
	}

	/** Publish an external setpoint. age_us shifts the timestamp into the past. */
	void publish(float vx, float vy, float vz, bool valid = true, uint64_t age_us = 0)
	{
		external_setpoint_s msg{};
		msg.timestamp = hrt_absolute_time() - age_us;
		msg.velocity[0] = vx;
		msg.velocity[1] = vy;
		msg.velocity[2] = vz;
		msg.acceleration[0] = NAN;
		msg.acceleration[1] = NAN;
		msg.acceleration[2] = NAN;
		msg.yaw = NAN;
		msg.yawspeed = NAN;
		msg.valid = valid;

		uORB::Publication<external_setpoint_s> pub{ORB_ID(external_setpoint)};
		pub.publish(msg);
	}

	/** A trajectory setpoint representing a mission leg: position and velocity commanded. */
	static trajectory_setpoint_s missionSetpoint()
	{
		trajectory_setpoint_s sp{};

		for (int i = 0; i < 3; i++) {
			sp.position[i] = 10.f + i;
			sp.velocity[i] = 1.f + i;
			sp.acceleration[i] = NAN;
		}

		sp.yaw = 0.5f;
		sp.yawspeed = NAN;
		return sp;
	}
};

TEST_F(ExternalSetpointTest, instantiation) { ExternalSetpoint es(nullptr); }

TEST_F(ExternalSetpointTest, disabledByDefaultIsNoOp)
{
	// GIVEN: a fresh, valid external setpoint but the feature left at its default
	TestExternalSetpoint es;
	publish(5.f, 5.f, 5.f);

	trajectory_setpoint_s sp = missionSetpoint();
	const trajectory_setpoint_s original = sp;

	// WHEN: the setpoint is run through fusion
	es.modifySetpoint(sp, kPosctl);

	// THEN: nothing is touched and the feature reports inactive
	EXPECT_FALSE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], original.velocity[0]);
	EXPECT_FLOAT_EQ(sp.position[0], original.position[0]);
}

TEST_F(ExternalSetpointTest, unsupportedModeIsNoOp)
{
	// GIVEN: the feature enabled with a fresh setpoint, but the vehicle in Offboard
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, 5.f, 5.f);

	trajectory_setpoint_s sp = missionSetpoint();
	const trajectory_setpoint_s original = sp;

	// WHEN: fusion runs in a mode that is not Position or Mission
	es.modifySetpoint(sp, kOffboard);

	// THEN: the setpoint is left alone, so Offboard keeps its own semantics
	EXPECT_FALSE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], original.velocity[0]);
	EXPECT_FLOAT_EQ(sp.position[0], original.position[0]);
}

TEST_F(ExternalSetpointTest, staleSetpointIsNoOp)
{
	// GIVEN: an external setpoint far older than the default EXT_SP_TIMEOUT of 0.5 s
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, 5.f, 5.f, true, 10_s);

	trajectory_setpoint_s sp = missionSetpoint();
	const trajectory_setpoint_s original = sp;

	// WHEN: fusion runs
	es.modifySetpoint(sp, kPosctl);

	// THEN: the underlying setpoint resumes untouched
	EXPECT_FALSE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], original.velocity[0]);
	EXPECT_FLOAT_EQ(sp.position[0], original.position[0]);
}

TEST_F(ExternalSetpointTest, invalidSetpointIsNoOp)
{
	// GIVEN: a fresh setpoint explicitly marked invalid
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, 5.f, 5.f, false);

	trajectory_setpoint_s sp = missionSetpoint();
	const trajectory_setpoint_s original = sp;

	es.modifySetpoint(sp, kPosctl);

	EXPECT_FALSE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], original.velocity[0]);
}

TEST_F(ExternalSetpointTest, fusesVelocityAndReleasesPositionInMission)
{
	// GIVEN: a mission leg with both position and velocity commanded
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, -5.f, 0.5f);

	trajectory_setpoint_s sp = missionSetpoint();

	// WHEN: a fresh external setpoint is fused in Mission mode
	es.modifySetpoint(sp, kMission);

	// THEN: velocity is overridden on every commanded axis
	EXPECT_TRUE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], 5.f);
	EXPECT_FLOAT_EQ(sp.velocity[1], -5.f);
	EXPECT_FLOAT_EQ(sp.velocity[2], 0.5f);

	// AND: the matching position axes are released, so the position controller's
	// P-loop cannot drag the vehicle back toward the blocked waypoint
	EXPECT_FALSE(PX4_ISFINITE(sp.position[0]));
	EXPECT_FALSE(PX4_ISFINITE(sp.position[1]));
	EXPECT_FALSE(PX4_ISFINITE(sp.position[2]));
}

TEST_F(ExternalSetpointTest, fusesInPositionMode)
{
	TestExternalSetpoint es;
	enable(es);
	publish(3.f, 0.f, 0.f);

	trajectory_setpoint_s sp = missionSetpoint();
	es.modifySetpoint(sp, kPosctl);

	EXPECT_TRUE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], 3.f);
}

TEST_F(ExternalSetpointTest, uncommandedAxesAreLeftUntouched)
{
	// GIVEN: an external setpoint that commands only the North axis
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, NAN, NAN);

	trajectory_setpoint_s sp = missionSetpoint();
	const trajectory_setpoint_s original = sp;

	// WHEN: fusion runs
	es.modifySetpoint(sp, kMission);

	// THEN: only the commanded axis is overridden and released
	EXPECT_TRUE(es.isActive());
	EXPECT_FLOAT_EQ(sp.velocity[0], 5.f);
	EXPECT_FALSE(PX4_ISFINITE(sp.position[0]));

	// AND: the uncommanded axes keep both their velocity and their position
	EXPECT_FLOAT_EQ(sp.velocity[1], original.velocity[1]);
	EXPECT_FLOAT_EQ(sp.velocity[2], original.velocity[2]);
	EXPECT_FLOAT_EQ(sp.position[1], original.position[1]);
	EXPECT_FLOAT_EQ(sp.position[2], original.position[2]);
}

TEST_F(ExternalSetpointTest, disablingMidStreamRestoresBaseline)
{
	// GIVEN: fusion active on a fresh stream
	TestExternalSetpoint es;
	enable(es);
	publish(5.f, 0.f, 0.f);

	trajectory_setpoint_s sp = missionSetpoint();
	es.modifySetpoint(sp, kMission);
	ASSERT_TRUE(es.isActive());

	// WHEN: the feature is disabled while the stream is still fresh
	enable(es, false);

	trajectory_setpoint_s sp2 = missionSetpoint();
	const trajectory_setpoint_s original = sp2;
	es.modifySetpoint(sp2, kMission);

	// THEN: the baseline setpoint is handed through unmodified again
	EXPECT_FALSE(es.isActive());
	EXPECT_FLOAT_EQ(sp2.velocity[0], original.velocity[0]);
	EXPECT_FLOAT_EQ(sp2.position[0], original.position[0]);
}
