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
#include <PnGuidance.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;
using through::AttitudeCommand;
using through::PnGuidance;

namespace
{

constexpr float g = PnGuidance::GRAVITY;

PnGuidance makeGuidance()
{
	PnGuidance guidance;
	PnGuidance::Params params{};
	params.thr_min = 0.3f;
	params.thr_max = 0.9f;
	params.tilt_max = math::radians(60.f);
	params.k_vdamp = 1.5f;
	params.pn_n = 3.5f;
	params.a_max = 9.8f;
	params.v_max = 10.f;
	params.k_vlim = 2.f;
	params.vc_min = 1.f;
	guidance.setParams(params);
	return guidance;
}

float tiltOf(const Quatf &q)
{
	// angle between body z axis and the world down axis
	const Vector3f body_z = Dcmf(q).col(2);
	return acosf(math::constrain(body_z(2), -1.f, 1.f));
}

} // namespace

TEST(ThroughPnGuidanceTest, HoverAtRestIsLevelAtHoverThrust)
{
	const PnGuidance guidance = makeGuidance();
	const float hover_thrust = 0.6f;

	const Vector3f a = guidance.hoverAcceleration(Vector3f(0.f, 0.f, 0.f));
	const AttitudeCommand cmd = guidance.forceToAttitude(PnGuidance::specificForce(a), 0.f, hover_thrust);

	EXPECT_NEAR(cmd.thrust, hover_thrust, 1e-5f);
	EXPECT_NEAR(tiltOf(cmd.q_d), 0.f, 1e-4f);
	EXPECT_FALSE(cmd.tilt_limited);
	EXPECT_FALSE(cmd.thrust_saturated);
	EXPECT_NEAR(cmd.f_cmd(2), -g, 1e-4f);
}

TEST(ThroughPnGuidanceTest, HoverDampsHorizontalVelocity)
{
	const PnGuidance guidance = makeGuidance();

	// moving north: the hover command must accelerate south, i.e. pitch nose up
	const Vector3f a = guidance.hoverAcceleration(Vector3f(1.f, 0.f, 0.f));
	EXPECT_NEAR(a(0), -1.5f, 1e-6f);

	const AttitudeCommand cmd = guidance.forceToAttitude(PnGuidance::specificForce(a), 0.f, 0.6f);
	const Eulerf euler(cmd.q_d);
	EXPECT_GT(euler.theta(), 0.f);
	EXPECT_NEAR(euler.phi(), 0.f, 1e-5f);
}

TEST(ThroughPnGuidanceTest, YawSetpointIsApplied)
{
	const PnGuidance guidance = makeGuidance();
	const float yaw = math::radians(120.f);

	const AttitudeCommand cmd = guidance.forceToAttitude(Vector3f(0.f, 0.f, -g), yaw, 0.6f);
	EXPECT_NEAR(Eulerf(cmd.q_d).psi(), yaw, 1e-4f);
}

TEST(ThroughPnGuidanceTest, TiltLimitClampsToMaximumKeepingAzimuth)
{
	const PnGuidance guidance = makeGuidance();

	// strong eastward demand: 80 deg from vertical
	const float angle = math::radians(80.f);
	const Vector3f f(0.f, g * sinf(angle), -g * cosf(angle));
	const AttitudeCommand cmd = guidance.forceToAttitude(f, 0.f, 0.6f);

	EXPECT_TRUE(cmd.tilt_limited);
	EXPECT_NEAR(tiltOf(cmd.q_d), math::radians(60.f), 1e-3f);

	// limited specific force keeps the eastward direction
	EXPECT_NEAR(cmd.f_cmd(0), 0.f, 1e-4f);
	EXPECT_GT(cmd.f_cmd(1), 0.f);
	EXPECT_LT(cmd.f_cmd(2), 0.f);
}

TEST(ThroughPnGuidanceTest, ThrustIsClampedAndFlagged)
{
	const PnGuidance guidance = makeGuidance();
	const float hover_thrust = 0.6f;

	const AttitudeCommand high = guidance.forceToAttitude(Vector3f(0.f, 0.f, -2.f * g), 0.f, hover_thrust);
	EXPECT_NEAR(high.thrust, 0.9f, 1e-6f);
	EXPECT_TRUE(high.thrust_saturated);

	const AttitudeCommand low = guidance.forceToAttitude(Vector3f(0.f, 0.f, -0.2f * g), 0.f, hover_thrust);
	EXPECT_NEAR(low.thrust, 0.3f, 1e-6f);
	EXPECT_TRUE(low.thrust_saturated);
}

TEST(ThroughPnGuidanceTest, LosYawFollowsAzimuth)
{
	const Vector3f u = Vector3f(1.f, 1.f, 0.5f).normalized();
	EXPECT_NEAR(PnGuidance::losYaw(u, 0.f), math::radians(45.f), 1e-5f);
}

TEST(ThroughPnGuidanceTest, LosYawHeldWhenTargetBelow)
{
	// target almost straight below: keep the previous yaw
	const Vector3f u = Vector3f(0.05f, -0.05f, 1.f).normalized();
	const float yaw_prev = math::radians(30.f);
	EXPECT_FLOAT_EQ(PnGuidance::losYaw(u, yaw_prev), yaw_prev);
}

TEST(ThroughPnGuidanceTest, YawRateLimitTakesShortestWay)
{
	const float max_rate = math::radians(60.f);
	const float dt = 0.1f;

	// small step reaches the target
	EXPECT_NEAR(PnGuidance::rateLimitYaw(0.f, math::radians(3.f), max_rate, dt), math::radians(3.f), 1e-6f);

	// large step is limited to max_rate * dt
	EXPECT_NEAR(PnGuidance::rateLimitYaw(0.f, math::radians(90.f), max_rate, dt), math::radians(6.f), 1e-6f);

	// across +-pi: from 170 deg to -170 deg goes through 180 (positive direction)
	const float yaw = PnGuidance::rateLimitYaw(math::radians(170.f), math::radians(-170.f), max_rate, dt);
	EXPECT_NEAR(wrap_pi(yaw - math::radians(176.f)), 0.f, 1e-5f);
}

TEST(ThroughPnGuidanceTest, HorizontalLosFixedThrustBalancesGravity)
{
	const PnGuidance guidance = makeGuidance();
	const float hover_thrust = 0.6f;
	const float thr_base = 0.7f;

	// a_perp = 0 (no LOS rotation), horizontal LOS north, at rest
	const Vector3f u(1.f, 0.f, 0.f);
	const through::GuidanceCommand gc = guidance.guide(u, Vector3f(), Vector3f(), thr_base, hover_thrust);

	EXPECT_NEAR(gc.a_perp.norm(), 0.f, 1e-6f);
	EXPECT_FALSE(gc.a_perp_limited);

	// no vertical acceleration, forward acceleration from the excess thrust
	const float f0 = g * thr_base / hover_thrust;
	EXPECT_NEAR(gc.a(2), 0.f, 1e-4f);
	EXPECT_NEAR(gc.s, sqrtf(f0 * f0 - g * g), 1e-3f);

	const AttitudeCommand cmd = guidance.forceToAttitude(PnGuidance::specificForce(gc.a), 0.f, hover_thrust);
	EXPECT_NEAR(cmd.thrust, thr_base, 1e-4f);
	EXPECT_FALSE(cmd.thrust_saturated);
	EXPECT_LT(Eulerf(cmd.q_d).theta(), 0.f); // nose down to accelerate north
}

TEST(ThroughPnGuidanceTest, DiveAlongLosKeepsBaseThrust)
{
	const PnGuidance guidance = makeGuidance();
	const float hover_thrust = 0.6f;
	const float thr_base = 0.7f;

	// LOS 31 deg below the horizon
	const float el = math::radians(31.f);
	const Vector3f u(cosf(el), 0.f, sinf(el));
	const through::GuidanceCommand gc = guidance.guide(u, Vector3f(), Vector3f(), thr_base, hover_thrust);

	// acceleration stays on the LOS and |f| = f0
	EXPECT_NEAR(gc.a.cross(u).norm(), 0.f, 1e-4f);
	EXPECT_GT(gc.s, 0.f);
	EXPECT_NEAR(PnGuidance::specificForce(gc.a).norm(), g * thr_base / hover_thrust, 1e-3f);
}

TEST(ThroughPnGuidanceTest, InsufficientBaseThrustTakesMinimumThrustSolution)
{
	const PnGuidance guidance = makeGuidance();
	const float hover_thrust = 0.6f;
	const float thr_base = 0.5f; // below hover: disc < 0 for a horizontal LOS

	const Vector3f u(1.f, 0.f, 0.f);
	const through::GuidanceCommand gc = guidance.guide(u, Vector3f(), Vector3f(), thr_base, hover_thrust);

	EXPECT_NEAR(gc.s, 0.f, 1e-5f);
	const AttitudeCommand cmd = guidance.forceToAttitude(PnGuidance::specificForce(gc.a), 0.f, hover_thrust);
	EXPECT_GT(cmd.thrust, thr_base);
	EXPECT_NEAR(cmd.thrust, hover_thrust, 1e-4f);
}

TEST(ThroughPnGuidanceTest, SpeedLimitReducesAlongLosAcceleration)
{
	const PnGuidance guidance = makeGuidance();
	const Vector3f u(1.f, 0.f, 0.f);

	const through::GuidanceCommand slow = guidance.guide(u, Vector3f(), Vector3f(2.f, 0.f, 0.f), 0.7f, 0.6f);
	const through::GuidanceCommand fast = guidance.guide(u, Vector3f(), Vector3f(9.8f, 0.f, 0.f), 0.7f, 0.6f);
	const through::GuidanceCommand over = guidance.guide(u, Vector3f(), Vector3f(12.f, 0.f, 0.f), 0.7f, 0.6f);

	EXPECT_LT(fast.s, slow.s);
	EXPECT_NEAR(fast.s, 2.f * (10.f - 9.8f), 1e-4f);
	EXPECT_LT(over.s, 0.f);
	EXPECT_NEAR(over.vc, 12.f, 1e-5f);
}

TEST(ThroughPnGuidanceTest, PnAccelerationTurnsTowardsLosRotation)
{
	const PnGuidance guidance = makeGuidance();

	// LOS north, rotating towards east (omega about down): a_perp must point east
	const Vector3f u(1.f, 0.f, 0.f);
	const Vector3f omega(0.f, 0.f, 0.1f);
	const Vector3f v(5.f, 0.f, 0.f);
	const through::GuidanceCommand gc = guidance.guide(u, omega, v, 0.7f, 0.6f);

	EXPECT_NEAR(gc.a_perp(0), 0.f, 1e-5f);
	EXPECT_NEAR(gc.a_perp(1), 3.5f * 5.f * 0.1f, 1e-4f);
	EXPECT_NEAR(gc.a_perp(2), 0.f, 1e-5f);
	EXPECT_FALSE(gc.a_perp_limited);

	// opening (vc < 0): PN uses vc_min
	const through::GuidanceCommand opening = guidance.guide(u, omega, -v, 0.7f, 0.6f);
	EXPECT_NEAR(opening.vc, -5.f, 1e-5f);
	EXPECT_NEAR(opening.a_perp(1), 3.5f * 1.f * 0.1f, 1e-4f);
}

TEST(ThroughPnGuidanceTest, LateralAccelerationIsLimited)
{
	const PnGuidance guidance = makeGuidance();
	const Vector3f u(1.f, 0.f, 0.f);

	// |a_perp| above a_max is scaled down keeping the direction
	const through::GuidanceCommand gc = guidance.guide(u, Vector3f(0.f, 0.f, 1.f), Vector3f(10.f, 0.f, 0.f), 0.7f, 0.6f);
	EXPECT_TRUE(gc.a_perp_limited);
	EXPECT_NEAR(gc.a_perp.norm(), 9.8f, 1e-3f);
	EXPECT_GT(gc.a_perp(1), 0.f);
}

TEST(ThroughPnGuidanceTest, PerpendicularDemandIsLimitedToMaximumThrust)
{
	PnGuidance guidance = makeGuidance();
	PnGuidance::Params params = guidance.params();
	params.a_max = 100.f;
	guidance.setParams(params);

	// horizontal LOS, strong upward a_perp: |a_perp - g_perp| would exceed f_max = 1.5 g
	const Vector3f u(1.f, 0.f, 0.f);
	const Vector3f omega(0.f, 1.f, 0.f); // u rotating up: omega x u = -z (up)
	const through::GuidanceCommand gc = guidance.guide(u, omega, Vector3f(10.f, 0.f, 0.f), 0.7f, 0.6f);

	EXPECT_TRUE(gc.a_perp_limited);
	EXPECT_LT(gc.a_perp(2), 0.f);
	const Vector3f f = PnGuidance::specificForce(gc.a);
	const Vector3f f_perp = f - f.dot(u) * u;
	EXPECT_NEAR(f_perp.norm(), 1.5f * g, 1e-3f);
}
