/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
#include <AttitudeControl.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	Vector3f rate_setpoint = attitude_control.update(Quatf());
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlConvergenceTest : public ::testing::Test
{
public:
	AttitudeControlConvergenceTest()
	{
		_attitude_control.setProportionalGain(Vector3f(.5f, .6f, .3f), .4f);
		_attitude_control.setRateLimit(Vector3f(100.f, 100.f, 100.f));
	}

	void checkConvergence()
	{
		int i; // need function scope to check how many steps
		Vector3f rate_setpoint(1000.f, 1000.f, 1000.f);

		_attitude_control.setAttitudeSetpoint(_quat_goal, 0.f);

		for (i = 100; i > 0; i--) {
			// run attitude control to get rate setpoints
			const Vector3f rate_setpoint_new = _attitude_control.update(_quat_state);
			// rotate the simulated state quaternion according to the rate setpoint
			_quat_state = _quat_state * Quatf(AxisAnglef(rate_setpoint_new));
			_quat_state = -_quat_state; // produce intermittent antipodal quaternion states to test against unwinding problem

			// expect the error and hence also the output to get smaller with each iteration
			if (rate_setpoint_new.norm() >= rate_setpoint.norm()) {
				break;
			}

			rate_setpoint = rate_setpoint_new;
		}

		EXPECT_EQ(_quat_state.canonical(), _quat_goal.canonical());
		// it shouldn't have taken longer than an iteration timeout to converge
		EXPECT_GT(i, 0);
	}

	AttitudeControl _attitude_control;
	Quatf _quat_state;
	Quatf _quat_goal;
};

TEST_F(AttitudeControlConvergenceTest, AttitudeControlConvergence)
{
	const int inputs = 8;

	const Quatf QArray[inputs] = {
		Quatf(),
		Quatf(0, 1, 0, 0),
		Quatf(0, 0, 1, 0),
		Quatf(0, 0, 0, 1),
		Quatf(0.698f, 0.024f, -0.681f, -0.220f),
		Quatf(-0.820f, -0.313f, 0.225f, -0.423f),
		Quatf(0.599f, -0.172f, 0.755f, -0.204f),
		Quatf(0.216f, -0.662f, 0.290f, -0.656f)
	};

	for (int i = 0; i < inputs; i++) {
		for (int j = 0; j < inputs; j++) {
			printf("--- Input combination: %d %d\n", i, j);
			_quat_state = QArray[i];
			_quat_goal = QArray[j];
			_quat_state.normalize();
			_quat_goal.normalize();
			checkConvergence();
		}
	}
}

TEST(AttitudeControlTest, YawWeightScaling)
{
	// GIVEN: default tuning and pure yaw turn command
	AttitudeControl attitude_control;
	const float yaw_gain = 2.8f;
	const float yaw_sp = .1f;
	Quatf pure_yaw_attitude(cosf(yaw_sp / 2.f), 0, 0, sinf(yaw_sp / 2.f));
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), .4f);
	attitude_control.setRateLimit(Vector3f(1000.f, 1000.f, 1000.f));
	attitude_control.setAttitudeSetpoint(pure_yaw_attitude, 0.f);

	// WHEN: we run one iteration of the controller
	Vector3f rate_setpoint = attitude_control.update(Quatf());

	// THEN: no actuation in roll, pitch
	EXPECT_EQ(Vector2f(rate_setpoint), Vector2f());
	// THEN: actuation error * gain in yaw
	EXPECT_NEAR(rate_setpoint(2), yaw_sp * yaw_gain, 1e-4f);

	// GIVEN: additional corner case of zero yaw weight
	attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, yaw_gain), 0.f);
	// WHEN: we run one iteration of the controller
	rate_setpoint = attitude_control.update(Quatf());
	// THEN: no actuation (also no NAN)
	EXPECT_EQ(rate_setpoint, Vector3f());
}

class AttitudeControlFeedforwardTest : public ::testing::Test
{
public:
	AttitudeControlFeedforwardTest()
	{
		_attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), 0.4f);
		_attitude_control.setRateLimit(Vector3f(10.f, 10.f, 10.f));
		_attitude_control.setRefModelFrequency(10.f);
	}

	// Push a constant-rate ramp around the given body axis until the reference model is settled.
	// First call uses dt < 0 to reset the model to the current sample (matches the wrapper's
	// behaviour on the very first setpoint after boot).
	Quatf rampSetpoint(const Vector3f &body_rate, float yawspeed_sp, int steps)
	{
		Quatf q_d;

		for (int i = 0; i < steps; i++) {
			q_d = q_d * Quatf(AxisAnglef(body_rate * kDt));
			_attitude_control.setAttitudeSetpoint(q_d, yawspeed_sp, (i == 0) ? -1.f : kDt);
		}

		return q_d;
	}

	AttitudeControl _attitude_control;

	static constexpr float kDt = 0.004f;          // 250 Hz setpoint rate
	static constexpr int kSettleSteps = 500;      // generous settling window for any default omega_n
};

TEST_F(AttitudeControlFeedforwardTest, ConstantSetpointGivesNoFeedforward)
{
	// GIVEN: a constant tilted setpoint repeated with valid dt
	const Quatf q_d(AxisAnglef(Vector3f(0.1f, 0.f, 0.f)));

	for (int i = 0; i < kSettleSteps; i++) {
		_attitude_control.setAttitudeSetpoint(q_d, 0.f, (i == 0) ? -1.f : kDt);
	}

	// WHEN: vehicle is at the setpoint (no error)
	const Vector3f rate_setpoint = _attitude_control.update(q_d);

	// THEN: rate setpoint is zero — non-moving SP gives a zero model rate output
	EXPECT_NEAR(rate_setpoint.norm(), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, RollRampProducesRollFeedforward)
{
	// GIVEN: a steady roll ramp, reference model settled
	const float omega = 0.5f;
	rampSetpoint(Vector3f(omega, 0.f, 0.f), 0.f, kSettleSteps);

	// WHEN: vehicle is at the reference (no P error → P term = 0)
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: rate setpoint is the FF from omega_ref, which has converged to the ramp rate
	EXPECT_NEAR(rate_setpoint(0), omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, PitchRampProducesPitchFeedforward)
{
	// GIVEN: a steady pitch ramp, reference model settled
	const float omega = 0.5f;
	rampSetpoint(Vector3f(0.f, omega, 0.f), 0.f, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, HighRateRampStillExact)
{
	// GIVEN: a high-rate pitch ramp (~90 dps). Reference model steady-state holds at
	// the ramp rate independent of magnitude (the model's equilibrium tracking is
	// rate-invariant for any constant-rate command).
	const float omega = 1.5708f;    // ~90 dps
	rampSetpoint(Vector3f(0.f, omega, 0.f), 0.f, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, YawRampOnlyAnalyticalFeedforwardContributes)
{
	// GIVEN: a yaw ramp with the analytical yawspeed setpoint matching. The reference
	// model's damping is biased toward this known rate, so omega_ref settles to
	// (0,0,omega) in q_ref's body frame and the FF reads out the body-z component.
	const float omega = 0.5f;
	rampSetpoint(Vector3f(0.f, 0.f, omega), omega, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), omega, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, TiltedYawDoesNotDoubleCount)
{
	// GIVEN: body locked at constant tilt, yawing around world-z at constant rate.
	// Truth body angular velocity: ω_body = R_BW · (0, 0, yaw_rate)
	//                                     = (-sin(tilt)·yaw_rate, 0, cos(tilt)·yaw_rate)
	// The reference model bakes yaw_sp_move_rate into omega_ref via its damping bias,
	// so the FF reproduces the body-frame projection of the world-z rotation —
	// no separate analytical path, no double-count possible.
	const float tilt = 0.5f;        // ~28.6° pitch
	const float yaw_rate = 0.5f;    // ~28.6 dps
	const Quatf q_pitch(AxisAnglef(Vector3f(0.f, tilt, 0.f)));
	Quatf q_d;

	for (int i = 0; i < kSettleSteps; i++) {
		const Quatf q_yaw(AxisAnglef(Vector3f(0.f, 0.f, yaw_rate * kDt * i)));
		q_d = q_yaw * q_pitch;
		_attitude_control.setAttitudeSetpoint(q_d, yaw_rate, (i == 0) ? -1.f : kDt);
	}

	// WHEN: vehicle is at the reference attitude
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: rate setpoint matches the analytical world-z rotation in body frame
	EXPECT_NEAR(rate_setpoint(0), -sinf(tilt) * yaw_rate, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2),  cosf(tilt) * yaw_rate, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, FeedForwardDisabledSuppressesContribution)
{
	// GIVEN: a settled roll-ramp reference
	const float omega = 0.5f;
	rampSetpoint(Vector3f(omega, 0.f, 0.f), 0.f, kSettleSteps);

	// WHEN: the feedforward gain is 0, evaluated at the reference attitude so the P term is zero
	_attitude_control.setFeedForwardGain(0.f);
	Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: the anticipation feedforward is fully suppressed
	EXPECT_NEAR(rate_setpoint.norm(), 0.f, 1e-3f);

	// AND WHEN: the gain is restored, the anticipation returns (reference model preserved)
	_attitude_control.setFeedForwardGain(1.f);
	rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	EXPECT_NEAR(rate_setpoint(0), omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, UnlockedYawDoesNotFeedBackSlavedHeading)
{
	// GIVEN: the manual yaw-rate regime. StickYaw slaves the setpoint heading to the measured yaw,
	// so q_d.yaw ramps at the (large) vehicle yaw rate while the commanded analytical rate is only
	// the small decaying filter tail. Differentiating the slaved heading would latch the FF onto the
	// measured rate (the uncommanded-yaw runaway). Reproduce: q_d.yaw ramps fast, yawspeed_sp small.
	const float ramp_rate  = 0.8f;   // slaved-heading rate (≈ measured yaw rate)
	const float commanded  = 0.05f;  // small but > FLT_EPSILON: yaw stays "unlocked"
	rampSetpoint(Vector3f(0.f, 0.f, ramp_rate), commanded, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: yaw FF tracks only the analytical commanded rate, NOT the slaved-heading ramp (no latch)
	EXPECT_NEAR(rate_setpoint(2), commanded, 1e-3f);
	EXPECT_LT(fabsf(rate_setpoint(2)), 0.2f);
	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, LockedYawRampStillFeedsForward)
{
	// GIVEN: a genuine yaw-angle target slewed with no commanded rate (heading-hold / auto yaw).
	// yawspeed_sp = 0 → the heading is exogenous, not slaved, so the full reference-model FF must
	// still differentiate it. This guards against over-gating killing the legitimate yaw FF.
	const float omega = 0.5f;
	rampSetpoint(Vector3f(0.f, 0.f, omega), 0.f, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	EXPECT_NEAR(rate_setpoint(2), omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, TiltedUnlockedYawUsesCommandedRateOnly)
{
	// GIVEN: tilted while the slaved heading ramps fast (q_d.yaw := measured) but the commanded
	// analytical rate is small — the tilted form of the uncommanded-yaw runaway.
	const float tilt        = 0.5f;
	const float slaved_rate = 0.8f;   // q_d.yaw ramp (≈ measured yaw rate)
	const float commanded   = 0.05f;  // small but > FLT_EPSILON: yaw stays "unlocked"
	const Quatf q_pitch(AxisAnglef(Vector3f(0.f, tilt, 0.f)));
	Quatf q_d;

	for (int i = 0; i < kSettleSteps; i++) {
		const Quatf q_yaw(AxisAnglef(Vector3f(0.f, 0.f, slaved_rate * kDt * i)));
		q_d = q_yaw * q_pitch;
		_attitude_control.setAttitudeSetpoint(q_d, commanded, (i == 0) ? -1.f : kDt);
	}

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: FF is the commanded world-z rate projected into the body frame (not the slaved ramp),
	// with no roll/pitch leak from the stripped heading error.
	EXPECT_NEAR(rate_setpoint(0), -sinf(tilt) * commanded, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2),  cosf(tilt) * commanded, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, CommandedYawRateFedForwardWhenFFDisabled)
{
	// GIVEN: a settled manual yaw-rate command (heading slaved to measurement, small commanded rate)
	const float commanded = 0.05f;
	rampSetpoint(Vector3f(0.f, 0.f, 0.8f), commanded, kSettleSteps);

	// WHEN: the feedforward gain is 0, evaluated at the reference attitude so the P term is zero
	_attitude_control.setFeedForwardGain(0.f);
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: the commanded yaw rate still bypasses the ref model and is fed forward at unity
	EXPECT_NEAR(rate_setpoint(2), commanded, 1e-3f);
	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, FractionalGainDoesNotWeakenCommandedYaw)
{
	// GIVEN: a manual yaw-rate command with the anticipation gain detuned to 0.1
	const float commanded = 0.05f;
	_attitude_control.setFeedForwardGain(0.1f);
	rampSetpoint(Vector3f(0.f, 0.f, 0.8f), commanded, kSettleSteps);

	// Evaluate at the reference attitude so the P term is zero; the only yaw contribution is the FF.
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: commanded yaw authority is preserved at full strength (NOT scaled to 0.1 x commanded).
	EXPECT_NEAR(rate_setpoint(2), commanded, 1e-3f);
	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
}

TEST_F(AttitudeControlFeedforwardTest, FractionalGainScalesAnticipation)
{
	// GIVEN: a settled roll-ramp reference (pure error-driven anticipation, no commanded rate), gain 0.1
	const float omega = 0.5f;
	_attitude_control.setFeedForwardGain(0.1f);
	rampSetpoint(Vector3f(omega, 0.f, 0.f), 0.f, kSettleSteps);

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: the anticipation IS scaled by the gain (0.1 x the settled reference rate), unlike the command.
	EXPECT_NEAR(rate_setpoint(0), 0.1f * omega, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

class AttitudeControlShapingTest : public ::testing::Test
{
public:
	AttitudeControlShapingTest()
	{
		_attitude_control.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), 0.4f);
		_attitude_control.setRateLimit(Vector3f(kRateMax, kRateMax, kRateMax));
		_attitude_control.setFeedForwardGain(1.f);
		_attitude_control.setRefModelAccelerationLimit(Vector3f(kAccelMax, kAccelMax, kAccelMax), kJerkMax);
		// reset the reference to the identity attitude
		_attitude_control.setAttitudeSetpoint(Quatf(), 0.f, -1.f);
	}

	// Apply a constant setpoint and record the per-step reference rate (body frame, from consecutive q_ref).
	// Returns the peak rate, acceleration and jerk magnitudes observed on the given axis and the peak angle.
	void stepSetpoint(const Quatf &q_d, int axis, int steps, float &max_rate, float &max_accel, float &max_jerk,
			  float &max_angle)
	{
		max_rate = max_accel = max_jerk = max_angle = 0.f;
		float rate_prev = 0.f;
		float accel_prev = 0.f;
		Quatf q_ref_prev = _attitude_control.getReferenceAttitude();

		for (int i = 0; i < steps; i++) {
			_attitude_control.setAttitudeSetpoint(q_d, 0.f, kDt);
			const Quatf q_ref = _attitude_control.getReferenceAttitude();

			const Vector3f delta_phi = 2.f * (q_ref_prev.inversed() * q_ref).canonical().imag();
			const float rate = delta_phi(axis) / kDt;
			const float accel = (rate - rate_prev) / kDt;
			const float jerk = (accel - accel_prev) / kDt;

			max_rate = math::max(max_rate, fabsf(rate));
			max_accel = math::max(max_accel, fabsf(accel));
			max_jerk = math::max(max_jerk, fabsf(jerk));
			max_angle = math::max(max_angle, fabsf(2.f * q_ref.canonical().imag()(axis)));

			rate_prev = rate;
			accel_prev = accel;
			q_ref_prev = q_ref;
		}
	}

	AttitudeControl _attitude_control;

	static constexpr float kDt = 0.004f;
	static constexpr float kRateMax = 2.f;    // rad/s
	static constexpr float kAccelMax = 5.f;   // rad/s^2
	static constexpr float kJerkMax = 50.f;   // rad/s^3
};

TEST_F(AttitudeControlShapingTest, StepRespectsRateAccelJerkLimitsAndConverges)
{
	// GIVEN: a large roll step that saturates rate, acceleration and jerk
	const float step = 1.5f;
	const Quatf q_d(AxisAnglef(Vector3f(step, 0.f, 0.f)));

	// WHEN: the reference model is propagated until it settled
	float max_rate, max_accel, max_jerk, max_angle;
	stepSetpoint(q_d, 0, 1000, max_rate, max_accel, max_jerk, max_angle);

	// THEN: all limits are saturated but never exceeded (the jerk is a second finite difference of
	// the float quaternion reference and hence carries some numerical noise)
	EXPECT_LE(max_rate, kRateMax * 1.01f);
	EXPECT_GT(max_rate, kRateMax * 0.95f);
	EXPECT_LE(max_accel, kAccelMax * 1.02f);
	EXPECT_GT(max_accel, kAccelMax * 0.95f);
	EXPECT_LE(max_jerk, kJerkMax * 1.15f);
	EXPECT_GT(max_jerk, kJerkMax * 0.9f);

	// THEN: the reference reached the setpoint without overshoot
	EXPECT_LE(max_angle, step * 1.005f);
	const Vector3f error = 2.f * (_attitude_control.getReferenceAttitude().inversed() * q_d).canonical().imag();
	EXPECT_NEAR(error.norm(), 0.f, 1e-3f);

	// THEN: at the settled reference no rate is fed forward anymore
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());
	EXPECT_NEAR(rate_setpoint.norm(), 0.f, 1e-3f);
}

TEST_F(AttitudeControlShapingTest, SmallStepStaysBelowRateLimit)
{
	// GIVEN: a small pitch step that cannot reach the rate limit within the acceleration limit
	const float step = 0.1f;
	const Quatf q_d(AxisAnglef(Vector3f(0.f, step, 0.f)));

	float max_rate, max_accel, max_jerk, max_angle;
	stepSetpoint(q_d, 1, 1000, max_rate, max_accel, max_jerk, max_angle);

	// THEN: rate stays well below the limit, acceleration and jerk limits hold, no overshoot, converged
	EXPECT_LT(max_rate, kRateMax * 0.5f);
	EXPECT_LE(max_accel, kAccelMax * 1.02f);
	EXPECT_LE(max_jerk, kJerkMax * 1.15f);
	EXPECT_LE(max_angle, step * 1.005f);
	const Vector3f error = 2.f * (_attitude_control.getReferenceAttitude().inversed() * q_d).canonical().imag();
	EXPECT_NEAR(error.norm(), 0.f, 1e-3f);
}

TEST_F(AttitudeControlShapingTest, ShapedReferenceRateIsFedForward)
{
	// GIVEN: a large roll step, propagated only until the reference is moving fast
	const Quatf q_d(AxisAnglef(Vector3f(1.5f, 0.f, 0.f)));
	Quatf q_ref_prev;

	for (int i = 0; i < 125; i++) {
		q_ref_prev = _attitude_control.getReferenceAttitude();
		_attitude_control.setAttitudeSetpoint(q_d, 0.f, kDt);
	}

	// reference roll rate from the last propagation step
	const Vector3f delta_phi = 2.f * (q_ref_prev.inversed() * _attitude_control.getReferenceAttitude()).canonical().imag();
	const float ref_rate = delta_phi(0) / kDt;
	EXPECT_GT(ref_rate, 0.5f * kRateMax);
	EXPECT_LE(ref_rate, kRateMax * 1.01f);

	// WHEN: the vehicle sits exactly on the reference (no P error)
	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: the feed-forward equals the shaped reference rate
	EXPECT_NEAR(rate_setpoint(0), ref_rate, 1e-2f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(2), 0.f, 1e-3f);
}

TEST_F(AttitudeControlShapingTest, CommandedYawRateBypassesShaping)
{
	// GIVEN: a manual yaw-rate command (heading slaved to the measurement, small commanded rate)
	const float commanded = 0.05f;
	Quatf q_d;

	for (int i = 0; i < 500; i++) {
		q_d = q_d * Quatf(AxisAnglef(Vector3f(0.f, 0.f, 0.8f * kDt)));
		_attitude_control.setAttitudeSetpoint(q_d, commanded, kDt);
	}

	const Vector3f rate_setpoint = _attitude_control.update(_attitude_control.getReferenceAttitude());

	// THEN: only the commanded yaw rate is fed forward, the shaped error-driven yaw rate is stripped
	EXPECT_NEAR(rate_setpoint(2), commanded, 1e-3f);
	EXPECT_NEAR(rate_setpoint(0), 0.f, 1e-3f);
	EXPECT_NEAR(rate_setpoint(1), 0.f, 1e-3f);
}

TEST_F(AttitudeControlShapingTest, DisabledAxisKeepsLinearModel)
{
	// GIVEN: two controllers, one with shaping disabled, one with only the roll axis limited
	AttitudeControl linear;
	linear.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), 0.4f);
	linear.setRateLimit(Vector3f(kRateMax, kRateMax, kRateMax));
	linear.setRefModelFrequency(10.f);
	linear.setAttitudeSetpoint(Quatf(), 0.f, -1.f);

	AttitudeControl mixed;
	mixed.setProportionalGain(Vector3f(6.5f, 6.5f, 2.8f), 0.4f);
	mixed.setRateLimit(Vector3f(kRateMax, kRateMax, kRateMax));
	mixed.setRefModelFrequency(10.f);
	mixed.setRefModelAccelerationLimit(Vector3f(kAccelMax, 0.f, 0.f), kJerkMax);
	mixed.setAttitudeSetpoint(Quatf(), 0.f, -1.f);

	// WHEN: a pure pitch step is applied to both
	const Quatf q_d(AxisAnglef(Vector3f(0.f, 0.3f, 0.f)));

	for (int i = 0; i < 50; i++) {
		linear.setAttitudeSetpoint(q_d, 0.f, kDt);
		mixed.setAttitudeSetpoint(q_d, 0.f, kDt);

		// THEN: the pitch axis (no limit) evolves identically to the pure linear model
		const Vector3f diff = 2.f * (linear.getReferenceAttitude().inversed() * mixed.getReferenceAttitude()).canonical().imag();
		EXPECT_NEAR(diff.norm(), 0.f, 1e-5f);
	}
}
