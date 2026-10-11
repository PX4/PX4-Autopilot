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

/**
 * Output predictor attitude tracking tests.
 *
 * The predictor measures its attitude error against the EKF at the delayed fusion
 * horizon and applies the correction to delta angles sampled at the current time. The
 * body frame turns by r * tau between those two instants, so the correction has to be
 * rotated into the current frame before it is applied. Without that rotation the
 * tracking loop's feedback changes sign once r * tau exceeds pi / 2 - about 15.7 rad/s
 * at the default 100 ms horizon - and the published attitude diverges while the EKF's
 * own solution stays correct.
 *
 * These tests drive the predictor directly with a synthetic spin and a transverse
 * attitude step, and assert that the loop converges.
 */

#include <gtest/gtest.h>

#include "EKF/output_predictor/output_predictor.h"

using matrix::AxisAnglef;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector3f;

class OutputPredictorTest : public ::testing::Test
{
public:
	static constexpr float kDt = 0.004f;		///< 250 Hz

	/*
	 * The ring buffer hands back the entry written kSize - 1 pushes ago, so a 26-deep
	 * buffer driven at 250 Hz puts the fusion horizon exactly 100 ms back, matching the
	 * stock EKF2_DELAY_MAX.
	 */
	static constexpr uint8_t kBufferLength = 26;
	static constexpr int kLagSamples = kBufferLength - 1;
	static constexpr uint64_t kDelayUs = static_cast<uint64_t>(kLagSamples *kDt * 1e6f);

	// hold still until the buffer is full, then spin, then let the loop settle before
	// stepping the EKF sideways - so every phase starts from a converged state
	static constexpr int kSpinStartStep = 50;
	static constexpr int kDisturbanceStep = 150;

	struct Result {
		float tracking_error_final{0.f};	///< angle between EKF and INS at the horizon (rad)
		float tracking_error_peak{0.f};		///< worst value after the step was applied
		float attitude_error_final{0.f};	///< angle between the published attitude and reference (rad)
	};

	void SetUp() override
	{
		ASSERT_TRUE(_predictor.allocate(kBufferLength));
	}

	/**
	 * Spin about body z at a constant rate, then step the EKF attitude sideways by
	 * `disturbance` about body x and let the tracking loop chase it.
	 *
	 * @param negate_ekf_quat publish -q instead of q for the EKF attitude. The two
	 *                        describe the same rotation, so the result must not change.
	 */
	Result run(float yaw_rate, float disturbance, float duration, bool negate_ekf_quat = false)
	{
		_predictor.reset();
		// gravity plays no part in the attitude tracking loop
		_predictor.set_gravity(0.f);

		// about earth x, i.e. transverse to the spin axis
		const Quatf q_step(AxisAnglef(Vector3f(disturbance, 0.f, 0.f)));
		const int steps = static_cast<int>(duration / kDt);

		Result result{};
		Quatf quat_ekf_last;

		for (int i = 0; i < steps; i++) {
			// start clear of zero: a zero timestamp means "not yet initialised"
			const uint64_t time_us = 10000000 + static_cast<uint64_t>(i) * 4000;
			const float rate = (i >= kSpinStartStep) ? yaw_rate : 0.f;

			_predictor.calculateOutputStates(time_us, Vector3f(0.f, 0.f, rate * kDt), kDt, Vector3f(), kDt);

			// the EKF solution at the fusion horizon. Integrating body-z increments is a
			// pure right-multiplication, so the attitude after n spinning steps is just
			// R_z(yaw_rate * n * dt).
			const int spin_steps_delayed = math::max((i + 1 - kLagSamples) - kSpinStartStep, 0);
			Quatf quat_ekf(AxisAnglef(Vector3f(0.f, 0.f, yaw_rate * spin_steps_delayed * kDt)));

			if (i >= kDisturbanceStep) {
				// an attitude offset is a rotation of the earth frame, applied on the left -
				// the same convention resetQuaternion() uses. A body-frame offset would not
				// be reachable by integrating body-z rates, so the loop would be chasing a
				// target moving at a different angular velocity.
				quat_ekf = q_step * quat_ekf;
			}

			quat_ekf_last = quat_ekf;

			_predictor.correctOutputStates(time_us - kDelayUs, negate_ekf_quat ? -quat_ekf : quat_ekf,
						       Vector3f(), LatLonAlt(0.0, 0.0, 0.f), Vector3f(), Vector3f());

			if (i >= kDisturbanceStep) {
				result.tracking_error_peak = math::max(result.tracking_error_peak,
								       _predictor.getOutputTrackingError()(0));
			}
		}

		result.tracking_error_final = _predictor.getOutputTrackingError()(0);

		// a converged INS sits exactly one horizon ahead of the EKF attitude it is
		// tracking, and that propagation is the same right-multiplication
		const Quatf quat_expected = quat_ekf_last * Quatf(AxisAnglef(Vector3f(0.f, 0.f,
					    yaw_rate * kLagSamples * kDt)));
		result.attitude_error_final = angleBetween(_predictor.getQuaternion(), quat_expected);

		return result;
	}

	static float angleBetween(const Quatf &a, const Quatf &b)
	{
		Quatf delta((a.inversed() * b).normalized());

		if (delta(0) < 0.f) {
			delta = -delta;
		}

		return AxisAnglef(delta).norm();
	}

	OutputPredictor _predictor;
};

// the harness itself: with no disturbance the INS reproduces the EKF exactly, so any
// residual here would be a modelling error rather than a filter one
TEST_F(OutputPredictorTest, harnessIsExactWithoutDisturbance)
{
	const Result result = run(20.f, 0.f, 4.f);

	EXPECT_LT(result.tracking_error_final, 1e-5f);
	EXPECT_LT(result.attitude_error_final, 1e-4f);
}

// 20 rad/s is past the r * tau = pi / 2 threshold where the uncorrected loop's feedback
// changes sign. Without the frame rotation the error grows to the better part of a
// radian and stays there; with it the step is nulled.
TEST_F(OutputPredictorTest, convergesUnderRapidYaw)
{
	const float disturbance = math::radians(1.f);
	const Result result = run(20.f, disturbance, 4.f);

	EXPECT_LT(result.tracking_error_final, 0.1f * disturbance);
	EXPECT_LT(result.tracking_error_peak, 2.f * disturbance);
	EXPECT_LT(result.attitude_error_final, math::radians(0.5f));
}

// the sign of the rotation must not matter: the correction is rotated by the vehicle's
// own attitude history, not by an assumed direction
TEST_F(OutputPredictorTest, convergesUnderRapidYawReversed)
{
	const float disturbance = math::radians(1.f);
	const Result result = run(-20.f, disturbance, 4.f);

	EXPECT_LT(result.tracking_error_final, 0.1f * disturbance);
	EXPECT_LT(result.tracking_error_peak, 2.f * disturbance);
	EXPECT_LT(result.attitude_error_final, math::radians(0.5f));
}

// both spin directions have to behave identically for a symmetric disturbance
TEST_F(OutputPredictorTest, spinDirectionSymmetry)
{
	const float disturbance = math::radians(1.f);

	const Result forward = run(20.f, disturbance, 4.f);

	const Result reverse = run(-20.f, disturbance, 4.f);

	EXPECT_NEAR(forward.tracking_error_final, reverse.tracking_error_final, 1e-6f);
	EXPECT_NEAR(forward.tracking_error_peak, reverse.tracking_error_peak, 1e-6f);
}

// q and -q are the same attitude; the shortest-branch selection must make the loop blind
// to which one the EKF happens to publish
TEST_F(OutputPredictorTest, quaternionSignEquivalence)
{
	const float disturbance = math::radians(1.f);

	const Result positive = run(20.f, disturbance, 4.f);

	const Result negated = run(20.f, disturbance, 4.f, true);

	EXPECT_NEAR(positive.tracking_error_final, negated.tracking_error_final, 1e-6f);
	EXPECT_NEAR(positive.tracking_error_peak, negated.tracking_error_peak, 1e-6f);
	EXPECT_NEAR(positive.attitude_error_final, negated.attitude_error_final, 1e-6f);
}

// a conventional flight profile is well below the threshold and was never affected; the
// frame rotation must not disturb it
TEST_F(OutputPredictorTest, convergesAtConventionalRate)
{
	const float disturbance = math::radians(1.f);
	const Result result = run(1.f, disturbance, 4.f);

	EXPECT_LT(result.tracking_error_final, 0.1f * disturbance);
	EXPECT_LT(result.attitude_error_final, math::radians(0.5f));
}

// with no rotation at all the correction needs no reframing, so the loop has to behave
// exactly as it always did
TEST_F(OutputPredictorTest, convergesWithoutRotation)
{
	const float disturbance = math::radians(1.f);
	const Result result = run(0.f, disturbance, 4.f);

	EXPECT_LT(result.tracking_error_final, 0.1f * disturbance);
	EXPECT_LT(result.attitude_error_final, math::radians(0.5f));
}

// a large attitude error still has to be recoverable while spinning. This is the regime
// where the exact delta angle differs from the small-angle form.
TEST_F(OutputPredictorTest, recoversFromLargeErrorUnderRapidYaw)
{
	const float disturbance = math::radians(170.f);
	const Result result = run(20.f, disturbance, 12.f);

	EXPECT_LT(result.tracking_error_final, math::radians(1.f));
	EXPECT_LT(result.attitude_error_final, math::radians(2.f));
}

/*
 * The delta angle is extracted exactly, as theta * axis, rather than with the
 * small-angle 2 * sin(theta / 2) * axis. Both are monotonic over the theta in [0, pi]
 * that the shortest-branch selection guarantees, so this is a gain change and not a
 * correctness fix. Pin the numbers quoted in correctOutputStates(): the two agree to
 * well inside a tenth of a percent below 10 deg, and the approximation loses a
 * quarter of the correction by 150 deg.
 */
TEST_F(OutputPredictorTest, exactDeltaAngleGain)
{
	struct {
		float angle_deg;
		float min_ratio;	///< small-angle magnitude as a fraction of the exact angle
		float max_ratio;
	} const cases[] = {
		{  1.f, 0.9999f, 1.0000f},
		{ 10.f, 0.9985f, 1.0000f},
		{ 90.f, 0.8990f, 0.9010f},
		{150.f, 0.7370f, 0.7390f},
		{170.f, 0.6700f, 0.6730f},
	};

	for (const auto &c : cases) {
		const float angle = math::radians(c.angle_deg);
		const Quatf q(AxisAnglef(Vector3f(0.f, 0.f, angle)));

		// what the code now does
		const float exact = AxisAnglef(q).norm();
		EXPECT_NEAR(exact, angle, 1e-5f) << "at " << c.angle_deg << " deg";

		// what it did before: 2 * vec, i.e. 2 * sin(theta / 2)
		const float small_angle = 2.f * Vector3f(q(1), q(2), q(3)).norm();

		const float ratio = small_angle / exact;
		EXPECT_GE(ratio, c.min_ratio) << "at " << c.angle_deg << " deg";
		EXPECT_LE(ratio, c.max_ratio) << "at " << c.angle_deg << " deg";
	}
}
