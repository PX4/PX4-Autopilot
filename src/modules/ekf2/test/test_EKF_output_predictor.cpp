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
 * @file test_EKF_output_predictor.cpp
 *
 * @brief Unit tests for the output predictor attitude tracking
 *
 * The output predictor integrates the gyro at the current time horizon and is pulled towards the EKF
 * attitude, which is known at the delayed fusion time horizon. These tests drive it directly with a
 * known rotation and a perfect (but delayed) EKF attitude.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <tuple>

#include "EKF/output_predictor/output_predictor.h"

using namespace matrix;

namespace
{
constexpr uint64_t kImuDtUs = 4000;   // 250 Hz IMU
constexpr float kImuDt = kImuDtUs * 1e-6f;
constexpr int kImuPerUpdate = 2;      // EKF update every 8 ms
constexpr uint8_t kBufferLength = 20; // delay = (length - 1) * 8 ms = 152 ms
constexpr float kGravity = 9.80665f;

float angleBetween(const Quatf &a, const Quatf &b)
{
	const Quatf q_err = (a.inversed() * b).normalized();
	return 2.f * asinf(math::min(Vector3f(q_err(1), q_err(2), q_err(3)).norm(), 1.f));
}

// Vehicle spinning at a constant body rate, an output predictor and a perfect EKF that lags by the buffer delay
class OutputPredictorSim
{
public:
	OutputPredictorSim()
	{
		_op.allocate(kBufferLength);
		_op.set_gravity(kGravity);
	}

	void setBodyRate(const Vector3f &body_rate)
	{
		_body_rate = body_rate;
		_tick_spin_start = _tick;
	}

	// attitude of the vehicle at IMU tick n, spinning at a constant body rate since setBodyRate()
	Quatf truth(int tick) const
	{
		const float spin_time = static_cast<float>(math::max(tick - _tick_spin_start, 0)) * kImuDt;
		return Quatf(AxisAnglef(_body_rate * spin_time));
	}

	Quatf truthNow() const { return truth(_tick); }

	void run(float duration, const Quatf &ekf_frame = Quatf())
	{
		const int steps = static_cast<int>(lroundf(duration / kImuDt));
		const int delay_ticks = (kBufferLength - 1) * kImuPerUpdate;

		for (int i = 0; i < steps; i++) {
			_tick++;
			const uint64_t time_us = static_cast<uint64_t>(_tick) * kImuDtUs;

			const Vector3f delta_angle = _body_rate * kImuDt;
			const Vector3f delta_velocity = truth(_tick).rotateVectorInverse(Vector3f(0.f, 0.f, -kGravity)) * kImuDt;
			_op.calculateOutputStates(time_us, delta_angle, kImuDt, delta_velocity, kImuDt);

			if (_tick % kImuPerUpdate == 0) {
				const uint64_t time_delayed_us = time_us - static_cast<uint64_t>(delay_ticks) * kImuDtUs;
				_op.correctOutputStates(time_delayed_us, ekf_frame * truth(_tick - delay_ticks), Vector3f(), _gpos,
							Vector3f(), Vector3f());
			}
		}
	}

	OutputPredictor _op;

private:
	Vector3f _body_rate{};
	int _tick{0};
	int _tick_spin_start{0};
	LatLonAlt _gpos{47.3977, 8.5456, 400.f};
};
}

class OutputPredictorSpin : public ::testing::TestWithParam<std::tuple<int, float>> {};

// An attitude error must be removed whatever the rotation rate, about roll, pitch or yaw. Before the correction
// was held in earth frame, it was applied about axes that had turned by rate x delay, and the error grew above
// about 7 rad/s.
TEST_P(OutputPredictorSpin, attitudeErrorConvergesWhileSpinning)
{
	const int axis = std::get<0>(GetParam());
	const float rate = std::get<1>(GetParam());

	Vector3f body_rate{};
	body_rate(axis) = rate;

	// an error about an axis perpendicular to the spin; an error along it is not affected
	Vector3f error_axis{};
	error_axis(axis == 0 ? 1 : 0) = 1.f;

	OutputPredictorSim sim;
	sim.run(1.f); // settle at rest

	sim.setBodyRate(body_rate);
	sim.run(0.5f);

	// rotate the output 2 degrees away from the EKF (and from the truth)
	sim._op.resetQuaternion(Quatf(AxisAnglef(error_axis * math::radians(2.f))));
	EXPECT_NEAR(math::degrees(angleBetween(sim._op.getQuaternion(), sim.truthNow())), 2.f, 0.01f);

	sim.run(4.f);
	EXPECT_LT(math::degrees(angleBetween(sim._op.getQuaternion(), sim.truthNow())), 0.05f)
			<< "axis " << axis << ", rate " << rate << " rad/s";
}

INSTANTIATE_TEST_SUITE_P(RollPitchYaw, OutputPredictorSpin,
			 ::testing::Combine(::testing::Values(0, 1, 2), ::testing::Values(0.f, 3.f, 7.f, 15.f, 25.f)));

// Rotating the earth frame (e.g. a yaw reset) must commute with the output predictor:
// resetting and then running must give the same result as running and then resetting.
TEST(OutputPredictor, yawResetCommutesWithAttitudeCorrection)
{
	OutputPredictorSim a;
	OutputPredictorSim b;

	for (OutputPredictorSim *sim : {&a, &b}) {
		sim->run(1.f);
		sim->setBodyRate(Vector3f(0.f, 0.f, 10.f));
		sim->run(0.5f);
		// a large pending attitude correction
		sim->_op.resetQuaternion(Quatf(AxisAnglef(Vector3f(math::radians(5.f), 0.f, 0.f))));
		sim->run(0.02f);
	}

	const Quatf yaw_reset(AxisAnglef(Vector3f(0.f, 0.f, math::radians(30.f))));
	a._op.resetQuaternion(yaw_reset);

	float max_diff = 0.f;

	for (int i = 0; i < 50; i++) {
		a.run(kImuDt, yaw_reset);
		b.run(kImuDt);
		max_diff = math::max(max_diff, angleBetween(a._op.getQuaternion(), yaw_reset * b._op.getQuaternion()));
	}

	EXPECT_LT(max_diff, 5e-5f);
}
