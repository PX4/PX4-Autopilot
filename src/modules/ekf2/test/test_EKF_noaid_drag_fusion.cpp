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
 * Multirotor without any horizontal aiding (no GNSS, flow or vision) using drag
 * fusion, see https://github.com/PX4/PX4-Autopilot/issues/20277
 *
 * Truth model: rotor momentum drag (specific force proportional to the body
 * horizontal velocity, as modelled by the EKF drag fusion) with the thrust
 * holding the altitude, so the vehicle converges to a constant ground speed
 * while pitched forward. There is no wind.
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfNoAidDragFusionTest : public ::testing::Test
{
public:
	EkfNoAidDragFusionTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	static constexpr float kDt = 1e-3f;
	static constexpr float kDragCoef = 0.3f; ///< rotor momentum drag coefficient (1/s)
	const Vector3f _mag_earth{0.218f, 0.f, 0.43f};

	struct TruthState {
		float pitch{0.f};
		float vel_n{0.f};
	};

	TruthState _truth{};
	Vector3f _gyro_bias{};

	struct Result {
		float tilt_err_max_deg{0.f};
		float tilt_err_end_deg{0.f};
		bool fake_pos_in_air{false};
	};

	void SetUp() override
	{
		_ekf->init(0);

		_ekf_wrapper.setDragFusionParameters(0.f, 0.f, kDragCoef);
		_ekf_wrapper.enableDragFusion();
		_ekf->getParamHandle()->ekf2_imu_ctrl |= static_cast<int32_t>(ImuCtrl::GravityVector);

		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
		_sensor_simulator.runSeconds(10);

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);
	}

	float tiltErrorDeg() const
	{
		// earth z axis in body frame, not affected by heading errors
		const Vector3f z_true = Quatf{Eulerf{0.f, _truth.pitch, 0.f}}.rotateVectorInverse(Vector3f{0.f, 0.f, 1.f});
		const Vector3f z_est = _ekf->getQuaternion().rotateVectorInverse(Vector3f{0.f, 0.f, 1.f});
		return math::degrees(atan2f(z_true.cross(z_est).norm(), z_true.dot(z_est)));
	}

	void step(float pitch_rate)
	{
		_truth.pitch += pitch_rate * kDt;

		const Dcmf R_body_to_earth{Quatf{Eulerf{0.f, _truth.pitch, 0.f}}};
		const Dcmf R_earth_to_body = R_body_to_earth.transpose();
		const Vector3f vel_body = R_earth_to_body * Vector3f{_truth.vel_n, 0.f, 0.f};

		// thrust along body -z, sized so that the total vertical specific force balances gravity
		Vector3f specific_force_body{-kDragCoef * vel_body(0), -kDragCoef * vel_body(1), 0.f};
		specific_force_body(2) = -(CONSTANTS_ONE_G
					   + R_body_to_earth(2, 0) * specific_force_body(0)
					   + R_body_to_earth(2, 1) * specific_force_body(1)) / R_body_to_earth(2, 2);
		const Vector3f specific_force_earth = R_body_to_earth * specific_force_body;

		_truth.vel_n += specific_force_earth(0) * kDt;

		_sensor_simulator._imu.setData(specific_force_body, Vector3f{0.f, pitch_rate, 0.f} + _gyro_bias);
		_sensor_simulator._mag.setData(R_earth_to_body * _mag_earth);

		_sensor_simulator.runMicroseconds(1000);
	}

	// Hover, pitch forward, cruise, pitch back to level and brake
	Result fly(float pitch_deg, float cruise_duration)
	{
		Result res{};
		const float tilt_duration = 1.f;
		const float pitch_rate = -math::radians(pitch_deg) / tilt_duration;

		struct Phase {
			float duration;
			float pitch_rate;
		};

		const Phase phases[] = {
			{5.f, 0.f},
			{tilt_duration, pitch_rate},
			{cruise_duration, 0.f},
			{tilt_duration, -pitch_rate},
			{20.f, 0.f},
		};

		for (const Phase &phase : phases) {
			const int steps = static_cast<int>(lroundf(phase.duration / kDt));

			for (int i = 0; i < steps; i++) {
				step(phase.pitch_rate);
				res.tilt_err_max_deg = fmaxf(res.tilt_err_max_deg, tiltErrorDeg());
				res.fake_pos_in_air |= _ekf->control_status_flags().fake_pos;
			}
		}

		res.tilt_err_end_deg = tiltErrorDeg();
		return res;
	}
};

TEST_F(EkfNoAidDragFusionTest, sustainedPitchForward)
{
	// WHEN: the vehicle accelerates, cruises pitched forward and brakes to a stop
	const Result res = fly(20.f, 60.f);

	// THEN: the horizontal accelerations are not mistaken for gravity
	EXPECT_LT(res.tilt_err_max_deg, 0.5f);
	EXPECT_LT(res.tilt_err_end_deg, 0.2f);
	EXPECT_FALSE(res.fake_pos_in_air);
}

TEST_F(EkfNoAidDragFusionTest, sustainedPitchForwardWithGyroBiasChange)
{
	// WHEN: the roll gyro bias changes after takeoff and the vehicle flies forward for a long time
	_gyro_bias(0) = math::radians(0.3f);
	const Result res = fly(20.f, 180.f);

	// THEN: the bias is learned through the drag fusion and the tilt converges back
	EXPECT_LT(res.tilt_err_max_deg, 5.f);
	EXPECT_LT(res.tilt_err_end_deg, 0.5f);
	EXPECT_FALSE(res.fake_pos_in_air);
}

TEST_F(EkfNoAidDragFusionTest, longHoverWithGyroBiasChange)
{
	// WHEN: the vehicle hovers for a long time, where the drag fusion barely constrains the tilt
	_gyro_bias(0) = math::radians(0.3f);
	const Result res = fly(0.f, 300.f);

	// THEN: gravity fusion keeps the tilt bounded instead of fake position fusion,
	// which would corrupt the velocity states used by the drag fusion
	EXPECT_LT(res.tilt_err_max_deg, 3.f);
	EXPECT_LT(res.tilt_err_end_deg, 0.5f);
	EXPECT_FALSE(res.fake_pos_in_air);
}
