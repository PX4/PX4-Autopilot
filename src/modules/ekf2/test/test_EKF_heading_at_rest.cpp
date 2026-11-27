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
 * Heading of a vehicle at rest with GNSS as the only heading aiding.
 *
 * Scenario:
 *  - the magnetometer is only used to initialize the heading (EKF2_MAG_TYPE = INIT)
 *  - the vehicle sits on the ground, slightly tilted
 *  - GNSS velocity and position are fused; a static receiver reports a slowly
 *    wandering velocity and position error
 *
 * Without horizontal acceleration, GNSS velocity and position carry no heading
 * information, so the heading must only follow the gyro while at rest.
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfHeadingAtRestTest : public ::testing::Test
{
public:
	EkfHeadingAtRestTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	static constexpr float kTilt = math::radians(3.f);   ///< roll and pitch of the vehicle on the ground (rad)
	static constexpr float kYawTrue = math::radians(30.f);
	static constexpr float kGnssDelay = 0.11f;            ///< GNSS delay used by the simulator (s)

	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		// the magnetometer only initializes the heading
		_ekf->getParamHandle()->ekf2_mag_type = MagFuseType::INIT;
		_sensor_simulator.simulateOrientation(Quatf(Eulerf(kTilt, kTilt, kYawTrue)));
		_sensor_simulator.runSeconds(5);

		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(12);
	}

	// Error of a static GNSS receiver: velocity and position wander slowly and independently,
	// as observed on real receivers (position error is not the integral of the velocity error).
	// Kept below the GNSS drift checks so that GNSS fusion keeps running.
	static Vector3f gnssVelocityError(float t)
	{
		return Vector3f(0.08f * sinf(2.f * M_PI_F * t / 23.f) + 0.05f * sinf(2.f * M_PI_F * t / 61.f + 1.f),
				0.07f * sinf(2.f * M_PI_F * t / 29.f + 0.5f) + 0.06f * sinf(2.f * M_PI_F * t / 47.f + 2.f),
				0.f);
	}

	static Vector3f gnssPositionDriftRate(float t)
	{
		return Vector3f(0.12f * sinf(2.f * M_PI_F * t / 71.f + 0.3f) + 0.06f * sinf(2.f * M_PI_F * t / 37.f),
				0.10f * sinf(2.f * M_PI_F * t / 83.f + 1.2f) + 0.07f * sinf(2.f * M_PI_F * t / 41.f + 0.7f),
				0.f);
	}

	// Stay at rest while the GNSS receiver reports a wandering velocity and position
	void runAtRestWithGnssWander(float duration_s)
	{
		const float dt = 0.01f;

		for (float t = 0.f; t < duration_s; t += dt) {
			_sensor_simulator._gps.setVelocity(gnssVelocityError(t));
			_sensor_simulator._gps.setPositionRateNED(gnssPositionDriftRate(t));
			_sensor_simulator.runMicroseconds(10'000);
		}

		_sensor_simulator._gps.setVelocity(Vector3f());
		_sensor_simulator._gps.setPositionRateNED(Vector3f());
	}

	// Accelerate north, then east, then back: the GNSS velocity now observes the heading
	void runAcceleratingTrajectory()
	{
		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);

		// keep the attitude the vehicle had on the ground
		const Dcmf R_body_to_world(Quatf(Eulerf(kTilt, kTilt, kYawTrue)));
		_sensor_simulator.setOrientation(R_body_to_world);

		const float dt = 1e-3f;
		const float accel = 2.f;   // m/s^2
		const float leg_duration = 4.f;
		const Vector3f leg_accel[] {{accel, 0.f, 0.f}, {0.f, accel, 0.f}, {-accel, 0.f, 0.f}, {0.f, -accel, 0.f}};

		// the simulator timestamps the GNSS sample kGnssDelay in the past, so the
		// sample must contain the velocity of the vehicle at that (delayed) time
		std::vector<Vector3f> vel_history{Vector3f{}};
		const size_t gnss_delay_steps = static_cast<size_t>(kGnssDelay / dt);

		for (int repeat = 0; repeat < 3; repeat++) {
			for (const Vector3f &accel_world : leg_accel) {
				for (float t = 0.f; t < leg_duration; t += dt) {
					const Vector3f specific_force = R_body_to_world.transpose() * (accel_world + Vector3f(0.f, 0.f, -CONSTANTS_ONE_G));
					_sensor_simulator._imu.setData(specific_force, Vector3f{});

					vel_history.push_back(vel_history.back() + accel_world * dt);
					const size_t n = vel_history.size();
					const Vector3f vel_gnss = (n > gnss_delay_steps) ? vel_history[n - 1 - gnss_delay_steps] : Vector3f{};
					_sensor_simulator._gps.setVelocity(vel_gnss);
					_sensor_simulator._gps.setPositionRateNED(vel_gnss);

					_sensor_simulator.runMicroseconds(1000);
				}
			}
		}
	}

	float yawErrorDeg() const
	{
		return math::degrees(wrap_pi(_ekf_wrapper.getYawAngle() - kYawTrue));
	}
};

TEST_F(EkfHeadingAtRestTest, gnssWanderDoesNotMoveHeading)
{
	// GIVEN: heading initialized by the mag and GNSS fusion running, at rest
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	ASSERT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
	const float yaw_err_start = yawErrorDeg();
	const float yaw_var_start = _ekf->getYawVar();

	// WHEN: the vehicle stays at rest for 5 minutes with a wandering GNSS error
	runAtRestWithGnssWander(300.f);

	const float yaw_err_end = yawErrorDeg();
	const float yaw_var_end = _ekf->getYawVar();
	printf("yaw_err start=%.2f end=%.2f deg  yaw_std start=%.2f end=%.2f deg\n",
	       (double)yaw_err_start, (double)yaw_err_end,
	       (double)math::degrees(sqrtf(yaw_var_start)), (double)math::degrees(sqrtf(yaw_var_end)));

	// THEN: GNSS does not make the heading observable and the heading does not move
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_FALSE(_ekf->control_status_flags().heading_observable);
	EXPECT_LT(fabsf(yaw_err_end - yaw_err_start), 0.5f);

	// AND: the filter does not become more confident about the heading
	EXPECT_GE(yaw_var_end, yaw_var_start * 0.99f);
	EXPECT_LE(yaw_var_end, sq(_ekf->getParamHandle()->ekf2_head_noise) * 1.01f);
}

TEST_F(EkfHeadingAtRestTest, manualHeadingHoldsAtRest)
{
	// GIVEN: the operator sets the heading without accuracy (default 20 deg)
	const float yaw_set = kYawTrue + math::radians(10.f);
	_ekf->resetHeadingToExternalObservation(yaw_set, math::radians(20.f));
	_sensor_simulator.runSeconds(1);
	const float yaw_after_set = _ekf_wrapper.getYawAngle();
	EXPECT_NEAR(yaw_after_set, yaw_set, math::radians(0.5f));

	// WHEN: the vehicle stays at rest with a wandering GNSS error
	runAtRestWithGnssWander(120.f);

	const float yaw_drift = math::degrees(wrap_pi(_ekf_wrapper.getYawAngle() - yaw_after_set));
	printf("yaw drift after manual set=%.2f deg\n", (double)yaw_drift);

	// THEN: the manual heading is kept
	EXPECT_LT(fabsf(yaw_drift), 0.5f);
}

TEST_F(EkfHeadingAtRestTest, gnssCorrectsHeadingWhenMoving)
{
	// GIVEN: a wrong heading set while at rest
	const float yaw_err_initial = math::radians(20.f);
	_ekf->resetHeadingToExternalObservation(kYawTrue + yaw_err_initial, math::radians(20.f));
	_sensor_simulator.runSeconds(1);
	EXPECT_NEAR(yawErrorDeg(), math::degrees(yaw_err_initial), 0.5f);

	// WHEN: the vehicle accelerates with GNSS velocity and position fused
	runAcceleratingTrajectory();
	printf("yaw_err after manoeuvre=%.2f deg\n", (double)yawErrorDeg());

	// THEN: the heading is observable again and GNSS corrects it
	EXPECT_TRUE(_ekf->control_status_flags().heading_observable);
	EXPECT_LT(fabsf(yawErrorDeg()), 3.f);
}
