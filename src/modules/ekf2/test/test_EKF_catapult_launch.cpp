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
 * Catapult launch of a fixed-wing without magnetometer.
 *
 * Scenario:
 *  - heading is aligned manually (external heading observation), no mag
 *  - GNSS velocity and position are fused
 *  - the vehicle sits on the catapult long enough for the heading uncertainty
 *    to grow (heading is unobservable at zero ground speed)
 *  - the catapult produces ~15g while the GNSS receiver's internal dynamic
 *    model is limited to ~4g, so the reported GNSS velocity/position lag
 *    behind the truth and produce a large innovation along the launch
 *    direction
 *
 * The launch direction is purely a magnitude error, so a correct vector
 * (or a correctly sequenced scalar) update must not rotate the heading.
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"

class EkfCatapultLaunchTest : public ::testing::Test
{
public:
	EkfCatapultLaunchTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	static constexpr float kCatapultAccel = 15.f * CONSTANTS_ONE_G;  ///< catapult acceleration (m/s^2)
	static constexpr float kGnssAccelLimit = 4.f * CONSTANTS_ONE_G;  ///< GNSS internal dynamic model limit (m/s^2)
	static constexpr float kLaunchSpeed = 30.f;                      ///< speed at end of catapult stroke (m/s)
	static constexpr float kGnssDelay = 0.11f;                       ///< GNSS delay used by the simulator (s)

	float _yaw_ref{0.f};

	void SetUp() override
	{
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		// no magnetometer
		_ekf_wrapper.setMagFuseTypeNone();
		_sensor_simulator.runSeconds(2);
	}

	// Align the heading manually and start fusing GNSS
	void alignHeadingAndStartGnss(float yaw, float yaw_accuracy_deg)
	{
		_yaw_ref = yaw;
		_sensor_simulator.setOrientation(Quatf(Eulerf(0.f, 0.f, yaw)));
		_ekf->resetHeadingToExternalObservation(yaw, math::radians(yaw_accuracy_deg));

		_ekf_wrapper.enableGpsFusion();
		_sensor_simulator.startGps();
		_sensor_simulator.runSeconds(12);
	}

	// Sit on the catapult: heading is unobservable at zero ground speed
	void waitOnCatapult(float duration_s)
	{
		_sensor_simulator.runSeconds(duration_s);
	}

	struct LaunchResult {
		float yaw_err_max{0.f};
		float yaw_err_final{0.f};
		float yaw_std_at_release{0.f};
	};

	// Simulate the catapult stroke followed by a constant-velocity phase
	LaunchResult runLaunch(float duration_s, bool verbose = false)
	{
		LaunchResult res{};
		res.yaw_std_at_release = sqrtf(_ekf->getYawVar());

		const Vector3f dir(cosf(_yaw_ref), sinf(_yaw_ref), 0.f);
		const float stroke_duration = kLaunchSpeed / kCatapultAccel;

		_ekf->set_in_air_status(true);
		_ekf->set_vehicle_at_rest(false);

		const float dt = 1e-3f;

		for (float t = 0.f; t < duration_s; t += dt) {
			// true specific force in body frame (level attitude, x forward)
			const float accel_x = (t < stroke_duration) ? kCatapultAccel : 0.f;
			_sensor_simulator._imu.setData(Vector3f{accel_x, 0.f, -CONSTANTS_ONE_G}, Vector3f{});

			// the simulator timestamps the GNSS sample kGnssDelay in the past, so the
			// sample must contain the state of the world at that (delayed) time
			const float t_gnss = math::max(t - kGnssDelay, 0.f);
			const float v_true = math::min(kCatapultAccel * t_gnss, kLaunchSpeed);
			// the receiver's dynamic model limits how fast the reported velocity can change
			const float v_gnss = math::min(kGnssAccelLimit * t_gnss, v_true);

			_sensor_simulator._gps.setVelocity(dir * v_gnss);
			_sensor_simulator._gps.setPositionRateNED(dir * v_gnss);

			_sensor_simulator.runMicroseconds(1000);

			const float yaw_err = wrap_pi(_ekf_wrapper.getYawAngle() - _yaw_ref);

			if (fabsf(yaw_err) > fabsf(res.yaw_err_max)) {
				res.yaw_err_max = yaw_err;
			}

			if (verbose && (fmodf(t, 0.05f) < dt)) {
				const auto &vel = _ekf->aid_src_gnss_vel();
				printf("t=%5.2f yaw_err=%7.2f deg yaw_std=%6.2f deg vel=[%6.2f %6.2f] innov=[%6.2f %6.2f]\n",
				       (double)t, (double)math::degrees(yaw_err), (double)math::degrees(sqrtf(_ekf->getYawVar())),
				       (double)_ekf->getVelocity()(0), (double)_ekf->getVelocity()(1),
				       (double)vel.innovation[0], (double)vel.innovation[1]);
			}
		}

		res.yaw_err_final = wrap_pi(_ekf_wrapper.getYawAngle() - _yaw_ref);
		return res;
	}
};

// Heading pointing 45 deg (both N and E velocity components are non-zero)
TEST_F(EkfCatapultLaunchTest, headingPullNorthEast)
{
	alignHeadingAndStartGnss(math::radians(45.f), 5.f);
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	waitOnCatapult(300.f);

	const LaunchResult res = runLaunch(3.f);

	printf("[45 deg] yaw_std_at_release=%.2f deg  yaw_err_max=%.2f deg  yaw_err_final=%.2f deg\n",
	       (double)math::degrees(res.yaw_std_at_release),
	       (double)math::degrees(res.yaw_err_max),
	       (double)math::degrees(res.yaw_err_final));

	// THEN: the heading is not pulled away by the velocity/position innovation.
	// With the stale sequential update this used to reach ~34 deg.
	EXPECT_LT(fabsf(math::degrees(res.yaw_err_max)), 2.f);
	EXPECT_LT(fabsf(math::degrees(res.yaw_err_final)), 2.f);
}

// Heading pointing North: the launch is aligned with the first fused axis
TEST_F(EkfCatapultLaunchTest, headingPullNorth)
{
	alignHeadingAndStartGnss(0.f, 5.f);
	ASSERT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	waitOnCatapult(300.f);

	const LaunchResult res = runLaunch(3.f);

	printf("[0 deg] yaw_std_at_release=%.2f deg  yaw_err_max=%.2f deg  yaw_err_final=%.2f deg\n",
	       (double)math::degrees(res.yaw_std_at_release),
	       (double)math::degrees(res.yaw_err_max),
	       (double)math::degrees(res.yaw_err_final));

	// THEN: no heading error either. Note that this case was already almost unaffected by the
	// stale sequential update because the launch is aligned with the first fused axis, so there
	// is no cross-axis coupling to get wrong.
	EXPECT_LT(fabsf(math::degrees(res.yaw_err_max)), 2.f);
	EXPECT_LT(fabsf(math::degrees(res.yaw_err_final)), 2.f);
}
