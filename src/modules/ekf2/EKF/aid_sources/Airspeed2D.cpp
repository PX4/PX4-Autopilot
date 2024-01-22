/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "Airspeed2D.hpp"

#include "../ekf.h"

#include <ekf_derivation/generated/compute_airspeed_2d_innov_innov_var_and_hx.h>
#include <ekf_derivation/generated/compute_airspeed_2d_y_innov_innov_var_and_hy.h>

bool Airspeed2D::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_sensor_airflow_sub.updated()) {
		sensor_airflow_s sensor_airflow;

		if (_sensor_airflow_sub.copy(&sensor_airflow)) {

			AirflowSample sample{};
			sample.time_us = sensor_airflow.timestamp;
			sample.speed = sensor_airflow.speed;
			sample.direction_rad = sensor_airflow.direction;

			_airflow_buffer.push(sample);
			_time_last_buffer_push = imu_delayed.time_us;
		}
	}

#endif // MODULE_NAME



	AirflowSample sample;

	if (_airflow_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {




		//ECL_INFO("%lu: %lu sensor wind fusion", _time_delayed_us, sample_delayed.time_us);

		ekf.resetEstimatorAidStatus(_aid_src);

		_aid_src.timestamp_sample = sample.time_us;

		// body frame wind speed and direction
		const Vector2f measurement{sample.speed * cosf(sample.direction_rad), sample.speed * sinf(sample.direction_rad)};

		float wind_noise = 4.f; // m/s
		const float R = sq(wind_noise);

		const auto state_vector = ekf.state().vector();
		const auto &P = ekf.covariances();

		Vector2f innov{};
		Vector2f innov_var{};
		Ekf::VectorState H;

		sym::ComputeAirspeed2DInnovInnovVarAndHx(state_vector, P, measurement, R, FLT_EPSILON,
				&innov, &innov_var, &H);

		_aid_src.observation[0] = measurement(0);
		_aid_src.observation[1] = measurement(1);
		_aid_src.innovation[0] = innov(0);
		_aid_src.innovation[1] = innov(1);

		_aid_src.observation_variance[0] = R;
		_aid_src.observation_variance[1] = R;
		_aid_src.innovation_variance[0] = innov_var(0);
		_aid_src.innovation_variance[1] = innov_var(1);

		float innov_gate = 3.f; // innovation gate
		ekf.setEstimatorAidStatusTestRatio(_aid_src, innov_gate);

		bool conditions_passing = ekf.control_status_flags().wind;

		bool update_all_states = ekf.control_status_flags().wind && ekf.control_status_flags().in_air
					 && (_aid_src.test_ratio[0] < 0.1f) && (_aid_src.test_ratio[1] < 0.1f)
					 && !ekf.getWindVelocityVariance().longerThan(0.1f * R);

		// const bool update_wind_only = !ekf.control_status_flags().wind_dead_reckoning;

		bool fused[2] {false, false};

		if (ekf.control_status_flags().wind
		    && conditions_passing
		    && (_aid_src.test_ratio[0] < 1.f)
		    && (_aid_src.test_ratio[1] < 1.f)
		   ) {
			// fuse x
			{
				// H computed above
				Ekf::VectorState Kfusion = P * H / innov_var(0);

				if (!update_all_states) {
					for (unsigned row = 0; row <= 21; row++) {
						Kfusion(row) = 0.f;
					}
				}

				fused[0] = ekf.measurementUpdate(Kfusion, innov_var(0), innov(0));
			}

			// fuse y
			{
				// recalculate innovation variance because state covariances have changed due to previous fusion (linearise using the same initial state for all axes)
				sym::ComputeAirspeed2DYInnovInnovVarAndHy(state_vector, P, measurement, R, FLT_EPSILON,
						&innov(1), &innov_var(1), &H);

				Ekf::VectorState Kfusion = P * H / innov_var(1);

				if (!update_all_states) {
					for (unsigned row = 0; row <= 21; row++) {
						Kfusion(row) = 0.f;
					}
				}

				fused[1] = ekf.measurementUpdate(Kfusion, innov_var(1), innov(1));
			}
		}

		if (fused[0] && fused[1]) {
			_aid_src.time_last_fuse = imu_delayed.time_us;
			_aid_src.fused = true;
		}


		if (!ekf.control_status_flags().wind && ekf.local_position_is_valid()) {

			Vector3f airflow_bf{measurement(0), measurement(1), 0.f};
			Vector3f offset_body{0.f, 0.f, 0.f};

			const Dcmf R_to_earth{ekf.state().quat_nominal};

			Vector3f airflow_ned = (R_to_earth * (airflow_bf - offset_body));

			Vector2f wind_vel{
				ekf.state().vel(0) - airflow_ned(0),
				ekf.state().vel(1) - airflow_ned(1)
			};

			ekf.resetWind(wind_vel, R);

			_aid_src.time_last_fuse = imu_delayed.time_us;
		}

#if defined(MODULE_NAME)
		_aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_airspeed2d_pub.publish(_aid_src);
#endif // MODULE_NAME

		return true;
	}

	return false;
}
