/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "ekf.h"

#include "aid_sources/aux_global_position/aux_global_position.hpp"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

void AuxGlobalPosition::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{

#if defined(MODULE_NAME)

	if (_aux_global_position_sub.updated()) {

		vehicle_global_position_s aux_global_position{};
		_aux_global_position_sub.copy(&aux_global_position);

		const int64_t time_us = aux_global_position.timestamp_sample - static_cast<int64_t>(_param_ekf2_agp_delay.get() * 1000);

		AuxGlobalPositionSample sample{};
		sample.time_us = time_us;
		sample.latitude = aux_global_position.lat;
		sample.longitude = aux_global_position.lon;
		sample.altitude_amsl = aux_global_position.alt;
		sample.eph = aux_global_position.eph;
		sample.epv = aux_global_position.epv;
		sample.lat_lon_reset_counter = aux_global_position.lat_lon_reset_counter;

		_aux_global_position_buffer.push(sample);

		_time_last_buffer_push = imu_delayed.time_us;
	}

#endif // MODULE_NAME

	AuxGlobalPositionSample sample;

	if (_aux_global_position_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

		if (!(_param_ekf2_agp_ctrl.get() & static_cast<int32_t>(Ctrl::HPOS))) {
			return;
		}

		estimator_aid_source2d_s aid_src{};
		const LatLonAlt position(sample.latitude, sample.longitude, sample.altitude_amsl);
		const Vector2f innovation = (ekf.getLatLonAlt() - position).xy(); // altitude measurements are not used

		// relax the upper observation noise limit which prevents bad measurements perturbing the position estimate
		float pos_noise = math::max(sample.eph, _param_ekf2_agp_noise.get(), 0.01f);
		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);

		ekf.updateAidSourceStatus(aid_src,
					  sample.time_us,                                      // sample timestamp
					  matrix::Vector2d(sample.latitude, sample.longitude), // observation
					  pos_obs_var,                                         // observation variance
					  innovation,                                          // innovation
					  Vector2f(ekf.getPositionVariance()) + pos_obs_var,   // innovation variance
					  math::max(_param_ekf2_agp_gate.get(), 1.f));         // innovation gate

		const bool starting_conditions = PX4_ISFINITE(sample.latitude) && PX4_ISFINITE(sample.longitude)
						 && ekf.control_status_flags().yaw_align;
		const bool continuing_conditions = starting_conditions
						   && ekf.global_origin_valid();

		switch (_state) {
		case State::stopped:

		/* FALLTHROUGH */
		case State::starting:
			if (starting_conditions) {
				_state = State::starting;

				if (ekf.global_origin_valid()) {
					ekf.enableControlStatusAuxGpos();
					_reset_counters.lat_lon = sample.lat_lon_reset_counter;
					_state = State::active;

				} else {
					// Try to initialize using measurement
					if (ekf.resetGlobalPositionTo(sample.latitude, sample.longitude, sample.altitude_amsl, pos_var,
								      sq(sample.epv))) {
						ekf.enableControlStatusAuxGpos();
						_reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::active;
					}
				}
			}

			break;

		case State::active:
			if (continuing_conditions) {
				ekf.fuseHorizontalPosition(aid_src);

				if (isTimedOut(aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.no_aid_timeout_max)
				    || (_reset_counters.lat_lon != sample.lat_lon_reset_counter)) {

					ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(aid_src.observation_variance));

					ekf.resetAidSourceStatusZeroInnovation(aid_src);

					_reset_counters.lat_lon = sample.lat_lon_reset_counter;
				}

			} else {
				ekf.disableControlStatusAuxGpos();
				_state = State::stopped;
			}

			break;

		default:
			break;
		}

#if defined(MODULE_NAME)
		aid_src.timestamp = hrt_absolute_time();
		_estimator_aid_src_aux_global_position_pub.publish(aid_src);

		_test_ratio_filtered = math::max(fabsf(aid_src.test_ratio_filtered[0]), fabsf(aid_src.test_ratio_filtered[1]));
#endif // MODULE_NAME

	} else if ((_state != State::stopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		ekf.disableControlStatusAuxGpos();
		_state = State::stopped;
		ECL_WARN("Aux global position data stopped");
	}
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
