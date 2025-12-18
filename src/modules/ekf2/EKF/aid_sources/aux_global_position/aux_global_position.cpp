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

	for (int instance = 0; instance < MAX_AGP_IDS; instance++) {
		if (_aux_global_position_subs[instance].updated()) {

			aux_global_position_s aux_global_position{};
			_aux_global_position_subs[instance].copy(&aux_global_position);

			const uint8_t sensor_id = aux_global_position.id;
			const int slot = mapSensorIdToSlot(sensor_id);

			if (slot >= MAX_AGP_IDS) {
				// All parameter slots are full, cannot handle this sensor. Set ID of unused slot to 0
				continue;
			}

			const int64_t time_us = aux_global_position.timestamp_sample - static_cast<int64_t>(getDelayParam(slot) * 1000);

			AuxGlobalPositionSample sample{};
			sample.time_us = time_us;
			sample.id = sensor_id;
			sample.latitude = aux_global_position.lat;
			sample.longitude = aux_global_position.lon;
			sample.altitude_amsl = aux_global_position.alt;
			sample.eph = aux_global_position.eph;
			sample.epv = aux_global_position.epv;
			sample.lat_lon_reset_counter = aux_global_position.lat_lon_reset_counter;

			_sources[slot].buffer.push(sample);
			_sources[slot].time_last_buffer_push = imu_delayed.time_us;
		}
	}

#endif // MODULE_NAME

	for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
		SourceData &source = _sources[slot];
		AuxGlobalPositionSample sample;

		if (source.buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

			if (!(getCtrlParam(slot) & static_cast<int32_t>(Ctrl::kHPos))) {
				continue;
			}

			estimator_aid_source2d_s &aid_src = source.aid_src;
			const LatLonAlt position(sample.latitude, sample.longitude, sample.altitude_amsl);
			const Vector2f innovation = (ekf.getLatLonAlt() - position).xy(); // altitude measurements are not used

			float pos_noise = math::max(sample.eph, getNoiseParam(slot), 0.01f);
			const float pos_var = sq(pos_noise);
			const Vector2f pos_obs_var(pos_var, pos_var);

			ekf.updateAidSourceStatus(aid_src,
						  sample.time_us,                                      // sample timestamp
						  matrix::Vector2d(sample.latitude, sample.longitude), // observation
						  pos_obs_var,                                         // observation variance
						  innovation,                                          // innovation
						  Vector2f(ekf.getPositionVariance()) + pos_obs_var,   // innovation variance
						  math::max(getGateParam(slot), 1.f));                   // innovation gate

			const bool starting_conditions = PX4_ISFINITE(sample.latitude) && PX4_ISFINITE(sample.longitude)
							 && ekf.control_status_flags().yaw_align;
			const bool continuing_conditions = starting_conditions
							   && ekf.global_origin_valid();

			switch (source.state) {
			case State::kStopped:

			/* FALLTHROUGH */
			case State::kStarting:
				if (starting_conditions) {
					source.state = State::kStarting;

					if (ekf.global_origin_valid()) {
						const bool fused = ekf.fuseHorizontalPosition(aid_src);
						bool reset = false;

						if (!fused && isResetAllowed(ekf, slot)) {
							ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(aid_src.observation_variance));
							ekf.resetAidSourceStatusZeroInnovation(aid_src);
							reset = true;
						}

						if (fused || reset) {
							ekf.enableControlStatusAuxGpos();
							source.reset_counters.lat_lon = sample.lat_lon_reset_counter;
							source.state = State::kActive;
						}

					} else {
						// Try to initialize using measurement
						if (ekf.resetGlobalPositionTo(sample.latitude, sample.longitude, sample.altitude_amsl, pos_var,
									      sq(sample.epv))) {
							ekf.resetAidSourceStatusZeroInnovation(aid_src);
							ekf.enableControlStatusAuxGpos();
							source.reset_counters.lat_lon = sample.lat_lon_reset_counter;
							source.state = State::kActive;
						}
					}
				}

				break;

			case State::kActive:
				if (continuing_conditions) {
					ekf.fuseHorizontalPosition(aid_src);

					if (isTimedOut(aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.reset_timeout_max)
					    || (source.reset_counters.lat_lon != sample.lat_lon_reset_counter)) {
						if (isResetAllowed(ekf, slot)) {

							ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(aid_src.observation_variance));

							ekf.resetAidSourceStatusZeroInnovation(aid_src);

							source.reset_counters.lat_lon = sample.lat_lon_reset_counter;

						} else {
							ekf.disableControlStatusAuxGpos();
							source.state = State::kStopped;
						}
					}

				} else {
					ekf.disableControlStatusAuxGpos();
					source.state = State::kStopped;
				}

				break;

			default:
				break;
			}

#if defined(MODULE_NAME)
			aid_src.timestamp = hrt_absolute_time();
			source.aid_src_pub.publish(aid_src);

			source.test_ratio_filtered = math::max(fabsf(aid_src.test_ratio_filtered[0]), fabsf(aid_src.test_ratio_filtered[1]));
#endif // MODULE_NAME

		} else if ((source.state != State::kStopped) && isTimedOut(source.time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
			ekf.disableControlStatusAuxGpos();
			source.state = State::kStopped;
			ECL_WARN("Aux global position data stopped for slot %d (sensor ID %d)", slot, getIdParam(slot));
		}
	}
}

bool AuxGlobalPosition::isResetAllowed(const Ekf &ekf, int slot) const
{
	return ((static_cast<Mode>(getModeParam(slot)) == Mode::kAuto)
		&& !ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf.control_status_flags().aux_gpos))
	       || ((static_cast<Mode>(getModeParam(slot)) == Mode::kDeadReckoning)
		   && !ekf.isOtherSourceOfHorizontalAidingThan(ekf.control_status_flags().aux_gpos));
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION
