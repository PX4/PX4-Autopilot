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
#include <aid_sources/aux_global_position/aux_global_position_control.hpp>
#include <aid_sources/aux_global_position/aux_global_position.hpp>

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

AgpSource::AgpSource(int instance_id, AuxGlobalPosition *manager)
	: _agp_sub(ORB_ID(aux_global_position), instance_id)
	, _manager(manager)
	, _instance_id(instance_id) {}

void AgpSource::checkAndBufferData(const estimator::imuSample &imu_delayed)
{
	if (_agp_sub.updated()) {

		aux_global_position_s aux_global_position{};
		_agp_sub.copy(&aux_global_position);

		const uint8_t sensor_id = aux_global_position.id;
		const int slot = _manager->mapSensorIdToSlot(sensor_id);

		if (slot >= AuxGlobalPosition::MAX_AGP_IDS) {
			// All parameter slots are full, cannot handle this sensor
			return;
		}

		if (slot != _instance_id) {
			// This sensor is mapped to a different instance
			return;
		}

		const int64_t time_us = aux_global_position.timestamp_sample
					- static_cast<int64_t>(getDelayParam() * 1000);

		AuxGlobalPositionSample sample{};
		sample.time_us = time_us;
		sample.id = sensor_id;
		sample.latitude = aux_global_position.lat;
		sample.longitude = aux_global_position.lon;
		sample.altitude_amsl = aux_global_position.alt;
		sample.eph = aux_global_position.eph;
		sample.epv = aux_global_position.epv;
		sample.lat_lon_reset_counter = aux_global_position.lat_lon_reset_counter;

		_buffer.push(sample);
		_time_last_buffer_push = imu_delayed.time_us;
	}
}

void AgpSource::update(Ekf &ekf, const estimator::imuSample &imu_delayed)
{
	AuxGlobalPositionSample sample;

	if (_buffer.pop_first_older_than(imu_delayed.time_us, &sample)) {

		if (!(getCtrlParam() & static_cast<int32_t>(Ctrl::kHPos))) {
			return;
		}

		const LatLonAlt position(sample.latitude, sample.longitude, sample.altitude_amsl);
		const Vector2f innovation = (ekf.getLatLonAlt() - position).xy(); // altitude measurements are not used

		float pos_noise = math::max(sample.eph, getNoiseParam(), 0.01f);
		const float pos_var = sq(pos_noise);
		const Vector2f pos_obs_var(pos_var, pos_var);

		ekf.updateAidSourceStatus(_aid_src,
					  sample.time_us,                                      // sample timestamp
					  matrix::Vector2d(sample.latitude, sample.longitude), // observation
					  pos_obs_var,                                         // observation variance
					  innovation,                                          // innovation
					  Vector2f(ekf.getPositionVariance()) + pos_obs_var,   // innovation variance
					  math::max(getGateParam(), 1.f));                     // innovation gate

		const bool starting_conditions = PX4_ISFINITE(sample.latitude) && PX4_ISFINITE(sample.longitude)
						 && ekf.control_status_flags().yaw_align;
		const bool continuing_conditions = starting_conditions
						   && ekf.global_origin_valid();

		switch (_state) {
		case State::kStopped:

		/* FALLTHROUGH */
		case State::kStarting:
			if (starting_conditions) {
				_state = State::kStarting;

				if (ekf.global_origin_valid()) {
					const bool fused = ekf.fuseHorizontalPosition(_aid_src);
					bool reset = false;

					if (!fused && isResetAllowed(ekf)) {
						ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(_aid_src.observation_variance));
						ekf.resetAidSourceStatusZeroInnovation(_aid_src);
						reset = true;
					}

					if (fused || reset) {
						ekf.enableControlStatusAuxGpos();
						_reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::kActive;
					}

				} else {
					// Try to initialize using measurement
					if (ekf.resetGlobalPositionTo(sample.latitude, sample.longitude, sample.altitude_amsl, pos_var,
								      sq(sample.epv))) {
						ekf.resetAidSourceStatusZeroInnovation(_aid_src);
						ekf.enableControlStatusAuxGpos();
						_reset_counters.lat_lon = sample.lat_lon_reset_counter;
						_state = State::kActive;
					}
				}
			}

			break;

		case State::kActive:
			if (continuing_conditions) {
				ekf.fuseHorizontalPosition(_aid_src);

				if (isTimedOut(_aid_src.time_last_fuse, imu_delayed.time_us, ekf._params.reset_timeout_max)
				    || (_reset_counters.lat_lon != sample.lat_lon_reset_counter)) {
					if (isResetAllowed(ekf)) {

						ekf.resetHorizontalPositionTo(sample.latitude, sample.longitude, Vector2f(_aid_src.observation_variance));

						ekf.resetAidSourceStatusZeroInnovation(_aid_src);

						_reset_counters.lat_lon = sample.lat_lon_reset_counter;

					} else {
						_state = State::kStopped;

						if (!_manager->anySourceFusing()) {
							ekf.disableControlStatusAuxGpos();
						}
					}
				}

			} else {
				_state = State::kStopped;

				if (!_manager->anySourceFusing()) {
					ekf.disableControlStatusAuxGpos();
				}
			}

			break;

		default:
			break;
		}

		_aid_src.timestamp = hrt_absolute_time();
		_aid_src_pub.publish(_aid_src);

		_test_ratio_filtered = math::max(fabsf(_aid_src.test_ratio_filtered[0]), fabsf(_aid_src.test_ratio_filtered[1]));

	} else if ((_state != State::kStopped) && isTimedOut(_time_last_buffer_push, imu_delayed.time_us, (uint64_t)5e6)) {
		_state = State::kStopped;

		if (!_manager->anySourceFusing()) {
			ekf.disableControlStatusAuxGpos();
		}

		ECL_WARN("Aux global position data stopped for instance %d", _instance_id);
	}
}

bool AgpSource::isResetAllowed(const Ekf &ekf) const
{
	return ((static_cast<Mode>(getModeParam()) == Mode::kAuto)
		&& !ekf.isOtherSourceOfHorizontalPositionAidingThan(ekf.control_status_flags().aux_gpos))
	       || ((static_cast<Mode>(getModeParam()) == Mode::kDeadReckoning)
		   && !ekf.isOtherSourceOfHorizontalAidingThan(ekf.control_status_flags().aux_gpos));
}

bool AgpSource::isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const
{
	return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < time_delayed_us);
}

int32_t AgpSource::getCtrlParam() const
{
	return _manager->getCtrlParam(_instance_id);
}

int32_t AgpSource::getModeParam() const
{
	return _manager->getModeParam(_instance_id);
}

float AgpSource::getDelayParam() const
{
	return _manager->getDelayParam(_instance_id);
}

float AgpSource::getNoiseParam() const
{
	return _manager->getNoiseParam(_instance_id);
}

float AgpSource::getGateParam() const
{
	return _manager->getGateParam(_instance_id);
}

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME
