/****************************************************************************
 *
 *   Copyright (c) 2022 PX4. All rights reserved.
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
 * @file height_control.cpp
 */

#include "ekf.h"

void Ekf::controlHeightFusion(const imuSample &imu_delayed)
{
	checkVerticalAccelerationHealth(imu_delayed);

	updateGroundEffect();

	controlBaroHeightFusion();
	controlGnssHeightFusion(_gps_sample_delayed);
#if defined(CONFIG_EKF2_RANGE_FINDER)
	controlRangeHeightFusion();
#endif // CONFIG_EKF2_RANGE_FINDER

	checkHeightSensorRefFallback();
}

void Ekf::checkHeightSensorRefFallback()
{
	if (_height_sensor_ref != HeightSensor::UNKNOWN) {
		// The reference sensor is running, all good
		return;
	}

	HeightSensor fallback_list[4];

	switch (_params.height_sensor_ref) {
	default:

	/* FALLTHROUGH */
	case HeightSensor::UNKNOWN:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::BARO:
		fallback_list[0] = HeightSensor::BARO;
		fallback_list[1] = HeightSensor::GNSS;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::GNSS:
		fallback_list[0] = HeightSensor::GNSS;
		fallback_list[1] = HeightSensor::BARO;
		fallback_list[2] = HeightSensor::EV;
		fallback_list[3] = HeightSensor::RANGE;
		break;

	case HeightSensor::RANGE:
		fallback_list[0] = HeightSensor::RANGE;
		fallback_list[1] = HeightSensor::EV;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;

	case HeightSensor::EV:
		fallback_list[0] = HeightSensor::EV;
		fallback_list[1] = HeightSensor::RANGE;
		fallback_list[2] = HeightSensor::BARO;
		fallback_list[3] = HeightSensor::GNSS;
		break;
	}

	for (unsigned i = 0; i < 4; i++) {
		if (((fallback_list[i] == HeightSensor::BARO) && _control_status.flags.baro_hgt)
		    || ((fallback_list[i] == HeightSensor::GNSS) && _control_status.flags.gps_hgt)
		    || ((fallback_list[i] == HeightSensor::RANGE) && _control_status.flags.rng_hgt)
		    || ((fallback_list[i] == HeightSensor::EV) && _control_status.flags.ev_hgt)) {
			ECL_INFO("fallback to secondary height reference");
			_height_sensor_ref = fallback_list[i];
			break;
		}
	}
}

void Ekf::checkVerticalAccelerationHealth(const imuSample &imu_delayed)
{
	// Check for IMU accelerometer vibration induced clipping as evidenced by the vertical
	// innovations being positive and not stale.
	// Clipping usually causes the average accel reading to move towards zero which makes the INS
	// think it is falling and produces positive vertical innovations.

	Likelihood inertial_nav_falling_likelihood = estimateInertialNavFallingLikelihood();

	// Check for more than 50% clipping affected IMU samples within the past 1 second
	const uint16_t clip_count_limit = 1.f / _dt_ekf_avg;
	const bool is_clipping = imu_delayed.delta_vel_clipping[0] ||
				 imu_delayed.delta_vel_clipping[1] ||
				 imu_delayed.delta_vel_clipping[2];

	if (is_clipping && _clip_counter < clip_count_limit) {
		_clip_counter++;

	} else if (_clip_counter > 0) {
		_clip_counter--;
	}

	_fault_status.flags.bad_acc_clipping = _clip_counter > clip_count_limit / 2;

	const bool is_clipping_frequently = _clip_counter > 0;

	// Do not require evidence of clipping if the likelihood of having the INS falling is high
	const bool bad_vert_accel = (is_clipping_frequently && (inertial_nav_falling_likelihood == Likelihood::MEDIUM))
				    || (inertial_nav_falling_likelihood == Likelihood::HIGH);

	if (bad_vert_accel) {
		_time_bad_vert_accel = imu_delayed.time_us;

	} else {
		_time_good_vert_accel = imu_delayed.time_us;
	}

	// declare a bad vertical acceleration measurement and make the declaration persist
	// for a minimum of BADACC_PROBATION seconds
	if (_fault_status.flags.bad_acc_vertical) {
		_fault_status.flags.bad_acc_vertical = isRecent(_time_bad_vert_accel, BADACC_PROBATION);

	} else {
		_fault_status.flags.bad_acc_vertical = bad_vert_accel;
	}
}

Likelihood Ekf::estimateInertialNavFallingLikelihood() const
{
	bool likelihood_high = false;
	bool likelihood_medium = false;

	enum class ReferenceType { PRESSURE, GNSS, GROUND };

	struct {
		ReferenceType ref_type{};
		float innov{0.f};
		float innov_var{0.f};
		bool failed_min{false};
		bool failed_lim{false};
	} checks[6] {};

	if (_control_status.flags.baro_hgt) {
		checks[0] = {ReferenceType::PRESSURE, _aid_src_baro_hgt.innovation, _aid_src_baro_hgt.innovation_variance};
	}

	if (_control_status.flags.gps_hgt) {
		checks[1] = {ReferenceType::GNSS, _aid_src_gnss_hgt.innovation, _aid_src_gnss_hgt.innovation_variance};
	}

	if (_control_status.flags.gps) {
		checks[2] = {ReferenceType::GNSS, _aid_src_gnss_vel.innovation[2], _aid_src_gnss_vel.innovation_variance[2]};
	}

#if defined(CONFIG_EKF2_RANGE_FINDER)
	if (_control_status.flags.rng_hgt) {
		checks[3] = {ReferenceType::GROUND, _aid_src_rng_hgt.innovation, _aid_src_rng_hgt.innovation_variance};
	}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_EXTERNAL_VISION)
	if (_control_status.flags.ev_hgt) {
		checks[4] = {ReferenceType::GROUND, _aid_src_ev_hgt.innovation, _aid_src_ev_hgt.innovation_variance};
	}

	if (_control_status.flags.ev_vel) {
		checks[5] = {ReferenceType::GROUND, _aid_src_ev_vel.innovation[2], _aid_src_ev_vel.innovation_variance[2]};
	}
#endif // CONFIG_EKF2_EXTERNAL_VISION

	// Compute the check based on innovation ratio for all the sources
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].innov_var < FLT_EPSILON) {
			continue;
		}

		const float innov_ratio = checks[i].innov / sqrtf(checks[i].innov_var);
		checks[i].failed_min = innov_ratio > _params.vert_innov_test_min;
		checks[i].failed_lim = innov_ratio > _params.vert_innov_test_lim;
	}

	// Check all the sources agains each other
	for (unsigned i = 0; i < 6; i++) {
		if (checks[i].failed_lim) {
			// There is a chance that the inertial nav is falling if one source is failing the test
			likelihood_medium = true;
		}

		for (unsigned j = 0; j < 6; j++) {

			if ((checks[i].ref_type != checks[j].ref_type) && checks[i].failed_lim && checks[j].failed_min) {
				// There is a high chance that the inertial nav is failing if two sources are failing the test
				likelihood_high = true;
			}
		}
	}

	if (likelihood_high) {
		return Likelihood::HIGH;

	} else if (likelihood_medium) {
		return Likelihood::MEDIUM;
	}

	return Likelihood::LOW;
}
