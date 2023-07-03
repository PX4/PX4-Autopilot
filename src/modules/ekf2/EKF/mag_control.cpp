/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlMagFusion()
{
	bool mag_data_ready = false;

	magSample mag_sample;

	if (_mag_buffer) {
		mag_data_ready = _mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample);

		if (mag_data_ready) {

			// sensor or calibration has changed, clear any mag bias and reset low pass filter
			if (mag_sample.reset || (_mag_counter == 0)) {
				// Zero the magnetometer bias states
				_state.mag_B.zero();

				// Zero the corresponding covariances and set
				// variances to the values use for initial alignment
				P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

				// reset any saved covariance data for re-use when auto-switching between heading and 3-axis fusion
				_saved_mag_bf_variance.zero();

				_control_status.flags.mag_fault = false;

				_mag_lpf.reset(mag_sample.mag);
				_mag_counter = 1;

			} else {
				_mag_lpf.update(mag_sample.mag);
				_mag_counter++;
			}

			// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetomter Z component value.
			// this is useful if there is a lot of interference on the sensor measurement.
			if (_params.synthesize_mag_z && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
			    && (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps))
			   ) {
				const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
				mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
				_control_status.flags.synthetic_mag_z = true;

			} else {
				_control_status.flags.synthetic_mag_z = false;
			}

			_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_sample.mag);


			// compute mag heading innovation (for estimator_aid_src_mag_heading logging)
			const Vector3f mag_observation = mag_sample.mag - _state.mag_B;
			const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
			const Vector3f mag_earth_pred = R_to_earth * mag_observation;

			resetEstimatorAidStatus(_aid_src_mag_heading);
			_aid_src_mag_heading.timestamp_sample = mag_sample.time_us;
			_aid_src_mag_heading.observation = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
			_aid_src_mag_heading.innovation = wrap_pi(getEulerYaw(_R_to_earth) - _aid_src_mag_heading.observation);

			// compute magnetometer innovations (for estimator_aid_src_mag logging)
			//  rotate magnetometer earth field state into body frame
			const Vector3f mag_I_body = _state.quat_nominal.rotateVectorInverse(_state.mag_I);
			const Vector3f mag_innov = mag_I_body - mag_observation;

			resetEstimatorAidStatus(_aid_src_mag);
			_aid_src_mag.timestamp_sample = mag_sample.time_us;
			mag_observation.copyTo(_aid_src_mag.observation);
			mag_innov.copyTo(_aid_src_mag.innovation);

		} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
			// No data anymore. Stop until it comes back.
			stopMagFusion();
		}
	}

	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	if (mag_data_ready && !_control_status.flags.tilt_align && !_control_status.flags.yaw_align) {
		// calculate the initial magnetic field and yaw alignment
		// but do not mark the yaw alignement complete as it needs to be
		// reset once the leveling phase is done
		if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
			if ((_mag_counter > 1) && isTimedOut(_aid_src_mag_heading.time_last_fuse, (uint64_t)100'000)) {
				// rotate the magnetometer measurements into earth frame using a zero yaw angle
				// the angle of the projection onto the horizontal gives the yaw angle
				const Vector3f mag_earth_pred = updateYawInRotMat(0.f, _R_to_earth) * _mag_lpf.getState();
				const float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

				const float yaw_prev = getEulerYaw(_R_to_earth);

				if (fabsf(yaw_new - yaw_prev) > math::radians(1.f)) {

					ECL_INFO("mag heading init %.3f -> %.3f rad (declination %.1f)", (double)yaw_prev, (double)yaw_new, (double)getMagDeclination());

					// update the rotation matrix using the new yaw value
					_R_to_earth = updateYawInRotMat(yaw_new, Dcmf(_state.quat_nominal));
					_state.quat_nominal = _R_to_earth;

					// reset the output predictor state history to match the EKF initial values
					_output_predictor.alignOutputFilter(_state.quat_nominal, _state.vel, _state.pos);

					// set the earth magnetic field states using the updated rotation
					_state.mag_I = _R_to_earth * _mag_lpf.getState();
					_state.mag_B.zero();

					_aid_src_mag_heading.time_last_fuse = _time_delayed_us;
					_time_last_heading_fuse = _time_delayed_us;
				}
			}
		}

		return;
	}

	if (_params.mag_fusion_type >= MagFuseType::NONE
	    || _control_status.flags.mag_fault
	    || !_control_status.flags.tilt_align) {

		stopMagFusion();
		return;
	}

	_mag_yaw_reset_req |= !_control_status.flags.yaw_align;

	if (mag_data_ready && !_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw) {

		if (shouldInhibitMag()) {
			if (uint32_t(_time_delayed_us - _mag_use_not_inhibit_us) > (uint32_t)5e6) {
				// If magnetometer use has been inhibited continuously then stop the fusion
				stopMagFusion();
			}

			return;

		} else {
			_mag_use_not_inhibit_us = _time_delayed_us;
		}

		const bool mag_enabled_previously = _control_status_prev.flags.mag_hdg || _control_status_prev.flags.mag_3D;

		// Determine if we should use simple magnetic heading fusion which works better when
		// there are large external disturbances or the more accurate 3-axis fusion
		switch (_params.mag_fusion_type) {
		default:
		// FALLTHROUGH
		case MagFuseType::AUTO:
			selectMagAuto();
			break;

		case MagFuseType::INDOOR:

		/* fallthrough */
		case MagFuseType::HEADING:
			startMagHdgFusion();
			break;

		case MagFuseType::MAG_3D:
			startMag3DFusion();
			break;
		}

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {

			if (_mag_yaw_reset_req || !_control_status.flags.yaw_align || mag_sample.reset || !mag_enabled_previously || haglYawResetReq()) {

				if (magReset()) {
					_mag_yaw_reset_req = false;

				} else {
					// mag reset failed, try again next time
					_mag_yaw_reset_req = true;
				}
			}
		}

		if (!_control_status.flags.yaw_align) {
			// Having the yaw aligned is mandatory to continue
			return;
		}

		checkMagDeclRequired();

		runMagAndMagDeclFusions(mag_sample.mag);
	}
}

bool Ekf::haglYawResetReq()
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
#if defined(CONFIG_EKF2_RANGE_FINDER)
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
#else
		return true;
#endif // CONFIG_EKF2_RANGE_FINDER
	}

	return false;
}

bool Ekf::magReset()
{
	// prevent a reset being performed more than once on the same frame
	if ((_flt_mag_align_start_time == _time_delayed_us)
	    || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align)) {
		return false;
	}

	bool has_realigned_yaw = false;

	// use yaw estimator if available
	if (_control_status.flags.gps && isYawEmergencyEstimateAvailable()
	    && (_mag_counter > 1) // mag LPF available
	   ) {

		resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

		_information_events.flags.yaw_aligned_to_imu_gps = true;

		// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
		if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
			// use predicted earth field to reset states
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
			_state.mag_I = mag_earth_pred;

			const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
			_state.mag_B = _mag_lpf.getState() - (R_to_body * mag_earth_pred);

		} else {
			// Use the last magnetometer measurements to reset the field states
			// calculate initial earth magnetic field states
			_state.mag_I = _R_to_earth * _mag_lpf.getState();
			_state.mag_B.zero();
		}

		ECL_DEBUG("resetting mag I: [%.3f, %.3f, %.3f], B: [%.3f, %.3f, %.3f]",
			(double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
			(double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			);

		resetMagCov();

		has_realigned_yaw = true;
	}

	if (!has_realigned_yaw) {
		has_realigned_yaw = resetMagHeading();
	}

	if (has_realigned_yaw) {
		_control_status.flags.yaw_align = true;

		if (_control_status.flags.in_air) {
			_control_status.flags.mag_aligned_in_flight = true;

			// record the time for the magnetic field alignment event
			_flt_mag_align_start_time = _time_delayed_us;
		}

		return true;
	}

	return false;
}

void Ekf::selectMagAuto()
{
	check3DMagFusionSuitability();
	canUse3DMagFusion() ? startMag3DFusion() : startMagHdgFusion();
}

void Ekf::check3DMagFusionSuitability()
{
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _time_delayed_us;
	}
}

void Ekf::checkYawAngleObservability()
{
	if (_control_status.flags.gps) {
		// Check if there has been enough change in horizontal velocity to make yaw observable
		// Apply hysteresis to check to avoid rapid toggling
		if (_yaw_angle_observable) {
			_yaw_angle_observable = _accel_lpf_NE.norm() > _params.mag_acc_gate;

		} else {
			_yaw_angle_observable = _accel_lpf_NE.norm() > _params.mag_acc_gate * 2.f;
		}

	} else {
		_yaw_angle_observable = false;
	}
}

void Ekf::checkMagBiasObservability()
{
	// check if there is enough yaw rotation to make the mag bias states observable
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// initial yaw motion is detected
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// require sustained yaw motion of 50% the initial yaw rate threshold
		const float yaw_dt = 1e-6f * (float)(_time_delayed_us - _time_yaw_started);
		const float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _time_delayed_us;
}

bool Ekf::canUse3DMagFusion() const
{
	// Use of 3D fusion requires an in-air heading alignment but it should not
	// be used when the heading and mag biases are not observable for more than 2 seconds
	return _control_status.flags.mag_aligned_in_flight
	       && ((_time_delayed_us - _time_last_mov_3d_mag_suitable) < (uint64_t)2e6);
}

void Ekf::checkMagDeclRequired()
{
	// if we are using 3-axis magnetometer fusion, but without external NE aiding,
	// then the declination must be fused as an observation to prevent long term heading drift
	// fusing declination when gps aiding is available is optional, but recommended to prevent
	// problem if the vehicle is static for extended periods of time
	const bool user_selected = (_params.mag_declination_source & GeoDeclinationMask::FUSE_DECL);
	const bool not_using_ne_aiding = !_control_status.flags.gps;
	_control_status.flags.mag_dec = (_control_status.flags.mag_3D && (not_using_ne_aiding || user_selected));
}

bool Ekf::shouldInhibitMag() const
{
	// If the user has selected auto protection against indoor magnetic field errors, only use the magnetometer
	// if a yaw angle relative to true North is required for navigation. If no GPS or other earth frame aiding
	// is available, assume that we are operating indoors and the magnetometer should not be used.
	// Also inhibit mag fusion when a strong magnetic field interference is detected or the user
	// has explicitly stopped magnetometer use.
	const bool user_selected = (_params.mag_fusion_type == MagFuseType::INDOOR);

	const bool heading_not_required_for_navigation = !_control_status.flags.gps;

	return (user_selected && heading_not_required_for_navigation) || _control_status.flags.mag_field_disturbed;
}

bool Ekf::magFieldStrengthDisturbed(const Vector3f &mag_sample) const
{
	if (_params.check_mag_strength
	    && ((_params.mag_fusion_type <= MagFuseType::MAG_3D) || (_params.mag_fusion_type == MagFuseType::INDOOR && _control_status.flags.gps))) {

		if (PX4_ISFINITE(_mag_strength_gps)) {
			constexpr float wmm_gate_size = 0.2f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), _mag_strength_gps, wmm_gate_size);

		} else {
			constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
			constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss
			return !isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size);
		}
	}

	return false;
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

void Ekf::runMagAndMagDeclFusions(const Vector3f &mag)
{
	if (_control_status.flags.mag_3D) {
		run3DMagAndDeclFusions(mag);

	} else if (_control_status.flags.mag_hdg) {
		// Rotate the measurements into earth frame using the zero yaw angle
		Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		Vector3f mag_earth_pred = R_to_earth * (mag - _state.mag_B);

		// the angle of the projection onto the horizontal gives the yaw angle
		// calculate the yaw innovation and wrap to the interval between +-pi
		float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		float obs_var = fmaxf(sq(_params.mag_heading_noise), 1.e-4f);

		_aid_src_mag_heading.observation = measured_hdg;
		_aid_src_mag_heading.observation_variance = obs_var;
		_aid_src_mag_heading.innovation = innovation;
		_aid_src_mag_heading.fusion_enabled = _control_status.flags.mag_hdg;

		fuseYaw(_aid_src_mag_heading);
	}
}

void Ekf::run3DMagAndDeclFusions(const Vector3f &mag)
{
	// sanity check mag_B before they are used to constrain heading drift
	const Vector3f mag_bias_var = P.slice<3, 3>(19, 19).diag();
	const bool mag_bias_var_good = (mag_bias_var.min() > 0.f) && (mag_bias_var.max() < sq(0.02f));

	const bool update_all_states = _control_status.flags.mag_aligned_in_flight && mag_bias_var_good;

	if (!_mag_decl_cov_reset) {
		// After any magnetic field covariance reset event the earth field state
		// covariances need to be corrected to incorporate knowledge of the declination
		// before fusing magnetometer data to prevent rapid rotation of the earth field
		// states for the first few observations.
		fuseDeclination(0.02f);
		_mag_decl_cov_reset = true;
		fuseMag(mag, _aid_src_mag, update_all_states);

	} else {
		// The normal sequence is to fuse the magnetometer data first before fusing
		// declination angle at a higher uncertainty to allow some learning of
		// declination angle over time.
		fuseMag(mag, _aid_src_mag, update_all_states);

		if (_control_status.flags.mag_dec) {
			fuseDeclination(0.5f);
		}
	}
}

bool Ekf::resetMagHeading()
{
	// prevent a reset being performed more than once on the same frame
	if ((_flt_mag_align_start_time == _time_delayed_us) || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align)) {
		return false;
	}

	const Vector3f mag_init = _mag_lpf.getState();

	const bool mag_available = (_mag_counter > 1) && !magFieldStrengthDisturbed(mag_init);

	// low pass filtered mag required
	if (!mag_available) {
		return false;
	}

	const bool heading_required_for_navigation = _control_status.flags.gps;

	if ((_params.mag_fusion_type <= MagFuseType::MAG_3D) || ((_params.mag_fusion_type == MagFuseType::INDOOR) && heading_required_for_navigation)) {

		// rotate the magnetometer measurements into earth frame using a zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		// the angle of the projection onto the horizontal gives the yaw angle
		const Vector3f mag_earth_pred = R_to_earth * mag_init;

		// calculate the observed yaw angle and yaw variance
		float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();
		float yaw_new_variance = sq(fmaxf(_params.mag_heading_noise, 1.e-2f));

		ECL_INFO("reset mag heading %.3f -> %.3f rad (declination %.1f)", (double)getEulerYaw(_R_to_earth), (double)yaw_new, (double)getMagDeclination());

		// update quaternion states and corresponding covarainces
		resetQuatStateYaw(yaw_new, yaw_new_variance);

		// set the earth magnetic field states using the updated rotation
		_state.mag_I = _R_to_earth * mag_init;

		resetMagCov();

		// record the time for the magnetic field alignment event
		_flt_mag_align_start_time = _time_delayed_us;

		return true;
	}

	return false;
}

float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (_NED_origin_initialised || PX4_ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;

		} else {
			return math::radians(_params.mag_declination_deg);
		}

	} else {
		// always use the parameter value
		return math::radians(_params.mag_declination_deg);
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {
		ECL_INFO("stopping all mag fusion");
		stopMag3DFusion();
		stopMagHdgFusion();
		clearMagCov();
	}
}

void Ekf::stopMag3DFusion()
{
	// save covariance data for re-use if currently doing 3-axis fusion
	if (_control_status.flags.mag_3D) {
		saveMagCovData();

		_control_status.flags.mag_3D = false;
		_control_status.flags.mag_dec = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}

void Ekf::stopMagHdgFusion()
{
	if (_control_status.flags.mag_hdg) {
		_control_status.flags.mag_hdg = false;

		_fault_status.flags.bad_hdg = false;
	}
}

void Ekf::startMagHdgFusion()
{
	if (!_control_status.flags.mag_hdg) {
		stopMag3DFusion();
		ECL_INFO("starting mag heading fusion");
		_control_status.flags.mag_hdg = true;
	}
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {

		stopMagHdgFusion();

		zeroMagCov();
		loadMagCovData();
		_control_status.flags.mag_3D = true;
	}
}
