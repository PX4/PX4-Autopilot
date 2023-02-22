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
	if (_params.mag_fusion_type == MagFuseType::NONE) {
		stopMag3DFusion();
		stopMagHdgFusion();
		stopMagFusion();
		return;
	}

	bool mag_data_ready = false;

	magSample mag_sample;

	if (_mag_buffer) {
		mag_data_ready = _mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample);

		if (mag_data_ready) {

			// sensor or calibration has changed, clear any mag bias and reset low pass filter
			if (mag_sample.reset || (_mag_counter == 0)) {
				// Zero the magnetometer bias states
				_state.mag_B.zero();

				_control_status.flags.mag_fault = false;

				stopMagFusion();

				// reset any saved covariance data for re-use when auto-switching between heading and 3-axis fusion
				resetMagCov();

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

		} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
			// No data anymore. Stop until it comes back.
			stopMag3DFusion();
			stopMagHdgFusion();
			stopMagFusion();
			return;
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

					_last_static_yaw = NAN;
				}
			}
		}

		return;
	}

	if (!_control_status.flags.tilt_align) {
		stopMag3DFusion();
		stopMagHdgFusion();
		stopMagFusion();
		return;
	}

	// check mag state observability
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _time_delayed_us;
	}

	if (mag_data_ready) {

		if (_control_status.flags.mag_fault
		    || _control_status.flags.ev_yaw
		    || _control_status.flags.gps_yaw) {

			stopMag3DFusion();
			stopMagHdgFusion();

		} else if (shouldInhibitMag()) {

			if (isTimedOut(_mag_use_not_inhibit_us, (uint32_t)5e6)) {
				// If magnetometer use has been inhibited continuously then stop the fusion
				stopMag3DFusion();
				stopMagHdgFusion();
			}

		} else {
			_mag_use_not_inhibit_us = _time_delayed_us;

			// Determine if we should use simple magnetic heading fusion which works better when
			// there are large external disturbances or the more accurate 3-axis fusion
			switch (_params.mag_fusion_type) {
			case MagFuseType::AUTO:
				if (_control_status.flags.mag && _control_status.flags.mag_aligned_in_flight) {
					startMag3DFusion();

				} else {
					startMagHdgFusion();
				}

				break;

			case MagFuseType::INDOOR:

			// FALLTHROUGH
			case MagFuseType::HEADING:
				startMagHdgFusion();
				break;

			case MagFuseType::MAG_3D:
				startMag3DFusion();
				break;

			default:
				stopMag3DFusion();
				stopMagHdgFusion();
				break;
			}
		}

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {

			if (_mag_yaw_reset_req || !_control_status.flags.yaw_align || mag_sample.reset || checkHaglYawResetReq()) {

				magReset();
			}
		}

		if (!_control_status.flags.yaw_align) {
			// Having the yaw aligned is mandatory to continue
			stopMagFusion();
			stopMag3DFusion();
			stopMagHdgFusion();
			return;
		}

		const bool mag_3d_forced_always = (_params.mag_fusion_type == MagFuseType::MAG_3D);

		if (_mag_bias_observable
		    || _yaw_angle_observable
		    || isRecent(_time_last_mov_3d_mag_suitable, (uint64_t)2e6)
		    || mag_3d_forced_always
		   ) {
			startMagFusion();

		} else {
			stopMagFusion();
		}

		checkMagDeclRequired();


		// mag
		resetEstimatorAidStatus(_aid_src_mag);
		_aid_src_mag.timestamp_sample = mag_sample.time_us;

		if (_control_status.flags.mag) {

			// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
			// before they are used to constrain heading drift
			const bool update_all_states = _control_status.flags.mag_3D
				&& ((_control_status.flags.mag_aligned_in_flight && isTimedOut(_flt_mag_align_start_time, (uint64_t)5e6)) || mag_3d_forced_always);

			if (!_mag_decl_cov_reset) {
				// After any magnetic field covariance reset event the earth field state
				// covariances need to be corrected to incorporate knowledge of the declination
				// before fusing magnetometer data to prevent rapid rotation of the earth field
				// states for the first few observations.

				// reset to default
				P.uncorrelateCovarianceSetVariance<3>(16, sq(_params.mag_noise));
				P.uncorrelateCovarianceSetVariance<3>(19, sq(_params.mag_noise));

				fuseDeclination(0.02f);
				_mag_decl_cov_reset = true;
				fuseMag(mag_sample.mag, _aid_src_mag, update_all_states);

			} else {
				// The normal sequence is to fuse the magnetometer data first before fusing
				// declination angle at a higher uncertainty to allow some learning of
				// declination angle over time.
				fuseMag(mag_sample.mag, _aid_src_mag, update_all_states);

				if (_control_status.flags.mag_dec) {
					fuseDeclination(0.5f);
				}
			}

		} else {
			// otherwise compute magnetometer innovations for estimator_aid_src_mag logging

			//  rotate magnetometer earth field state into body frame
			const Vector3f mag_I_body = _state.quat_nominal.rotateVectorInverse(_state.mag_I);
			const Vector3f mag_observation = mag_sample.mag - _state.mag_B;
			const Vector3f mag_innov = mag_I_body - mag_observation;

			mag_observation.copyTo(_aid_src_mag.observation);
			mag_innov.copyTo(_aid_src_mag.innovation);
		}


		// mag heading
		const float obs_var = fmaxf(sq(_params.mag_heading_noise), 1.e-4f);

		// Rotate the measurements into earth frame using the zero yaw angle
		Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

		Vector3f mag = mag_sample.mag;

		// use mag bias if variance good (unless in HEADING only configured)
		if (_params.mag_fusion_type != MagFuseType::HEADING) {
			const Vector3f mag_bias_var = getMagBiasVariance();

			if (mag_bias_var.longerThan(0.f) && !getMagBiasVariance().longerThan(0.001f)) {
				mag -= _state.mag_B;
			}
		}

		const Vector3f mag_earth_pred = R_to_earth * mag;

		// the angle of the projection onto the horizontal gives the yaw angle
		// calculate the yaw innovation and wrap to the interval between +-pi
		const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

		// update aid source status
		resetEstimatorAidStatus(_aid_src_mag_heading);
		_aid_src_mag_heading.fusion_enabled = _control_status.flags.mag_hdg;
		_aid_src_mag_heading.timestamp_sample = mag_sample.time_us;
		_aid_src_mag_heading.observation = measured_hdg;
		_aid_src_mag_heading.observation_variance = obs_var;

		if (_control_status.flags.mag_hdg) {
			const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
			_aid_src_mag_heading.innovation = wrap_pi(getEulerYaw(_R_to_earth) - _aid_src_mag_heading.observation);

			fuseYaw(innovation, obs_var, _aid_src_mag_heading);

		} else {
			// use delta yaw for innovation logging
			_aid_src_mag_heading.innovation = wrap_pi(wrap_pi(getEulerYaw(_R_to_earth) - _mag_heading_pred_prev)
							  - wrap_pi(measured_hdg - _mag_heading_prev));

			_aid_src_mag_heading.observation_variance = obs_var;
		}

		// record corresponding mag heading and yaw state for future mag heading delta heading innovation (logging only)
		_mag_heading_prev = measured_hdg;
		_mag_heading_pred_prev = getEulerYaw(_state.quat_nominal);
	}
}

bool Ekf::checkHaglYawResetReq()
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

	return false;
}

void Ekf::magReset()
{
	// prevent a reset being performed more than once on the same frame
	if ((_flt_mag_align_start_time == _time_delayed_us)
	    || (_control_status_prev.flags.yaw_align != _control_status.flags.yaw_align)) {
		return;
	}


	bool has_realigned_yaw = false;

	// use yaw estimator if available
	if (_control_status.flags.gps && isYawEmergencyEstimateAvailable() &&
	    (_mag_counter != 0) && isNewestSampleRecent(_time_last_mag_buffer_push, 500'000) // mag LPF available
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
		const Vector3f mag_init = _mag_lpf.getState();

		const bool mag_available = (_mag_counter > 1) && !magFieldStrengthDisturbed(mag_init);

		if (mag_available) {

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

			has_realigned_yaw = true;
		}
	}

	if (has_realigned_yaw) {
		_mag_yaw_reset_req = false;
		_control_status.flags.yaw_align = true;

		if (_control_status.flags.in_air) {
			_control_status.flags.mag_aligned_in_flight = true;

			// record the time for the magnetic field alignment event
			_flt_mag_align_start_time = _time_delayed_us;
		}
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

void Ekf::checkMagDeclRequired()
{
	// if we are using 3-axis magnetometer fusion, but without external NE aiding,
	// then the declination must be fused as an observation to prevent long term heading drift
	// fusing declination when gps aiding is available is optional, but recommended to prevent
	// problem if the vehicle is static for extended periods of time
	const bool user_selected = (_params.mag_declination_source & GeoDeclinationMask::FUSE_DECL);
	const bool not_using_ne_aiding = !_control_status.flags.gps;
	_control_status.flags.mag_dec = (_control_status.flags.mag && (not_using_ne_aiding || user_selected));
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
	    && ((_params.mag_fusion_type <= MagFuseType::MAG_3D)
		|| (_params.mag_fusion_type == MagFuseType::INDOOR && _control_status.flags.gps))
	   ) {

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

void Ekf::saveMagCovData()
{
	// save the NED axis covariance sub-matrix
	_saved_mag_ef_covmat = P.slice<3, 3>(16, 16);

	// save the XYZ body covariance sub-matrix
	_saved_mag_bf_covmat = P.slice<3, 3>(19, 19);
}

void Ekf::loadMagCovData()
{
	// re-instate the NE axis covariance sub-matrix
	P.uncorrelateCovarianceSetVariance<3>(16, 0.0f);
	P.slice<3, 3>(16, 16) = _saved_mag_ef_covmat;

	// re-instate the XYZ body axis covariance sub-matrix
	P.uncorrelateCovarianceSetVariance<3>(19, 0.0f);
	P.slice<3, 3>(19, 19) = _saved_mag_bf_covmat;
}

void Ekf::startMagHdgFusion()
{
	if (!_control_status.flags.mag_hdg) {
		stopMag3DFusion();
		ECL_INFO("starting mag heading fusion");
		_control_status.flags.mag_hdg = true;
	}
}

void Ekf::stopMagHdgFusion()
{
	if (_control_status.flags.mag_hdg) {
		_control_status.flags.mag_hdg = false;

		_fault_status.flags.bad_hdg = false;
	}
}

void Ekf::startMagFusion()
{
	if (!_control_status.flags.mag) {
		ECL_INFO("starting mag fusion");
		loadMagCovData();
		_control_status.flags.mag = true;
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag) {
		ECL_INFO("stopping mag fusion");

		stopMag3DFusion();

		_control_status.flags.mag = false;
		_control_status.flags.mag_dec = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;

		saveMagCovData();
	}
}

void Ekf::startMag3DFusion()
{
	if (!_control_status.flags.mag_3D) {
		ECL_INFO("starting mag 3D fusion");
		stopMagHdgFusion();
		_control_status.flags.mag_3D = true;
	}
}

void Ekf::stopMag3DFusion()
{
	// save covariance data for re-use if currently doing 3-axis fusion
	if (_control_status.flags.mag_3D) {
		ECL_INFO("stopping mag 3D fusion");
		_control_status.flags.mag_3D = false;
	}
}
