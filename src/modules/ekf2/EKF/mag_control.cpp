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
	// If we are on ground, reset the flight alignment flag so that the mag fields will be
	// re-initialised next time we achieve flight altitude
	if (!_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &mag_sample)) {
		// if enabled, use knowledge of theoretical magnetic field vector to calculate a synthetic magnetometer Z component value.
		// this is useful if there is a lot of interference on the sensor measurement.
		if (_params.synthesize_mag_z
		    && (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL)
		    && PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)
		   ) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
							* Vector3f(_mag_strength_gps, 0, 0);
			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);
			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		// sensor or calibration has changed, clear any mag bias and reset low pass filter
		if (mag_sample.reset) {
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

		_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_sample.mag - _state.mag_B);

		const bool mag_enabled_previously = _control_status.flags.mag_hdg || _control_status.flags.mag_3D;

		if (!_control_status.flags.ev_yaw && !_control_status.flags.gps_yaw
		    && !_control_status.flags.mag_fault && _control_status.flags.tilt_align) {
			// Determine if we should use simple magnetic heading fusion which works better when
			// there are large external disturbances or the more accurate 3-axis fusion
			switch (_params.mag_fusion_type) {
			default:

			// FALLTHROUGH
			case MagFuseType::AUTO:
				check3DMagFusionSuitability();

				if (canUse3DMagFusion()) {
					startMag3DFusion();

				} else {
					startMagHdgFusion();
				}

				break;

			case MagFuseType::INDOOR:

			/* fallthrough */
			case MagFuseType::HEADING:
				startMagHdgFusion();
				break;

			case MagFuseType::MAG_3D:
				startMag3DFusion();
				break;

			case MagFuseType::NONE:
				stopMagFusion();
				break;
			}

		} else {
			stopMagFusion();
		}

		if (_control_status.flags.mag_hdg || _control_status.flags.mag_3D) {

			const bool declination_changed = (fabsf(_mag_declination - getMagDeclination()) > math::radians(1.f));

			bool reset_required = !_control_status.flags.yaw_align || !mag_enabled_previously
					      || declination_changed || haglYawResetReq();

			if (reset_required && !shouldInhibitMag() && !_control_status.flags.mag_fault) {
				magYawReset(_mag_lpf.getState());
			}
		}

		// mag 3D
		if (_control_status.flags.yaw_align) {
			resetEstimatorAidStatus(_aid_src_mag);
			_aid_src_mag.timestamp_sample = mag_sample.time_us;
			_aid_src_mag.fusion_enabled = _control_status.flags.mag_3D;

			checkMagDeclRequired();

			if (!_mag_decl_cov_reset) {
				if (_control_status.flags.mag_3D) {
					// After any magnetic field covariance reset event the earth field state
					// covariances need to be corrected to incorporate knowledge of the declination
					// before fusing magnetomer data to prevent rapid rotation of the earth field
					// states for the first few observations.
					fuseDeclination(0.02f);
					_mag_decl_cov_reset = true;
				}

				fuseMag(mag_sample.mag, _aid_src_mag);

			} else {
				// For the first few seconds after in-flight alignment we allow the magnetic field state estimates to stabilise
				// before they are used to constrain heading drift
				const bool update_all_states = ((_imu_sample_delayed.time_us - _flt_mag_align_start_time) > (uint64_t)5e6)
							       && _control_status.flags.mag_aligned_in_flight
							       && _control_status.flags.yaw_align
							       && !_control_status.flags.mag_fault
							       && !_control_status.flags.mag_field_disturbed
							       && !shouldInhibitMag();

				// The normal sequence is to fuse the magnetometer data first before fusing
				// declination angle at a higher uncertainty to allow some learning of
				// declination angle over time.
				fuseMag(mag_sample.mag, _aid_src_mag, update_all_states);

				if (_control_status.flags.mag_dec) {
					fuseDeclination(0.5f);
				}
			}
		}

		// mag heading
		const Vector3f mag_observation = mag_sample.mag - _state.mag_B;

		_control_status.flags.mag_field_disturbed = magFieldStrengthDisturbed(mag_observation);

		// Rotate the measurements into earth frame using the zero yaw angle
		const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);
		const Vector3f mag_earth_pred = R_to_earth * mag_observation;

		// the angle of the projection onto the horizontal gives the yaw angle
		// calculate the yaw innovation and wrap to the interval between +-pi
		_mag_declination = getMagDeclination();
		const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + _mag_declination;

		resetEstimatorAidStatus(_aid_src_mag_heading);
		_aid_src_mag_heading.timestamp_sample = mag_sample.time_us;
		_aid_src_mag_heading.observation = measured_hdg;
		_aid_src_mag_heading.observation_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));
		_aid_src_mag_heading.innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		_aid_src_mag_heading.fusion_enabled = _control_status.flags.mag_hdg
						      && _control_status.flags.yaw_align
						      && !_control_status.flags.mag_fault
						      && !_control_status.flags.mag_field_disturbed
						      && !shouldInhibitMag();

		fuseYaw(_aid_src_mag_heading);

	} else if ((_control_status.flags.mag_hdg || _control_status.flags.mag_3D)
		   && !isNewestSampleRecent(_time_last_mag_buffer_push, (int32_t)5e6)) {

		stopMagFusion();
	}
}

bool Ekf::haglYawResetReq() const
{
	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

	return false;
}

bool Ekf::magYawReset(const Vector3f &mag)
{
	bool has_realigned_yaw = false;

	// use yaw estimator if available
	if (_control_status.flags.gps && isYawEmergencyEstimateAvailable()) {

		resetQuatStateYaw(_yawEstimator.getYaw(), _yawEstimator.getYawVar());

		_information_events.flags.yaw_aligned_to_imu_gps = true;

		// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
		if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {
			// use predicted earth field to reset states
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps,
							     _mag_declination_gps)) * Vector3f(_mag_strength_gps, 0, 0);
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

	} else if (!magFieldStrengthDisturbed(mag - _state.mag_B)) {
		has_realigned_yaw = resetMagHeading(mag - _state.mag_B);
	}

	if (has_realigned_yaw) {
		_control_status.flags.yaw_align = true;

		if (_control_status.flags.in_air) {
			_control_status.flags.mag_aligned_in_flight = true;

			// record the time for the magnetic field alignment event
			_flt_mag_align_start_time = _imu_sample_delayed.time_us;
		}

		// Handle the special case where we have not been constraining yaw drift or learning yaw bias due
		// to assumed invalid mag field associated with indoor operation with a downwards looking flow sensor.
		static constexpr uint64_t YAW_AIDING_TIMEOUT_US = (uint64_t)5e6;

		if (!_yaw_angle_observable
		    && isTimedOut(_aid_src_mag_heading.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_mag.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_gnss_yaw.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		    && isTimedOut(_aid_src_ev_yaw.time_last_fuse, YAW_AIDING_TIMEOUT_US)
		   ) {
			// Zero the yaw bias covariance and set the variance to the initial alignment uncertainty
			resetZDeltaAngBiasCov();
		}

		_aid_src_mag.time_last_fuse = _imu_sample_delayed.time_us;
		_aid_src_mag_heading.time_last_fuse = _imu_sample_delayed.time_us;

		return true;
	}

	return false;
}

void Ekf::check3DMagFusionSuitability()
{
	checkYawAngleObservability();
	checkMagBiasObservability();

	if (_mag_bias_observable || _yaw_angle_observable) {
		_time_last_mov_3d_mag_suitable = _imu_sample_delayed.time_us;
	}
}

void Ekf::checkYawAngleObservability()
{
	// Check if there has been enough change in horizontal velocity to make yaw observable
	// Apply hysteresis to check to avoid rapid toggling
	_yaw_angle_observable = _yaw_angle_observable
				? _accel_lpf_NE.norm() > _params.mag_acc_gate
				: _accel_lpf_NE.norm() > 2.0f * _params.mag_acc_gate;

	_yaw_angle_observable = _yaw_angle_observable
				&& (_control_status.flags.gps || _control_status.flags.ev_pos); // Do we have to add ev_vel here?
}

void Ekf::checkMagBiasObservability()
{
	// check if there is enough yaw rotation to make the mag bias states observable
	if (!_mag_bias_observable && (fabsf(_yaw_rate_lpf_ef) > _params.mag_yaw_rate_gate)) {
		// initial yaw motion is detected
		_mag_bias_observable = true;

	} else if (_mag_bias_observable) {
		// require sustained yaw motion of 50% the initial yaw rate threshold
		const float yaw_dt = 1e-6f * (float)(_imu_sample_delayed.time_us - _time_yaw_started);
		const float min_yaw_change_req =  0.5f * _params.mag_yaw_rate_gate * yaw_dt;
		_mag_bias_observable = fabsf(_yaw_delta_ef) > min_yaw_change_req;
	}

	_yaw_delta_ef = 0.0f;
	_time_yaw_started = _imu_sample_delayed.time_us;
}

bool Ekf::canUse3DMagFusion() const
{
	// Use of 3D fusion requires an in-air heading alignment but it should not
	// be used when the heading and mag biases are not observable for more than 2 seconds
	return _control_status.flags.mag_aligned_in_flight
	       && ((_imu_sample_delayed.time_us - _time_last_mov_3d_mag_suitable) < (uint64_t)2e6);
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

	const bool heading_not_required_for_navigation = !_control_status.flags.gps
			&& !_control_status.flags.ev_pos
			&& !_control_status.flags.ev_vel;

	return (user_selected && heading_not_required_for_navigation);
}

bool Ekf::magFieldStrengthDisturbed(const Vector3f &mag_sample) const
{
	if (_params.check_mag_strength) {

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
