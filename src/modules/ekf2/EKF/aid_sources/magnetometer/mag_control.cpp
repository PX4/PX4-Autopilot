/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * @file mag_control.cpp
 * Control functions for ekf magnetic field fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

#include <ekf_derivation/generated/compute_mag_innov_innov_var_and_hx.h>

void Ekf::controlMagFusion()
{
	static constexpr const char *AID_SRC_NAME = "mag";
	estimator_aid_source3d_s &aid_src = _aid_src_mag;

	// reset the flight alignment flag so that the mag fields will be
	//  re-initialised next time we achieve flight altitude
	if (!_control_status_prev.flags.in_air && _control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = false;
	}

	if (_params.mag_fusion_type == MagFuseType::NONE) {
		stopMagFusion();
		return;
	}

	magSample mag_sample;

	if (_mag_buffer && _mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample)) {

		if (mag_sample.reset || (_mag_counter == 0)) {
			// sensor or calibration has changed, reset low pass filter
			_control_status.flags.mag_fault = false;

			_state.mag_B.zero();
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
		    && (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps))
		   ) {
			const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
							* Vector3f(_mag_strength_gps, 0, 0);

			mag_sample.mag(2) = calculate_synthetic_mag_z_measurement(mag_sample.mag, mag_earth_pred);

			_control_status.flags.synthetic_mag_z = true;

		} else {
			_control_status.flags.synthetic_mag_z = false;
		}

		// reset flags
		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;


		resetEstimatorAidStatus(aid_src);
		aid_src.timestamp_sample = mag_sample.time_us;

		// XYZ Measurement uncertainty. Need to consider timing errors for fast rotations
		const float R_MAG = math::max(sq(_params.mag_noise), sq(0.01f));

		// calculate intermediate variables used for X axis innovation variance, observation Jacobians and Kalman gains
		Vector3f mag_innov;
		Vector3f innov_var;

		// Observation jacobian and Kalman gain vectors
		VectorState H;
		sym::ComputeMagInnovInnovVarAndHx(_state.vector(), P, mag_sample.mag, R_MAG, FLT_EPSILON, &mag_innov, &innov_var, &H);

		for (int i = 0; i < 3; i++) {
			aid_src.observation[i] = mag_sample.mag(i);
			aid_src.observation_variance[i] = R_MAG;
			aid_src.innovation[i] = mag_innov(i);
			aid_src.innovation_variance[i] = innov_var(i);
		}

		const float innov_gate = math::max(_params.mag_innov_gate, 1.f);
		setEstimatorAidStatusTestRatio(aid_src, innov_gate);

		// Perform an innovation consistency check and report the result
		_innov_check_fail_status.flags.reject_mag_x = (aid_src.test_ratio[0] > 1.f);
		_innov_check_fail_status.flags.reject_mag_y = (aid_src.test_ratio[1] > 1.f);
		_innov_check_fail_status.flags.reject_mag_z = (aid_src.test_ratio[2] > 1.f);

		// determine if we should use mag fusion
		bool continuing_conditions_passing = ((_params.mag_fusion_type == MagFuseType::INIT)
						      || (_params.mag_fusion_type == MagFuseType::AUTO)
						      || (_params.mag_fusion_type == MagFuseType::HEADING))
						     && _control_status.flags.tilt_align
						     && (_control_status.flags.yaw_align || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
						     && mag_sample.mag.longerThan(0.f)
						     && mag_sample.mag.isAllFinite();

		const bool starting_conditions_passing = continuing_conditions_passing
				&& checkMagField(mag_sample.mag)
				&& (_mag_counter > 3) // wait until we have more than a few samples through the filter
				&& (_control_status.flags.yaw_align == _control_status_prev.flags.yaw_align) // no yaw alignment change this frame
				&& (_state_reset_status.reset_count.quat == _state_reset_count_prev.quat) // don't allow starting on same frame as yaw reset
				&& isNewestSampleRecent(_time_last_mag_buffer_push, MAG_MAX_INTERVAL);

		checkMagHeadingConsistency(mag_sample);

		// WMM update can occur on the last epoch, just after mag fusion
		const bool wmm_updated = (_wmm_gps_time_last_set >= aid_src.time_last_fuse);
		const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;


		{
		const bool mag_consistent_or_no_ne_aiding = _control_status.flags.mag_heading_consistent || !using_ne_aiding;
		const bool common_conditions_passing = _control_status.flags.mag
						       && ((_control_status.flags.yaw_align && mag_consistent_or_no_ne_aiding)
						           || (!_control_status.flags.ev_yaw && !_control_status.flags.yaw_align))
						       && !_control_status.flags.mag_fault
						       && !_control_status.flags.mag_field_disturbed
						       && !_control_status.flags.ev_yaw
						       && !_control_status.flags.gps_yaw;

		_control_status.flags.mag_3D = common_conditions_passing
					       && (_params.mag_fusion_type == MagFuseType::AUTO)
					       && _control_status.flags.mag_aligned_in_flight;

		_control_status.flags.mag_hdg = common_conditions_passing
						&& ((_params.mag_fusion_type == MagFuseType::HEADING)
						    || (_params.mag_fusion_type == MagFuseType::AUTO && !_control_status.flags.mag_3D));
		}

		// TODO: allow clearing mag_fault if mag_3d is good?

		if (_control_status.flags.mag_3D && !_control_status_prev.flags.mag_3D) {
			ECL_INFO("starting mag 3D fusion");

		} else if (!_control_status.flags.mag_3D && _control_status_prev.flags.mag_3D) {
			ECL_INFO("stopping mag 3D fusion");
		}

		// if we are using 3-axis magnetometer fusion, but without external NE aiding,
		// then the declination must be fused as an observation to prevent long term heading drift
		const bool no_ne_aiding_or_pre_takeoff = !using_ne_aiding || !_control_status.flags.in_air;
		_control_status.flags.mag_dec = _control_status.flags.mag && no_ne_aiding_or_pre_takeoff;

		if (_control_status.flags.mag) {

			if (continuing_conditions_passing && _control_status.flags.yaw_align) {

				if (mag_sample.reset || checkHaglYawResetReq() || (wmm_updated && no_ne_aiding_or_pre_takeoff)) {
					ECL_INFO("reset to %s", AID_SRC_NAME);
					resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
					aid_src.time_last_fuse = _time_delayed_us;

				} else {
					// The normal sequence is to fuse the magnetometer data first before fusing
					// declination angle at a higher uncertainty to allow some learning of
					// declination angle over time.
					const bool update_all_states = _control_status.flags.mag_3D || _control_status.flags.mag_hdg;
					const bool update_tilt = _control_status.flags.mag_3D;
					fuseMag(mag_sample.mag, R_MAG, H, aid_src, update_all_states, update_tilt);

					// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
					if (update_all_states && update_tilt) {
						_fault_status.flags.bad_mag_x = (aid_src.innovation_variance[0] < aid_src.observation_variance[0]);
						_fault_status.flags.bad_mag_y = (aid_src.innovation_variance[1] < aid_src.observation_variance[1]);
						_fault_status.flags.bad_mag_z = (aid_src.innovation_variance[2] < aid_src.observation_variance[2]);
					}

					if (_control_status.flags.mag_dec) {
						fuseDeclination(0.5f);
					}
				}

				const bool is_fusion_failing = isTimedOut(aid_src.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					if (no_ne_aiding_or_pre_takeoff) {
						ECL_WARN("%s fusion failing, resetting", AID_SRC_NAME);
						resetMagStates(_mag_lpf.getState(), _control_status.flags.mag_hdg || _control_status.flags.mag_3D);
						aid_src.time_last_fuse = _time_delayed_us;

					} else {
						ECL_WARN("stopping %s, fusion failing", AID_SRC_NAME);
						stopMagFusion();
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				ECL_DEBUG("stopping %s fusion, continuing conditions no longer passing", AID_SRC_NAME);
				stopMagFusion();
			}

		} else {
			if (starting_conditions_passing) {

				// activate fusion, reset mag states and initialize variance if first init or in flight reset
				if (!_control_status.flags.yaw_align
				    || wmm_updated
				    || !_state.mag_I.longerThan(0.f)
				    || (getStateVariance<State::mag_I>().min() < kMagVarianceMin)
				    || (getStateVariance<State::mag_B>().min() < kMagVarianceMin)
				   ) {
					ECL_INFO("starting %s fusion, resetting states", AID_SRC_NAME);

					bool reset_heading = !_control_status.flags.yaw_align;

					resetMagStates(_mag_lpf.getState(), reset_heading);
					aid_src.time_last_fuse = _time_delayed_us;

					if (reset_heading) {
						_control_status.flags.yaw_align = true;
					}

					_control_status.flags.mag = true;

				} else {
					if (fuseMag(mag_sample.mag, R_MAG, H, aid_src)) {
						ECL_INFO("starting %s fusion", AID_SRC_NAME);
						_control_status.flags.mag = true;
					}
				}
			}
		}

	} else if (!isNewestSampleRecent(_time_last_mag_buffer_push, 2 * MAG_MAX_INTERVAL)) {
		// No data anymore. Stop until it comes back.
		stopMagFusion();
	}
}

void Ekf::stopMagFusion()
{
	if (_control_status.flags.mag) {
		ECL_INFO("stopping mag fusion");

		if (_control_status.flags.yaw_align && (_control_status.flags.mag_3D || _control_status.flags.mag_hdg)) {
			// reset yaw alignment from mag unless using GNSS aiding
			const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;

			if (!using_ne_aiding) {
				_control_status.flags.yaw_align = false;
			}
		}

		_control_status.flags.mag = false;
		_control_status.flags.mag_dec = false;

		if (_control_status.flags.mag_3D) {
			ECL_INFO("stopping mag 3D fusion");
			_control_status.flags.mag_3D = false;
		}

		if (_control_status.flags.mag_hdg) {
			ECL_INFO("stopping mag heading fusion");
			_control_status.flags.mag_hdg = false;
			_fault_status.flags.bad_hdg = false;
		}

		_control_status.flags.mag_aligned_in_flight = false;

		_fault_status.flags.bad_mag_x = false;
		_fault_status.flags.bad_mag_y = false;
		_fault_status.flags.bad_mag_z = false;

		_fault_status.flags.bad_mag_decl = false;
	}
}

bool Ekf::checkHaglYawResetReq() const
{
#if defined(CONFIG_EKF2_TERRAIN)

	// We need to reset the yaw angle after climbing away from the ground to enable
	// recovery from ground level magnetic interference.
	if (_control_status.flags.in_air && _control_status.flags.yaw_align && !_control_status.flags.mag_aligned_in_flight) {
		// Check if height has increased sufficiently to be away from ground magnetic anomalies
		// and request a yaw reset if not already requested.
		static constexpr float mag_anomalies_max_hagl = 1.5f;
		const bool above_mag_anomalies = (getTerrainVPos() - _state.pos(2)) > mag_anomalies_max_hagl;
		return above_mag_anomalies;
	}

#endif // CONFIG_EKF2_TERRAIN

	return false;
}

void Ekf::resetMagStates(const Vector3f &mag, bool reset_heading)
{
	// reinit mag states
	const Vector3f mag_I_before_reset = _state.mag_I;
	const Vector3f mag_B_before_reset = _state.mag_B;

	// reset covariances to default
	resetMagCov();

	// if world magnetic model (inclination, declination, strength) available then use it to reset mag states
	if (PX4_ISFINITE(_mag_inclination_gps) && PX4_ISFINITE(_mag_declination_gps) && PX4_ISFINITE(_mag_strength_gps)) {

		// use expected earth field to reset states
		const Vector3f mag_earth_pred = Dcmf(Eulerf(0, -_mag_inclination_gps, _mag_declination_gps))
						* Vector3f(_mag_strength_gps, 0, 0);

		// mag_B: reset
		if (!reset_heading && _control_status.flags.yaw_align) {
			// mag_B: reset using WMM
			const Dcmf R_to_body = quatToInverseRotMat(_state.quat_nominal);
			_state.mag_B = mag - (R_to_body * mag_earth_pred);

		} else {
			_state.mag_B.zero();
		}

		// mag_I: reset, skipped if no change in state and variance good
		_state.mag_I = mag_earth_pred;

		if (reset_heading) {
			resetMagHeading(mag);
		}

	} else {
		// mag_B: reset
		_state.mag_B.zero();

		// Use the magnetometer measurement to reset the field states
		if (reset_heading) {
			resetMagHeading(mag);
		}

		// mag_I: use the last magnetometer measurements to reset the field states
		_state.mag_I = _R_to_earth * mag;
	}

	if (!mag_I_before_reset.longerThan(0.f)) {
		ECL_INFO("initializing mag I [%.3f, %.3f, %.3f], mag B [%.3f, %.3f, %.3f]",
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2),
			 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
			);

	} else {
		ECL_INFO("resetting mag I [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
			 (double)mag_I_before_reset(0), (double)mag_I_before_reset(1), (double)mag_I_before_reset(2),
			 (double)_state.mag_I(0), (double)_state.mag_I(1), (double)_state.mag_I(2)
			);

		if (mag_B_before_reset.longerThan(0.f) || _state.mag_B.longerThan(0.f)) {
			ECL_INFO("resetting mag B [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]",
				 (double)mag_B_before_reset(0), (double)mag_B_before_reset(1), (double)mag_B_before_reset(2),
				 (double)_state.mag_B(0), (double)_state.mag_B(1), (double)_state.mag_B(2)
				);
		}
	}

	// record the start time for the magnetic field alignment
	if (_control_status.flags.in_air) {
		_control_status.flags.mag_aligned_in_flight = true;
		_flt_mag_align_start_time = _time_delayed_us;
	}
}

void Ekf::checkMagHeadingConsistency(const magSample &mag_sample)
{
	// use mag bias if variance good
	Vector3f mag_bias{0.f, 0.f, 0.f};
	const Vector3f mag_bias_var = getMagBiasVariance();

	if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
		mag_bias = _state.mag_B;
	}

	// calculate mag heading
	// Rotate the measurements into earth frame using the zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	// calculate the yaw innovation and wrap to the interval between +-pi
	const Vector3f mag_earth_pred = R_to_earth * (mag_sample.mag - mag_bias);
	const float declination = getMagDeclination();
	const float measured_hdg = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;

	if (_control_status.flags.yaw_align) {
		const float innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		_mag_heading_innov_lpf.update(innovation);

	} else {
		_mag_heading_innov_lpf.reset(0.f);
	}

	if (fabsf(_mag_heading_innov_lpf.getState()) < _params.mag_heading_noise) {
		// Check if there has been enough change in horizontal velocity to make yaw observable
		const bool using_ne_aiding = _control_status.flags.gps || _control_status.flags.aux_gpos;

		if (using_ne_aiding && (_accel_lpf_NE.norm() > _params.mag_acc_gate)) {
			// yaw angle must be observable to consider consistency
			_control_status.flags.mag_heading_consistent = true;
		}

	} else {
		_control_status.flags.mag_heading_consistent = false;
	}
}

bool Ekf::checkMagField(const Vector3f &mag_sample)
{
	_control_status.flags.mag_field_disturbed = false;

	if (_params.mag_check == 0) {
		// skip all checks
		return true;
	}

	bool is_check_failing = false;
	_mag_strength = mag_sample.length();

	if (_params.mag_check & static_cast<int32_t>(MagCheckMask::STRENGTH)) {
		if (PX4_ISFINITE(_mag_strength_gps)) {
			if (!isMeasuredMatchingExpected(_mag_strength, _mag_strength_gps, _params.mag_check_strength_tolerance_gs)) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}

		} else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
			is_check_failing = true;

		} else {
			constexpr float average_earth_mag_field_strength = 0.45f; // Gauss
			constexpr float average_earth_mag_gate_size = 0.40f; // +/- Gauss

			if (!isMeasuredMatchingExpected(mag_sample.length(), average_earth_mag_field_strength, average_earth_mag_gate_size)) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}
		}
	}

	const Vector3f mag_earth = _R_to_earth * mag_sample;
	_mag_inclination = asinf(mag_earth(2) / fmaxf(mag_earth.norm(), 1e-4f));

	if (_params.mag_check & static_cast<int32_t>(MagCheckMask::INCLINATION)) {
		if (PX4_ISFINITE(_mag_inclination_gps)) {
			const float inc_tol_rad = radians(_params.mag_check_inclination_tolerance_deg);
			const float inc_error_rad = wrap_pi(_mag_inclination - _mag_inclination_gps);

			if (fabsf(inc_error_rad) > inc_tol_rad) {
				_control_status.flags.mag_field_disturbed = true;
				is_check_failing = true;
			}

		} else if (_params.mag_check & static_cast<int32_t>(MagCheckMask::FORCE_WMM)) {
			is_check_failing = true;

		} else {
			// No check possible when the global position is unknown
			// TODO: add parameter to remember the inclination between boots
		}
	}

	if (is_check_failing || (_time_last_mag_check_failing == 0)) {
		_time_last_mag_check_failing = _time_delayed_us;
	}

	return ((_time_delayed_us - _time_last_mag_check_failing) > (uint64_t)_min_mag_health_time_us);
}

bool Ekf::isMeasuredMatchingExpected(const float measured, const float expected, const float gate)
{
	return (measured >= expected - gate)
	       && (measured <= expected + gate);
}

void Ekf::resetMagHeading(const Vector3f &mag)
{
	// use mag bias if variance good (unless configured for HEADING only)
	Vector3f mag_bias{0.f, 0.f, 0.f};
	const Vector3f mag_bias_var = getMagBiasVariance();

	if ((mag_bias_var.min() > 0.f) && (mag_bias_var.max() <= sq(_params.mag_noise))) {
		mag_bias = _state.mag_B;
	}

	// calculate mag heading
	// rotate the magnetometer measurements into earth frame using a zero yaw angle
	const Dcmf R_to_earth = updateYawInRotMat(0.f, _R_to_earth);

	// the angle of the projection onto the horizontal gives the yaw angle
	const Vector3f mag_earth_pred = R_to_earth * (mag - mag_bias);

	// calculate the observed yaw angle and yaw variance
	const float declination = getMagDeclination();
	float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + declination;
	float yaw_new_variance = math::max(sq(_params.mag_heading_noise), sq(0.01f));

	ECL_INFO("reset mag heading %.3f -> %.3f rad (bias:[%.3f, %.3f, %.3f], declination:%.1f)",
		 (double)getEulerYaw(_R_to_earth), (double)yaw_new,
		 (double)mag_bias(0), (double)mag_bias(1), (double)mag_bias(2),
		 (double)declination);

	// update quaternion states and corresponding covarainces
	resetQuatStateYaw(yaw_new, yaw_new_variance);

	_time_last_heading_fuse = _time_delayed_us;

	_mag_heading_innov_lpf.reset(0.f);
	_control_status.flags.mag_heading_consistent = true;
}

float Ekf::getMagDeclination()
{
	// set source of magnetic declination for internal use
	if (_control_status.flags.mag_aligned_in_flight) {
		// Use value consistent with earth field state
		return atan2f(_state.mag_I(1), _state.mag_I(0));

	} else if (_params.mag_declination_source & GeoDeclinationMask::USE_GEO_DECL) {
		// use parameter value until GPS is available, then use value returned by geo library
		if (PX4_ISFINITE(_mag_declination_gps)) {
			return _mag_declination_gps;
		}
	}

	// otherwise use the parameter value
	return math::radians(_params.mag_declination_deg);
}

