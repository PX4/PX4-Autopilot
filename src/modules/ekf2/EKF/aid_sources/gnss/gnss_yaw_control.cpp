/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file gnss_yaw_control.cpp
 * Definition of functions required to use yaw obtained from GNSS dual antenna measurements.
 * Equations generated using src/modules/ekf2/EKF/python/ekf_derivation/derivation.py
 *
 */

#include "ekf.h"

#include <mathlib/mathlib.h>
#include <cstdlib>

#include <ekf_derivation/generated/compute_gnss_yaw_pred_innov_var_and_h.h>

void Ekf::controlGnssYawFusion(const gnssSample &gnss_sample)
{
	if (!(_params.gnss_ctrl & static_cast<int32_t>(GnssCtrl::YAW))
	    || _control_status.flags.gnss_yaw_fault) {

		stopGnssYawFusion();
		return;
	}

	const bool is_new_data_available = PX4_ISFINITE(gnss_sample.yaw);

	if (is_new_data_available) {

		updateGnssYaw(gnss_sample);

		const bool continuing_conditions_passing = _control_status.flags.tilt_align;

		const bool is_gnss_yaw_data_intermittent = !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push,
				2 * GNSS_YAW_MAX_INTERVAL);

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _gps_checks_passed
				&& !is_gnss_yaw_data_intermittent
				&& !_gps_intermittent;

		if (_control_status.flags.gnss_yaw) {
			if (continuing_conditions_passing) {

				fuseGnssYaw(gnss_sample.yaw_offset);

				const bool is_fusion_failing = isTimedOut(_aid_src_gnss_yaw.time_last_fuse, _params.reset_timeout_max);

				if (is_fusion_failing) {
					stopGnssYawFusion();

					// Before takeoff, we do not want to continue to rely on the current heading
					// if we had to stop the fusion
					if (!_control_status.flags.in_air) {
						ECL_INFO("clearing yaw alignment");
						_control_status.flags.yaw_align = false;
					}
				}

			} else {
				// Stop GNSS yaw fusion but do not declare it faulty
				stopGnssYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate GNSS yaw fusion
				const bool not_using_ne_aiding = !_control_status.flags.gps && !_control_status.flags.aux_gpos;

				if (!_control_status.flags.in_air
				    || !_control_status.flags.yaw_align
				    || not_using_ne_aiding) {

					// Reset before starting the fusion
					if (resetYawToGnss(gnss_sample.yaw, gnss_sample.yaw_offset)) {

						resetAidSourceStatusZeroInnovation(_aid_src_gnss_yaw);

						_control_status.flags.gnss_yaw = true;
						_control_status.flags.yaw_align = true;
					}

				} else if (!_aid_src_gnss_yaw.innovation_rejected) {
					// Do not force a reset but wait for the consistency check to pass
					_control_status.flags.gnss_yaw = true;
					fuseGnssYaw(gnss_sample.yaw_offset);
				}

				if (_control_status.flags.gnss_yaw) {
					ECL_INFO("starting GNSS yaw fusion");
				}
			}
		}

	} else if (_control_status.flags.gnss_yaw
		   && !isNewestSampleRecent(_time_last_gnss_yaw_buffer_push, _params.reset_timeout_max)) {

		// No yaw data in the message anymore. Stop until it comes back.
		stopGnssYawFusion();
	}
}

void Ekf::updateGnssYaw(const gnssSample &gnss_sample)
{
	// calculate the observed yaw angle of antenna array, converting a from body to antenna yaw measurement
	const float measured_hdg = wrap_pi(gnss_sample.yaw + gnss_sample.yaw_offset);

	const float yaw_acc = PX4_ISFINITE(gnss_sample.yaw_acc) ? gnss_sample.yaw_acc : 0.f;
	const float R_YAW = sq(fmaxf(yaw_acc, _params.gnss_heading_noise));

	float heading_pred;
	float heading_innov_var;

	VectorState H;
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, gnss_sample.yaw_offset, R_YAW, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	updateAidSourceStatus(_aid_src_gnss_yaw,
			      gnss_sample.time_us,                          // sample timestamp
			      measured_hdg,                                // observation
			      R_YAW,                                       // observation variance
			      wrap_pi(heading_pred - measured_hdg),        // innovation
			      heading_innov_var,                           // innovation variance
			      math::max(_params.heading_innov_gate, 1.f)); // innovation gate
}

void Ekf::fuseGnssYaw(float antenna_yaw_offset)
{
	auto &aid_src = _aid_src_gnss_yaw;

	if (aid_src.innovation_rejected) {
		_innov_check_fail_status.flags.reject_yaw = true;
		return;
	}

	if (!PX4_ISFINITE(antenna_yaw_offset)) {
		antenna_yaw_offset = 0.f;
	}

	float heading_pred;
	float heading_innov_var;
	VectorState H;

	// Note: we recompute innov and innov_var because it doesn't cost much more than just computing H
	// making a separate function just for H uses more flash space without reducing CPU load significantly
	sym::ComputeGnssYawPredInnovVarAndH(_state.vector(), P, antenna_yaw_offset, aid_src.observation_variance, FLT_EPSILON,
					    &heading_pred, &heading_innov_var, &H);

	// check if the innovation variance calculation is badly conditioned
	if (aid_src.innovation_variance < aid_src.observation_variance) {
		// the innovation variance contribution from the state covariances is negative which means the covariance matrix is badly conditioned
		_fault_status.flags.bad_hdg = true;

		// we reinitialise the covariance matrix and abort this fusion step
		initialiseCovariance();
		ECL_ERR("GNSS yaw numerical error - covariance reset");
		stopGnssYawFusion();
		return;
	}

	_fault_status.flags.bad_hdg = false;
	_innov_check_fail_status.flags.reject_yaw = false;

	if ((fabsf(aid_src.test_ratio_filtered) > 0.2f)
	    && !_control_status.flags.in_air && isTimedOut(aid_src.time_last_fuse, (uint64_t)1e6)
	   ) {
		// A constant large signed test ratio is a sign of wrong gyro bias
		// Reset the yaw gyro variance to converge faster and avoid
		// being stuck on a previous bad estimate
		resetGyroBiasZCov();
	}

	// calculate the Kalman gains
	// only calculate gains for states we are using
	VectorState Kfusion = P * H / aid_src.innovation_variance;

	measurementUpdate(Kfusion, H, aid_src.observation_variance, aid_src.innovation);

	_fault_status.flags.bad_hdg = false;
	aid_src.fused = true;
	aid_src.time_last_fuse = _time_delayed_us;

	_time_last_heading_fuse = _time_delayed_us;
}

bool Ekf::resetYawToGnss(const float gnss_yaw, const float gnss_yaw_offset)
{
	// define the predicted antenna array vector and rotate into earth frame
	const Vector3f ant_vec_bf = {cosf(gnss_yaw_offset), sinf(gnss_yaw_offset), 0.0f};
	const Vector3f ant_vec_ef = _R_to_earth * ant_vec_bf;

	// check if antenna array vector is within 30 degrees of vertical and therefore unable to provide a reliable heading
	if (fabsf(ant_vec_ef(2)) > cosf(math::radians(30.0f)))  {
		return false;
	}

	// GNSS yaw measurement is already compensated for antenna offset in the driver
	const float measured_yaw = gnss_yaw;

	const float yaw_variance = sq(fmaxf(_params.gnss_heading_noise, 1.e-2f));
	resetQuatStateYaw(measured_yaw, yaw_variance);

	return true;
}

void Ekf::stopGnssYawFusion()
{
	if (_control_status.flags.gnss_yaw) {

		_control_status.flags.gnss_yaw = false;

		ECL_INFO("stopping GNSS yaw fusion");
	}
}
