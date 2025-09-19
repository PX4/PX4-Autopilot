/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file VTEOrientation.cpp
 * @brief Estimate the orientation of a target by processing and fusing sensor data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "VTEOrientation.h"

#define SEC2USEC_F 1e6f

namespace vision_target_estimator
{

using namespace matrix;

VTEOrientation::VTEOrientation() :
	ModuleParams(nullptr)
{
	_vte_aid_ev_yaw_pub.advertise();
	_targetOrientationPub.advertise();

	updateParams();
}

VTEOrientation::~VTEOrientation() = default;

bool VTEOrientation::init()
{
	_target_est_yaw = KF_orientation{};
	return true;
}

void VTEOrientation::resetFilter()
{
	_estimator_initialized = false;
}

void VTEOrientation::update(const matrix::Quaternionf &q_att)
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	checkMeasurementInputs();

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {
		predictionStep();
		_last_predict = hrt_absolute_time();
	}

	// update and fuse the observations and pulishes innovations
	if (updateStep(q_att)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool VTEOrientation::initEstimator(const ObsValidMaskU &vte_fusion_aid_mask,
				   const TargetObs observations[ObsType::Type_count])
{
	// Until a sensor measures the yaw rate, state_init(State::yaw_rate) is init at zero
	matrix::Vector<float, State::size> state_init{};

	// Yaw init
	if (vte_fusion_aid_mask.flags.fuse_vision) {
		state_init(State::yaw) = observations[ObsType::Fiducial_marker].meas;

	} else if (vte_fusion_aid_mask.flags.fuse_uwb) {
		state_init(State::yaw) = observations[ObsType::Uwb].meas;
	}

	matrix::Vector<float, State::size> state_var_init;

	for (int i = 0; i < State::size; i++) {
		state_var_init(i) = _yaw_unc;
	}

	_target_est_yaw.set_state(state_init);
	_target_est_yaw.set_state_covariance(state_var_init);

	PX4_INFO("Orientation filter init yaw: %.2f [rad] yaw_rate: %.2f [rad/s]", (double)state_init(State::yaw),
		 (double)state_init(State::yaw_rate));

	PX4_INFO("VTE Orientation Estimator properly initialized.");
	_estimator_initialized = true;
	_last_update = hrt_absolute_time();
	_last_predict = _last_update;

	return true;
}

void VTEOrientation::predictionStep()
{
	// Time from last prediciton
	float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC_F;

	_target_est_yaw.predictState(dt);
	_target_est_yaw.predictCov(dt);
}

bool VTEOrientation::updateStep(const matrix::Quaternionf &q_att)
{
	TargetObs obs_fiducial_marker_orientation;

	ObsValidMaskU vte_fusion_aid_mask{};
	TargetObs observations[ObsType::Type_count];
	processObservations(q_att, vte_fusion_aid_mask, observations);

	// No new observations --> no fusion.
	if (vte_fusion_aid_mask.value == 0) {
		return false;
	}

	// Initialize estimator if not already initialized
	if (!_estimator_initialized && !initEstimator(vte_fusion_aid_mask, observations)) {
		resetFilter();
		return false;
	}

	// Fuse new sensor data
	return fuseNewSensorData(vte_fusion_aid_mask, observations);
}

void VTEOrientation::processObservations(const matrix::Quaternionf &q_att, ObsValidMaskU &vte_fusion_aid_mask,
		TargetObs obs[ObsType::Type_count])
{
	handleVisionData(vte_fusion_aid_mask, obs[ObsType::Fiducial_marker]);

	handleUwbData(q_att, vte_fusion_aid_mask, obs[ObsType::Uwb]);
}

void VTEOrientation::handleVisionData(ObsValidMaskU &vte_fusion_aid_mask, TargetObs &obs_fiducial_marker)
{

	if (!_vte_aid_mask.flags.use_vision_pos) {
		return;
	}

	fiducial_marker_yaw_report_s fiducial_marker_yaw;

	if (!_fiducial_marker_yaw_report_sub.update(&fiducial_marker_yaw)) {
		return;
	}

	if (!isVisionDataValid(fiducial_marker_yaw)) {
		return;
	}

	if (processObsVision(fiducial_marker_yaw, obs_fiducial_marker)) {
		vte_fusion_aid_mask.flags.fuse_vision = true;
	}
}

bool VTEOrientation::isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw)
{
	if (!isMeasValid(fiducial_marker_yaw.timestamp)) {
		PX4_DEBUG("Vision yaw is outdated!");
		return false;
	}

	if (!PX4_ISFINITE(fiducial_marker_yaw.yaw_ned)) {
		PX4_DEBUG("Vision yaw is corrupt!");
		return false;
	}

	// TODO: extend checks
	return true;
}

bool VTEOrientation::processObsVision(const fiducial_marker_yaw_report_s &fiducial_marker_yaw, TargetObs &obs)
{
	float yaw_unc;

	if (_ev_noise_md) {
		yaw_unc = _range_sensor.valid ? (_ev_angle_noise * _ev_angle_noise * fmaxf(_range_sensor.dist_bottom,
						 1.f)) : (_ev_angle_noise * _ev_angle_noise * 10);

	} else {
		yaw_unc = fmaxf(fiducial_marker_yaw.yaw_var_ned, _ev_angle_noise * _ev_angle_noise);
	}

	obs.timestamp = fiducial_marker_yaw.timestamp;
	obs.updated = true;

	obs.meas = wrap_pi(fiducial_marker_yaw.yaw_ned);
	obs.meas_unc = yaw_unc;
	obs.meas_h_theta(State::yaw) = 1;

	obs.updated = true;
	obs.type = ObsType::Fiducial_marker;

	return true;
}

void VTEOrientation::handleUwbData(const matrix::Quaternionf &q_att, ObsValidMaskU &vte_fusion_aid_mask,
				   TargetObs &obs_uwb)
{
	sensor_uwb_s uwb_report;

	if (!_vte_aid_mask.flags.use_uwb) {
		return;
	}

	if (!_sensor_uwb_sub.update(&uwb_report)) {
		return;
	}

	if (!isUwbDataValid(uwb_report)) {
		return;
	}

	if (processObsUwb(q_att, uwb_report, obs_uwb)) {
		vte_fusion_aid_mask.flags.fuse_uwb = true;
	}
}

// TODO: check
bool VTEOrientation::isUwbDataValid(const sensor_uwb_s &uwb_report)
{
	if (!isMeasValid(uwb_report.timestamp)) {
		return false;
	}

	if (!PX4_ISFINITE(uwb_report.distance) ||
	    fabsf(uwb_report.aoa_azimuth_dev) > max_uwb_aoa_angle_degree ||
	    fabsf(uwb_report.aoa_elevation_dev) > max_uwb_aoa_angle_degree ||
	    fabsf(uwb_report.aoa_azimuth_resp) > max_uwb_aoa_angle_degree) {
		return false;
	}

	return true;
}

bool VTEOrientation::processObsUwb(const matrix::Quaternionf &q_att, const sensor_uwb_s &uwb_report, TargetObs &obs)
{
	matrix::Vector3f rel_pos_ned{};

	if (!uwbMeasurementToNed(uwb_report, q_att, rel_pos_ned)) {
		return false;
	}

	const float horizontal_sq = rel_pos_ned(0) * rel_pos_ned(0) + rel_pos_ned(1) * rel_pos_ned(1);

	if (!PX4_ISFINITE(horizontal_sq) || horizontal_sq < 1e-4f) {
		return false;
	}

	const float bearing_drone_to_target = atan2f(rel_pos_ned(1), rel_pos_ned(0));
	const float bearing_target_to_drone = wrap_pi(bearing_drone_to_target + M_PI_F);
	const float aoa_resp_rad = math::radians(uwb_report.aoa_azimuth_resp);

	if (!PX4_ISFINITE(aoa_resp_rad)) {
		return false;
	}

	obs.timestamp = uwb_report.timestamp;
	obs.meas = wrap_pi(bearing_target_to_drone - aoa_resp_rad);
	obs.meas_unc = fmaxf(math::sq(math::radians(3.f)), _uwb_angle_noise_min);
	obs.meas_h_theta(State::yaw) = 1;
	obs.meas_h_theta(State::yaw_rate) = 0;
	obs.updated = true;
	obs.type = ObsType::Uwb;

	return true;
}

bool VTEOrientation::fuseNewSensorData(ObsValidMaskU &vte_fusion_aid_mask,
				       const TargetObs observations[ObsType::Type_count])
{
	bool meas_fused = false;

	// Fuse vision position
	if (vte_fusion_aid_mask.flags.fuse_vision) {
		meas_fused |= fuseMeas(observations[ObsType::Fiducial_marker]);
	}

	// Fuse uwb
	if (vte_fusion_aid_mask.flags.fuse_uwb) {
		meas_fused |= fuseMeas(observations[ObsType::Uwb]);
	}

	return meas_fused;
}

bool VTEOrientation::fuseMeas(const TargetObs &target_obs)
{
	// update step for orientation
	bool meas_fused = false;

	estimator_aid_source1d_s target_innov;

	target_innov.time_last_fuse = _last_predict;
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_obs.timestamp);

	if (dt_sync_us > kMeasValidTimeoutUs || dt_sync_us < 0.f) {

		PX4_INFO("Obs type: %d too old or in the future. Time sync: %.2f [ms] > timeout: %.2f [ms]",
			 static_cast<int>(target_obs.type), (double)(dt_sync_us / 1000), (double)(kMeasValidTimeoutUs / 1000));

		// No measurement update, set to false
		target_innov.fused = false;
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	if (!target_obs.updated) {

		PX4_DEBUG("Obs i = %d: non-valid", static_cast<int>(target_obs.type));
		target_innov.fused = false;
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	// Convert time sync to seconds
	const float dt_sync_s = dt_sync_us / SEC2USEC_F;

	_target_est_yaw.syncState(dt_sync_s);
	_target_est_yaw.set_H(target_obs.meas_h_theta);
	target_innov.innovation_variance = _target_est_yaw.computeInnovCov(target_obs.meas_unc);
	target_innov.innovation = _target_est_yaw.computeInnov(target_obs.meas);

	// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
	_target_est_yaw.set_nis_threshold(_nis_threshold);
	meas_fused = _target_est_yaw.update();

	// Fill the target innovation field
	target_innov.innovation_rejected = !meas_fused;
	target_innov.fused = meas_fused;

	target_innov.observation = target_obs.meas;
	target_innov.observation_variance = target_obs.meas_unc;

	target_innov.test_ratio = _target_est_yaw.get_test_ratio();

	_vte_aid_ev_yaw_pub.publish(target_innov);

	return meas_fused;
}

void VTEOrientation::publishTarget()
{
	vision_target_est_orientation_s vision_target_orientation{};

	matrix::Vector<float, State::size> state = _target_est_yaw.get_state();
	matrix::Vector<float, State::size> state_var = _target_est_yaw.get_state_covariance();

	vision_target_orientation.timestamp = _last_predict;
	vision_target_orientation.orientation_valid = !hasTimedOut(_last_update, kTargetValidTimeoutUs);

	vision_target_orientation.theta = state(State::yaw);
	vision_target_orientation.cov_theta = state_var(State::yaw);

	vision_target_orientation.v_theta = state(State::yaw_rate);
	vision_target_orientation.cov_v_theta =  state_var(State::yaw_rate);

	_targetOrientationPub.publish(vision_target_orientation);
}

void VTEOrientation::checkMeasurementInputs()
{
	if (_range_sensor.valid) {
		_range_sensor.valid = isMeasUpdated(_range_sensor.timestamp);
	}

	if (_local_orientation.valid && !isMeasUpdated(_local_orientation.last_update)) {
		_local_orientation.valid = false;
	}
}

void VTEOrientation::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();

	_yaw_unc = _param_vte_yaw_unc_in.get();
	const float new_ev_angle_noise = _param_vte_ev_angle_noise.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	const float new_nis_threshold = _param_vte_yaw_nis_thre.get();
	const float new_uwb_angle_noise_min = _param_vte_uwb_a_noise.get();

	if (PX4_ISFINITE(new_ev_angle_noise) && new_ev_angle_noise > kMinObservationNoise) {
		_ev_angle_noise = new_ev_angle_noise;

	} else {
		PX4_WARN("VTE: VTE_EVA_NOISE %.1f <= %.1f, keeping previous value",
			 (double)new_ev_angle_noise, (double)kMinObservationNoise);
	}

	if (PX4_ISFINITE(new_nis_threshold) && new_nis_threshold > kMinNisThreshold) {
		_nis_threshold = new_nis_threshold;

	} else {
		PX4_WARN("VTE: VTE_YAW_NIS_THRE %.1f <= %.1f, keeping previous value",
			 (double)new_nis_threshold, (double)kMinNisThreshold);
	}

	if (PX4_ISFINITE(new_uwb_angle_noise_min) && new_uwb_angle_noise_min > 0.f) {
		_uwb_angle_noise_min = new_uwb_angle_noise_min;

	} else {
		PX4_WARN("VTE: VTE_UWB_A_NOISE %.1f <= 0, keeping previous value",
			 (double)new_uwb_angle_noise_min);
	}
}

void VTEOrientation::set_range_sensor(const float dist, const bool valid, hrt_abstime timestamp)
{
	_range_sensor.valid = valid && isMeasUpdated(timestamp) && (PX4_ISFINITE(dist) && dist > 0.f);
	_range_sensor.dist_bottom = dist;
	_range_sensor.timestamp = timestamp;
}

void VTEOrientation::set_local_orientation(const float yaw, const bool valid, const hrt_abstime timestamp)
{
	_local_orientation.valid = valid && isMeasUpdated(timestamp);
	_local_orientation.yaw = wrap_pi(yaw);
	_local_orientation.last_update = timestamp;
}

} // namespace vision_target_estimator
