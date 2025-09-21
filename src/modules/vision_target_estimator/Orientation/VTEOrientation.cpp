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

constexpr float kDefaultVisionYawDistance = 10.f;
constexpr float kDefaultUwbAngleVar = math::radians(3.f) * math::radians(3.f);

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

	// Predict the target yaw state using the configured kinematic model.
	if (_estimator_initialized) {
		predictionStep();
		_last_predict = hrt_absolute_time();
	}

	// Update with the latest observations and publish any resulting innovations.
	if (performUpdateStep(q_att)) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {
		publishTarget();
	}
}

bool VTEOrientation::initEstimator(const ObsValidMaskU &fusion_mask,
				   const TargetObs observations[kObsTypeCount])
{
	// Until a sensor measures the yaw rate, state_init(State::yaw_rate) is init at zero
	matrix::Vector<float, State::size> state_init{};

	// Yaw init
	if (fusion_mask.flags.fuse_vision) {
		state_init(State::yaw) = observations[obsIndex(ObsType::Fiducial_marker)].meas;

	} else if (fusion_mask.flags.fuse_uwb) {
		state_init(State::yaw) = observations[obsIndex(ObsType::Uwb)].meas;
	}

	matrix::Vector<float, State::size> state_var_init;
	state_var_init.setAll(_yaw_unc);

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
	// Time since the last prediction
	const float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC_F;

	_target_est_yaw.predictState(dt);
	_target_est_yaw.predictCov(dt);
}

bool VTEOrientation::performUpdateStep(const matrix::Quaternionf &q_att)
{
	ObsValidMaskU fusion_mask{};
	resetObservations();
	// Gather the latest observations from each enabled data source.
	processObservations(q_att, fusion_mask, _obs_buffer);

	if (fusion_mask.value == 0) {
		return false;
	}

	if (!_estimator_initialized && !initEstimator(fusion_mask, _obs_buffer)) {
		return false;
	}

	return fuseActiveMeasurements(fusion_mask, _obs_buffer);
}

void VTEOrientation::resetObservations()
{
	for (auto &obs : _obs_buffer) {
		obs = {};
	}
}

void VTEOrientation::processObservations(const matrix::Quaternionf &q_att, ObsValidMaskU &fusion_mask,
		TargetObs observations[kObsTypeCount])
{
	handleVisionData(fusion_mask, observations[obsIndex(ObsType::Fiducial_marker)]);

	handleUwbData(q_att, fusion_mask, observations[obsIndex(ObsType::Uwb)]);
}

void VTEOrientation::handleVisionData(ObsValidMaskU &fusion_mask, TargetObs &vision_obs)
{
	if (!_vte_aid_mask.flags.use_vision_pos) {
		return;
	}

	fiducial_marker_yaw_report_s fiducial_marker_yaw;

	if (!_fiducial_marker_yaw_report_sub.update(&fiducial_marker_yaw)
	    || !isVisionDataValid(fiducial_marker_yaw)) {
		return;
	}

	if (processObsVision(fiducial_marker_yaw, vision_obs)) {
		fusion_mask.flags.fuse_vision = true;
	}
}

bool VTEOrientation::isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw) const
{
	if (!isMeasRecent(fiducial_marker_yaw.timestamp)) {
		PX4_DEBUG("Vision yaw is outdated!");
		return false;
	}

	if (!PX4_ISFINITE(fiducial_marker_yaw.yaw_ned)) {
		PX4_DEBUG("Vision yaw is corrupt!");
		return false;
	}

	return true;
}

bool VTEOrientation::processObsVision(const fiducial_marker_yaw_report_s &fiducial_marker_yaw, TargetObs &obs) const
{
	float yaw_unc = fmaxf(fiducial_marker_yaw.yaw_var_ned, _min_ev_angle_var);

	if (_ev_noise_md) {
		const float range = _range_sensor.valid ? fmaxf(_range_sensor.dist_bottom, 1.f) : kDefaultVisionYawDistance;
		yaw_unc = _min_ev_angle_var * range;

	}

	obs.timestamp = fiducial_marker_yaw.timestamp;
	obs.updated = true;
	obs.meas = wrap_pi(fiducial_marker_yaw.yaw_ned);
	obs.meas_unc = yaw_unc;
	obs.meas_h_theta(State::yaw) = 1;
	obs.type = ObsType::Fiducial_marker;

	return true;
}

void VTEOrientation::handleUwbData(const matrix::Quaternionf &q_att, ObsValidMaskU &fusion_mask,
				   TargetObs &uwb_obs)
{
	sensor_uwb_s uwb_report;

	if (!_vte_aid_mask.flags.use_uwb) {
		return;
	}

	if (!_sensor_uwb_sub.update(&uwb_report) || !isUwbDataValid(uwb_report)) {
		return;
	}

	if (processObsUwb(q_att, uwb_report, uwb_obs)) {
		fusion_mask.flags.fuse_uwb = true;
	}
}

// TODO: check
bool VTEOrientation::isUwbDataValid(const sensor_uwb_s &uwb_report) const
{
	if (!isMeasRecent(uwb_report.timestamp)) {
		return false;
	}

	const bool finite_range = PX4_ISFINITE(uwb_report.distance);
	const bool angles_within_bounds = fabsf(uwb_report.aoa_azimuth_dev) <= max_uwb_aoa_angle_degree
					  && fabsf(uwb_report.aoa_elevation_dev) <= max_uwb_aoa_angle_degree
					  && fabsf(uwb_report.aoa_azimuth_resp) <= max_uwb_aoa_angle_degree;

	return finite_range && angles_within_bounds;
}

bool VTEOrientation::processObsUwb(const matrix::Quaternionf &q_att, const sensor_uwb_s &uwb_report,
				   TargetObs &obs) const
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
	obs.meas_unc = fmaxf(kDefaultUwbAngleVar, _min_uwb_angle_var);
	obs.meas_h_theta(State::yaw) = 1;
	obs.updated = true;
	obs.type = ObsType::Uwb;

	return true;
}

bool VTEOrientation::fuseActiveMeasurements(ObsValidMaskU &fusion_mask,
		const TargetObs observations[kObsTypeCount])
{
	bool meas_fused = false;

	// Fuse vision position
	if (fusion_mask.flags.fuse_vision) {
		meas_fused |= fuseMeas(observations[obsIndex(ObsType::Fiducial_marker)]);
	}

	// Fuse uwb
	if (fusion_mask.flags.fuse_uwb) {
		meas_fused |= fuseMeas(observations[obsIndex(ObsType::Uwb)]);
	}

	return meas_fused;
}

bool VTEOrientation::fuseMeas(const TargetObs &target_obs)
{
	estimator_aid_source1d_s &target_innov = _aid_src1d_buffer;
	target_innov = {};

	target_innov.time_last_fuse = _last_predict;
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();

	const int64_t dt_sync_us = signedTimeDiffUs(_last_predict, target_obs.timestamp);
	const bool measurement_stale = (dt_sync_us > static_cast<int64_t>(_meas_recent_timeout_us)) || (dt_sync_us < 0);

	if (measurement_stale || !target_obs.updated) {
		if (measurement_stale) {
			PX4_INFO("Obs type: %d too old or in the future. Time sync: %.2f [ms] > timeout: %.2f [ms]",
				 static_cast<int>(target_obs.type), static_cast<double>(dt_sync_us) / 1000.,
				 static_cast<double>(_meas_recent_timeout_us) / 1000.);

		} else {
			PX4_DEBUG("Obs i = %d: non-valid", static_cast<int>(target_obs.type));
		}

		target_innov.fused = false;
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	const float dt_sync_s = static_cast<float>(dt_sync_us) / SEC2USEC_F;

	_target_est_yaw.syncState(dt_sync_s);
	_target_est_yaw.set_H(target_obs.meas_h_theta);
	target_innov.innovation_variance = _target_est_yaw.computeInnovCov(target_obs.meas_unc);
	target_innov.innovation = _target_est_yaw.computeInnov(target_obs.meas);

	_target_est_yaw.set_nis_threshold(_nis_threshold);
	const bool meas_fused = _target_est_yaw.update();

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
	vision_target_est_orientation_s &vision_target_orientation = _orientation_msg;
	vision_target_orientation = {};

	matrix::Vector<float, State::size> state = _target_est_yaw.get_state();
	matrix::Vector<float, State::size> state_var = _target_est_yaw.get_state_covariance();

	vision_target_orientation.timestamp = _last_predict;
	vision_target_orientation.orientation_valid = !hasTimedOut(_last_update, _target_valid_timeout_us);

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
	const float new_uwb_angle_noise = _param_vte_uwb_a_noise.get();

	if (PX4_ISFINITE(new_ev_angle_noise) && new_ev_angle_noise > kMinObservationNoise) {
		_min_ev_angle_var = new_ev_angle_noise * new_ev_angle_noise;

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

	if (PX4_ISFINITE(new_uwb_angle_noise) && new_uwb_angle_noise > 0.f) {
		_min_uwb_angle_var = new_uwb_angle_noise * new_uwb_angle_noise;

	} else {
		PX4_WARN("VTE: VTE_UWB_A_NOISE %.1f <= 0, keeping previous value",
			 (double)new_uwb_angle_noise);
	}
}

void VTEOrientation::set_range_sensor(const float dist, const bool valid, hrt_abstime timestamp)
{
	_range_sensor.valid = valid && isMeasUpdated(timestamp) && (PX4_ISFINITE(dist) && dist > 0.f);
	_range_sensor.dist_bottom = dist;
	_range_sensor.timestamp = timestamp;
}

} // namespace vision_target_estimator
