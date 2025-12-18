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

#define USEC2SEC_F 1e-6f

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
	_target_est_yaw.resetHistory();
	return true;
}

void VTEOrientation::resetFilter()
{
	_estimator_initialized = false;
	_target_est_yaw.resetHistory();
}

void VTEOrientation::update()
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
	if (performUpdateStep()) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {
		// Store the post-fusion posterior snapshot at the current timestamp for OOSM fusion.
		_target_est_yaw.pushHistory(_last_predict);
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
	}

	matrix::Vector<float, State::size> state_var_init;
	state_var_init.setAll(_yaw_unc);

	_target_est_yaw.setState(state_init);
	_target_est_yaw.setStateCovarianceDiag(state_var_init);
	_target_est_yaw.resetHistory();

	PX4_DEBUG("Orientation filter init yaw: %.2f [rad] yaw_rate: %.2f [rad/s]", (double)state_init(State::yaw),
		 (double)state_init(State::yaw_rate));

	PX4_DEBUG("VTE Orientation Estimator properly initialized.");
	_estimator_initialized = true;
	_last_update = hrt_absolute_time();
	_last_predict = _last_update;

	return true;
}

void VTEOrientation::predictionStep()
{
	// Time since the last prediction
	const float dt = (hrt_absolute_time() - _last_predict) * USEC2SEC_F;

	_target_est_yaw.predict(dt);
}

bool VTEOrientation::performUpdateStep()
{
	ObsValidMaskU fusion_mask{};
	resetObservations();
	// Gather the latest observations from each enabled data source.
	processObservations(fusion_mask, _obs_buffer);

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

void VTEOrientation::processObservations(ObsValidMaskU &fusion_mask,
		TargetObs observations[kObsTypeCount])
{
	fusion_mask.flags.fuse_vision = _vte_aid_mask.flags.use_vision_pos
					&& processObsVision(observations[obsIndex(ObsType::Fiducial_marker)]);
}

bool VTEOrientation::isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw) const
{
	if (!isMeasRecent(fiducial_marker_yaw.timestamp_sample)) {
		PX4_DEBUG("Vision yaw is outdated!"); // TODO: throttle, similar to _vision_pos_warn_last
		return false;
	}

	if (!PX4_ISFINITE(fiducial_marker_yaw.yaw_ned)) {
		PX4_DEBUG("Vision yaw is corrupt!"); // TODO: throttle, similar to _vision_pos_warn_last
		return false;
	}

	if (!_ev_noise_md && !PX4_ISFINITE(fiducial_marker_yaw.yaw_var_ned)) {
		PX4_DEBUG("Vision yaw var is corrupt!"); // TODO: throttle, similar to _vision_pos_warn_last
		return false;
	}

	return true;
}

bool VTEOrientation::processObsVision(TargetObs &obs)
{
	fiducial_marker_yaw_report_s fiducial_marker_yaw;

	if (!_fiducial_marker_yaw_report_sub.update(&fiducial_marker_yaw)
	    || !isVisionDataValid(fiducial_marker_yaw)) {
		return false;
	}

	float yaw_unc = fmaxf(fiducial_marker_yaw.yaw_var_ned, _min_ev_angle_var);

	static constexpr float kDefaultVisionYawDistance = 10.f;

	if (_ev_noise_md) {
		const float range = _range_sensor.valid ? _range_sensor.dist_bottom : kDefaultVisionYawDistance;
		yaw_unc = fmaxf(sq(sqrtf(_min_ev_angle_var) * range), _min_ev_angle_var);
	}

	obs.timestamp = fiducial_marker_yaw.timestamp_sample;
	obs.updated = true;
	obs.meas = wrap_pi(fiducial_marker_yaw.yaw_ned);
	obs.meas_unc = yaw_unc;
	obs.meas_h_theta(State::yaw) = 1;
	obs.type = ObsType::Fiducial_marker;

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

	return meas_fused;
}

bool VTEOrientation::fuseMeas(const TargetObs &target_obs)
{
	vte_aid_source1d_s target_innov = {};

	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();
	target_innov.time_last_predict = _last_predict;
	target_innov.time_since_meas_ms = static_cast<float>(signedTimeDiffUs(_last_predict, target_obs.timestamp)) * 1e-3f;

	if (!target_obs.updated) {
		target_innov.fusion_status = static_cast<uint8_t>(FusionStatus::IDLE);
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	const KF_orientation::ScalarMeas meas_input{target_obs.timestamp, target_obs.meas, target_obs.meas_unc,
			target_obs.meas_h_theta};

	target_innov.observation = meas_input.val;
	target_innov.observation_variance = meas_input.unc;

	const FusionResult result = _target_est_yaw.fuseScalarAtTime(meas_input, _last_predict, _nis_threshold);

	target_innov.innovation          = result.innov;
	target_innov.innovation_variance = result.innov_var;
	target_innov.test_ratio          = result.test_ratio;
	target_innov.fusion_status       = static_cast<uint8_t>(result.status);
	target_innov.history_steps       = result.history_steps;

	_vte_aid_ev_yaw_pub.publish(target_innov);

	return (result.status == FusionStatus::FUSED_CURRENT || result.status == FusionStatus::FUSED_OOSM);
}

void VTEOrientation::publishTarget()
{
	vision_target_est_orientation_s &vision_target_orientation = _orientation_msg;
	vision_target_orientation = {};

	const matrix::Vector<float, State::size> &state = _target_est_yaw.getState();
	const matrix::Vector<float, State::size> state_var = _target_est_yaw.getStateCovarianceDiag();

	vision_target_orientation.timestamp = _last_predict;
	vision_target_orientation.orientation_valid = !hasTimedOut(_last_update, _target_valid_timeout_us);

	vision_target_orientation.yaw = state(State::yaw);
	vision_target_orientation.cov_yaw = state_var(State::yaw);

	vision_target_orientation.yaw_rate = state(State::yaw_rate);
	vision_target_orientation.cov_yaw_rate =  state_var(State::yaw_rate);

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
	const float new_yaw_acc_var = _param_vte_yaw_acc_unc.get();
	const float new_ev_angle_noise = _param_vte_ev_angle_noise.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	const float new_nis_threshold = _param_vte_yaw_nis_thre.get();

	if (PX4_ISFINITE(new_yaw_acc_var) && (new_yaw_acc_var >= 0.f)) {
		_yaw_acc_var = new_yaw_acc_var;
		_target_est_yaw.setYawAccVar(_yaw_acc_var);
	}

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

}

void VTEOrientation::set_range_sensor(const float dist, const bool valid, hrt_abstime timestamp)
{
	_range_sensor.valid = valid && isMeasUpdated(timestamp) && (PX4_ISFINITE(dist) && dist > 0.f);
	_range_sensor.dist_bottom = dist;
	_range_sensor.timestamp = timestamp;
}

} // namespace vision_target_estimator
