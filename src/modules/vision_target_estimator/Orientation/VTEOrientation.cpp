/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @brief Estimate the orientation of a target by processing and fusing sensor
 * data in a Kalman Filter.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/defines.h>

#include <mathlib/mathlib.h>

#include "VTEOrientation.h"


namespace vision_target_estimator
{

using namespace matrix;
using OrientationStateVec = KF_orientation::VectorState;

VTEOrientation::VTEOrientation() : ModuleParams(nullptr)
{
	_vte_aid_ev_yaw_pub.advertise();
	_target_orientation_pub.advertise();

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

	// Predict the target yaw state using the constant yaw-rate model.
	if (_estimator_initialized) {
		const hrt_abstime now_us = nowUs();

		if (now_us < _last_predict) {
			resetFilter();
			PX4_WARN("VTE orientation invalid _last_predict time");
			return;
		}

		const uint64_t dt_us = now_us - _last_predict;

		// Guard against long scheduling gaps for which the constant yaw-rate assumption breaks down.
		static constexpr uint64_t kMaxPredictionDeltaTimeUs = 100_ms; // Filter runs at 50 Hz, expect dt = 20 ms.

		if (dt_us > kMaxPredictionDeltaTimeUs) {
			resetFilter();
			PX4_WARN("VTE orientation stale, resetting filter");
			return;
		}

		predictionStep(dt_us * kMicrosecondsToSeconds);
		_last_predict = now_us;
	}

	// Fuse the latest observation and publish innovations.
	if (performUpdateStep()) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {
		// Store the post-fusion posterior snapshot at the current timestamp for OOSM fusion.
		_target_est_yaw.pushHistory(_last_predict);
		publishTarget();
	}
}

void VTEOrientation::initEstimator(const VisionObs &vision_obs)
{
	// Seed from the first valid yaw sample; yaw-rate starts at zero until an observer provides it.
	OrientationStateVec state_init{};
	state_init(KF_orientation::kYaw) = vision_obs.yaw;

	OrientationStateVec state_var_init;
	state_var_init.setAll(fmaxf(_param_vte_yaw_unc_in.get(), kMinVariance));

	_target_est_yaw.setState(state_init);
	_target_est_yaw.setStateCovarianceDiag(state_var_init);
	_target_est_yaw.resetHistory();

	PX4_DEBUG("VTE Orientation initialized with yaw: %.2f [rad]", (double)vision_obs.yaw);

	_estimator_initialized = true;
	_last_update = nowUs();
	_last_predict = _last_update;
}

void VTEOrientation::predictionStep(float dt)
{
	_target_est_yaw.setYawAccVar(fmaxf(_param_vte_yaw_acc_unc.get(), kMinVariance));
	_target_est_yaw.predict(dt);
}

bool VTEOrientation::performUpdateStep()
{
	VisionObs vision_obs{};

	if (!_vte_aid_mask.flags.use_vision_pos || !processObsVision(vision_obs)) {
		return false;
	}

	if (!_estimator_initialized) {
		initEstimator(vision_obs);
	}

	return fuseMeas(vision_obs);
}

bool VTEOrientation::shouldEmitWarning(hrt_abstime &last_warn)
{
	const hrt_abstime now = nowUs();

	if ((last_warn == 0) || (now < last_warn) || ((now - last_warn) > kWarnThrottleIntervalUs)) {
		last_warn = now;
		return true;
	}

	return false;
}

bool VTEOrientation::isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw)
{
	if (!isMeasRecent(fiducial_marker_yaw.timestamp_sample)) {
		if (shouldEmitWarning(_vision_warn_last)) {
			PX4_WARN("VTE: Vision yaw too old!");
		}

		return false;
	}

	if (!PX4_ISFINITE(fiducial_marker_yaw.yaw_ned) || !PX4_ISFINITE(fiducial_marker_yaw.yaw_var_ned)) {
		if (shouldEmitWarning(_vision_warn_last)) {
			PX4_WARN("VTE: Vision yaw is corrupt!");
		}

		return false;
	}

	return true;
}

bool VTEOrientation::processObsVision(VisionObs &vision_obs)
{
	fiducial_marker_yaw_report_s fiducial_marker_yaw;

	if (!_fiducial_marker_yaw_report_sub.update(&fiducial_marker_yaw) ||
	    !isVisionDataValid(fiducial_marker_yaw)) {
		return false;
	}

	const float min_ev_angle_var = sq(fmaxf(_param_vte_ev_angle_noise.get(), kMinObservationNoise));
	const float yaw_unc = fmaxf(fiducial_marker_yaw.yaw_var_ned, min_ev_angle_var);

	vision_obs.timestamp = fiducial_marker_yaw.timestamp_sample;
	vision_obs.yaw = wrap_pi(fiducial_marker_yaw.yaw_ned);
	vision_obs.yaw_var = yaw_unc;

	return true;
}

bool VTEOrientation::fuseMeas(const VisionObs &vision_obs)
{
	vte_aid_source1d_s target_innov = {};
	const float nis_threshold = fmaxf(_param_vte_yaw_nis_thre.get(), kMinNisThreshold);

	target_innov.timestamp_sample = vision_obs.timestamp;
	target_innov.timestamp = nowUs();
	target_innov.time_last_predict = _last_predict;
	target_innov.time_since_meas_ms = static_cast<float>(signedTimeDiffUs(
			_last_predict, vision_obs.timestamp)) * kMicrosecondsToMilliseconds;

	OrientationStateVec meas_h_theta{};
	meas_h_theta(KF_orientation::kYaw) = 1.f;

	const KF_orientation::ScalarMeas meas_input{
		vision_obs.timestamp, vision_obs.yaw, vision_obs.yaw_var, meas_h_theta};

	target_innov.observation = meas_input.val;
	target_innov.observation_variance = meas_input.unc;

	const FusionResult result = _target_est_yaw.fuseScalarAtTime(meas_input, _last_predict, nis_threshold);

	target_innov.innovation          = result.innov;
	target_innov.innovation_variance = result.innov_var;
	target_innov.test_ratio          = result.test_ratio;
	target_innov.fusion_status       = static_cast<uint8_t>(result.status);
	target_innov.history_steps       = result.history_steps;

	_vte_aid_ev_yaw_pub.publish(target_innov);

	return (result.status == FusionStatus::STATUS_FUSED_CURRENT || result.status == FusionStatus::STATUS_FUSED_OOSM);
}

void VTEOrientation::publishTarget()
{
	_orientation_msg = {};

	const OrientationStateVec &state = _target_est_yaw.getState();
	const OrientationStateVec state_var = _target_est_yaw.getStateCovarianceDiag();

	_orientation_msg.timestamp = _last_predict;
	_orientation_msg.orientation_valid = !hasTimedOut(_last_update, _target_valid_timeout_us);

	_orientation_msg.yaw = state(KF_orientation::kYaw);
	_orientation_msg.cov_yaw = state_var(KF_orientation::kYaw);

	_orientation_msg.yaw_rate = state(KF_orientation::kYawRate);
	_orientation_msg.cov_yaw_rate = state_var(KF_orientation::kYawRate);

	_target_orientation_pub.publish(_orientation_msg);
}

void VTEOrientation::print_status() const
{
	const auto yes_no = [](const bool value) { return value ? "yes" : "no"; };
	const auto age_s = [](const hrt_abstime timestamp) -> double {
		if (timestamp == 0)
		{
			return -1.;
		}

		return signedTimeDiffUs(nowUs(), timestamp) * 1e-6;
	};

	const bool orientation_valid = _estimator_initialized && !hasTimedOut(_last_update, _target_valid_timeout_us);

	PX4_INFO("  orientation state: initialized: %s, valid: %s, last predict/update age: %.3f / %.3f s",
		 yes_no(_estimator_initialized), yes_no(orientation_valid), age_s(_last_predict), age_s(_last_update));

	if (!_estimator_initialized) {
		return;
	}

	const OrientationStateVec &state = _target_est_yaw.getState();

	PX4_INFO("    yaw: %.3f rad (%.1f deg)",
		 (double)state(KF_orientation::kYaw),
		 (double)math::degrees(state(KF_orientation::kYaw)));
	PX4_INFO("    yaw rate: %.3f rad/s",
		 (double)state(KF_orientation::kYawRate));
}

void VTEOrientation::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();
}

} // namespace vision_target_estimator
