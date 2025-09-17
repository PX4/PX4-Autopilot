/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

// TODO: implement aid mask. In the future there might be more measurements

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

VTEOrientation::~VTEOrientation()
{
	delete _target_est_yaw;
}

bool VTEOrientation::init()
{
	return createEstimator();
}

void VTEOrientation::reset_filter()
{
	_estimator_initialized = false;
	_has_timed_out = false;
}

void VTEOrientation::update()
{
	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	checkMeasurementInputs();

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _vte_TIMEOUT_US) {
			PX4_WARN("VTE orientation estimator has timed out");
			_has_timed_out = true;

		} else {
			predictionStep();
			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and pulishes innovations
	if (updateStep()) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool VTEOrientation::initEstimator(const ObsValidMask &vte_fusion_aid_mask,
				   const targetObs observations[ObsType::Type_count])
{
	// Until a sensor measures the yaw rate, state_init(State::yaw_rate) is init at zero
	matrix::Vector<float, State::size> state_init{};

	// Yaw init
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_VISION) {
		state_init(State::yaw) = observations[ObsType::Fiducial_marker].meas;

	} else if (vte_fusion_aid_mask & ObsValidMask::FUSE_UWB) {
		state_init(State::yaw) = observations[ObsType::Uwb].meas;
	}

	matrix::Vector<float, State::size> state_var_init;

	for (int i = 0; i < State::size; i++) {
		state_var_init(i) = _yaw_unc;
	}

	_target_est_yaw->setState(state_init);
	_target_est_yaw->setStateVar(state_var_init);

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

	_target_est_yaw->predictState(dt);
	_target_est_yaw->predictCov(dt);
}

bool VTEOrientation::updateStep()
{
	targetObs obs_fiducial_marker_orientation;

	ObsValidMask vte_fusion_aid_mask = ObsValidMask::NO_VALID_DATA;
	targetObs observations[ObsType::Type_count];
	processObservations(vte_fusion_aid_mask, observations);

	// No new observations --> no fusion.
	if (vte_fusion_aid_mask == ObsValidMask::NO_VALID_DATA) {
		return false;
	}

	// Initialize estimator if not already initialized
	if (!_estimator_initialized && !initEstimator(vte_fusion_aid_mask, observations)) {
		reset_filter();
		return false;
	}

	// Fuse new sensor data
	return fuseNewSensorData(vte_fusion_aid_mask, observations);
}

void VTEOrientation::processObservations(ObsValidMask &vte_fusion_aid_mask, targetObs obs[ObsType::Type_count])
{
	handleVisionData(vte_fusion_aid_mask, obs[ObsType::Fiducial_marker]);

	handleUwbData(vte_fusion_aid_mask, obs[ObsType::Uwb]);
}

void VTEOrientation::handleVisionData(ObsValidMask &vte_fusion_aid_mask, targetObs &obs_fiducial_marker)
{

	if (!(_vte_aid_mask & SensorFusionMask::USE_EXT_VIS_POS)) {
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
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_VISION);
	}
}

bool VTEOrientation::isVisionDataValid(const fiducial_marker_yaw_report_s &fiducial_marker_yaw)
{
	if (!isMeasValid(fiducial_marker_yaw.timestamp)) {
		PX4_WARN("Vision yaw is outdated!");
		return false;
	}

	if (!PX4_ISFINITE(fiducial_marker_yaw.yaw_ned)) {
		PX4_WARN("Vision yaw is corrupt!");
		return false;
	}

	// TODO: extend checks
	return true;
}

bool VTEOrientation::processObsVision(const fiducial_marker_yaw_report_s &fiducial_marker_yaw, targetObs &obs)
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


void VTEOrientation::handleUwbData(ObsValidMask &vte_fusion_aid_mask, targetObs &obs_uwb)
{
	sensor_uwb_s uwb_report;

	if (!(_vte_aid_mask & SensorFusionMask::USE_UWB)) {
		return;
	}

	if (!_sensor_uwb_sub.update(&uwb_report)) {
		return;
	}

	if (!isUwbDataValid(uwb_report)) {
		return;
	}

	if (processObsUwb(uwb_report, obs_uwb)) {
		vte_fusion_aid_mask = static_cast<ObsValidMask>(vte_fusion_aid_mask | ObsValidMask::FUSE_UWB);
	}
}

// TODO: check
bool VTEOrientation::isUwbDataValid(const sensor_uwb_s &uwb_report)
{
	if (!isMeasValid(uwb_report.timestamp)) {
		return false;
	}

	if (fabsf(uwb_report.aoa_azimuth_dev) > max_uwb_aoa_angle_degree ||
	    fabsf(uwb_report.aoa_elevation_dev) > max_uwb_aoa_angle_degree) {
		return false;
	}

	return true;
}

// TODO: complete
bool VTEOrientation::processObsUwb(const sensor_uwb_s &uwb_report, targetObs &obs)
{
	obs.timestamp = uwb_report.timestamp;
	// obs.updated = ;

	// obs.meas = ;
	// obs.meas_unc = ;

	// obs.meas_h_theta(State::yaw) = 1; // Set to 1 if observed, zero otherwise
	// obs.meas_h_theta(State::yaw_rate) = 1; // Set to 1 if observed, zero otherwise

	// obs.updated = true;
	obs.type = ObsType::Uwb;

	return false; // TODO: change to true once implemented
}

bool VTEOrientation::fuseNewSensorData(ObsValidMask &vte_fusion_aid_mask,
				       const targetObs observations[ObsType::Type_count])
{
	bool meas_fused = false;

	// Fuse vision position
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_VISION) {
		meas_fused |= fuseMeas(observations[ObsType::Fiducial_marker]);
	}

	// Fuse uwb
	if (vte_fusion_aid_mask & ObsValidMask::FUSE_UWB) {
		meas_fused |= fuseMeas(observations[ObsType::Uwb]);
	}

	return meas_fused;
}


bool VTEOrientation::fuseMeas(const targetObs &target_obs)
{
	// Update step for orientation
	bool meas_fused = false;

	estimator_aid_source1d_s target_innov;

	target_innov.time_last_fuse = _last_predict;
	target_innov.timestamp_sample = target_obs.timestamp;
	target_innov.timestamp = hrt_absolute_time();

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_obs.timestamp);

	if (dt_sync_us > meas_valid_TIMEOUT_US || dt_sync_us < 0.f) {

		PX4_INFO("Obs type: %d too old or in the future. Time sync: %.2f [ms] > timeout: %.2f [ms]",
			 target_obs.type, (double)(dt_sync_us / 1000), (double)(meas_valid_TIMEOUT_US / 1000));

		// No measurement update, set to false
		target_innov.fused = false;
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	if (!target_obs.updated) {

		PX4_DEBUG("Obs i = %d: non-valid", target_obs.type);
		target_innov.fused = false;
		_vte_aid_ev_yaw_pub.publish(target_innov);
		return false;
	}

	// Convert time sync to seconds
	const float dt_sync_s = dt_sync_us / SEC2USEC_F;

	_target_est_yaw->syncState(dt_sync_s);
	_target_est_yaw->setH(target_obs.meas_h_theta);
	target_innov.innovation_variance = _target_est_yaw->computeInnovCov(target_obs.meas_unc);
	target_innov.innovation = _target_est_yaw->computeInnov(target_obs.meas);

	// Set the Normalized Innovation Squared (NIS) check threshold. Used to reject outlier measurements
	_target_est_yaw->setNISthreshold(_nis_threshold);
	meas_fused = _target_est_yaw->update();

	// Fill the target innovation field
	target_innov.innovation_rejected = !meas_fused;
	target_innov.fused = meas_fused;

	target_innov.observation = target_obs.meas;
	target_innov.observation_variance = target_obs.meas_unc;

	target_innov.test_ratio = _target_est_yaw->getTestRatio();

	_vte_aid_ev_yaw_pub.publish(target_innov);

	return meas_fused;
}

void VTEOrientation::publishTarget()
{
	vision_target_est_orientation_s vision_target_orientation{};

	matrix::Vector<float, State::size> state = _target_est_yaw->getState();
	matrix::Vector<float, State::size> state_var = _target_est_yaw->getStateVar();

	vision_target_orientation.timestamp = _last_predict;
	vision_target_orientation.orientation_valid = (hrt_absolute_time() - _last_update < target_valid_TIMEOUT_US);

	vision_target_orientation.theta = state(State::yaw);
	vision_target_orientation.cov_theta = state_var(State::yaw);

	vision_target_orientation.v_theta = state(State::yaw_rate);
	vision_target_orientation.cov_v_theta =  state_var(State::yaw_rate);

	_targetOrientationPub.publish(vision_target_orientation);
}

void VTEOrientation::checkMeasurementInputs()
{
	if (_range_sensor.valid) {
		_range_sensor.valid = isMeasUpdated(_range_sensor.last_update);
	}

	// TODO: check other measurements?
}

void VTEOrientation::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();

	_yaw_unc = _param_vte_yaw_unc_in.get();
	_ev_angle_noise = _param_vte_ev_angle_noise.get();
	_ev_noise_md = _param_vte_ev_noise_md.get();
	_nis_threshold = _param_vte_yaw_nis_thre.get();
}

void VTEOrientation::set_range_sensor(const float dist, const bool valid)
{
	_range_sensor.valid = valid;
	_range_sensor.dist_bottom = dist;
	_range_sensor.last_update = hrt_absolute_time();
}

bool VTEOrientation::createEstimator()
{
	KF_orientation *tmp_theta = new KF_orientation;

	if (tmp_theta == nullptr) {
		PX4_ERR("VTE orientation creation failed");
		return false;

	} else {

		delete _target_est_yaw;
		_target_est_yaw = tmp_theta;

		return true;
	}
}

} // namespace vision_target_estimator
