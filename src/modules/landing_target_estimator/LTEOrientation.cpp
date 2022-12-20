/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/*
 * @file LTEOrientation.cpp
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 * @author Mohammed Kabir <kabir@uasys.io>
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

#include "LTEOrientation.h"

#define SEC2USEC 1000000.0f

namespace landing_target_estimator
{

using namespace matrix;

LTEOrientation::LTEOrientation() :
	ModuleParams(nullptr)
{
	_target_estimator_aid_ev_yaw_pub.advertise();
	_targetOrientationPub.advertise();

	_check_params(true);
}

LTEOrientation::~LTEOrientation()
{
	delete _target_estimator_orientation;
}

void LTEOrientation::resetFilter()
{
	_estimator_initialized = false;
	_new_pos_sensor_acquired_time = 0;
}

void LTEOrientation::update()
{
	_check_params(false);

	// Rename to update topics
	update_topics();

	// Next waypoint is not land: early return;
	if (!_estimate_orientation || !_start_filter) {
		return;
	}

	// predict the target state using a constant relative acceleration model
	if (_estimator_initialized) {

		if (hrt_absolute_time() - _last_update > _ltest_TIMEOUT_US) {
			PX4_WARN("LTE orientation estimator timeout");
			resetFilter();

		} else {
			predictionStep();
			_last_predict = hrt_absolute_time();
		}
	}

	// Update and fuse the observations and pulishes innovations
	if (update_step()) {
		_last_update = _last_predict;
	}

	if (_estimator_initialized) {publishTarget();}
}

bool LTEOrientation::initEstimator(float theta_init)
{

	PX4_INFO("Theta init %.2f", (double)theta_init);

	_target_estimator_orientation->setPosition(theta_init);
	_target_estimator_orientation->setVelocity(0.f);
	_target_estimator_orientation->setStatePosVar(_yaw_unc);
	_target_estimator_orientation->setStateVelVar(_yaw_unc);

	return true;
}


void LTEOrientation::predictionStep()
{
	// Time from last prediciton
	float dt = (hrt_absolute_time() - _last_predict) / SEC2USEC;

	_target_estimator_orientation->predictState(dt);
	_target_estimator_orientation->predictCov(dt);
}



bool LTEOrientation::update_step()
{
	landing_target_orientation_s fiducial_marker_orientation;
	targetObsOrientation obs_fiducial_marker_orientation;

	bool orientation_valid = false;

	// Process data from all topics
	if (_fiducial_marker_orientation_sub.update(&fiducial_marker_orientation)) {

		if (processObsVisionOrientation(fiducial_marker_orientation, obs_fiducial_marker_orientation)) {

			orientation_valid = ((hrt_absolute_time() - obs_fiducial_marker_orientation.timestamp) < measurement_valid_TIMEOUT_US);
		}
	}

	// If we have a new sensor and the estimator is initialized: fuse available measurements and publish innov.
	if (orientation_valid && _estimator_initialized) {

		return fuse_orientation(obs_fiducial_marker_orientation);

	} else if (orientation_valid && !_estimator_initialized) {

		// Wait 1 second before initilazing the estimator to have a velocity initial estimate.
		if (!_new_pos_sensor_acquired_time) {
			_new_pos_sensor_acquired_time = hrt_absolute_time();

		} else if ((hrt_absolute_time() - _new_pos_sensor_acquired_time) > 1000000) {

			float theta_init = obs_fiducial_marker_orientation.meas_theta;

			if (initEstimator(theta_init)) {
				PX4_INFO("LTE Orientation Estimator properly initialized.");
				_estimator_initialized = true;
				_last_update = hrt_absolute_time();
				_last_predict = _last_update;

			} else {
				resetFilter();
			}
		}
	}

	return false;
}


bool LTEOrientation::processObsVisionOrientation(const landing_target_orientation_s &fiducial_marker_orientation,
		targetObsOrientation &obs)
{

	// TODO complete mavlink message to include orientation
	float vision_r_theta = matrix::wrap_pi(fiducial_marker_orientation.theta_rel);
	float vision_r_theta_unc = 0.5f; // eventually use fiducial_marker_orientation.cov_theta_rel
	hrt_abstime vision_timestamp = fiducial_marker_orientation.timestamp;

	/* ORIENTATION */
	if (!PX4_ISFINITE(vision_r_theta)) {
		PX4_WARN("VISION orientation is corrupt!");

	} else {

		// TODO: obtain relative orientation using the orientation of the drone.
		// (vision gives orientation between body and target frame. We can thus use the orientation between the body frame and NED to obtain the orientation between NED and target)
		obs.timestamp = vision_timestamp;
		obs.updated_theta = true;
		obs.meas_unc_theta = vision_r_theta_unc;
		obs.meas_theta = vision_r_theta;
		obs.meas_h_theta(0) = 1;

		// TODO: uncomment once mavlink message is complete
		return true;
	}

	return false;
}

bool LTEOrientation::fuse_orientation(const targetObsOrientation &target_orientation_obs)
{
	// Update step for orientation
	bool meas_fused = false;

	estimator_aid_source_1d_s target_innov;

	// Compute the measurement's time delay (difference between state and measurement time on validity)
	const float dt_sync_us = (_last_predict - target_orientation_obs.timestamp);

	if (dt_sync_us > measurement_valid_TIMEOUT_US) {

		PX4_INFO("Orientation obs. rejected because too old. Time sync: %.2f [seconds] > timeout: %.2f [seconds]",
			 (double)(dt_sync_us / SEC2USEC), (double)(measurement_valid_TIMEOUT_US / SEC2USEC));

		// No measurement update, set to false
		target_innov.fusion_enabled = false;
		target_innov.fused = false;

	} else if (target_orientation_obs.updated_theta) {

		// TODO: Eventually remove, for now to debug, assume prediction time = measurement time
		const float dt_sync_s = 0.f;

		// Convert time sync to seconds
		// const float dt_sync_s = dt_sync_us / SEC2USEC;

		_target_estimator_orientation->syncState(dt_sync_s);
		_target_estimator_orientation->setH(target_orientation_obs.meas_h_theta);
		target_innov.innovation_variance = _target_estimator_orientation->computeInnovCov(
				target_orientation_obs.meas_unc_theta);
		target_innov.innovation = _target_estimator_orientation->computeInnov(target_orientation_obs.meas_theta);
		meas_fused = _target_estimator_orientation->update();

		// Fill the target innovation field
		target_innov.fusion_enabled = true;
		target_innov.innovation_rejected = !meas_fused;
		target_innov.fused = meas_fused;

		target_innov.observation = target_orientation_obs.meas_theta;
		target_innov.observation_variance = target_orientation_obs.meas_unc_theta;
		target_innov.timestamp_sample = target_orientation_obs.timestamp;
		target_innov.timestamp = hrt_absolute_time(); // TODO: check if correct hrt_absolute_time() or _last_predict

	} else {
		// No yaw measurement
		target_innov.fusion_enabled = false;
		target_innov.fused = false;
	}

	_target_estimator_aid_ev_yaw_pub.publish(target_innov);

	return meas_fused;
}


void LTEOrientation::publishTarget()
{
	landing_target_orientation_s target_orientation{};

	target_orientation.timestamp = _last_predict;
	target_orientation.rel_orientation_valid = (hrt_absolute_time() - _last_update < landing_target_valid_TIMEOUT_US);

	// TODO: get velocity if (_target_mode == TargetMode::Moving) {
	target_orientation.theta_rel = _target_estimator_orientation->getPosition();
	target_orientation.cov_theta_rel =  _target_estimator_orientation->getPosVar();

	target_orientation.v_theta_rel = _target_estimator_orientation->getVelocity();
	target_orientation.cov_v_theta_rel =  _target_estimator_orientation->getVelVar();

	if (_local_pos.valid) {

		target_orientation.abs_pos_valid = true;
		target_orientation.theta_abs = matrix::wrap_pi(target_orientation.theta_rel - _local_pos.heading);

	} else {
		target_orientation.abs_pos_valid = false;
	}

	_targetOrientationPub.publish(target_orientation);
}

void LTEOrientation::_check_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		updateParams();
	}
}

void LTEOrientation::update_topics()
{
	vehicle_local_position_s	vehicle_local_position;
	vehicle_status_s vehicle_status;
	position_setpoint_triplet_s pos_sp_triplet;
	vehicle_land_detected_s vehicle_land_detected;

	//Update topics
	bool vehicle_local_position_valid = _vehicleLocalPositionSub.update(&vehicle_local_position);
	bool pos_sp_triplet_valid = _pos_sp_triplet_sub.update(&pos_sp_triplet);

	// Update nav state
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_nave_state_mission = (vehicle_status.nav_state  == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	}

	// Stop computations once the drone has landed
	if (_start_filter && _vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
		PX4_INFO("Land detected, orientation target estimator stoped.");

		resetFilter();

		_start_filter = false;
		_land_time = hrt_absolute_time();

		return;
	}

	if (!_start_filter && ((hrt_absolute_time() - _land_time) > 5000000) && pos_sp_triplet_valid) {
		_start_filter = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);
	}

	// To save stack space, only use x,y,z,valid as global variables (_local_pos is used when the target is published)
	_local_pos.heading = vehicle_local_position.heading; // Already in [-Pi; Pi]
	_local_pos.valid = (vehicle_local_position_valid && vehicle_local_position.heading_good_for_control);
}

void LTEOrientation::updateParams()
{

	ModuleParams::updateParams();

	const TargetMode param_target_mode = (TargetMode)_param_ltest_mode.get();
	_estimate_orientation = _param_ltest_yaw_en.get();
	_yaw_unc = _param_ltest_yaw_unc_in.get();

	PX4_INFO("LTE orientation estimator enabled.");

	// TODO: add orientation
	if (_target_mode != param_target_mode) {

		// Define the target mode and model
		_target_mode = param_target_mode;

		if (!selectTargetEstimator()) {
			// TODO: decide on behaviour
		}

		// Define LTEST timeout
		_ltest_TIMEOUT_US = (uint32_t)(_param_ltest_btout.get() * SEC2USEC);
	}

	if ((_target_estimator_orientation == nullptr)) {
		// TODO: should return false
		return;
	}
}

bool LTEOrientation::selectTargetEstimator()
{
	TargetEstimatorOrientation *tmp_theta = nullptr;

	bool init_failed = true;

	if (_target_mode == TargetMode::Moving) {
		tmp_theta = new KF_orientation_moving;

	} else {
		tmp_theta = new KF_orientation_static;
	}

	init_failed = (tmp_theta == nullptr);

	if (init_failed) {
		PX4_ERR("LTE orientation init failed");
		return false;
		// TODO: decide on a behaviour

	} else {

		delete _target_estimator_orientation;
		_target_estimator_orientation = tmp_theta;

		return true;
	}
}

} // namespace landing_target_estimator
