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
 * @file VisionTargetEst.cpp
 * @brief Handles the position and orientation estimators.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>

#include "VisionTargetEst.h"
#include "common.h"

namespace vision_target_estimator
{

constexpr uint32_t kAccDownsampleTimeoutUs = 40_ms; // 40 ms -> 25Hz
constexpr uint32_t kEstRestartTimeUs = 3_s; // Wait at least 3 second before re-starting the filter
constexpr float kGravity = 9.80665f;  // m/s^2
constexpr uint32_t kAccUpdatedTimeoutUs = 20_ms; // TODO: check if we can lower it
constexpr float kMinGpsOffsetM = 0.01f; // Consider GNSS not offset below 1cm

VisionTargetEst::VisionTargetEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::vte)
{
	_vision_target_est_input_pub.advertise();

	_param_ekf2_gps_pos_x = param_find("EKF2_GPS_POS_X");
	_param_ekf2_gps_pos_y = param_find("EKF2_GPS_POS_Y");
	_param_ekf2_gps_pos_z = param_find("EKF2_GPS_POS_Z");
}

VisionTargetEst::~VisionTargetEst()
{
	perf_free(_cycle_perf_pos);
	perf_free(_cycle_perf_yaw);
	perf_free(_cycle_perf);
}

int VisionTargetEst::task_spawn(int argc, char *argv[])
{

#if !defined(CONFIG_MODULES_VISION_TARGET_ESTIMATOR) || !CONFIG_MODULES_VISION_TARGET_ESTIMATOR
	PX4_ERR("Vision Target Estimator cannot run without CONFIG_MODULES_VISION_TARGET_ESTIMATOR (missing topics).");
	return PX4_ERROR;
#endif // !CONFIG_MODULES_VISION_TARGET_ESTIMATOR

#if defined(CONFIG_MODULES_LANDING_TARGET_ESTIMATOR) && CONFIG_MODULES_LANDING_TARGET_ESTIMATOR
	PX4_ERR("Vision Target Estimator cannot run with CONFIG_MODULES_LANDING_TARGET_ESTIMATOR enabled.");
	return PX4_ERROR;
#endif // CONFIG_MODULES_LANDING_TARGET_ESTIMATOR

	VisionTargetEst *instance = new VisionTargetEst();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void VisionTargetEst::Run()
{
	if (should_exit()) {
		handleExit();
		return;
	}

	updateTaskTopics();

	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	// If a new task is available, stop the estimators if they were already running
	if (isNewTaskAvailable()) {
		stopAllEstimators();
	}

	// No task running, early return
	if (noActiveTask()) {
		return;
	}

	// Task is running, check if an estimator must be started or re-started
	startEstIfNeeded();

	const bool task_completed = isCurrentTaskComplete();

	if (task_completed) {
		stopAllEstimators();
		_current_task = VisionTargetEstTaskMaskU{};
		return;
	}

	if (noEstRunning()) {
		return;
	}

	if (allEstStoppedDueToTimeout()) {
		return;
	}

	// Early return checks passed, start filter computations.
	updateEstimators();
}

void VisionTargetEst::handleExit()
{
	_vehicle_attitude_sub.unregisterCallback();
	exit_and_cleanup();
}

bool VisionTargetEst::init()
{
	if (!_param_vte_pos_en.get() && !_param_vte_yaw_en.get()) {
		PX4_WARN("VTE not enabled, update VTE_POS_EN or VTE_YAW_EN");
		return false;
	}

	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("VTE vehicle_attitude callback registration failed!");
		return false;
	}

	_orientation_estimator_running = false;
	_position_estimator_running = false;

	_vte_position_enabled = false;
	_vte_orientation_enabled = false;

	// Params with reboot required
	if (_param_vte_pos_en.get()) {
		PX4_INFO("VTE position estimator enabled.");

		if (_vte_position.init()) {
			_vte_position_enabled = true;

		} else {
			PX4_ERR("VTE position estimator initialisation failed");
		}
	}

	if (_param_vte_yaw_en.get()) {
		PX4_INFO("VTE yaw estimator enabled.");

		if (_vte_orientation.init()) {
			_vte_orientation_enabled = true;

		} else {
			PX4_ERR("VTE yaw estimator initialisation failed");
		}
	}

	// Params that impact the estimators
	updateParams();

	return _vte_position_enabled || _vte_orientation_enabled;
}

void VisionTargetEst::updateParams()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	ModuleParams::updateParams();

	const uint8_t new_vte_task_mask = static_cast<uint8_t>(_param_vte_task_mask.get());

	if (new_vte_task_mask != _vte_task_mask.value) {

		_vte_task_mask.value = new_vte_task_mask;

		if (_vte_task_mask.value < 1) {
			PX4_WARN("VTE no task, update VTE_TASK_MASK");

		} else {
			PX4_INFO("VTE VTE_TASK_MASK config: ");

			if (_vte_task_mask.flags.for_prec_land) { PX4_INFO("    Precision landing"); }

			if (_vte_task_mask.flags.debug) { PX4_WARN("    DEBUG, always active"); }
		}
	}

	matrix::Vector3f gps_antenna_offset{};

	if (_param_ekf2_gps_pos_x != PARAM_INVALID) {
		param_get(_param_ekf2_gps_pos_x, &gps_antenna_offset(0));
	}

	if (_param_ekf2_gps_pos_y != PARAM_INVALID) {
		param_get(_param_ekf2_gps_pos_y, &gps_antenna_offset(1));
	}

	if (_param_ekf2_gps_pos_z != PARAM_INVALID) {
		param_get(_param_ekf2_gps_pos_z, &gps_antenna_offset(2));
	}

	_gps_pos_is_offset = (fabsf(gps_antenna_offset(0)) > kMinGpsOffsetM)
			     || (fabsf(gps_antenna_offset(1)) > kMinGpsOffsetM)
			     || (fabsf(gps_antenna_offset(2)) > kMinGpsOffsetM);
	_gps_pos_offset_xyz = gps_antenna_offset;

	const uint16_t requested_aid_mask = adjustAidMask(_param_vte_aid_mask.get());

	if (requested_aid_mask != _vte_aid_mask.value) {
		_vte_aid_mask.value = requested_aid_mask;

		if (_vte_position_enabled) {
			_vte_position.set_vte_aid_mask(requested_aid_mask);
		}

		if (_vte_orientation_enabled) {
			_vte_orientation.set_vte_aid_mask(requested_aid_mask);
		}

		printAidMask();
	}

	const uint32_t new_vte_timeout_us = static_cast<uint32_t>(_param_vte_btout.get() * 1_s);

	if (new_vte_timeout_us != _vte_timeout_us) {

		PX4_INFO("VTE timeout: %.1f [s] (previous: %.1f [s])", static_cast<double>(new_vte_timeout_us) / 1e6,
			 static_cast<double>(_vte_timeout_us) / 1e6);

		_vte_timeout_us = new_vte_timeout_us;

		if (_vte_position_enabled) {
			_vte_position.set_vte_timeout(new_vte_timeout_us);
		}

		if (_vte_orientation_enabled) {
			_vte_orientation.set_vte_timeout(new_vte_timeout_us);
		}
	}

	const uint32_t new_target_valid_timeout_us = static_cast<uint32_t>(_param_vte_tgt_tout.get() * 1_s);

	if (new_target_valid_timeout_us != _target_valid_timeout_us) {

		PX4_INFO("VTE target validity timeout: %.2f [s] (previous: %.2f [s])",
			 static_cast<double>(new_target_valid_timeout_us) / 1e6,
			 static_cast<double>(_target_valid_timeout_us) / 1e6);

		_target_valid_timeout_us = new_target_valid_timeout_us;

		if (_vte_position_enabled) {
			_vte_position.set_target_valid_timeout(new_target_valid_timeout_us);
		}

		if (_vte_orientation_enabled) {
			_vte_orientation.set_target_valid_timeout(new_target_valid_timeout_us);
		}
	}

	const uint32_t new_meas_recent_timeout_us = static_cast<uint32_t>(_param_vte_mrec_tout.get() * 1_s);

	if (new_meas_recent_timeout_us != _meas_recent_timeout_us) {

		PX4_INFO("VTE measurement recent timeout: %.2f [s] (previous: %.2f [s])",
			 static_cast<double>(new_meas_recent_timeout_us) / 1e6,
			 static_cast<double>(_meas_recent_timeout_us) / 1e6);

		_meas_recent_timeout_us = new_meas_recent_timeout_us;

		if (_vte_position_enabled) {
			_vte_position.set_meas_recent_timeout(new_meas_recent_timeout_us);
		}

		if (_vte_orientation_enabled) {
			_vte_orientation.set_meas_recent_timeout(new_meas_recent_timeout_us);
		}
	}

	const uint32_t new_meas_updated_timeout_us = static_cast<uint32_t>(_param_vte_mupd_tout.get() * 1_s);

	if (new_meas_updated_timeout_us != _meas_updated_timeout_us) {

		PX4_INFO("VTE measurement updated timeout: %.3f [s] (previous: %.3f [s])",
			 static_cast<double>(new_meas_updated_timeout_us) / 1e6,
			 static_cast<double>(_meas_updated_timeout_us) / 1e6);

		_meas_updated_timeout_us = new_meas_updated_timeout_us;

		if (_vte_position_enabled) {
			_vte_position.set_meas_updated_timeout(new_meas_updated_timeout_us);
		}

		if (_vte_orientation_enabled) {
			_vte_orientation.set_meas_updated_timeout(new_meas_updated_timeout_us);
		}
	}

	const float new_pos_update_rate_hz = _param_vte_pos_rate.get();

	if (PX4_ISFINITE(new_pos_update_rate_hz) && new_pos_update_rate_hz > 0.f) {

		hrt_abstime new_pos_update_period_us = static_cast<hrt_abstime>(fmaxf(roundf(1e6f / new_pos_update_rate_hz), 1.f));

		if ((fabsf(new_pos_update_rate_hz - _pos_update_rate_hz) > FLT_EPSILON)
		    || (new_pos_update_period_us != _pos_update_period_us)) {
			PX4_INFO("VTE position update rate: %.1f Hz (previous: %.1f Hz)",
				 static_cast<double>(new_pos_update_rate_hz),
				 static_cast<double>(_pos_update_rate_hz));
		}

		_pos_update_rate_hz = new_pos_update_rate_hz;
		_pos_update_period_us = new_pos_update_period_us;

	} else {
		PX4_WARN("VTE_POS_RATE %.2f invalid, keeping previous value",
			 static_cast<double>(new_pos_update_rate_hz));
	}

	const float new_yaw_update_rate_hz = _param_vte_yaw_rate.get();

	if (PX4_ISFINITE(new_yaw_update_rate_hz) && new_yaw_update_rate_hz > 0.f) {

		hrt_abstime new_yaw_update_period_us = static_cast<hrt_abstime>(fmaxf(roundf(1e6f / new_yaw_update_rate_hz), 1.f));

		if ((fabsf(new_yaw_update_rate_hz - _yaw_update_rate_hz) > FLT_EPSILON)
		    || (new_yaw_update_period_us != _yaw_update_period_us)) {
			PX4_INFO("VTE yaw update rate: %.1f Hz (previous: %.1f Hz)",
				 static_cast<double>(new_yaw_update_rate_hz),
				 static_cast<double>(_yaw_update_rate_hz));
		}

		_yaw_update_rate_hz = new_yaw_update_rate_hz;
		_yaw_update_period_us = new_yaw_update_period_us;

	} else {
		PX4_WARN("VTE_YAW_RATE %.2f invalid, keeping previous value",
			 static_cast<double>(new_yaw_update_rate_hz));
	}

	if (_vte_position_enabled && _position_estimator_running
	    && !_vte_position.fusionEnabled()) {
		PX4_INFO("VTE position estimator stopped, no fusion source selected.");
		stopPosEst();
	}

	if (_vte_orientation_enabled && _orientation_estimator_running
	    && !_vte_orientation.fusionEnabled()) {
		PX4_INFO("VTE yaw estimator stopped, no fusion source selected.");
		stopYawEst();
	}
}

uint16_t VisionTargetEst::adjustAidMask(const int input_vte_aid_mask)
{
	SensorFusionMaskU new_aid_mask{};
	new_aid_mask.value = input_vte_aid_mask;

#if defined(CONFIG_VTEST_MOVING)

	if (new_aid_mask.flags.use_mission_pos) {
		PX4_WARN("VTE for moving target. Disabling mission land position data fusion.");
		new_aid_mask.flags.use_mission_pos = false;
	}

#endif // CONFIG_VTEST_MOVING

	if (new_aid_mask.flags.use_target_gps_pos && new_aid_mask.flags.use_mission_pos) {
		PX4_WARN("VTE both target GPS position and mission land position data fusion cannot be enabled simultaneously.");
		PX4_WARN("Disabling mission land position fusion.");
		new_aid_mask.flags.use_mission_pos = false;
	}

	return new_aid_mask.value;
}

void VisionTargetEst::printAidMask()
{
	PX4_INFO("VTE VTE_AID_MASK config: ");

	if (_vte_aid_mask.flags.use_vision_pos) {PX4_INFO("    vision relative position fusion enabled");}

	if (_vte_aid_mask.flags.use_target_gps_pos) {PX4_INFO("    target GPS position fusion enabled");}

	if (_vte_aid_mask.flags.use_target_gps_vel) {PX4_INFO("    target GPS velocity fusion enabled");}

	if (_vte_aid_mask.flags.use_mission_pos) {PX4_INFO("    mission land position fusion enabled");}

	if (_vte_aid_mask.flags.use_uav_gps_vel) {PX4_INFO("    UAV GPS velocity fusion enabled");}


	if (_vte_aid_mask.value == 0) {PX4_WARN("    no data fusion. Modify VTE_AID_MASK");}
}

void VisionTargetEst::resetAccDownsample()
{
	_vehicle_acc_ned_sum.setAll(0);
	_acc_sample_count = 0;
	_last_acc_reset = hrt_absolute_time();
}

bool VisionTargetEst::startYawEst()
{
	if (!_vte_orientation_enabled) {
		return false;
	}

	if (!hasTimedOut(_vte_orientation_stop_time, kEstRestartTimeUs)) {
		return false;
	}

	PX4_INFO("Starting Orientation Vision Target Estimator.");
	_vte_orientation.resetFilter();
	return true;
}

bool VisionTargetEst::startPosEst()
{
	if (!_vte_position_enabled) {
		return false;
	}

	if (!hasTimedOut(_vte_position_stop_time, kEstRestartTimeUs)) {
		return false;
	}

	resetAccDownsample();

	PX4_INFO("Starting Position Vision Target Estimator.");
	_vte_position.resetFilter();

	if (_current_task.flags.for_prec_land) {
		if (const position_setpoint_s *land_setpoint = findLandSetpoint()) {
			_vte_position.set_mission_position(land_setpoint->lat, land_setpoint->lon, land_setpoint->alt);

		} else {
			PX4_WARN("VTE for precision landing, land position cannot be used.");
			_vte_position.set_mission_position(0.0, 0.0, NAN);
		}
	}

	return true;
}

const position_setpoint_s *VisionTargetEst::findLandSetpoint()
{
	if (!_pos_sp_triplet_sub.update(&_pos_sp_triplet_buffer)) {
		return nullptr;
	}

	if (_pos_sp_triplet_buffer.next.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		PX4_INFO("VTE for precision landing, next sp is land.");
		return &_pos_sp_triplet_buffer.next;
	}

	if (_pos_sp_triplet_buffer.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
		PX4_INFO("VTE for precision landing, current sp is land.");
		return &_pos_sp_triplet_buffer.current;
	}

	return nullptr;
}

bool VisionTargetEst::isNewTaskAvailable()
{
	if (_vte_task_mask.flags.for_prec_land && _is_in_prec_land) {

		// Precision land task already running
		if (_current_task.flags.for_prec_land) {
			return false;
		}

		PX4_INFO("VTE, precision landing task requested.");
		VisionTargetEstTaskMaskU new_task{};
		new_task.flags.for_prec_land = 1;
		_current_task = new_task;
		return true;

	} else if (_vte_task_mask.flags.debug) {

		// DEBUG task already running
		if (_current_task.flags.debug) {
			return false;
		}

		PX4_WARN("VTE, DEBUG task requested.");
		VisionTargetEstTaskMaskU new_task{};
		new_task.flags.debug = 1;
		_current_task = new_task;
		return true;
	}

	// To add a new task:
	// else if ((_vte_task_mask.value & <new task bit>) && _is_in_new_task) {...}

	return false;
}

bool VisionTargetEst::isCurrentTaskComplete()
{
	// Prec-land
	if (_current_task.flags.for_prec_land) {

		if (!_vte_task_mask.flags.for_prec_land) {
			PX4_INFO("VTE_TASK_MASK updated, precision landing task completed.");
			return true;
		}

		vehicle_land_detected_s vehicle_land_detected;

		// Stop computations once the drone has landed
		if (_vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
			PX4_INFO("Land detected, precision landing task completed.");
			_is_in_prec_land = false;
			return true;
		}

		// Stop computations once precision landing is over
		if (!_is_in_prec_land) {
			PX4_INFO("Precision landing task completed.");
			return true;
		}

	} else if (_current_task.flags.debug) {

		if (!_vte_task_mask.flags.debug) {
			PX4_INFO("VTE_TASK_MASK updated, DEBUG task completed.");
			return true;
		}
	}

	// The structure allows to add additional tasks here E.g. precision delivery, follow me, precision takeoff.

	return false;
}

void VisionTargetEst::updateTaskTopics()
{
	// The structure allows to add additional tasks status here E.g. precision delivery, follow me, precision takeoff.

#if !defined(CONSTRAINED_FLASH)

	if (_vte_task_mask.flags.for_prec_land) {
		prec_land_status_s prec_land_status;

		if (_prec_land_status_sub.update(&prec_land_status)) {
			_is_in_prec_land = prec_land_status.state == prec_land_status_s::PREC_LAND_STATE_ONGOING;
		}
	}

#endif
}

void VisionTargetEst::stopAllEstimators()
{
	if (_vte_position_enabled && _position_estimator_running) {
		stopPosEst();
	}

	if (_vte_orientation_enabled && _orientation_estimator_running) {
		stopYawEst();
	}
}

void VisionTargetEst::stopPosEst()
{

	PX4_INFO("Stopping Position Vision Target Estimator.");

	if (_vte_position_enabled) {
		_vte_position.resetFilter();
	}

	_position_estimator_running = false;
	_vte_position_stop_time = hrt_absolute_time();
}

void VisionTargetEst::stopYawEst()
{

	PX4_INFO("Stopping Orientation Vision Target Estimator.");

	if (_vte_orientation_enabled) {
		_vte_orientation.resetFilter();
	}

	_orientation_estimator_running = false;
	_vte_orientation_stop_time = hrt_absolute_time();
}

void VisionTargetEst::startEstIfNeeded()
{

	// Only start the estimator if fusion is enabled, otherwise it will timeout
	if (!_position_estimator_running && _vte_position_enabled && _vte_position.fusionEnabled()) {
		_position_estimator_running = startPosEst();
	}

	if (!_orientation_estimator_running && _vte_orientation_enabled && _vte_orientation.fusionEnabled()) {
		_orientation_estimator_running = startYawEst();
	}
}

bool VisionTargetEst::allEstStoppedDueToTimeout()
{
	bool timed_out = false;

	if (_position_estimator_running && _vte_position.timedOut()) {
		stopPosEst();
		PX4_WARN("Estimator TIMEOUT, position VTE stopped.");
		timed_out = true;
	}

	if (_orientation_estimator_running && _vte_orientation.timedOut()) {
		stopYawEst();
		PX4_WARN("Estimator TIMEOUT, orientation VTE stopped.");
		timed_out = true;
	}

	return timed_out && !_position_estimator_running && !_orientation_estimator_running;
}

void VisionTargetEst::updateEstimators()
{
	perf_begin(_cycle_perf);

	LocalPose local_pose;
	const bool local_pose_updated = pollLocalPose(local_pose);

	matrix::Vector3f vel_offset_body{};
	const bool vel_offset_updated = computeGpsVelocityOffset(vel_offset_body);

	matrix::Vector3f vehicle_acc_ned{};
	matrix::Quaternionf q_att{};
	matrix::Vector3f gps_pos_offset_ned{};
	matrix::Vector3f vel_offset_ned = vel_offset_body;
	bool acc_valid = false;

	// Minimal requirements: attitude (always) and acceleration for position estimator
	if (!pollEstimatorInput(vehicle_acc_ned, q_att, gps_pos_offset_ned, vel_offset_ned, vel_offset_updated, acc_valid)) {
		perf_end(_cycle_perf);
		return;
	}

	_last_att = q_att;

	if (_vte_position_enabled && _position_estimator_running) {
		if (acc_valid) {
			updatePosEst(local_pose, local_pose_updated, vehicle_acc_ned,
				     gps_pos_offset_ned, vel_offset_ned, vel_offset_updated);

		} else {
			if ((_vehicle_acc_body.timestamp != 0) && (hrt_elapsed_time(&_acc_sample_warn_last) > kWarnThrottleIntervalUs)) {
				PX4_WARN("VTE acc sample stale (%.1f ms)",
					 static_cast<double>((hrt_absolute_time() - _vehicle_acc_body.timestamp) / 1e3));
				_acc_sample_warn_last = hrt_absolute_time();
			}

			resetAccDownsample();
		}
	}

	if (_vte_orientation_enabled && _orientation_estimator_running) {
		updateYawEst(local_pose, local_pose_updated);
	}

	perf_end(_cycle_perf);
}

void VisionTargetEst::updatePosEst(const LocalPose &local_pose, const bool local_pose_updated,
				   const matrix::Vector3f &vehicle_acc_ned,
				   const matrix::Vector3f &gps_pos_offset_ned,
				   const matrix::Vector3f &vel_offset_ned,
				   const bool vel_offset_updated)
{
	/* If the acceleration has been averaged for too long, early return */
	if (hasTimedOut(_last_acc_reset, kAccDownsampleTimeoutUs)) {
		PX4_DEBUG("Forced acc downsample reset");
		resetAccDownsample();
		return;
	}

	_vehicle_acc_ned_sum += vehicle_acc_ned;
	_acc_sample_count++;

	if (!updateWhenIntervalElapsed(_last_update_pos, _pos_update_period_us)) {
		return;
	}

	perf_begin(_cycle_perf_pos);

	if (local_pose_updated) {
		_vte_position.set_local_velocity(local_pose.vel_xyz, local_pose.vel_valid, local_pose.timestamp);
		_vte_position.set_local_position(local_pose.xyz, local_pose.pos_valid, local_pose.timestamp);
		_vte_position.set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid, local_pose.timestamp);
	}

	_vte_position.set_gps_pos_offset(gps_pos_offset_ned, _gps_pos_is_offset);

	if (vel_offset_updated) {
		_vte_position.set_vel_offset(vel_offset_ned);
	}

	const matrix::Vector3f vehicle_acc_ned_sampled = _vehicle_acc_ned_sum / _acc_sample_count;

	_vte_position.update(vehicle_acc_ned_sampled);
	publishVteInput(vehicle_acc_ned_sampled, _last_att);

	resetAccDownsample();

	perf_end(_cycle_perf_pos);
}

void VisionTargetEst::updateYawEst(const LocalPose &local_pose, const bool local_pose_updated)
{

	if (!updateWhenIntervalElapsed(_last_update_yaw, _yaw_update_period_us)) {
		return;
	}

	perf_begin(_cycle_perf_yaw);

	if (local_pose_updated) {
		_vte_orientation.set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid, local_pose.timestamp);
	}

	_vte_orientation.update();
	perf_end(_cycle_perf_yaw);
}

void VisionTargetEst::publishVteInput(const matrix::Vector3f &vehicle_acc_ned_sampled,
				      const matrix::Quaternionf &q_att_sampled)
{
	vision_target_est_input_s vte_input_report{};
	vte_input_report.timestamp = hrt_absolute_time();
	vte_input_report.timestamp_sample = _vehicle_acc_body.timestamp;

	for (int i = 0; i < 3; i++) {
		vte_input_report.acc_xyz[i] = vehicle_acc_ned_sampled(i);
	}

	for (int i = 0; i < 4; i++) {
		vte_input_report.q_att[i] = q_att_sampled(i);
	}

	_vision_target_est_input_pub.publish(vte_input_report);
}

bool VisionTargetEst::computeGpsVelocityOffset(matrix::Vector3f &vel_offset_body)
{
	if (!_gps_pos_is_offset) {
		return false;
	}

	vehicle_angular_velocity_s vehicle_angular_velocity;

	if (!_vehicle_angular_velocity_sub.update(&vehicle_angular_velocity)) {
		return false;
	}

	// If the GPS antenna is not at the center of mass, when the drone rotates around the center of mass, the GPS will record a velocity.
	const matrix::Vector3f ang_vel = matrix::Vector3f(vehicle_angular_velocity.xyz);
	vel_offset_body = ang_vel % _gps_pos_offset_xyz; // Get extra velocity from drone's rotation

	return true;
}

bool VisionTargetEst::pollLocalPose(LocalPose &local_pose)
{

	vehicle_local_position_s vehicle_local_position;

	if (!_vehicle_local_position_sub.update(&vehicle_local_position)) {
		return false;
	}

	local_pose.xyz(0) = vehicle_local_position.x;
	local_pose.xyz(1) = vehicle_local_position.y;
	local_pose.xyz(2) = vehicle_local_position.z;
	local_pose.pos_valid = vehicle_local_position.xy_valid && vehicle_local_position.z_valid;

	local_pose.vel_xyz(0) = vehicle_local_position.vx;
	local_pose.vel_xyz(1) = vehicle_local_position.vy;
	local_pose.vel_xyz(2) = vehicle_local_position.vz;
	local_pose.vel_valid = vehicle_local_position.v_xy_valid && vehicle_local_position.v_z_valid;

	local_pose.dist_bottom = vehicle_local_position.dist_bottom;
	local_pose.dist_valid = vehicle_local_position.dist_bottom_valid;

	local_pose.yaw_valid = vehicle_local_position.heading_good_for_control;
	local_pose.yaw = vehicle_local_position.heading;

	local_pose.timestamp = vehicle_local_position.timestamp;

	return true;
}

bool VisionTargetEst::pollEstimatorInput(matrix::Vector3f &vehicle_acc_ned,
		matrix::Quaternionf &quat_att,
		matrix::Vector3f &gps_pos_offset_ned,
		matrix::Vector3f &vel_offset_ned,
		const bool vel_offset_updated,
		bool &acc_valid)
{
	vehicle_attitude_s vehicle_attitude{};
	vehicle_acceleration_s vehicle_acceleration{};

	const bool vehicle_attitude_updated = _vehicle_attitude_sub.update(&vehicle_attitude);
	const bool vehicle_acceleration_updated = _vehicle_acceleration_sub.update(&vehicle_acceleration);

	if (!vehicle_attitude_updated) {
		return false;
	}

	if (vehicle_acceleration_updated) {
		_vehicle_acc_body.xyz = matrix::Vector3f(vehicle_acceleration.xyz);
		_vehicle_acc_body.timestamp = vehicle_acceleration.timestamp;
	}

	_vehicle_acc_body.valid = (_vehicle_acc_body.timestamp != 0)
				  && !hasTimedOut(_vehicle_acc_body.timestamp, kAccUpdatedTimeoutUs);
	acc_valid = _vehicle_acc_body.valid;

	/* Transform FRD body acc to NED */
	const matrix::Quaternionf q_att(&vehicle_attitude.q[0]); // (w,x,y,z)
	quat_att = q_att;

	if (acc_valid) {
		/* Compensate for gravity. */
		const matrix::Vector3f gravity_ned(0, 0, kGravity);
		vehicle_acc_ned = quat_att.rotateVector(_vehicle_acc_body.xyz) + gravity_ned;

	} else {
		vehicle_acc_ned.setAll(0.f);
	}

	/* Rotate position and velocity offset into ned frame */
	if (_gps_pos_is_offset) {
		gps_pos_offset_ned = quat_att.rotateVector(_gps_pos_offset_xyz);

		if (vel_offset_updated) {
			vel_offset_ned = quat_att.rotateVector(vel_offset_ned);

		}

	} else {
		gps_pos_offset_ned.setAll(0.f);
		vel_offset_ned.setAll(0.f);
	}

	return true;
}

bool VisionTargetEst::updateWhenIntervalElapsed(hrt_abstime &last_time, const hrt_abstime interval) const
{

	if (hrt_elapsed_time(&last_time) > interval) {
		last_time = hrt_absolute_time();
		return true;
	}

	return false;
}

int VisionTargetEst::print_status()
{
	PX4_INFO("VTE running");
	return 0;
}
int VisionTargetEst::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to estimate the position and orientation of a target using relative sensors.

The module runs periodically on the HP work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vision_target_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vision_target_estimator_main(int argc, char *argv[])
{
	return VisionTargetEst::main(argc, argv);
}

} // namespace vision_target_est
