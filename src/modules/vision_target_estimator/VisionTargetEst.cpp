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


namespace vision_target_estimator
{

static constexpr uint32_t vte_pos_UPDATE_RATE_HZ = 50;
static constexpr uint32_t vte_yaw_UPDATE_RATE_HZ = 50;
static constexpr uint32_t acc_downsample_TIMEOUT_US = 40_ms; // 40 ms -> 25Hz
static constexpr uint32_t estimator_restart_time_US = 3_s; // Wait at least one second before re-starting the filter
static constexpr float CONSTANTS_ONE_G = 9.80665f;  // m/s^2

VisionTargetEst::VisionTargetEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::vte)
{
	_vte_acc_input_pub.advertise();
}

VisionTargetEst::~VisionTargetEst()
{
	perf_free(_cycle_perf_pos);
	perf_free(_cycle_perf_yaw);
	perf_free(_cycle_perf);
}

int VisionTargetEst::task_spawn(int argc, char *argv[])
{

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

bool VisionTargetEst::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	updateParams();

	delete _vte_position;
	delete _vte_orientation;

	_vte_position_valid = false;
	_vte_orientation_valid = false;

	// Structure allows for different tasks using the VTE in the future.
	_vte_task_mask = _param_vte_task_mask.get();

	if (_vte_task_mask < 1) {
		PX4_ERR("VTE invalid task mask, target estimator not enabled.");
		return false;

	} else {

		if (_vte_task_mask & VisionTargetEstTask::VTE_FOR_PREC_LAND) { PX4_INFO("VTE for precision landing.");}
	}

	if (_param_vte_pos_en.get()) {
		PX4_INFO("VTE position estimator enabled.");
		_vte_position = new VTEPosition;
		_vte_position_valid = (_vte_position != nullptr && _vte_position->init());
	}

	if (_param_vte_yaw_en.get()) {
		PX4_INFO("VTE yaw estimator enabled.");
		_vte_orientation = new VTEOrientation;
		_vte_orientation_valid = (_vte_orientation != nullptr && _vte_orientation->init());
	}

	return _vte_position_valid || _vte_orientation_valid;
}

void VisionTargetEst::updateParams()
{
	ModuleParams::updateParams();

	_vte_task_mask = _param_vte_task_mask.get();

	float gps_pos_x;
	param_get(param_find("EKF2_GPS_POS_X"), &gps_pos_x);

	float gps_pos_y;
	param_get(param_find("EKF2_GPS_POS_Y"), &gps_pos_y);

	float gps_pos_z;
	param_get(param_find("EKF2_GPS_POS_Z"), &gps_pos_z);

	_gps_pos_is_offset = ((gps_pos_x > 0.01f) || (gps_pos_y > 0.01f) || (gps_pos_z > 0.01f));
	_gps_pos_offset = matrix::Vector3f(gps_pos_x, gps_pos_y, gps_pos_z);
}

void VisionTargetEst::reset_acc_downsample()
{
	_vehicle_acc_ned_sum.setAll(0);
	_loops_count = 0;
	_last_acc_reset = hrt_absolute_time();
}

void VisionTargetEst::start_estimators()
{

	PX4_INFO("Starting Vision Target Estimator (VTE).");

	if (_vte_position_valid) {

		if (_vte_task_mask & VisionTargetEstTask::VTE_FOR_PREC_LAND) {

			bool next_sp_is_land = false;
			bool current_sp_is_land = false;

			position_setpoint_triplet_s pos_sp_triplet;

			if (_pos_sp_triplet_sub.update(&pos_sp_triplet)) {
				next_sp_is_land = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);
				current_sp_is_land = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);
			}

			if (next_sp_is_land) {
				PX4_INFO("VTE for precision landing, next sp is land.");
				_vte_position->set_mission_position(pos_sp_triplet.next.lat, pos_sp_triplet.next.lon,
								    pos_sp_triplet.next.alt);

			} else if (current_sp_is_land) {
				PX4_INFO("VTE for precision landing, current sp is land.");
				_vte_position->set_mission_position(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon,
								    pos_sp_triplet.current.alt);

			} else {
				PX4_WARN("VTE for precision landing, land position cannot be used.");
				_vte_position->set_mission_position(0.0, 0.0, NAN);
			}
		}
	}

	reset_acc_downsample();
	_estimators_started = true;
}

bool VisionTargetEst::must_start_estimators()
{
	// If we enter this function, the estimator is not running. Current task must be 0.
	_vte_current_task = 0;

	// Start target estimator for precision landing
	if (_vte_task_mask & VisionTargetEstTask::VTE_FOR_PREC_LAND) {
		if (_is_in_prec_land && (hrt_absolute_time() - _vte_stop_time) > estimator_restart_time_US) {
			_vte_current_task += VisionTargetEstTask::VTE_FOR_PREC_LAND;
			return true;
		}
	}

	return false;
}

bool VisionTargetEst::must_stop_estimators()
{

	// Stop computations if the position estimator timedout
	if (_vte_position->has_timed_out()) {
		PX4_INFO("TIMEOUT, target estimator stopped.");
		return true;
	}

	// Allow to keep the estimator running while on the ground if not used for precision landing. E.g. for precision takeoff.
	if (_vte_current_task & VisionTargetEstTask::VTE_FOR_PREC_LAND) {
		vehicle_land_detected_s vehicle_land_detected;

		// Stop computations once the drone has landed
		if (_vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
			PX4_INFO("Land detected, target estimator stopped.");
			return true;
		}

		// Stop computations once precision landing is over
		if (!_is_in_prec_land) {
			PX4_INFO("Precision operation finished, target estimator stopped.");
			return true;
		}
	}

	return false;
}

void VisionTargetEst::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

#if !defined(CONSTRAINED_FLASH)
	if (_vte_task_mask & VisionTargetEstTask::VTE_FOR_PREC_LAND) {
		prec_land_status_s prec_land_status;

		if (_prec_land_status_sub.update(&prec_land_status)) {
			_is_in_prec_land = prec_land_status.state == prec_land_status_s::PREC_LAND_STATE_ONGOING;
		}
	}
#endif

	// Check if estimator must be started
	if (!_estimators_started) {

		if (must_start_estimators()) {
			reset_estimators(); // Force a reset before starting
			start_estimators(); // Will set _estimators_started to true
		}

		return;
	}

	perf_begin(_cycle_perf);

	if (must_stop_estimators()) {
		_vte_stop_time = hrt_absolute_time();
		reset_estimators();
		return;
	}

	localPose local_pose;

	const bool local_pose_updated = get_local_pose(local_pose);

	/* Update position filter at vte_pos_UPDATE_RATE_HZ */
	if (_vte_position_valid) {

		matrix::Vector3f gps_pos_offset_ned;
		matrix::Vector3f vel_offset;
		const bool vel_offset_updated = get_gps_velocity_offset(vel_offset);

		matrix::Vector3f vehicle_acc_ned;

		/* Downsample acceleration ned */
		if (get_input(vehicle_acc_ned, gps_pos_offset_ned, vel_offset, vel_offset_updated)) {

			/* If the acceleration has been averaged for too long, early return */
			if ((hrt_absolute_time() - _last_acc_reset) > acc_downsample_TIMEOUT_US) {
				PX4_INFO("Forced acc downsample reset");
				reset_acc_downsample();
				return;
			}

			_vehicle_acc_ned_sum += vehicle_acc_ned;
			_loops_count ++;

			if ((hrt_absolute_time() - _last_update_pos) > (1_s / vte_pos_UPDATE_RATE_HZ)) {

				perf_begin(_cycle_perf_pos);

				if (local_pose_updated) {
					_vte_position->set_local_velocity(local_pose.vel_xyz, local_pose.vel_valid, local_pose.timestamp);
					_vte_position->set_local_position(local_pose.xyz, local_pose.pos_valid, local_pose.timestamp);
					_vte_position->set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid, local_pose.timestamp);
				}

				_vte_position->set_gps_pos_offset(gps_pos_offset_ned, _gps_pos_is_offset);

				if (vel_offset_updated) {
					_vte_position->set_velocity_offset(vel_offset);
				}

				const matrix::Vector3f vehicle_acc_ned_sampled = _vehicle_acc_ned_sum / _loops_count;

				_vte_position->update(vehicle_acc_ned_sampled);
				_last_update_pos = hrt_absolute_time();

				/* Publish downsampled acceleration*/
				vehicle_acceleration_s vte_acc_input_report;
				vte_acc_input_report.timestamp = hrt_absolute_time();

				for (int i = 0; i < 3; i++) {
					vte_acc_input_report.xyz[i] = vehicle_acc_ned_sampled(i);
				}

				_vte_acc_input_pub.publish(vte_acc_input_report);

				reset_acc_downsample();
				perf_end(_cycle_perf_pos);
			}
		}
	}

	/* Update orientation filter at vte_yaw_UPDATE_RATE_HZ */
	if (_vte_orientation_valid && ((hrt_absolute_time() - _last_update_yaw) > (1_s / vte_yaw_UPDATE_RATE_HZ))) {
		perf_begin(_cycle_perf_yaw);

		if (local_pose_updated) {
			_vte_orientation->set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid);
		}

		_vte_orientation->update();
		_last_update_yaw = hrt_absolute_time();
		perf_end(_cycle_perf_yaw);
	}

	perf_end(_cycle_perf);

}

void VisionTargetEst::reset_estimators()
{
	if (_vte_orientation_valid) {
		_vte_orientation->resetFilter();
	}

	if (_vte_position_valid) {
		_vte_position->resetFilter();
	}

	_vte_current_task = 0;
	_is_in_prec_land = false;
	_estimators_started = false;

}

bool VisionTargetEst::get_gps_velocity_offset(matrix::Vector3f &vel_offset_body)
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
	vel_offset_body = ang_vel % _gps_pos_offset; // Get extra velocity from drone's rotation

	return true;
}

bool VisionTargetEst::get_local_pose(localPose &local_pose)
{

	vehicle_local_position_s vehicle_local_position;

	if (!_vehicle_local_position_sub.update(&vehicle_local_position)) {
		return false;
	}

	if ((hrt_absolute_time() - vehicle_local_position.timestamp) > 100_ms) {
		PX4_WARN("Local position too old.");
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

bool VisionTargetEst::get_input(matrix::Vector3f &vehicle_acc_ned, matrix::Vector3f &gps_pos_offset_ned,
				matrix::Vector3f &vel_offset_rot_ned,
				const bool vel_offset_updated)
{

	vehicle_attitude_s	vehicle_attitude;
	vehicle_acceleration_s	vehicle_acceleration;

	const bool vehicle_attitude_updated = _vehicle_attitude_sub.update(&vehicle_attitude);
	const bool vehicle_acceleration_updated = _vehicle_acceleration_sub.update(&vehicle_acceleration);

	// Minimal requirement: acceleraion (for input) and attitude (to rotate acc in vehicle-carried NED frame)
	if (!vehicle_attitude_updated || !vehicle_acceleration_updated) {
		return false;
	}

	/* Transform FRD body acc to NED */
	const matrix::Quaternionf quat_att(&vehicle_attitude.q[0]);
	const matrix::Vector3f vehicle_acc{vehicle_acceleration.xyz};

	/* Compensate for gravity. */
	const matrix::Vector3f gravity_ned(0, 0, CONSTANTS_ONE_G);
	vehicle_acc_ned = quat_att.rotateVector(vehicle_acc) + gravity_ned;

	/* Rotate position and velocity offset into ned frame */
	if (_gps_pos_is_offset) {
		gps_pos_offset_ned = quat_att.rotateVector(_gps_pos_offset);

		if (vel_offset_updated) {
			vel_offset_rot_ned = quat_att.rotateVector(vel_offset_rot_ned);
		}

	} else {
		gps_pos_offset_ned.setAll(0.f);
		vel_offset_rot_ned.setAll(0.f);
	}

	return true;
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
Module to estimate the position and orientation of a target using a vision sensor and GNSS.

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
