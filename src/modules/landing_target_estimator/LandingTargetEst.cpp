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
 * @file LandingTargetEst.cpp
 * Landing target estimator. Filter and publish the position of a landing target on the ground as observed by an onboard sensor.
 *
 * @author Jonas Perolini <jonas.perolini@epfl.ch>
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

#include "LandingTargetEst.h"


namespace landing_target_estimator
{

static constexpr uint32_t ltest_pos_UPDATE_RATE_HZ = 50;
static constexpr uint32_t ltest_yaw_UPDATE_RATE_HZ = 50;
static constexpr uint32_t acc_downsample_TIMEOUT_US = 40000; // 40 ms -> 25Hz
static constexpr float CONSTANTS_ONE_G = 9.80665f;  // m/s^2

LandingTargetEst::LandingTargetEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_ltest_acc_input_pub.advertise();
}

LandingTargetEst::~LandingTargetEst()
{
	perf_free(_cycle_perf_pos);
	perf_free(_cycle_perf_yaw);
	perf_free(_cycle_perf);
}

int LandingTargetEst::task_spawn(int argc, char *argv[])
{

	LandingTargetEst *instance = new LandingTargetEst();

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

bool LandingTargetEst::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed!");
		return false;
	}

	updateParams();

	delete _ltest_position;
	delete _ltest_orientation;

	_ltest_position_valid = false;
	_ltest_orientation_valid = false;

	if (_param_ltest_pos_en.get()) {
		PX4_INFO("LTEst position estimator enabled.");
		_ltest_position = new LTEstPosition;
		_ltest_position_valid = (_ltest_position != nullptr && _ltest_position->init());
	}

	if (_param_ltest_yaw_en.get()) {
		PX4_INFO("LTEst yaw estimator enabled.");
		_ltest_orientation = new LTEstOrientation;
		_ltest_orientation_valid = (_ltest_orientation != nullptr && _ltest_orientation->init());
	}

	return _ltest_position_valid || _ltest_orientation_valid;
}

void LandingTargetEst::updateParams()
{
	ModuleParams::updateParams();
}

void LandingTargetEst::reset_acc_downsample()
{
	_vehicle_acc_ned_sum.setAll(0);
	_loops_count = 0;
	_last_acc_reset = hrt_absolute_time();
}

void LandingTargetEst::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (!_start_filters) {

		position_setpoint_triplet_s pos_sp_triplet;

		if (((hrt_absolute_time() - _land_time) > 5000000) && _pos_sp_triplet_sub.update(&pos_sp_triplet)) {
			_start_filters = (pos_sp_triplet.next.type == position_setpoint_s::SETPOINT_TYPE_LAND);

			if (_start_filters) {

				reset_acc_downsample();

				if (_ltest_position_valid) {
					// TODO: do we want to enable this feature only in mission mode? (_vehicle_status_sub.update(&vehicle_status) && (vehicle_status.nav_state  == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
					_ltest_position->set_landpoint((int)(pos_sp_triplet.next.lat * 1e7), (int)(pos_sp_triplet.next.lon * 1e7),
								       pos_sp_triplet.next.alt * 1000.f);
				}
			}
		}

	} else {

		perf_begin(_cycle_perf);

		vehicle_land_detected_s vehicle_land_detected;

		// Stop computations once the drone has landed
		if (_vehicle_land_detected_sub.update(&vehicle_land_detected) && vehicle_land_detected.landed) {
			PX4_INFO("Land detected, target estimator stoped.");
			_land_time = hrt_absolute_time();
			reset_filters();

			return;
		}

		localPose local_pose;

		bool local_pose_updated = get_local_pose(local_pose);

		/* Update position filter at ltest_pos_UPDATE_RATE_HZ */
		if (_ltest_position_valid) {

			matrix::Vector3f vehicle_acc_ned;

			/* Downsample acceleration ned */
			if (get_input(vehicle_acc_ned)) {

				/* If the acceleration has been averaged for too long, early return */
				if ((hrt_absolute_time() - _last_acc_reset) > acc_downsample_TIMEOUT_US) {
					PX4_INFO("Forced acc downsample reset");
					reset_acc_downsample();
					return;
				}

				_vehicle_acc_ned_sum += vehicle_acc_ned;
				_loops_count ++;

				if ((hrt_absolute_time() - _last_update_pos) > (1000000 / ltest_pos_UPDATE_RATE_HZ)) {
					perf_begin(_cycle_perf_pos);

					if (local_pose_updated) {
						_ltest_position->set_local_position(local_pose.xyz, local_pose.pos_valid);
						_ltest_position->set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid);
					}

					matrix::Vector3f vehicle_acc_ned_sampled = _vehicle_acc_ned_sum / _loops_count;

					_ltest_position->update(vehicle_acc_ned_sampled);
					_last_update_pos = hrt_absolute_time();

					/* Publish downsampled acceleration*/
					vehicle_acceleration_s ltest_acc_input_report;
					ltest_acc_input_report.timestamp = hrt_absolute_time();

					for (int i = 0; i < 3; i++) {
						ltest_acc_input_report.xyz[i] = vehicle_acc_ned_sampled(i);
					}

					_ltest_acc_input_pub.publish(ltest_acc_input_report);

					reset_acc_downsample();
					perf_end(_cycle_perf_pos);
				}
			}
		}

		/* Update orientation filter at ltest_yaw_UPDATE_RATE_HZ */
		if (_ltest_orientation_valid && ((hrt_absolute_time() - _last_update_yaw) > (1000000 / ltest_yaw_UPDATE_RATE_HZ))) {
			perf_begin(_cycle_perf_yaw);

			if (local_pose_updated) {
				_ltest_orientation->set_range_sensor(local_pose.dist_bottom, local_pose.dist_valid);
			}

			_ltest_orientation->update();
			_last_update_yaw = hrt_absolute_time();
			perf_end(_cycle_perf_yaw);
		}

		perf_end(_cycle_perf);
	}
}

void LandingTargetEst::reset_filters()
{
	if (_ltest_orientation_valid) {
		_ltest_orientation->resetFilter();
	}

	if (_ltest_position_valid) {
		_ltest_position->resetFilter();
	}

	_start_filters = false;

}

bool LandingTargetEst::get_local_pose(localPose &local_pose)
{

	vehicle_local_position_s vehicle_local_position;

	if (!_vehicle_local_position_sub.update(&vehicle_local_position)) {
		return false;

	} else {

		local_pose.xyz(0) = vehicle_local_position.x;
		local_pose.xyz(1) = vehicle_local_position.y;
		local_pose.xyz(2) = vehicle_local_position.z;
		local_pose.pos_valid = vehicle_local_position.xy_valid;

		local_pose.dist_bottom = vehicle_local_position.dist_bottom;
		local_pose.dist_valid = vehicle_local_position.dist_bottom_valid;

		local_pose.yaw_valid = vehicle_local_position.heading_good_for_control;
		local_pose.yaw = vehicle_local_position.heading;

		return true;
	}
}

bool LandingTargetEst::get_input(matrix::Vector3f &vehicle_acc_ned)
{

	vehicle_attitude_s	vehicle_attitude;
	vehicle_acceleration_s	vehicle_acceleration;

	bool vehicle_attitude_valid = _vehicle_attitude_sub.update(&vehicle_attitude);
	bool vehicle_acceleration_valid = _vehicle_acceleration_sub.update(&vehicle_acceleration);

	// Minimal requirement: acceleraion (for input) and attitude (to rotate acc in vehicle-carried NED frame)
	if (!vehicle_attitude_valid || !vehicle_acceleration_valid) {
		return false;

	} else {

		/* Transform FRD body acc to NED */
		matrix::Quaternionf quat_att(&vehicle_attitude.q[0]);
		matrix::Dcmf R_att = matrix::Dcm<float>(quat_att);

		matrix::Vector3f vehicle_acc{vehicle_acceleration.xyz};

		/* Compensate for gravity: the inverse of a rotation matrix is simply its transposed. */
		const matrix::Vector3f gravity_ned(0, 0, CONSTANTS_ONE_G);
		matrix::Vector3f gravity_body = R_att.transpose() * gravity_ned;

		vehicle_acc_ned = R_att * (vehicle_acc + gravity_body);
	}

	return true;
}


int LandingTargetEst::print_status()
{
	PX4_INFO("LTEst running");
	return 0;
}
int LandingTargetEst::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Module to estimate the position and orientation of a landing target.

The module runs periodically on the HP work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("landing_target_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int landing_target_estimator_main(int argc, char *argv[])
{
	return LandingTargetEst::main(argc, argv);
}

} // namespace landing_target_est