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
 * @file VisionTargetEst.cpp
 * @brief Handles the position and orientation estimators.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#include <px4_platform_common/defines.h>

#include <mathlib/mathlib.h>

#include "VisionTargetEst.h"

namespace vision_target_estimator
{

ModuleBase::Descriptor VisionTargetEst::desc{task_spawn, custom_command, print_usage};

VisionTargetEst::VisionTargetEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::vte)
{
	_vte_input_pub.advertise();
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
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

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
	if (setNewTaskIfAvailable()) {
		stopAllEstimators();
	}

	// No task running: switch to a low-rate polling mode instead of waking up on every attitude update
	if (noActiveTask()) {
		static constexpr uint32_t kIdleUpdateIntervalUs = 200_ms;

		if (_vehicle_attitude_sub.registered()) {
			_vehicle_attitude_sub.unregisterCallback();
			ScheduleOnInterval(kIdleUpdateIntervalUs);
		}

		return;
	}

	// Task is active: ensure we're callback-driven and not running a periodic timer
	if (!_vehicle_attitude_sub.registered()) {
		if (_vehicle_attitude_sub.registerCallback()) {
			ScheduleClear();

		} else {
			const uint32_t estimator_update_period_us = static_cast<uint32_t>(math::min(kPosUpdatePeriodUs,
					kYawUpdatePeriodUs));
			ScheduleOnInterval(estimator_update_period_us);
		}
	}

	// Task is running, check if an estimator must be started or re-started
	startEstIfNeeded();

	const bool task_completed = isCurrentTaskComplete();

	if (task_completed) {
		stopAllEstimators();
		_current_task_ptr = nullptr;
		return;
	}

	restartTimedOutEstimators();

	if (noEstRunning()) {
		return;
	}

	// Early return checks passed, start filter computations.
	updateEstimators();
}

void VisionTargetEst::handleExit()
{
	_vehicle_attitude_sub.unregisterCallback();
	exit_and_cleanup(desc);
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

	_vte_position_enabled = false;
	_vte_orientation_enabled = false;

	// Params with reboot required
	if (_param_vte_pos_en.get()) {
		PX4_DEBUG("VTE position estimator enabled.");

		if (_vte_position.init()) {
			_vte_position_enabled = true;

		} else {
			PX4_ERR("VTE position estimator initialisation failed");
		}
	}

	if (_param_vte_yaw_en.get()) {
		PX4_DEBUG("VTE yaw estimator enabled.");

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

	if (new_vte_task_mask != _vte_task_mask) {

		_vte_task_mask = new_vte_task_mask;

		if (_vte_task_mask < 1) {
			PX4_WARN("VTE no task, update VTE_TASK_MASK");

		} else {
			PX4_DEBUG("VTE VTE_TASK_MASK config: ");

			for (const VteTask *task : _task_registry) {
				if (isTaskMaskBitEnabled(task->maskBit())) {
					PX4_DEBUG("    %s", task->name());
				}
			}
		}
	}

	const uint16_t requested_aid_mask = adjustAidMask(_param_vte_aid_mask.get());
	const bool aid_mask_changed = requested_aid_mask != _vte_aid_mask.value;
	_vte_aid_mask.value = requested_aid_mask;

	if (_vte_position_enabled) {
		_vte_position.setVteAidMask(requested_aid_mask);
	}

	if (_vte_orientation_enabled) {
		_vte_orientation.setVteAidMask(requested_aid_mask);
	}

	if (aid_mask_changed) {
		printAidMask();
	}

	const hrt_abstime vte_timeout_us = static_cast<hrt_abstime>(_param_vte_btout.get() * 1_s);
	_vte_position.setVteTimeout(vte_timeout_us);
	_vte_orientation.setVteTimeout(vte_timeout_us);

	const hrt_abstime target_valid_timeout_us = static_cast<hrt_abstime>(_param_vte_tgt_tout.get() * 1_s);
	_vte_position.setTargetValidTimeout(target_valid_timeout_us);
	_vte_orientation.setTargetValidTimeout(target_valid_timeout_us);

	const hrt_abstime meas_recent_timeout_us = static_cast<hrt_abstime>(_param_vte_mrec_tout.get() * 1_s);
	_vte_position.setMeasRecentTimeout(meas_recent_timeout_us);
	_vte_orientation.setMeasRecentTimeout(meas_recent_timeout_us);

	const hrt_abstime meas_updated_timeout_us = static_cast<hrt_abstime>(_param_vte_mupd_tout.get() * 1_s);
	_vte_position.setMeasUpdatedTimeout(meas_updated_timeout_us);

	if (_vte_position_enabled && _position_estimator_running
	    && !_vte_position.fusionEnabled()) {
		PX4_DEBUG("VTE position estimator stopped, no fusion source selected.");
		stopPosEst();
	}

	if (_vte_orientation_enabled && _orientation_estimator_running
	    && !_vte_orientation.fusionEnabled()) {
		PX4_DEBUG("VTE yaw estimator stopped, no fusion source selected.");
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
		PX4_WARN("VTE invalid aid mask: both target GPS and mission land enabled.");
		PX4_WARN("Disabling mission land position fusion.");
		new_aid_mask.flags.use_mission_pos = false;
	}

	return new_aid_mask.value;
}

void VisionTargetEst::printAidMask()
{
	PX4_DEBUG("VTE VTE_AID_MASK config: ");

	if (_vte_aid_mask.flags.use_vision_pos) {PX4_DEBUG("    vision relative position fusion enabled");}

	if (_vte_aid_mask.flags.use_target_gps_pos) {PX4_DEBUG("    target GPS position fusion enabled");}

	if (_vte_aid_mask.flags.use_target_gps_vel) {PX4_DEBUG("    target GPS velocity fusion enabled");}

	if (_vte_aid_mask.flags.use_mission_pos) {PX4_DEBUG("    mission land position fusion enabled");}

	if (_vte_aid_mask.flags.use_uav_gps_vel) {PX4_DEBUG("    UAV GPS velocity fusion enabled");}

	if (_vte_aid_mask.value == 0) {PX4_WARN("    no data fusion. Modify VTE_AID_MASK");}
}

void VisionTargetEst::resetAccDownsample()
{
	_vehicle_acc_ned_sum.setAll(0);
	_acc_sample_count = 0;
	_last_acc_reset = nowUs();
}

bool VisionTargetEst::startYawEst()
{
	if (!_vte_orientation_enabled) {
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

	resetAccDownsample();

	PX4_INFO("Starting Position Vision Target Estimator.");
	_vte_position.resetFilter();

	if (_current_task_ptr) {
		_current_task_ptr->onPosEstStart(_vte_position);
	}

	return true;
}

bool VisionTargetEst::setNewTaskIfAvailable()
{
	// Registry is ordered by priority (highest first)
	for (VteTask *task : _task_registry) {
		if (!isTaskMaskBitEnabled(task->maskBit())) {
			continue;
		}

		if (!task->isReady()) {
			continue;
		}

		if (_current_task_ptr == task) {
			return false;
		}

		PX4_INFO("VTE, %s task requested.", task->name());
		task->onActivate();
		_current_task_ptr = task;
		return true;
	}

	return false;
}

bool VisionTargetEst::isCurrentTaskComplete()
{
	if (!_current_task_ptr) {
		return false;
	}

	// Mask-bit deselection (user disabled this task in VTE_TASK_MASK).
	if (!isTaskMaskBitEnabled(_current_task_ptr->maskBit())) {
		PX4_DEBUG("VTE_TASK_MASK updated, %s task completed.", _current_task_ptr->name());
		return true;
	}

	return _current_task_ptr->isComplete();
}

void VisionTargetEst::updateTaskTopics()
{
	for (VteTask *task : _task_registry) {
		if (isTaskMaskBitEnabled(task->maskBit())) {
			task->pollStatus();
		}
	}
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
}

void VisionTargetEst::stopYawEst()
{
	PX4_INFO("Stopping Orientation Vision Target Estimator.");

	if (_vte_orientation_enabled) {
		_vte_orientation.resetFilter();
	}

	_orientation_estimator_running = false;
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

void VisionTargetEst::restartTimedOutEstimators()
{
	bool restart_needed = false;

	if (_position_estimator_running && _vte_position.timedOut()) {
		stopPosEst();
		PX4_WARN("Estimator TIMEOUT, position VTE stopped.");
		restart_needed = true;
	}

	if (_orientation_estimator_running && _vte_orientation.timedOut()) {
		stopYawEst();
		PX4_WARN("Estimator TIMEOUT, orientation VTE stopped.");
		restart_needed = true;
	}

	if (restart_needed) {
		startEstIfNeeded();
	}
}

void VisionTargetEst::updateEstimators()
{
	perf_begin(_cycle_perf);

	if (_vte_position_enabled && _position_estimator_running) {
		updateGpsAntennaOffset();

		matrix::Vector3f vel_offset_body{};
		const bool vel_offset_updated = computeGpsVelocityOffset(vel_offset_body);

		matrix::Vector3f vehicle_acc_ned{};
		matrix::Quaternionf q_att{};
		matrix::Vector3f gps_pos_offset_ned{};
		matrix::Vector3f vel_offset_ned = vel_offset_body;
		bool acc_valid = false;

		if (pollEstimatorInput(vehicle_acc_ned, q_att, gps_pos_offset_ned, vel_offset_ned, vel_offset_updated,
				       acc_valid)) {
			_last_att = q_att;

			/* If the acceleration has been averaged for too long, reset the accumulator */
			if (hasTimedOut(_last_acc_reset, kMinAccDownsampleTimeoutUs)) {
				PX4_DEBUG("Forced acc downsample reset");
				resetAccDownsample();
			}

			if (acc_valid) {
				_vehicle_acc_ned_sum += vehicle_acc_ned;
				_acc_sample_count++;
			}

			const uint32_t acc_sample_count = _acc_sample_count;

			if ((acc_sample_count > 0) && updateWhenIntervalElapsed(_last_update_pos, kPosUpdatePeriodUs)) {
				updatePosEst(gps_pos_offset_ned, vel_offset_ned, vel_offset_updated, acc_sample_count);
				resetAccDownsample();
			}
		}
	}

	if (_vte_orientation_enabled && _orientation_estimator_running
	    && updateWhenIntervalElapsed(_last_update_yaw, kYawUpdatePeriodUs)) {
		updateYawEst();
	}

	perf_end(_cycle_perf);
}

void VisionTargetEst::updatePosEst(const matrix::Vector3f &gps_pos_offset_ned,
				   const matrix::Vector3f &vel_offset_ned,
				   const bool vel_offset_updated,
				   const uint32_t acc_sample_count)
{
	perf_begin(_cycle_perf_pos);

	if (acc_sample_count == 0) {
		perf_end(_cycle_perf_pos);
		return;
	}

	LocalPose local_pose{};
	const bool local_pose_updated = pollLocalPose(local_pose);

	if (local_pose_updated) {
		_vte_position.setLocalVelocity(local_pose.vel_xyz, local_pose.vel_valid, local_pose.timestamp);
		_vte_position.setLocalPosition(local_pose.xyz, local_pose.pos_valid, local_pose.timestamp);
	}

	_vte_position.setGpsPosOffset(gps_pos_offset_ned, _gps_pos_is_offset);

	if (vel_offset_updated) {
		_vte_position.setVelOffset(vel_offset_ned);
	}

	const matrix::Vector3f vehicle_acc_ned_sampled = _vehicle_acc_ned_sum / acc_sample_count;

	_vte_position.update(vehicle_acc_ned_sampled);
	publishVteInput(vehicle_acc_ned_sampled, _last_att, acc_sample_count);

	perf_end(_cycle_perf_pos);
}

void VisionTargetEst::updateYawEst()
{
	perf_begin(_cycle_perf_yaw);
	_vte_orientation.update();
	perf_end(_cycle_perf_yaw);
}

void VisionTargetEst::publishVteInput(const matrix::Vector3f &vehicle_acc_ned_sampled,
				      const matrix::Quaternionf &q_att_sampled,
				      uint32_t acc_sample_count)
{
	vte_input_s vte_input_report{};
	vte_input_report.timestamp = nowUs();
	vte_input_report.timestamp_sample = _vehicle_acc_body.timestamp;

	vehicle_acc_ned_sampled.copyTo(vte_input_report.acc_xyz);
	q_att_sampled.copyTo(vte_input_report.q_att);
	vte_input_report.acc_sample_count = acc_sample_count;

	_vte_input_pub.publish(vte_input_report);
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

bool VisionTargetEst::updateGpsAntennaOffset()
{
	sensor_gps_s vehicle_gps_position{};

	if (!_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
		return false;
	}

	_gps_pos_offset_xyz = matrix::Vector3f(vehicle_gps_position.antenna_offset_x,
					       vehicle_gps_position.antenna_offset_y,
					       vehicle_gps_position.antenna_offset_z);

	static constexpr float kMinGpsOffsetM = 0.01f; // Consider GNSS not offset below 1cm
	_gps_pos_is_offset = (fabsf(_gps_pos_offset_xyz(0)) > kMinGpsOffsetM)
			     || (fabsf(_gps_pos_offset_xyz(1)) > kMinGpsOffsetM)
			     || (fabsf(_gps_pos_offset_xyz(2)) > kMinGpsOffsetM);

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

	local_pose.timestamp = vehicle_local_position.timestamp;

	return true;
}

bool VisionTargetEst::pollEstimatorInput(matrix::Vector3f &vehicle_acc_ned, matrix::Quaternionf &quat_att,
		matrix::Vector3f &gps_pos_offset_ned, matrix::Vector3f &vel_offset_ned,
		const bool vel_offset_updated, bool &acc_valid)
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
		const matrix::Vector3f gravity_ned(0.f, 0.f, kGravityMps2);
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
	const hrt_abstime now = nowUs();

	if (hasTimedOutAt(now, last_time, interval)) {
		last_time = now;
		return true;
	}

	return false;
}

int VisionTargetEst::print_status()
{
	const auto yes_no = [](const bool value) { return value ? "yes" : "no"; };

	PX4_INFO("VTE running");
#if defined(CONFIG_VTEST_MOVING)
	PX4_INFO("target model: moving");
	PX4_WARN("Caution: VTE for moving targets is a beta feature");
#else
	PX4_INFO("target model: static");
#endif // CONFIG_VTEST_MOVING
	PX4_INFO("work queue: %s", px4::wq_configurations::vte.name);
	PX4_INFO("current task: %s", _current_task_ptr ? _current_task_ptr->name() : "none");
	PX4_INFO("task mask: 0x%02x (prec_land: %s, debug: %s)",
		 static_cast<unsigned>(_vte_task_mask),
		 yes_no(isTaskMaskBitEnabled(task_bits::kPrecLand)),
		 yes_no(isTaskMaskBitEnabled(task_bits::kDebug)));
	PX4_INFO("aid mask: 0x%04x", static_cast<unsigned>(_vte_aid_mask.value));
	PX4_INFO("  vision pos: %s, target gps pos: %s, mission pos: %s",
		 yes_no(_vte_aid_mask.flags.use_vision_pos),
		 yes_no(_vte_aid_mask.flags.use_target_gps_pos),
		 yes_no(_vte_aid_mask.flags.use_mission_pos));
	PX4_INFO("  uav gps vel: %s, target gps vel: %s",
		 yes_no(_vte_aid_mask.flags.use_uav_gps_vel),
		 yes_no(_vte_aid_mask.flags.use_target_gps_vel));

	PX4_INFO("position vte: enabled: %s, available: %s, running: %s, fusing: %s, timed out: %s",
		 yes_no(_param_vte_pos_en.get()),
		 yes_no(_vte_position_enabled),
		 yes_no(_position_estimator_running),
		 yes_no(_vte_position.fusionEnabled()),
		 yes_no(_vte_position.timedOut()));

	if (_position_estimator_running) {
		_vte_position.print_status();
	}

	PX4_INFO("orientation vte: enabled: %s, available: %s, running: %s, fusing: %s, timed out: %s",
		 yes_no(_param_vte_yaw_en.get()),
		 yes_no(_vte_orientation_enabled),
		 yes_no(_orientation_estimator_running),
		 yes_no(_vte_orientation.fusionEnabled()),
		 yes_no(_vte_orientation.timedOut()));

	if (_orientation_estimator_running) {
		_vte_orientation.print_status();
	}

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

The module runs periodically on the px4::wq_configurations::vte queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vision_target_estimator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int vision_target_estimator_main(int argc, char *argv[])
{
	return ModuleBase::main(VisionTargetEst::desc, argc, argv);
}

} // namespace vision_target_estimator
