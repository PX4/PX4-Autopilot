/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 *
 * This module is a modification of the hippocampus control module and is designed for the
 * BlueROV2.
 *
 * All the acknowledgments and credits for the fw wing app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

#include "uuv_pos_control.hpp"



/**
 * UUV pos_controller app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int uuv_pos_control_main(int argc, char *argv[]);


UUVPOSControl::UUVPOSControl():
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

UUVPOSControl::~UUVPOSControl()
{
	perf_free(_loop_perf);
}

bool UUVPOSControl::init()
{
	if (!_vehicle_local_position_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void UUVPOSControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}
}

void UUVPOSControl::pose_controller_6dof(const Vector3f &pos_des, vehicle_attitude_s &vehicle_attitude,
		vehicle_local_position_s &vlocal_pos, bool altitude_mode)
{
	//get current rotation of vehicle
	Quatf q_att(vehicle_attitude.q);

	// Assumes target 0 velocity
	Vector3f p_control_output = Vector3f(_param_pose_gain_x.get() * (pos_des(0) - vlocal_pos.x) - _param_pose_gain_d_x.get()
					     * vlocal_pos.vx,
					     _param_pose_gain_y.get() * (pos_des(1) - vlocal_pos.y) - _param_pose_gain_d_y.get() * vlocal_pos.vy,
					     _param_pose_gain_z.get() * (pos_des(2) - vlocal_pos.z) - _param_pose_gain_d_z.get() * vlocal_pos.vz);

	if (altitude_mode) {
		// In altitude mode, we only control the z-axis
		p_control_output(0) = 0.0f;
		p_control_output(1) = 0.0f;
	}

	Vector3f rotated_input = q_att.rotateVectorInverse(p_control_output); //rotate the coord.sys (from global to body)

	_attitude_setpoint.timestamp = hrt_absolute_time();
	_attitude_setpoint.q_d[0] = _trajectory_setpoint.quaternion[0];
	_attitude_setpoint.q_d[1] = _trajectory_setpoint.quaternion[1];
	_attitude_setpoint.q_d[2] = _trajectory_setpoint.quaternion[2];
	_attitude_setpoint.q_d[3] = _trajectory_setpoint.quaternion[3];
	_attitude_setpoint.thrust_body[0] = rotated_input(0);
	_attitude_setpoint.thrust_body[1] = rotated_input(1);
	_attitude_setpoint.thrust_body[2] = rotated_input(2);
}

void UUVPOSControl::check_setpoint_validity(vehicle_local_position_s &vlocal_pos)
{
	const float _setpoint_age = (hrt_absolute_time() - _trajectory_setpoint.timestamp) * 1e-6f;

	if (_setpoint_age < 0.0f || _setpoint_age > _param_setpoint_max_age.get()) {
		reset_trajectory_setpoint(vlocal_pos);
	}

	if (!PX4_ISFINITE(_trajectory_setpoint.position[0]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.position[1]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.position[2]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.quaternion[0]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.quaternion[1]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.quaternion[2]) ||
	    !PX4_ISFINITE(_trajectory_setpoint.quaternion[3])) {
		reset_trajectory_setpoint(vlocal_pos);
	}
}

void UUVPOSControl::generate_trajectory_setpoint(vehicle_local_position_s &vlocal_pos,
		vehicle_attitude_s &vehicle_attitude,
		float dt)
{
	float roll = Eulerf(matrix::Quatf(_trajectory_setpoint.quaternion)).phi();
	float pitch = Eulerf(matrix::Quatf(_trajectory_setpoint.quaternion)).theta();
	float yaw = Eulerf(matrix::Quatf(_trajectory_setpoint.quaternion)).psi();

	// Integrate manual control inputs
	// Info:
	//  - throttle is Z, roll is Y, pitch is X
	//  - if param_stab_mode == 1:
	//	- roll = 0
	//	- pitch = 0
	//  - if param_stab_mode == 0:
	//      - roll can be updated with D-pad  (joystick)
	//      - pitch can be updated with D-pad (joystick)
	float roll_setpoint = roll;
	float pitch_setpoint = pitch;

	if (_param_stab_mode.get()) {
		roll_setpoint = 0.0;
		pitch_setpoint = 0.0;

	} else {
		// Update target roll and pitch setpoint with D-pad
		switch (_manual_control_setpoint.buttons) {
		case 2048:
			pitch_setpoint -= dt * _param_sgm_pitch.get();
			break;

		case 4096:
			pitch_setpoint += dt * _param_sgm_pitch.get();
			break;

		case 8192:
			roll_setpoint -= dt * _param_sgm_roll.get();
			break;

		case 16384:
			roll_setpoint += dt * _param_sgm_roll.get();
			break;

		default:
			break;
		}
	}

	float yaw_setpoint = yaw + _manual_control_setpoint.yaw * dt * _param_sgm_yaw.get();

	// Update position setpoints based on manual control inputs
	float vx_sp = 0.0;

	if (_manual_control_setpoint.pitch > _param_pos_stick_db.get()
	    || _manual_control_setpoint.pitch < -_param_pos_stick_db.get()) {
		// If pitch is not zero, we use it to set the roll setpoint
		vx_sp = _manual_control_setpoint.pitch * _param_pgm_vel.get();
	}

	float vy_sp = 0.0;

	if (_manual_control_setpoint.roll > _param_pos_stick_db.get()
	    || _manual_control_setpoint.roll < -_param_pos_stick_db.get()) {
		// If roll is not zero, we use it to set the pitch setpoint
		vy_sp = _manual_control_setpoint.roll * _param_pgm_vel.get();
	}

	float vz_sp = 0.0;

	if (_manual_control_setpoint.throttle > _param_pos_stick_db.get()
	    || _manual_control_setpoint.throttle < -_param_pos_stick_db.get()) {
		// If throttle is not zero, we use it to set the vertical velocity
		vz_sp = -_manual_control_setpoint.throttle * _param_pgm_vel.get();
	}

	// rotate velocity setpoint in body frame to global frame
	Vector3f velocity_setpoint(vx_sp, vy_sp, vz_sp);
	Quatf q_att(vehicle_attitude.q);
	Vector3f rotated_velocity_setpoint = q_att.rotateVector(velocity_setpoint);

	// Generate target quaternion
	Eulerf euler_sp(roll_setpoint, pitch_setpoint, yaw_setpoint);
	Quatf q_sp = euler_sp;

	// Normalize the quaternion to avoid numerical issues
	q_sp.normalize();

	q_sp.copyTo(_trajectory_setpoint.quaternion);

	_trajectory_setpoint.timestamp = hrt_absolute_time();

	if (!_param_pos_mode.get()) {
		_trajectory_setpoint.position[0] = _trajectory_setpoint.position[0] + vx_sp * dt; // X in world frame
		_trajectory_setpoint.position[1] = _trajectory_setpoint.position[1] + vy_sp * dt; // Y in world frame
		_trajectory_setpoint.position[2] = _trajectory_setpoint.position[2] + vz_sp * dt; // Z in world frame

	} else {
		_trajectory_setpoint.position[0] = _trajectory_setpoint.position[0] + rotated_velocity_setpoint(
				0) * dt; // X in body frame
		_trajectory_setpoint.position[1] = _trajectory_setpoint.position[1] + rotated_velocity_setpoint(
				1) * dt; // Y in body frame
		_trajectory_setpoint.position[2] = _trajectory_setpoint.position[2] + rotated_velocity_setpoint(
				2) * dt; // Z in body frame
	}
}

void UUVPOSControl::reset_trajectory_setpoint(vehicle_local_position_s &vlocal_pos)
{
	// Reset trajectory setpoint to current position and attitude
	_trajectory_setpoint.timestamp = hrt_absolute_time();
	_trajectory_setpoint.position[0] = vlocal_pos.x;
	_trajectory_setpoint.position[1] = vlocal_pos.y;
	_trajectory_setpoint.position[2] = vlocal_pos.z;
	_trajectory_setpoint.quaternion[0] = _vehicle_attitude.q[0];
	_trajectory_setpoint.quaternion[1] = _vehicle_attitude.q[1];
	_trajectory_setpoint.quaternion[2] = _vehicle_attitude.q[2];
	_trajectory_setpoint.quaternion[3] = _vehicle_attitude.q[3];
}

void UUVPOSControl::Run()
{
	if (should_exit()) {
		_vehicle_local_position_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	/* check vehicle control mode for changes to publication state */
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	/* update parameters from storage */
	parameters_update();

	//vehicle_attitude_s attitude;
	vehicle_local_position_s vlocal_pos;

	/* only run controller if local_pos changed */
	if (_vehicle_local_position_sub.update(&vlocal_pos)) {
		const float dt = math::constrain(((vlocal_pos.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = vlocal_pos.timestamp_sample;

		// Update vehicle attitude
		_vehicle_attitude_sub.update(&_vehicle_attitude);

		/* Run position or altitude mode from manual setpoints*/
		if (_vcontrol_mode.flag_control_manual_enabled
		    && (_vcontrol_mode.flag_control_altitude_enabled
			|| _vcontrol_mode.flag_control_position_enabled)
		    && _vcontrol_mode.flag_armed) {
			/* Update manual setpoints */

			const bool altitude_only_flag = _vcontrol_mode.flag_control_altitude_enabled
							&& ! _vcontrol_mode.flag_control_position_enabled;

			_manual_control_setpoint_sub.update(&_manual_control_setpoint);

			// Ensure no nan and sufficiently recent setpoint
			check_setpoint_validity(vlocal_pos);

			// Generate _trajectory_setpoint -> creates _trajectory_setpoint
			generate_trajectory_setpoint(vlocal_pos, _vehicle_attitude, dt);

			pose_controller_6dof(Vector3f(_trajectory_setpoint.position), _vehicle_attitude,
					     vlocal_pos, altitude_only_flag);

		} else if (!_vcontrol_mode.flag_control_manual_enabled
			   && (_vcontrol_mode.flag_control_altitude_enabled
			       || _vcontrol_mode.flag_control_position_enabled)
			   && _vcontrol_mode.flag_armed) {
			/* Autonomous position mode - no manual inputs are used */
			const bool altitude_only_flag = _vcontrol_mode.flag_control_altitude_enabled
							&& ! _vcontrol_mode.flag_control_position_enabled;

			// get manual control setpoint
			_trajectory_setpoint_sub.update(&_trajectory_setpoint);

			pose_controller_6dof(Vector3f(_trajectory_setpoint.position), _vehicle_attitude,
					     vlocal_pos, altitude_only_flag);

		} else {
			// Reset if not in a valid mode (like attitude, rate, manual) to clear setpoint
			check_setpoint_validity(vlocal_pos);
		}
	}

	/* Only publish if any of the proper modes are enabled */
	if (_vcontrol_mode.flag_control_position_enabled ||
	    _vcontrol_mode.flag_control_altitude_enabled) {
		// Print attitude setpoint
		_att_sp_pub.publish(_attitude_setpoint);
	}

	perf_end(_loop_perf);
}

int UUVPOSControl::task_spawn(int argc, char *argv[])
{
	UUVPOSControl *instance = new UUVPOSControl();

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

int UUVPOSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int UUVPOSControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Controls the attitude of an unmanned underwater vehicle (UUV).
Publishes `attitude_setpoint` messages.
### Implementation
Currently, this implementation supports only a few modes:
 * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
 * Auto mission: The uuv runs missions
### Examples
CLI usage example:
$ uuv_pos_control start
$ uuv_pos_control status
$ uuv_pos_control stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("uuv_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start")
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int uuv_pos_control_main(int argc, char *argv[])
{
    return UUVPOSControl::main(argc, argv);
}
