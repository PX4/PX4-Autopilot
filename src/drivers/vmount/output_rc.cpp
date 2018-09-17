/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file output_rc.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output_rc.h"

#include <px4_defines.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_control_mode.h>

namespace vmount
{

OutputRC::OutputRC(const OutputConfig &output_config)
	: OutputBase(output_config)
{
	_position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
}

OutputRC::~OutputRC()
{
	if (_actuator_controls_pub) {
		orb_unadvertise(_actuator_controls_pub);
	}
}

int OutputRC::update(const ControlData *control_data)
{
	bool control_mode_updated = false;
	orb_check(_vehicle_control_mode_sub, &control_mode_updated);

	if (control_mode_updated) {
		vehicle_control_mode_s control_mode = {};

		if (orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub, &control_mode) == PX4_OK) {
			_armed = control_mode.flag_armed;
		}
	}

	bool position_setpoint_triplet_updated = false;
	orb_check(_position_setpoint_triplet_sub, &position_setpoint_triplet_updated);

	if (position_setpoint_triplet_updated) {
		position_setpoint_triplet_s position_setpoint_triplet = {};

		if (orb_copy(ORB_ID(position_setpoint_triplet), _position_setpoint_triplet_sub, &position_setpoint_triplet) == PX4_OK) {
			if (position_setpoint_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
				if (!_retract_gimbal) {
					_retract_gimbal = true;
					_retract_changed = hrt_absolute_time();
				}
			}
		}
	}

	if (control_data) {
		// got new command

		if (_retract_gimbal != control_data->gimbal_shutter_retract) {
			// TODO: only if armed?
			_retract_changed = hrt_absolute_time();
		}

		_retract_gimbal = control_data->gimbal_shutter_retract;

		_set_angle_setpoints(control_data);

		_zoom = math::constrain(control_data->zoom, -1.0f, 1.0f);
	}

	_handle_position_update();

	hrt_abstime t = hrt_absolute_time();
	_calculate_output_angles(t);

	_actuator_controls.timestamp = hrt_absolute_time();

	const float retract_elapsed = hrt_elapsed_time(&_retract_changed) / 1e6f;

	const float doors_delay = _config.doors_delay / 1000.0f;
	const float camera_safe_elapsed = _config.camera_safe_position_close_delay / 1000.0f;

	if (_retract_gimbal && _doors_open) { // TODO: readd _armed
		// retract gimbal camera
		if (_doors_open) {

			_actuator_controls.control[0] = _config.gimbal_roll_retracted_mode_value;
			_actuator_controls.control[1] = _config.gimbal_pitch_retracted_mode_value;
			_actuator_controls.control[2] = _config.gimbal_yaw_retracted_mode_value;

			if (retract_elapsed > camera_safe_elapsed) {
				// camera in safe position for closing, begin retracting
				_actuator_controls.control[3] = _config.gimbal_retracted_mode_value;

				if (retract_elapsed > camera_safe_elapsed + doors_delay) {
					// close doors
					_actuator_controls.control[4] = _config.doors_closed_value;
					_doors_open = false;
				}
			}
		}

	} else if (!_retract_gimbal && !_doors_open) { // TODO: readd _armed
		// deploy gimbal camera
		if (!_doors_open) {
			// first open doors then wait
			_actuator_controls.control[4] = _config.doors_open_value;

			if (retract_elapsed > doors_delay) {
				_actuator_controls.control[0] = _config.gimbal_roll_retracted_mode_value;
				_actuator_controls.control[1] = _config.gimbal_pitch_retracted_mode_value;
				_actuator_controls.control[2] = _config.gimbal_yaw_retracted_mode_value;

				_actuator_controls.control[3] = _config.gimbal_normal_mode_value;

				if (retract_elapsed > camera_safe_elapsed + doors_delay) {
					// report doors now open to complete
					_doors_open = true;
				}
			}
		}

	} else if (_retract_gimbal && !_doors_open) {
		// retracted and doors closed
		// lock in safe positions
		_actuator_controls.control[0] = _config.gimbal_roll_retracted_mode_value;
		_actuator_controls.control[1] = _config.gimbal_pitch_retracted_mode_value;
		_actuator_controls.control[2] = _config.gimbal_yaw_retracted_mode_value;

		_actuator_controls.control[3] = _config.gimbal_retracted_mode_value;
		_actuator_controls.control[4] = _config.doors_closed_value;

	} else {
		// otherwise allow normal movement

		// constrain pitch to defined range
		_angle_setpoints[1] = math::constrain(_angle_setpoints[1], _config.pitch_min, _config.pitch_max);
		_angle_outputs[1] = math::constrain(_angle_outputs[1], _config.pitch_min, _config.pitch_max);

		// map yaw to +- 180 degrees
		if (_angle_outputs[2] > M_PI_F) {
			//_angle_setpoints[2] = -(M_PI_F * 2.0f) + _angle_setpoints[2];
			_angle_outputs[2] = -(M_PI_F * 2.0f) + _angle_outputs[2];
		}

		_actuator_controls.control[0] = (_angle_outputs[0] - _config.roll_offset) * _config.roll_scale;
		_actuator_controls.control[1] = (_angle_outputs[1] - _config.pitch_offset) * _config.pitch_scale;
		_actuator_controls.control[2] = (_angle_outputs[2] - _config.yaw_offset) * _config.yaw_scale;
	}

	// limit outputs to within scale
	_actuator_controls.control[0] = math::constrain(_actuator_controls.control[0], -1.0f, 1.0f);
	_actuator_controls.control[1] = math::constrain(_actuator_controls.control[1], -1.0f, 1.0f);
	_actuator_controls.control[2] = math::constrain(_actuator_controls.control[2], -1.0f, 1.0f);

	// zoom (find a better home for this)
	_actuator_controls.control[5] = _zoom;

	//PX4_INFO("pitch (tilt) %.3f (%.3f)", (double)_angle_outputs[1], (double)_actuator_controls.control[1]);

	int instance;
	orb_publish_auto(ORB_ID(actuator_controls_2), &_actuator_controls_pub, &_actuator_controls, &instance,
			 ORB_PRIO_DEFAULT);

	_last_update = t;

	return 0;
}

void OutputRC::print_status()
{
	PX4_INFO("Output: AUX");

	PX4_INFO("Retract Gimbal: %d", _retract_gimbal);
	PX4_INFO("Doors Open: %d", _doors_open);

	PX4_INFO("roll %.3f", (double)math::degrees(_angle_outputs[0]));
	PX4_INFO("pitch (tilt) %.3f", (double)math::degrees(_angle_outputs[1]));
	PX4_INFO("yaw (pan) %.3f", (double)math::degrees(_angle_outputs[2]));
}

} /* namespace vmount */

