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
 * @file output.cpp
 * @author Leon Müller (thedevleon)
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 */

#include "output.h"
#include <errno.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/mount_orientation.h>
#include <px4_defines.h>
#include <geo/geo.h>
#include <math.h>
#include <mathlib/mathlib.h>

namespace vmount
{

OutputBase::OutputBase(const OutputConfig &output_config)
	: _config(output_config)
{
	_last_update = hrt_absolute_time();
}

OutputBase::~OutputBase()
{
	if (_vehicle_attitude_sub >= 0) {
		orb_unsubscribe(_vehicle_attitude_sub);
	}

	if (_vehicle_global_position_sub >= 0) {
		orb_unsubscribe(_vehicle_global_position_sub);
	}
}

int OutputBase::initialize()
{
	if ((_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude))) < 0) {
		return -errno;
	}

	if ((_vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position))) < 0) {
		return -errno;
	}

	return 0;
}

void OutputBase::publish()
{
	int instance;
	mount_orientation_s mount_orientation;

	for (unsigned i = 0; i < 3; ++i) {
		mount_orientation.attitude_euler_angle[i] = _angle_outputs[i];
	}

	orb_publish_auto(ORB_ID(mount_orientation), &_mount_orientation_pub, &mount_orientation, &instance, ORB_PRIO_DEFAULT);
}

float OutputBase::_calculate_pitch(double lon, double lat, float altitude,
				   const vehicle_global_position_s &global_position)
{
	if (!map_projection_initialized(&_projection_reference)) {
		map_projection_init(&_projection_reference, global_position.lat, global_position.lon);
	}

	float x1, y1, x2, y2;
	map_projection_project(&_projection_reference, lat, lon, &x1, &y1);
	map_projection_project(&_projection_reference, global_position.lat, global_position.lon, &x2, &y2);
	float dx = x1 - x2, dy = y1 - y2;
	float target_distance = sqrtf(dx * dx + dy * dy);
	float z = altitude - global_position.alt;

	return atan2f(z, target_distance);
}

void OutputBase::_set_angle_setpoints(const ControlData *control_data)
{
	_cur_control_data = control_data;

	for (int i = 0; i < 3; ++i) {
		_stabilize[i] = control_data->stabilize_axis[i];
		_angle_speeds[i] = 0.f;
	}

	switch (control_data->type) {
	case ControlData::Type::Angle:
		for (int i = 0; i < 3; ++i) {
			if (control_data->type_data.angle.is_speed[i]) {
				_angle_speeds[i] = control_data->type_data.angle.angles[i];

			} else {
				_angle_setpoints[i] = control_data->type_data.angle.angles[i];
			}
		}

		break;

	case ControlData::Type::LonLat:
		_handle_position_update(true);
		break;

	case ControlData::Type::Neutral:
		_angle_setpoints[0] = 0.f;
		_angle_setpoints[1] = 0.f;
		_angle_setpoints[2] = 0.f;
		break;
	}
}

void OutputBase::_handle_position_update(bool force_update)
{
	bool need_update = force_update;

	if (!_cur_control_data || _cur_control_data->type != ControlData::Type::LonLat) {
		return;
	}

	if (!force_update) {
		orb_check(_vehicle_global_position_sub, &need_update);
	}

	if (!need_update) {
		return;
	}

	vehicle_global_position_s vehicle_global_position;
	orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &vehicle_global_position);

	float pitch;
	const double &lon = _cur_control_data->type_data.lonlat.lon;
	const double &lat = _cur_control_data->type_data.lonlat.lat;
	const float &alt = _cur_control_data->type_data.lonlat.altitude;

	if (_cur_control_data->type_data.lonlat.pitch_fixed_angle >= -M_PI_F) {
		pitch = _cur_control_data->type_data.lonlat.pitch_fixed_angle;

	} else {
		pitch = _calculate_pitch(lon, lat, alt, vehicle_global_position);
	}

	float roll = _cur_control_data->type_data.lonlat.roll_angle;
	float yaw = get_bearing_to_next_waypoint(vehicle_global_position.lat, vehicle_global_position.lon, lat, lon);

	_angle_setpoints[0] = roll;
	_angle_setpoints[1] = pitch;
	_angle_setpoints[2] = yaw;
}

void OutputBase::_calculate_output_angles(const hrt_abstime &t)
{
	//take speed into account
	float dt = (t - _last_update) / 1.e6f;

	for (int i = 0; i < 3; ++i) {
		_angle_setpoints[i] += dt * _angle_speeds[i];
	}

	//get the output angles and stabilize if necessary
	vehicle_attitude_s vehicle_attitude;

	if (_stabilize[0] || _stabilize[1] || _stabilize[2]) {
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude);
	}

	matrix::Eulerf euler = matrix::Quatf(vehicle_attitude.q);

	for (int i = 0; i < 3; ++i) {
		if (_stabilize[i]) {
			_angle_outputs[i] = _angle_setpoints[i] - euler(i);

		} else {
			_angle_outputs[i] = _angle_setpoints[i];
		}

		//bring angles into proper range [-pi, pi]
		while (_angle_outputs[i] > M_PI_F) { _angle_outputs[i] -= 2.f * M_PI_F; }

		while (_angle_outputs[i] < -M_PI_F) { _angle_outputs[i] += 2.f * M_PI_F; }
	}
}

} /* namespace vmount */

