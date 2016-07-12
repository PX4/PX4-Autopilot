/****************************************************************************
*
*   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file vmount_onboard.cpp
 * @author Leon Müller (thedevleon)
 *
 */

#include "vmount_onboard.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>


/* uORB topics */
static struct actuator_controls_s actuator_controls;
static orb_advert_t actuator_controls_pub;

static struct vehicle_attitude_s vehicle_attitude;

static int mount_mode;
static float pitch;
static float roll;
static float yaw;
static float retracts;
static int stab_pitch;
static int stab_roll;
static int stab_yaw;
static double lat;
static double lon;
static float alt;

bool vmount_onboard_init()
{
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuator_controls);

	if (!actuator_controls_pub) { return false; }

	vmount_onboard_configure(vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT, 0.0f, 0.0f, 0.0f);

	return true;
}

void vmount_onboard_deinit()
{
	orb_unadvertise(actuator_controls_pub);
	//free(actuator_controls);
	//free(vehicle_attitude);
}

void vmount_onboard_configure(int new_mount_mode, bool new_stab_roll, bool new_stab_pitch, bool new_stab_yaw)
{
	mount_mode = new_mount_mode;
	stab_roll = new_stab_roll;
	stab_pitch = new_stab_pitch;
	stab_yaw = new_stab_yaw;

	switch (mount_mode) {
	case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
		retracts = -1.0f;
		vmount_onboard_set_manual(0.0f, 0.0f, 0.0f);
		break;

	case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
		retracts = 0.0f;
		vmount_onboard_set_manual(0.0f, 0.0f, 0.0f);
		break;

	case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
	case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
	case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
		retracts = 0.0f;

	default:
		break;
	}
}

void vmount_onboard_set_location(double lat_new, double lon_new, float alt_new)
{
	lat = lat_new;
	lon = lon_new;
	alt = alt_new;
}

void vmount_onboard_set_manual(double pitch_new, double roll_new, float yaw_new)
{
	pitch = pitch_new;
	roll = roll_new;
	yaw = yaw_new;
}

void vmount_onboard_set_mode(int new_mount_mode)
{
	vmount_onboard_configure(new_mount_mode, stab_roll, stab_pitch, stab_yaw);
}

void vmount_onboard_point(double global_lat, double global_lon, float global_alt)
{
	if (mount_mode == vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT) {
		pitch = vmount_onboard_calculate_pitch(global_lat, global_lon, global_alt);
		roll = 0.0f; // We want a level horizon, so leave roll at 0 degrees.
		yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon) * (float)M_RAD_TO_DEG;
	}

	vmount_onboard_point_manual(pitch, roll, yaw);
}

void vmount_onboard_point_manual(float pitch_target, float roll_target, float yaw_target)
{
	float pitch_new = pitch_target;
	float roll_new = roll_target;
	float yaw_new = yaw_target;

	if (mount_mode != vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT &&
	    mount_mode != vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL) {
		if (stab_roll) {
			roll_new += 1.0f / M_PI_F * -vehicle_attitude.roll;
		}

		if (stab_pitch) {
			pitch_new += 1.0f / M_PI_F * -vehicle_attitude.pitch;
		}

		if (stab_yaw) {
			yaw_new += 1.0f / M_PI_F * vehicle_attitude.yaw;
		}
	}

	actuator_controls.timestamp = hrt_absolute_time();
	actuator_controls.control[0] = pitch_new;
	actuator_controls.control[1] = roll_new;
	actuator_controls.control[2] = yaw_new;
	actuator_controls.control[4] = retracts;

	orb_publish(ORB_ID(actuator_controls_2), actuator_controls_pub, &actuator_controls);
}

void vmount_onboard_update_attitude(vehicle_attitude_s vehicle_attitude_new)
{
	vehicle_attitude = vehicle_attitude_new;
}

float vmount_onboard_calculate_pitch(double global_lat, double global_lon, float global_alt)
{
	float x = (lon - global_lon) * cos(M_DEG_TO_RAD * ((global_lat + lat) * 0.00000005)) * 0.01113195;
	float y = (lat - global_lat) * 0.01113195;
	float z = (alt - global_alt);
	float target_distance = sqrtf(powf(x, 2) + powf(y, 2));

	return atan2(z, target_distance) * M_RAD_TO_DEG;
}
