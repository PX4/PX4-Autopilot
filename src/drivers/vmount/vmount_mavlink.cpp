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
 * @file vmount_mavlink.cpp
 * @author Leon Müller (thedevleon)
 *
 */

#include "vmount_mavlink.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_roi.h>

#include <px4_posix.h>

/* uORB advertising */
static struct vehicle_command_s *vehicle_command;
static orb_advert_t vehicle_command_pub;

static int sys_id;
static int comp_id;
static double lat;
static double lon;
static float alt;

bool vmount_mavlink_init()
{
	memset(&vehicle_command, 0, sizeof(vehicle_command));
	vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &vehicle_command);

	if (!vehicle_command_pub) { return false; }

	return true;
}


void vmount_mavlink_deinit()
{
	orb_unadvertise(vehicle_command_pub);
	free(vehicle_command);
}

/*
 * MAV_CMD_DO_MOUNT_CONFIGURE (#204)
 *   param1 = mount_mode (1 = MAV_MOUNT_MODE_NEUTRAL recenters camera)
 */

void vmount_mavlink_configure(int roi_mode, bool man_control, int sysid, int compid)
{
	sys_id = sysid;
	comp_id = compid;

	vehicle_command->command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE;
	vehicle_command->target_system = sys_id;
	vehicle_command->target_component = comp_id;

	switch (roi_mode) {
	case vehicle_roi_s::VEHICLE_ROI_NONE:
		if (man_control) { vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING; }

		else { vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL; }

		break;

	case vehicle_roi_s::VEHICLE_ROI_WPNEXT:
		vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
		break;

	case vehicle_roi_s::VEHICLE_ROI_WPINDEX:
		vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
		break;

	case vehicle_roi_s::VEHICLE_ROI_LOCATION:
		vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
		break;

	case vehicle_roi_s::VEHICLE_ROI_TARGET:
		vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING;
		break;

	default:
		vehicle_command->param1 = vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL;
		break;
	}

	orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, vehicle_command);
}


/* MAV_CMD_DO_MOUNT_CONTROL (#205)
 *  param1 = pitch, angle in degree or pwm input
 *  param2 = roll, angle in degree or pwm input
 *  param3 = yaw, angle in degree or pwm input
 *  param6 = typeflags (uint16_t)
 *   0x0001: pitch, 0: unlimited, 1: limited, see CMD_SETANGLE
 *   0x0002: roll, 0: unlimited, 1: limited, see CMD_SETANGLE
 *   0x0004: yaw, 0: unlimited, 1: limited, see CMD_SETANGLE
 *   0x0100: input type, 0: angle input (see CMD_SETANGLE), 1: pwm input (see CMD_SETPITCHROLLYAW)
 */


void vmount_mavlink_set_location(double lat_new, double lon_new, float alt_new)
{
	lat = lat_new;
	lon = lon_new;
	alt = alt_new;
}

void vmount_mavlink_point(double global_lat, double global_lon, float global_alt)
{
	float pitch = vmount_mavlink_calculate_pitch(global_lat, global_lon, global_alt);
	float roll = 0.0f; // We want a level horizon, so leave roll at 0 degrees.
	float yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon) * (float)M_RAD_TO_DEG;

	vmount_mavlink_point_manual(pitch, roll, yaw);
}


void vmount_mavlink_point_manual(float pitch_new, float roll_new, float yaw_new)
{
	vehicle_command->command = vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL;
	vehicle_command->target_system = sys_id;
	vehicle_command->target_component = comp_id;

	vehicle_command->param1 = pitch_new;
	vehicle_command->param2 = roll_new;
	vehicle_command->param3 = yaw_new;

	orb_publish(ORB_ID(vehicle_command), vehicle_command_pub, vehicle_command);
}

float vmount_mavlink_calculate_pitch(double global_lat, double global_lon, float global_alt)
{
	float x = (lon - global_lon) * cos(M_DEG_TO_RAD * ((global_lat + lat) * 0.00000005)) * 0.01113195;
	float y = (lat - global_lat) * 0.01113195;
	float z = (alt - global_alt);
	float target_distance = sqrtf(powf(x, 2) + powf(y, 2));

	return atan2(z, target_distance) * M_RAD_TO_DEG;
}
