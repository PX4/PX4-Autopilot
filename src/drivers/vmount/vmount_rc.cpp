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
 * @file vmount_rc.c
 * @author Leon Müller (thedevleon)
 *
 */

#include "vmount_rc.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/actuator_controls.h>


/* uORB advertising */
static struct actuator_controls_s actuator_controls;
static orb_advert_t actuator_controls_pub;

static double lon;
static double lat;
static float alt;
static float normal_mode;
static float locked_mode;
static bool  locked;

bool vmount_rc_init()
{
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuator_controls);

	if (!actuator_controls_pub) { return false; }

	locked = false;
	normal_mode = -1.0f;
	locked_mode = 0.0f;

	return true;
}

void vmount_rc_deinit()
{
	orb_unadvertise(actuator_controls_pub);
	//free(&actuator_controls);
}

void vmount_rc_configure(int roi_mode, bool man_control, int normal_mode_new, int locked_mode_new)
{
	normal_mode = normal_mode_new;
	locked_mode = locked_mode_new;

	switch (roi_mode) {
	case vehicle_roi_s::VEHICLE_ROI_NONE:
		locked = false;

		if (!man_control) {vmount_rc_point_manual(0.0f, 0.0f, 0.0f);}

		break;

	case vehicle_roi_s::VEHICLE_ROI_WPNEXT:
	case vehicle_roi_s::VEHICLE_ROI_WPINDEX:
	case vehicle_roi_s::VEHICLE_ROI_LOCATION:
	case vehicle_roi_s::VEHICLE_ROI_TARGET:
		locked = true;
		break;

	default:
		locked = false;
		vmount_rc_point_manual(0.0f, 0.0f, 0.0f);
		break;
	}
}

void vmount_rc_set_location(double lat_new, double lon_new, float alt_new)
{
	lat = lat_new;
	lon = lon_new;
	alt = alt_new;
}

void vmount_rc_point(double global_lat, double global_lon, float global_alt)
{
	float pitch = vmount_rc_calculate_pitch(global_lat, global_lon, global_alt);
	float roll = 0.0f; // We want a level horizon, so leave roll at 0 degrees.
	float yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon) * (float)M_RAD_TO_DEG;

	vmount_rc_point_manual(pitch, roll, yaw);
}

void vmount_rc_point_manual(float pitch_new, float roll_new, float yaw_new)
{
	actuator_controls.timestamp = hrt_absolute_time();
	actuator_controls.control[0] = pitch_new;
	actuator_controls.control[1] = roll_new;
	actuator_controls.control[2] = yaw_new;
	actuator_controls.control[3] = (locked) ? locked_mode : normal_mode;

	/** for debugging purposes
	warnx("actuator_controls_2 values:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
		 (double)actuator_controls.control[0],
		 (double)actuator_controls.control[1],
		 (double)actuator_controls.control[2],
	     (double)actuator_controls.control[3],
	     (double)actuator_controls.control[4]);
	**/

	orb_publish(ORB_ID(actuator_controls_2), actuator_controls_pub, &actuator_controls);
}

float vmount_rc_calculate_pitch(double global_lat, double global_lon, float global_alt)
{
	float x = (lon - global_lon) * cos(M_DEG_TO_RAD * ((global_lat + lat) * 0.00000005)) * 0.01113195;
	float y = (lat - global_lat) * 0.01113195;
	float z = (alt - global_alt);
	float target_distance = sqrtf(powf(x, 2) + powf(y, 2));

	return atan2(z, target_distance) * M_RAD_TO_DEG;
}
