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
 * @file mount_rc.c
 * @author Leon Müller (thedevleon)
 *
 */

#include "mount_rc.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/actuator_controls.h>

/* uORB advertising */
static struct actuator_controls_s *actuator_controls;
static orb_advert_t actuator_controls_pub;

static float roll;
static float pitch;
static float yaw;

bool mount_rc_init()
{
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_3), &actuator_controls);

	if (!actuator_controls_pub) { return false; }

	return true;
}

void mount_rc_deinit()
{
	free(actuator_controls);
}

void mount_rc_configure(int roi_mode, bool man_control)
{
	switch (roi_mode) {
	case vehicle_roi_s::VEHICLE_ROI_NONE:
		if (!man_control) {mount_rc_set_manual(0.0f, 0.0f, 0.0f);}
		break;

	case vehicle_roi_s::VEHICLE_ROI_WPNEXT:
	case vehicle_roi_s::VEHICLE_ROI_WPINDEX:
	case vehicle_roi_s::VEHICLE_ROI_LOCATION:
	case vehicle_roi_s::VEHICLE_ROI_TARGET:
		break;

	default:
		mount_rc_set_manual(0.0f, 0.0f, 0.0f);
		break;
	}
}

void mount_rc_set_location(double global_lat, double global_lon, float global_alt, double lat, double lon, float alt)
{
	float new_roll = 0.0f; // We want a level horizon, so leave roll at 0 degrees.
	float new_pitch = mount_rc_calculate_pitch(global_lat, global_lon, global_alt, lat, lon, alt);
	float new_yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon) * (float)M_RAD_TO_DEG;

	mount_rc_set_manual(new_pitch, new_roll, new_yaw);
}

void mount_rc_set_manual(float new_pitch, float new_roll, float new_yaw)
{
	pitch = new_pitch;
	roll = new_roll;
	yaw = new_yaw;
}

void mount_rc_point()
{
	actuator_controls->timestamp = hrt_absolute_time();
	actuator_controls->control[0] = pitch;
	actuator_controls->control[1] = roll;
	actuator_controls->control[2] = yaw;

	orb_publish(ORB_ID(actuator_controls_3), actuator_controls_pub, &actuator_controls);
}

float mount_rc_calculate_pitch(double global_lat, double global_lon, float global_alt, double lat, double lon,
			       float alt)
{
	float x = (lon - global_lon) * cos(M_DEG_TO_RAD * ((global_lat + lat) * 0.00000005)) * 0.01113195;
	float y = (lat - global_lat) * 0.01113195;
	float z = (alt - global_alt);
	float target_distance = sqrtf(powf(x, 2) + powf(y, 2));

	return atan2(z, target_distance) * M_RAD_TO_DEG;
}
