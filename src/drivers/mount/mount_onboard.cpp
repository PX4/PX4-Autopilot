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
 * @file mount_onboard.cpp
 * @author Leon Müller (thedevleon)
 *
 */

#include "mount_onboard.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <arch/math.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_mount.h>


/* uORB topics */
static struct actuator_controls_s *actuator_controls;
static orb_advert_t actuator_controls_pub;

static struct vehicle_attitude_s *vehicle_attitude;
static int vehicle_attitude_sub;

static int mount_mode;
static float roll;
static float pitch;
static float yaw;
static float retracts;
static int stab_pitch;
static int stab_roll;
static int stab_yaw;

bool mount_onboard_init()
{
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_3), &actuator_controls);
	vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	if (!actuator_controls_pub || !vehicle_attitude_sub) { return false; }

	return true;
}

void mount_onboard_deinit()
{
	free(actuator_controls);
}

void mount_onboard_configure(int new_mount_mode, bool new_stab_roll, bool new_stab_pitch, bool new_stab_yaw)
{
	mount_mode = new_mount_mode;
	stab_roll = new_stab_roll;
	stab_pitch = new_stab_pitch;
	stab_yaw = new_stab_yaw;

	switch (mount_mode) {
	case vehicle_mount_s::VEHICLE_MOUNT_MODE_RETRACT:
		retracts = 1.0f;
		mount_onboard_set_manual(mount_mode, 0.0f, 0.0f, 0.0f);
		break;

	case vehicle_mount_s::VEHICLE_MOUNT_MODE_NEUTRAL:
		retracts = 0.0f;
		mount_onboard_set_manual(mount_mode, 0.0f, 0.0f, 0.0f);
		break;

	case vehicle_mount_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
	case vehicle_mount_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
	case vehicle_mount_s::VEHICLE_MOUNT_MODE_GPS_POINT:
		retracts = 0.0f;

	default:
		break;
	}
}

void mount_onboard_set_location(int new_mount_mode, double global_lat, double global_lon, float global_alt, double lat,
				double lon, float alt)
{
	float new_yaw = get_bearing_to_next_waypoint(global_lat, global_lon, lat, lon);
	float new_pitch = 0.0f; //TODO calculate pitch
	float new_roll = 0.0f; //TODO calculate yaw

	mount_onboard_set_manual(new_mount_mode, new_pitch, new_roll, new_yaw);
}

void mount_onboard_set_manual(int new_mount_mode, float new_pitch, float new_roll, float new_yaw)
{
	mount_mode = new_mount_mode;
	pitch = new_pitch;
	roll = new_roll;
	yaw = new_yaw;
}

void mount_onboard_point()
{
	if (mount_mode != vehicle_mount_s::VEHICLE_MOUNT_MODE_RETRACT &&
	    mount_mode != vehicle_mount_s::VEHICLE_MOUNT_MODE_NEUTRAL) {
		if (stab_roll) {
			roll += 1.0f / M_PI_F * -vehicle_attitude->roll;
		}

		if (stab_pitch) {
			pitch += 1.0f / M_PI_F * -vehicle_attitude->pitch;
		}

		if (stab_yaw) {
			yaw += 1.0f / M_PI_F * vehicle_attitude->yaw;
		}
	}

	actuator_controls->timestamp = hrt_absolute_time();
	actuator_controls->control[0] = pitch;
	actuator_controls->control[1] = roll;
	actuator_controls->control[2] = yaw;
	actuator_controls->control[3] = retracts;

	orb_publish(ORB_ID(actuator_controls_3), actuator_controls_pub, &actuator_controls);
}

void mount_onboard_update_topics()
{
	bool updated;
	orb_check(vehicle_attitude_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, vehicle_attitude);
	}
}
