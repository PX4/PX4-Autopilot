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
 * @file mount.cpp
 * @author Leon Müller (thedevleon)
 * MAV_MOUNT driver for controlling mavlink gimbals, rc gimbals/servors and
 * future kinds of mounts.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include "mount_mavlink.h"
#include "mount_rc.h"
#include "mount_onboard.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_mount.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <px4_config.h>

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int mount_task;
typedef enum { IDLE, MAVLINK, RC, ONBOARD } mount_state_t;
static mount_state_t mount_state = IDLE;

/* functions */
static void usage(void);
static void mount_update_topics(void);
static bool get_params(void);
static int mount_thread_main(int argc, char *argv[]);
__EXPORT int mount_main(int argc, char *argv[]);

/* uORB subscriptions */
static int vehicle_roi_sub = -1;
static int vehicle_global_position_sub = -1;
static int vehicle_mount_sub = -1;
static int position_setpoint_triplet_sub = -1;
static int manual_control_setpoint_sub = -1;

static struct vehicle_roi_s *vehicle_roi;
static bool   vehicle_roi_updated;

static struct vehicle_global_position_s *vehicle_global_position;
static bool   vehicle_global_position_updated;

static struct vehicle_mount_s *vehicle_mount;
static bool   vehicle_mount_updated;

static struct position_setpoint_triplet_s *position_setpoint_triplet;
static bool   position_setpoint_triplet_updated;

static struct manual_control_setpoint_s *manual_control_setpoint;
static bool   manual_control_setpoint_updated;

static struct {
	int mnt_mode;
	int mnt_mav_sysid;
	int mnt_mav_compid;
	int mnt_man_control;
	int mnt_man_roll;
	int mnt_man_pitch;
	int mnt_man_yaw;
} params;

static struct {
	param_t mnt_mode;
	param_t mnt_mav_sysid;
	param_t mnt_mav_compid;
	param_t mnt_man_control;
	param_t mnt_man_roll;
	param_t mnt_man_pitch;
	param_t mnt_man_yaw;
} params_handels;


/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: mount start\n"
		"       mount stop\n"
		"       mount status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int mount_thread_main(int argc, char *argv[])
{
	if(!get_params())
	{
		err(1, "could not get mount parameters!");
	}

	if(params.mnt_mode == 0) { mount_state = IDLE;}
    else if(params.mnt_mode == 1) { mount_state = MAVLINK;}
    else if(params.mnt_mode == 2) { mount_state = RC;}
	else if(params.mnt_mode == 3) { mount_state = ONBOARD;}

	//TODO is this needed?
	memset(&vehicle_roi, 0, sizeof(vehicle_roi));
	memset(&vehicle_global_position, 0, sizeof(vehicle_global_position));
	memset(&vehicle_mount, 0, sizeof(vehicle_mount));
	memset(&position_setpoint_triplet, 0, sizeof(position_setpoint_triplet));
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));

    vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));
	vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	vehicle_mount_sub = orb_subscribe(ORB_ID(vehicle_mount));
    position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	if(!vehicle_roi_sub || !position_setpoint_triplet_sub ||
	   !manual_control_setpoint_sub || !vehicle_global_position_sub ||
   	   !vehicle_mount_sub)
	{
		err(1, "could not subscribe to uORB topics");
	}

	thread_running = true;

	if (mount_state == MAVLINK) {
		if (!mount_mavlink_init(params.mnt_mav_sysid, params.mnt_mav_compid)) {
			err(1, "could not initiate mount_mavlink");
		}

		warnx("running mount driver in mavlink mode");

		while (!thread_should_exit) {
            mount_update_topics();

			if(vehicle_roi_updated)
			{
				vehicle_roi_updated = false;
				mount_mavlink_configure(vehicle_roi->mode, (params.mnt_man_control == 1));

				if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_NONE)
				{
					if(params.mnt_man_control && manual_control_setpoint_updated)
					{
						//TODO use mount_mavlink_point_manual to control gimbal
						//with specified aux channels via the parameters
					}
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT)
				{
					mount_mavlink_point_location(
						vehicle_global_position->lat,
						vehicle_global_position->lon,
						vehicle_global_position->alt,
						position_setpoint_triplet->next.lat,
						position_setpoint_triplet->next.lon,
						position_setpoint_triplet->next.alt
					);
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX)
				{
					//TODO how to do this?
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_LOCATION)
				{
					mount_mavlink_point_location(
						vehicle_global_position->lat,
						vehicle_global_position->lon,
						vehicle_global_position->alt,
						vehicle_roi->lat,
						vehicle_roi->lon,
						vehicle_roi->alt
					);
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_TARGET)
				{
					//TODO is this even suported?
				}
			}
		}

		mount_mavlink_deinit();
    }
    else if (mount_state == RC)
    {
        if (!mount_rc_init()) {
            err(1, "could not initiate mount_rc");
        }

        warnx("running mount driver in rc mode");

        while (!thread_should_exit) {
            mount_update_topics();

			if(vehicle_roi_updated)
			{
				vehicle_roi_updated = false;
				mount_rc_configure(vehicle_roi->mode, (params.mnt_man_control == 1));

				if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_NONE)
				{
					if(params.mnt_man_control && manual_control_setpoint_updated)
					{
						//TODO use mount_rc_point_manual to control gimbal
						//with specified aux channels via the parameters
					}
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT)
				{
					mount_rc_set_location(
						vehicle_global_position->lat,
						vehicle_global_position->lon,
						vehicle_global_position->alt,
						position_setpoint_triplet->next.lat,
						position_setpoint_triplet->next.lon,
						position_setpoint_triplet->next.alt
					);
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX)
				{
					//TODO how to do this?
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_LOCATION)
				{
					mount_rc_set_location(
						vehicle_global_position->lat,
						vehicle_global_position->lon,
						vehicle_global_position->alt,
						vehicle_roi->lat,
						vehicle_roi->lon,
						vehicle_roi->alt
					);
				}
				else if(vehicle_roi->mode == vehicle_roi_s::VEHICLE_ROI_TARGET)
				{
					//TODO is this even suported?
				}
			}
			mount_rc_point();
		}

        mount_rc_deinit();
    }
	else if (mount_state == ONBOARD)
	{
		if (!mount_onboard_init()) {
			err(1, "could not initiate mount_onboard");
		}

		warnx("running mount driver in onboard mode");

		while (!thread_should_exit) {
			mount_update_topics();
			mount_onboard_update_topics();

			if(vehicle_mount_updated)
			{
				if(vehicle_mount->config)
				{
					mount_onboard_configure(vehicle_mount->mode,
						vehicle_mount->stab_roll, vehicle_mount->stab_pitch, vehicle_mount->stab_yaw);
				}
				else
				{
					switch (vehicle_mount->mode) {
						case vehicle_mount_s::VEHICLE_MOUNT_MODE_RETRACT:
						case vehicle_mount_s::VEHICLE_MOUNT_MODE_NEUTRAL:
							break;
						case vehicle_mount_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
							mount_onboard_set_manual(vehicle_mount->mode, vehicle_mount->pitch_lat,
								vehicle_mount->roll_lon, vehicle_mount->yaw_alt);
						case vehicle_mount_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
							if(params.mnt_man_control && manual_control_setpoint_updated){
								//TODO use defined aux channels to control mount
							}
						case vehicle_mount_s::VEHICLE_MOUNT_MODE_GPS_POINT:
							mount_onboard_set_location(
								vehicle_mount->mode,
								vehicle_global_position->lat,
								vehicle_global_position->lon,
								vehicle_global_position->alt,
								vehicle_mount->pitch_lat,
								vehicle_mount->roll_lon,
								vehicle_mount->yaw_alt);
						default:
							break;
					}
				}
			}
			mount_onboard_point();
		}
	}

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int mount_main(int argc, char *argv[])
{
	if (argc < 1) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running) {
			errx(0, "mount driver already running");
		}

		thread_should_exit = false;
		mount_task = px4_task_spawn_cmd("mount",
						SCHED_DEFAULT, //TODO we might want a higher priority?
						200,
						1100,
						mount_thread_main,
						(char *const *)argv);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			errx(0, "mount driver already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(1000000);
			warnx(".");
		}

		warnx("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			switch (mount_state) {
			case IDLE:
				errx(0, "running: IDLE");
				break;

			case MAVLINK:
				errx(0, "running: MAVLINK");
				break;

			case RC:
				errx(0, "running: RC");
				break;

			case ONBOARD:
				errx(0, "running: ONBOARD");
				break;

			}

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	usage();
	/* not getting here */
	return 0;
}

/* Update oURB topics */
void mount_update_topics()
{
    orb_check(vehicle_roi_sub, &vehicle_roi_updated);
    if (vehicle_roi_updated) {
        orb_copy(ORB_ID(vehicle_roi), vehicle_roi_sub, vehicle_roi);
    }

	orb_check(vehicle_mount_sub, &vehicle_mount_updated);
	if (vehicle_mount_updated) {
		orb_copy(ORB_ID(vehicle_mount), vehicle_mount_sub, vehicle_mount);
	}

    orb_check(position_setpoint_triplet_sub, &position_setpoint_triplet_updated);
    if (position_setpoint_triplet_updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_sub, position_setpoint_triplet);
    }

    orb_check(manual_control_setpoint_sub, &manual_control_setpoint_updated);
    if (manual_control_setpoint_updated) {
        orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, manual_control_setpoint);
    }

	orb_check(vehicle_global_position_sub, &vehicle_global_position_updated);
	if (vehicle_global_position_updated) {
		orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, vehicle_global_position);
	}
}

bool get_params()
{
	params_handels.mnt_mode = param_find("MNT_MODE");
	params_handels.mnt_mav_sysid = param_find("MNT_MAV_SYSID");
	params_handels.mnt_mav_compid = param_find("MNT_MAV_COMPID");
	params_handels.mnt_man_control = param_find("MNT_MAN_CONTROL");
	params_handels.mnt_man_roll = param_find("MNT_MAN_ROLL");
	params_handels.mnt_man_pitch = param_find("MNT_MAN_PITCH");
	params_handels.mnt_man_yaw = param_find("MNT_MAN_YAW");


	if (!param_get(params_handels.mnt_mode, &params.mnt_mode) |
		!param_get(params_handels.mnt_mav_sysid, &params.mnt_mav_sysid) |
		!param_get(params_handels.mnt_mav_compid, &params.mnt_mav_compid) |
		!param_get(params_handels.mnt_man_control, &params.mnt_man_control) |
		!param_get(params_handels.mnt_man_roll, &params.mnt_man_roll) |
		!param_get(params_handels.mnt_man_pitch, &params.mnt_mode) |
		!param_get(params_handels.mnt_man_yaw, &params.mnt_man_yaw))
	{
		return false;
	}

	return true;

}
