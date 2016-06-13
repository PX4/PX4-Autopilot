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
 * @file vmount.cpp
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

#include "vmount_mavlink.h"
#include "vmount_rc.h"
#include "vmount_onboard.h"

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <px4_config.h>
#include <px4_posix.h>
#include <poll.h>

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static volatile bool thread_should_restart = false;
static int vmount_task;
typedef enum { IDLE = -1, MAVLINK = 0, RC = 1, ONBOARD = 2 } vmount_state_t;
static vmount_state_t vmount_state = IDLE;

/* polling */
px4_pollfd_struct_t polls[7];

/* functions */
static void usage(void);
static void vmount_update_topics(void);
static void update_params(void);
static bool get_params(void);
static float get_aux_value(int);
static void ack_mount_command(uint16_t command);
static int vmount_thread_main(int argc, char *argv[]);
extern "C" __EXPORT int vmount_main(int argc, char *argv[]);

/* uORB subscriptions */
static int vehicle_roi_sub = -1;
static int vehicle_global_position_sub = -1;
static int vehicle_command_sub = -1;
static int vehicle_attitude_sub = -1;
static int position_setpoint_triplet_sub = -1;
static int manual_control_setpoint_sub = -1;
static int parameter_update_sub = -1;

/* uORB publication */
static orb_advert_t vehicle_command_ack_pub;

static struct vehicle_roi_s vehicle_roi;
static bool   vehicle_roi_updated;

static struct vehicle_global_position_s vehicle_global_position;
static bool   vehicle_global_position_updated;

static struct vehicle_command_s vehicle_command;
static bool   vehicle_command_updated;

static struct vehicle_attitude_s vehicle_attitude;
static bool   vehicle_attitude_updated;

static struct position_setpoint_triplet_s position_setpoint_triplet;
static bool   position_setpoint_triplet_updated;

static struct manual_control_setpoint_s manual_control_setpoint;
static bool   manual_control_setpoint_updated;

static struct parameter_update_s parameter_update;
static bool   parameter_update_updated;

static struct vehicle_command_ack_s vehicle_command_ack;

static struct {
	int mnt_mode;
	int mnt_mav_sysid;
	int mnt_mav_compid;
	int mnt_ob_lock_mode;
	int mnt_ob_norm_mode;
	int mnt_man_control;
	int mnt_man_pitch;
	int mnt_man_roll;
	int mnt_man_yaw;
	int mnt_mode_ovr;
} params;

static struct {
	param_t mnt_mode;
	param_t mnt_mav_sysid;
	param_t mnt_mav_compid;
	param_t mnt_ob_lock_mode;
	param_t mnt_ob_norm_mode;
	param_t mnt_man_control;
	param_t mnt_man_pitch;
	param_t mnt_man_roll;
	param_t mnt_man_yaw;
	param_t mnt_mode_ovr;
} params_handels;

static bool manual_control_desired;

/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: vmount start\n"
		"       vmount stop\n"
		"       vmount status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int vmount_thread_main(int argc, char *argv[])
{
	if (!get_params()) {
		warnx("could not get mount parameters!");
	}

	if (params.mnt_mode == 0) { vmount_state = IDLE;}

	else if (params.mnt_mode == 1) { vmount_state = MAVLINK;}

	else if (params.mnt_mode == 2) { vmount_state = RC;}

	else if (params.mnt_mode == 3) { vmount_state = ONBOARD;}

	//TODO is this needed?
	memset(&vehicle_roi, 0, sizeof(vehicle_roi));
	memset(&vehicle_global_position, 0, sizeof(vehicle_global_position));
	memset(&vehicle_command, 0, sizeof(vehicle_command));
	memset(&vehicle_attitude, 0, sizeof(vehicle_attitude));
	memset(&position_setpoint_triplet, 0, sizeof(position_setpoint_triplet));
	memset(&manual_control_setpoint, 0, sizeof(manual_control_setpoint));
	memset(&parameter_update, 0, sizeof(parameter_update));
	memset(&vehicle_command_ack, 0, sizeof(vehicle_command_ack));


	vehicle_roi_sub = orb_subscribe(ORB_ID(vehicle_roi));
	vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	position_setpoint_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	vehicle_command_ack_pub = orb_advertise(ORB_ID(vehicle_command_ack), &vehicle_command_ack);

	if (!vehicle_roi_sub || !position_setpoint_triplet_sub ||
	    !manual_control_setpoint_sub || !vehicle_global_position_sub ||
	    !vehicle_command_sub || !parameter_update_sub || !vehicle_command_ack_pub ||
		!vehicle_attitude_sub) {
		err(1, "could not subscribe to uORB topics");
	}

	polls[0].fd = 		vehicle_roi_sub;
	polls[0].events = 	POLLIN;
	polls[1].fd = 		vehicle_global_position_sub;
	polls[1].events = 	POLLIN;
	polls[2].fd = 		vehicle_attitude_sub;
	polls[2].events = 	POLLIN;
	polls[3].fd = 		vehicle_command_sub;
	polls[3].events = 	POLLIN;
	polls[4].fd = 		position_setpoint_triplet_sub;
	polls[4].events = 	POLLIN;
	polls[5].fd = 		manual_control_setpoint_sub;
	polls[5].events = 	POLLIN;
	polls[6].fd = 		parameter_update_sub;
	polls[6].events = 	POLLIN;

	thread_running = true;

	run: {
		if (vmount_state == MAVLINK) {
			if (!vmount_mavlink_init()) {
				err(1, "could not initiate vmount_mavlink");
			}

			warnx("running mount driver in mavlink mode");

			if (params.mnt_man_control) manual_control_desired = true;

			while (!thread_should_exit && !thread_should_restart) {
				vmount_update_topics();

				if (vehicle_roi_updated) {
					vehicle_roi_updated = false;
					vmount_mavlink_configure(vehicle_roi.mode, (params.mnt_man_control == 1), params.mnt_mav_sysid, params.mnt_mav_compid);

					if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_NONE) {
						if (params.mnt_man_control) manual_control_desired = true;

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT) {
						manual_control_desired = false;
						vmount_mavlink_set_location(
							position_setpoint_triplet.next.lat,
							position_setpoint_triplet.next.lon,
							position_setpoint_triplet.next.alt
						);

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX) {
						manual_control_desired = false;
						//TODO how to do this?

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_LOCATION) {
						manual_control_desired = false;
						vmount_mavlink_set_location(
							vehicle_roi.lat,
							vehicle_roi.lon,
							vehicle_roi.alt
						);

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_TARGET) {
						manual_control_desired = false;
						//TODO is this even suported?
					}
				}

				else if (manual_control_desired && manual_control_setpoint_updated)
				{
					manual_control_setpoint_updated = false;
					vmount_mavlink_point_manual(get_aux_value(params.mnt_man_pitch), get_aux_value(params.mnt_man_roll), get_aux_value(params.mnt_man_yaw));
				}

				else if (!manual_control_desired && vehicle_global_position_updated)
				{
					vehicle_global_position_updated = false;
					vmount_mavlink_point(vehicle_global_position.lat, vehicle_global_position.lon, vehicle_global_position.alt);
				}

			vmount_mavlink_deinit();
			}

		} else if (vmount_state == RC) {
			if (!vmount_rc_init()) {
				err(1, "could not initiate vmount_rc");
			}

			warnx("running mount driver in rc mode");

			if (params.mnt_man_control) {
				manual_control_desired = true;
			}

			while (!thread_should_exit && !thread_should_restart) {
				vmount_update_topics();

				if (vehicle_roi_updated) {
					vehicle_roi_updated = false;
					vmount_rc_configure(vehicle_roi.mode, (params.mnt_man_control == 1), params.mnt_ob_norm_mode, params.mnt_ob_lock_mode);

					if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_NONE) {
						if (params.mnt_man_control) manual_control_desired = true;

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPNEXT) {
						manual_control_desired = false;
						vmount_rc_set_location(
							position_setpoint_triplet.next.lat,
							position_setpoint_triplet.next.lon,
							position_setpoint_triplet.next.alt
						);

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_WPINDEX) {
						manual_control_desired = false;
						//TODO how to do this?
					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_LOCATION) {
						manual_control_desired = false;
						vmount_rc_set_location(
							vehicle_roi.lat,
							vehicle_roi.lon,
							vehicle_roi.alt
						);

					} else if (vehicle_roi.mode == vehicle_roi_s::VEHICLE_ROI_TARGET) {
						manual_control_desired = false;
						//TODO is this even suported?
					}
				}

				else if (manual_control_desired && manual_control_setpoint_updated) {
					manual_control_setpoint_updated = false;
					vmount_rc_point_manual(get_aux_value(params.mnt_man_pitch), get_aux_value(params.mnt_man_roll), get_aux_value(params.mnt_man_yaw));
				}

				else if (!manual_control_desired && vehicle_global_position_updated)
				{
					vehicle_global_position_updated = false;
					vmount_rc_point(vehicle_global_position.lat, vehicle_global_position.lon, vehicle_global_position.alt);
				}
			}

			vmount_rc_deinit();

		} else if (vmount_state == ONBOARD) {
			if (!vmount_onboard_init()) {
				err(1, "could not initiate vmount_onboard");
			}

			warnx("running mount driver in onboard mode");

			if (params.mnt_man_control) manual_control_desired = true;

			while (!thread_should_exit && !thread_should_restart) {
				vmount_update_topics();

				if(vehicle_attitude_updated)
				{
					vmount_onboard_update_attitude(vehicle_attitude);
					vehicle_attitude_updated = false;
				}

				if (params.mnt_mode_ovr && manual_control_setpoint_updated) {
					manual_control_setpoint_updated = false;

					float ovr_value = get_aux_value(params.mnt_mode_ovr);

					if(ovr_value < 0.0f)
					{
						vmount_onboard_set_mode(vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT);
					}
					else if (ovr_value > 0.0f)
					{
						vmount_onboard_set_mode(vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING);
					}

				}

				else if (vehicle_command_updated) {
					vehicle_command_updated = false;

					if(vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONTROL)
					{
						switch ((int)vehicle_command.param7) {
						case vehicle_command_s::VEHICLE_MOUNT_MODE_RETRACT:
						case vehicle_command_s::VEHICLE_MOUNT_MODE_NEUTRAL:
							manual_control_desired = false;
							break;

						case vehicle_command_s::VEHICLE_MOUNT_MODE_MAVLINK_TARGETING:
							manual_control_desired = false;
							vmount_onboard_set_mode(vehicle_command.param7);
							vmount_onboard_set_manual(vehicle_command.param1,
										 vehicle_command.param2, vehicle_command.param3);

						case vehicle_command_s::VEHICLE_MOUNT_MODE_RC_TARGETING:
							if (params.mnt_man_control) {
								manual_control_desired = true;
								vmount_onboard_set_mode(vehicle_command.param7);
							}

						case vehicle_command_s::VEHICLE_MOUNT_MODE_GPS_POINT:
							manual_control_desired = false;
							vmount_onboard_set_mode(vehicle_command.param7);
							vmount_onboard_set_location(
								vehicle_command.param1,
								vehicle_command.param2,
								vehicle_command.param3);

						default:
							manual_control_desired = false;
							break;
						}

						ack_mount_command(vehicle_command.command);
					}

					else if(vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_MOUNT_CONFIGURE)
					{
						vmount_onboard_configure(vehicle_command.param1,
									((uint8_t) vehicle_command.param2 == 1), ((uint8_t) vehicle_command.param3 == 1), ((uint8_t) vehicle_command.param4 == 1));

						ack_mount_command(vehicle_command.command);
					}
				}

				else if (manual_control_desired && manual_control_setpoint_updated)
				{
					manual_control_setpoint_updated = false;
					vmount_onboard_point_manual(get_aux_value(params.mnt_man_pitch), get_aux_value(params.mnt_man_roll), get_aux_value(params.mnt_man_yaw));
				}
				else if (!manual_control_desired && vehicle_global_position_updated)
				{
					vehicle_global_position_updated = false;
					vmount_onboard_point(vehicle_global_position.lat, vehicle_global_position.lon, vehicle_global_position.alt);
				}
			}
		} else if (vmount_state == IDLE)
		{
			warnx("running mount driver in idle mode");

			while (!thread_should_exit && !thread_should_restart) {
				vmount_update_topics();
			}
		}
	}

	if (thread_should_restart)
	{
		thread_should_restart = false;
		warnx("parameter update, restarting mount driver!");
		goto run;
	}

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int vmount_main(int argc, char *argv[])
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
		vmount_task = px4_task_spawn_cmd("vmount",
						SCHED_DEFAULT,
						SCHED_PRIORITY_DEFAULT + 40, //TODO we might want a higher priority?
						1100,
						vmount_thread_main,
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
			switch (vmount_state) {
			case IDLE:
				errx(0, "running: IDLE");
				break;

			case MAVLINK:
				errx(0, "running: MAVLINK - Manual Control? %d", manual_control_desired);
				break;

			case RC:
				errx(0, "running: RC - Manual Control? %d", manual_control_desired);
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
void vmount_update_topics()
{
	/*
	polls = {
		{ .fd = vehicle_roi_sub,   .events = POLLIN },
		{ .fd = vehicle_global_position_sub,   .events = POLLIN },
		{ .fd = vehicle_attitude_sub,   .events = POLLIN },
		{ .fd = vehicle_command_sub,   .events = POLLIN },
		{ .fd = position_setpoint_triplet_sub,   .events = POLLIN },
		{ .fd = manual_control_setpoint_sub,   .events = POLLIN },
		{ .fd = parameter_update_sub,   .events = POLLIN },
	};
	*/

	/* wait for sensor update of 7 file descriptors for 100 ms */
	int poll_ret = px4_poll(polls, 7, 100);

	//Nothing updated.
	if(poll_ret == 0) return;

	if (polls[0].revents & POLLIN) {
		orb_copy(ORB_ID(vehicle_roi), vehicle_roi_sub, &vehicle_roi);
		vehicle_roi_updated = true;
	}

	if (polls[1].revents & POLLIN) {
		orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub, &vehicle_global_position);
		vehicle_global_position_updated = true;
	}

	if (polls[2].revents & POLLIN) {
		orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);
		vehicle_attitude_updated = true;
	}

	if (polls[3].revents & POLLIN) {
		orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &vehicle_command);
		vehicle_command_updated = true;
	}

	if (polls[4].revents & POLLIN) {
		orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_sub, &position_setpoint_triplet);
		position_setpoint_triplet_updated = true;
	}

	if (polls[5].revents & POLLIN) {
		orb_copy(ORB_ID(manual_control_setpoint), manual_control_setpoint_sub, &manual_control_setpoint);
		manual_control_setpoint_updated = true;
	}

	if (polls[6].revents & POLLIN) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &parameter_update);
		parameter_update_updated = true;
		update_params();
	}
}

void update_params()
{
	param_get(params_handels.mnt_mode, &params.mnt_mode);
	param_get(params_handels.mnt_mav_sysid, &params.mnt_mav_sysid);
	param_get(params_handels.mnt_mav_compid, &params.mnt_mav_compid);
	param_get(params_handels.mnt_ob_lock_mode, &params.mnt_ob_lock_mode);
	param_get(params_handels.mnt_ob_norm_mode, &params.mnt_ob_norm_mode);
	param_get(params_handels.mnt_man_control, &params.mnt_man_control);
	param_get(params_handels.mnt_man_pitch, &params.mnt_man_pitch);
	param_get(params_handels.mnt_man_roll, &params.mnt_man_roll);
	param_get(params_handels.mnt_man_yaw, &params.mnt_man_yaw);
	param_get(params_handels.mnt_mode_ovr, &params.mnt_mode_ovr);

	if (vmount_state != params.mnt_mode)
	{
		thread_should_restart = true;
	}
	else if (vmount_state == MAVLINK)
	{
		vmount_mavlink_configure(vehicle_roi.mode, (params.mnt_man_control == 1), params.mnt_mav_sysid, params.mnt_mav_compid);
	}
	else if (vmount_state == RC)
	{
		vmount_rc_configure(vehicle_roi.mode, (params.mnt_man_control == 1), params.mnt_ob_norm_mode, params.mnt_ob_lock_mode);
	}
	else if(vmount_state == ONBOARD)
	{
		//None of the parameter changes require a reconfiguration of the onboard mount.
	}
}

bool get_params()
{
	params_handels.mnt_mode = param_find("MNT_MODE");
	params_handels.mnt_mav_sysid = param_find("MNT_MAV_SYSID");
	params_handels.mnt_mav_compid = param_find("MNT_MAV_COMPID");
	params_handels.mnt_ob_lock_mode = param_find("MNT_OB_LOCK_MODE");
	params_handels.mnt_ob_norm_mode = param_find("MNT_OB_NORM_MODE");
	params_handels.mnt_man_control = param_find("MNT_MAN_CONTROL");
	params_handels.mnt_man_pitch = param_find("MNT_MAN_PITCH");
	params_handels.mnt_man_roll = param_find("MNT_MAN_ROLL");
	params_handels.mnt_man_yaw = param_find("MNT_MAN_YAW");
	params_handels.mnt_mode_ovr = param_find("MNT_MODE_OVR");

	if (param_get(params_handels.mnt_mode, &params.mnt_mode) ||
	    param_get(params_handels.mnt_mav_sysid, &params.mnt_mav_sysid) ||
	    param_get(params_handels.mnt_mav_compid, &params.mnt_mav_compid) ||
		param_get(params_handels.mnt_ob_lock_mode, &params.mnt_ob_lock_mode) ||
		param_get(params_handels.mnt_ob_norm_mode, &params.mnt_ob_norm_mode) ||
	    param_get(params_handels.mnt_man_control, &params.mnt_man_control) ||
	    param_get(params_handels.mnt_man_pitch, &params.mnt_man_pitch) ||
		param_get(params_handels.mnt_man_roll, &params.mnt_man_roll) ||
	    param_get(params_handels.mnt_man_yaw, &params.mnt_man_yaw) ||
		param_get(params_handels.mnt_mode_ovr, &params.mnt_mode_ovr)) {
		return false;
	}

	return true;

}

void ack_mount_command(uint16_t command)
{
	vehicle_command_ack.command = command;
	vehicle_command_ack.result = vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED;

	orb_publish(ORB_ID(vehicle_command_ack), vehicle_command_ack_pub, &vehicle_command_ack);
}

float get_aux_value(int aux)
{
	switch (aux)
	{
		case 0:
			return 0.0f;
		case 1:
			return manual_control_setpoint.aux1;
		case 2:
			return manual_control_setpoint.aux2;
		case 3:
			return manual_control_setpoint.aux3;
		case 4:
			return manual_control_setpoint.aux4;
		case 5:
			return manual_control_setpoint.aux5;
		default:
			return 0.0f;
	}
}
