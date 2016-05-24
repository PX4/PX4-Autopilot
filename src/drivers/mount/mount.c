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
 * @file mount.c
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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_roi.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rc_channels.h>

/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int mount_task;
typedef enum { IDLE, MAVLINK, RC } mount_state_t;
static mount_state_t mount_state = IDLE;

/* functions */
static void usage(void);
static void mount_update_topics(void);
static int mount_thread_main(int argc, char *argv[]);
__EXPORT int mount_main(int argc, char *argv[]);

/* uORB subscriptions */
static int vehicle_roi_sub = -1;
static int position_setpoint_triplet_sub = -1;
static int rc_channels_sub = -1;

static struct vehicle_roi_s *vehicle_roi;
static bool   vehicle_roi_updated;

static struct position_setpoint_triplet_s *position_setpoint_triplet;
static bool   position_setpoint_triplet_updated;

static struct rc_channels_s *rc_channels;
static bool   rc_channels_updated;


/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: mount start [-t mavlink|rc]\n"
		"       mount stop\n"
		"       mount status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int mount_thread_main(int argc, char *argv[])
{
	/* Default values for arguments */
	char *mount_type = "mavlink"; /* MAVLINK by default */

	/* Work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	int ch;

	while ((ch = getopt(argc, argv, "t:")) != EOF) {
		switch (ch) {
		case 't':
			mount_type = optarg;
			break;

		default:
			usage();
			break;
		}
	}

    if(mount_type == "mavlink") { mount_state = MAVLINK;}
    else if(mount_type == "rc") { mount_state = RC;}
    else                        { mount_state = IDLE;}

    vehicle_roi = malloc(sizeof(struct vehicle_roi_s));
    position_setpoint_triplet = malloc(sizeof(struct position_setpoint_triplet_s));
    rc_channels = malloc(sizeof(struct rc_channels_s));

    if(vehicle_roi == NULL || position_setpoint_triplet == NULL || rc_channels == NULL)
    {
        err(1, "could not allocate memory for uORB topics");
    }

    vehicle_roi = orb_subscribe(ORB_ID(vehicle_roi));
    position_setpoint_triplet = orb_subscribe(ORB_ID(position_setpoint_triplet));
    rc_channels = orb_subscribe(ORB_ID(rc_channels));

	thread_running = true;

	if (mount_state == MAVLINK) {
		if (!mount_mavlink_init()) {
			err(1, "could not initiate mount_mavlink");
		}

		warnx("running mount driver in mavlink mode");

		while (!thread_should_exit) {
            mount_update_topics();
            //TODO switch by roi type
            // ROI_NONE will call mount_mavlink_point_manual
            // with the values from the assigned rc channels for yaw and pitch
            // so the gimbal can be controlled by the remote if there's no ROI set.
            // other ones will call mount_mavlink_point_location
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
            //TODO same as above
        }

        mount_rc_deinit();
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
	if (argc < 2) {
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
						SCHED_DEFAULT,
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

    orb_check(position_setpoint_triplet_sub, &position_setpoint_triplet_updated);
    if (position_setpoint_triplet_updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), position_setpoint_triplet_sub, position_setpoint_triplet);
    }

    orb_check(rc_channels_sub, &rc_channel_updated);
    if (rc_channel_updated) {
        orb_copy(ORB_ID(rc_channels), rc_channels_sub, rc_channels);
    }
}
