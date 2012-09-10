/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file sdlog.c
 * Simple SD logger for flight data
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */

static const char *mountpoint = "/fs/microsd";

/**
 * Deamon management function.
 */
__EXPORT int sdlog_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int sdlog_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int file_exist(const char *filename);

/**
 * Create folder for current logging session.
 */
static int create_logfolder(char* folder_path);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	errx(1, "usage: sdlog {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int sdlog_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sdlog already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_create("sdlog", SCHED_PRIORITY_DEFAULT - 10, 4096, sdlog_thread_main, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tsdlog is running\n");
		} else {
			printf("\tsdlog not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int create_logfolder(char* folder_path) {
	/* make folder on sdcard */
	uint16_t foldernumber = 1; // start with folder 0001
	int mkdir_ret;

	/* look for the next folder that does not exist */
	while (foldernumber < MAX_NO_LOGFOLDER) {
		/* set up file path: e.g. /mnt/sdcard/sensorfile0001.txt */
		sprintf(folder_path, "%s/session%04u", mountpoint, foldernumber);
		mkdir_ret = mkdir(folder_path, S_IRWXU | S_IRWXG | S_IRWXO);
		/* the result is -1 if the folder exists */

		if (mkdir_ret == 0) {
			/* folder does not exist */
			break;

		} else if (mkdir_ret == -1) {
			/* folder exists already */
			foldernumber++; // to try next time
			continue;

		} else {
			warn("failed creating new folder");
			return -1;
		}
	}

	if (foldernumber >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		warn("all %d possible folders exist already", MAX_NO_LOGFOLDER);
		return -1;
	}
	return 0;
}


int sdlog_thread_main(int argc, char *argv[]) {

	printf("[sdlog] starting\n");

	if (file_exist(mountpoint) != OK) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	char folder_path[64];
	if (create_logfolder(folder_path))
		errx(1, "unable to create logging folder, exiting");

	/* create sensorfile */
	int sensorfile;
	FILE *gpsfile;
	// FILE *vehiclefile;

	char path_buf[64] = ""; // string to hold the path to the sensorfile

	/* set up file path: e.g. /mnt/sdcard/session0001/sensors_combined.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "sensors_combined");
	if (0 == (sensorfile = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	/* set up file path: e.g. /mnt/sdcard/session0001/gps.txt */
	sprintf(path_buf, "%s/%s.txt", folder_path, "gps");
	if (NULL == (gpsfile = fopen(path_buf, "w"))) {
		errx(1, "opening %s failed.\n", path_buf);
	}
	int gpsfile_no = fileno(gpsfile);

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 25;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];


	union {
		struct sensor_combined_s raw;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s actuators;
	} buf;

	struct {
		int sensor_sub;
		int att_sub;
		int spa_sub;
		int act_0_sub;
		int actuators_sub;
	} subs;

	/* --- SENSORS RAW VALUE --- */
	/* subscribe to ORB for sensors raw */
	subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = subs.sensor_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE VALUE --- */
	/* subscribe to ORB for attitude */
	subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(subs.att_sub, 100);
	fds[fdsc_count].fd = subs.att_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE SETPOINT VALUE --- */
	/* subscribe to ORB for attitude setpoint */
	/* struct already allocated */
	subs.spa_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	orb_set_interval(subs.spa_sub, 2000);	/* 0.5 Hz updates */
	fds[fdsc_count].fd = subs.spa_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/** --- ACTUATOR OUTPUTS --- */
	subs.act_0_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	fds[fdsc_count].fd = subs.act_0_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL VALUE --- */
	/* subscribe to ORB for actuator control */
	subs.actuators_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	fds[fdsc_count].fd = subs.actuators_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* WARNING: If you get the error message below,
	 * then the number of registered messages (fdsc)
	 * differs from the number of messages in the above list.
	 */
	if (fdsc_count > fdsc) {
		warn("WARNING: Not enough space for poll fds allocated. Check %s:%d.\n", __FILE__, __LINE__);
		fdsc_count = fdsc;
	}

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	thread_running = true;

	while (!thread_should_exit) {

		int poll_ret = poll(fds, fdsc_count, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* XXX this means none of our providers is giving us data - might be an error? */
		} else if (poll_ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {

			int ifds = 0;

			/* --- SENSORS RAW VALUE --- */
			if (fds[ifds++].revents & POLLIN) {

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.raw);
				/* write out */
				write(sensorfile, (const char*)&buf.raw, sizeof(buf.raw));
			}

			/* --- ATTITUDE VALUE --- */
			if (fds[ifds++].revents & POLLIN) {

				/* copy attitude data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), subs.att_sub, &buf.att);

				
			}

			/* --- VEHICLE ATTITUDE SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.spa_sub, &buf.att_sp);
				
			}

			/* --- ACTUATOR OUTPUTS 0 --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy actuator data into local buffer */
				orb_copy(ORB_ID(actuator_outputs_0), subs.act_0_sub, &buf.act_outputs);
				
			}

			/* --- ACTUATOR CONTROL --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.actuators_sub, &buf.actuators);
				
			}

			/* enforce write to disk */
			fsync(sensorfile);
			fsync(gpsfile_no);
		}
	}

	warn("exiting.");

	close(sensorfile);
	fclose(gpsfile);

	thread_running = false;

	return 0;
}

/**
 * @return 0 if file exists
 */
int file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer);
}

