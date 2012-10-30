/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
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
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <systemlib/systemlib.h>

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */

static const char *mountpoint = "/fs/microsd";
static const char *mfile_in = "/etc/logging/logconv.m";

/**
 * SD log management function.
 */
__EXPORT int sdlog_main(int argc, char *argv[]);

/**
 * Mainloop of sd log deamon.
 */
int sdlog_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int file_exist(const char *filename);

static int file_copy(const char* file_old, const char* file_new);

/**
 * Print the current status.
 */
static void print_sdlog_status(void);

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

// XXX turn this into a C++ class
unsigned sensor_combined_bytes = 0;
unsigned actuator_outputs_bytes = 0;
unsigned actuator_controls_bytes = 0;
unsigned sysvector_bytes = 0;
unsigned blackbox_file_bytes = 0;
uint64_t starttime = 0;

/**
 * The sd log deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
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
		deamon_task = task_spawn("sdlog",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT - 30,
					 4096,
					 sdlog_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			printf("\tsdlog is not started\n");
		}
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			print_sdlog_status();
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
			/* folder does not exist, success */

			/* now copy the Matlab/Octave file */
			char mfile_out[100];
			sprintf(mfile_out, "%s/session%04u/run_to_plot_data.m", mountpoint, foldernumber);
			int ret = file_copy(mfile_in, mfile_out);
			if (!ret) {
				warnx("copied m file to %s", mfile_out);
			} else {
				warnx("failed copying m file from %s to\n %s", mfile_in, mfile_out);
			}
			break;

		} else if (mkdir_ret == -1) {
			/* folder exists already */
			foldernumber++;
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

	warnx("starting\n");

	if (file_exist(mountpoint) != OK) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	char folder_path[64];
	if (create_logfolder(folder_path))
		errx(1, "unable to create logging folder, exiting.");

	/* create sensorfile */
	int sensorfile = -1;
	int actuator_outputs_file = -1;
	int actuator_controls_file = -1;
	int sysvector_file = -1;
	FILE *gpsfile;
	FILE *blackbox_file;
	// FILE *vehiclefile;

	char path_buf[64] = ""; // string to hold the path to the sensorfile

	warnx("logging to directory %s\n", folder_path);

	/* set up file path: e.g. /mnt/sdcard/session0001/sensor_combined.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "sensor_combined");
	if (0 == (sensorfile = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	// /* set up file path: e.g. /mnt/sdcard/session0001/actuator_outputs0.bin */
	// sprintf(path_buf, "%s/%s.bin", folder_path, "actuator_outputs0");
	// if (0 == (actuator_outputs_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
	// 	errx(1, "opening %s failed.\n", path_buf);
	// }

	/* set up file path: e.g. /mnt/sdcard/session0001/actuator_controls0.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "sysvector");
	if (0 == (sysvector_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	/* set up file path: e.g. /mnt/sdcard/session0001/actuator_controls0.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "actuator_controls0");
	if (0 == (actuator_controls_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	/* set up file path: e.g. /mnt/sdcard/session0001/gps.txt */
	sprintf(path_buf, "%s/%s.txt", folder_path, "gps");
	if (NULL == (gpsfile = fopen(path_buf, "w"))) {
		errx(1, "opening %s failed.\n", path_buf);
	}
	int gpsfile_no = fileno(gpsfile);

	/* set up file path: e.g. /mnt/sdcard/session0001/blackbox.txt */
	sprintf(path_buf, "%s/%s.txt", folder_path, "blackbox");
	if (NULL == (blackbox_file = fopen(path_buf, "w"))) {
		errx(1, "opening %s failed.\n", path_buf);
	}
	int blackbox_file_no = fileno(blackbox_file);

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 25;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];


	struct {
		struct sensor_combined_s raw;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s act_controls;
		struct vehicle_command_s cmd;
		struct vehicle_local_position_s local_pos;
		struct vehicle_global_position_s global_pos;
		struct vehicle_gps_position_s gps_pos;
	} buf;
	memset(&buf, 0, sizeof(buf));

	struct {
		int cmd_sub;
		int sensor_sub;
		int att_sub;
		int spa_sub;
		int act_0_sub;
		int controls0_sub;
		int local_pos_sub;
		int global_pos_sub;
		int gps_pos_sub;
	} subs;

	/* --- MANAGEMENT - LOGGING COMMAND --- */
	/* subscribe to ORB for sensors raw */
	subs.cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	fds[fdsc_count].fd = subs.cmd_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- SENSORS RAW VALUE --- */
	/* subscribe to ORB for sensors raw */
	subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = subs.sensor_sub;
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(subs.sensor_sub, 5);
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE VALUE --- */
	/* subscribe to ORB for attitude */
	subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	fds[fdsc_count].fd = subs.att_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE SETPOINT VALUE --- */
	/* subscribe to ORB for attitude setpoint */
	/* struct already allocated */
	subs.spa_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
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
	subs.controls0_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	fds[fdsc_count].fd = subs.controls0_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- LOCAL POSITION --- */
	/* subscribe to ORB for local position */
	subs.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	fds[fdsc_count].fd = subs.local_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GLOBAL POSITION --- */
	/* subscribe to ORB for global position */
	subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	fds[fdsc_count].fd = subs.global_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GPS POSITION --- */
	/* subscribe to ORB for global position */
	subs.gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	fds[fdsc_count].fd = subs.gps_pos_sub;
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
	// const int timeout = 1000;

	thread_running = true;

	int poll_count = 0;

	starttime = hrt_absolute_time();

	while (!thread_should_exit) {

		// int poll_ret = poll(fds, fdsc_count, timeout);

		// /* handle the poll result */
		// if (poll_ret == 0) {
		// 	/* XXX this means none of our providers is giving us data - might be an error? */
		// } else if (poll_ret < 0) {
		// 	/* XXX this is seriously bad - should be an emergency */
		// } else {

		// 	int ifds = 0;

		// 	if (poll_count % 5000 == 0) {
		// 		fsync(sensorfile);
		// 		fsync(actuator_outputs_file);
		// 		fsync(actuator_controls_file);
		// 		fsync(blackbox_file_no);
		// 	}

			

		// 	/* --- VEHICLE COMMAND VALUE --- */
		// 	if (fds[ifds++].revents & POLLIN) {
		// 		/* copy command into local buffer */
		// 		orb_copy(ORB_ID(vehicle_command), subs.cmd_sub, &buf.cmd);
		// 		blackbox_file_bytes += fprintf(blackbox_file, "[%10.4f\tVCMD] CMD #%d [%f\t%f\t%f\t%f\t%f\t%f\t%f]\n", hrt_absolute_time()/1000000.0d,
		// 			buf.cmd.command, (double)buf.cmd.param1, (double)buf.cmd.param2, (double)buf.cmd.param3, (double)buf.cmd.param4,
		// 			(double)buf.cmd.param5, (double)buf.cmd.param6, (double)buf.cmd.param7);
		// 	}

		// 	/* --- SENSORS RAW VALUE --- */
		// 	if (fds[ifds++].revents & POLLIN) {

		// 		/* copy sensors raw data into local buffer */
		// 		orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.raw);
		// 		/* write out */
		// 		sensor_combined_bytes += write(sensorfile, (const char*)&(buf.raw), sizeof(buf.raw));
		// 	}

		// 	/* --- ATTITUDE VALUE --- */
		// 	if (fds[ifds++].revents & POLLIN) {

		// 		/* copy attitude data into local buffer */
		// 		orb_copy(ORB_ID(vehicle_attitude), subs.att_sub, &buf.att);

				
		// 	}

		// 	/* --- VEHICLE ATTITUDE SETPOINT --- */
		// 	if (fds[ifds++].revents & POLLIN) {
		// 		/* copy local position data into local buffer */
		// 		orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.spa_sub, &buf.att_sp);
				
		// 	}

		// 	/* --- ACTUATOR OUTPUTS 0 --- */
		// 	if (fds[ifds++].revents & POLLIN) {
		// 		/* copy actuator data into local buffer */
		// 		orb_copy(ORB_ID(actuator_outputs_0), subs.act_0_sub, &buf.act_outputs);
		// 		/* write out */
		// 		// actuator_outputs_bytes += write(actuator_outputs_file, (const char*)&buf.act_outputs, sizeof(buf.act_outputs));
		// 	}

		// 	/* --- ACTUATOR CONTROL --- */
		// 	if (fds[ifds++].revents & POLLIN) {
		// 		orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.controls0_sub, &buf.act_controls);
		// 		/* write out */
		// 		actuator_controls_bytes += write(actuator_controls_file, (const char*)&buf.act_controls, sizeof(buf.act_controls));
		// 	}
		// }

		if (poll_count % 100 == 0) {
			fsync(sysvector_file);
		}

		poll_count++;


		/* copy sensors raw data into local buffer */
		orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.raw);
		orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.controls0_sub, &buf.act_controls);
		/* copy actuator data into local buffer */
		orb_copy(ORB_ID(actuator_outputs_0), subs.act_0_sub, &buf.act_outputs);
		orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.spa_sub, &buf.att_sp);
		orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
		orb_copy(ORB_ID(vehicle_local_position), subs.local_pos_sub, &buf.local_pos);
		orb_copy(ORB_ID(vehicle_global_position), subs.global_pos_sub, &buf.global_pos);

		#pragma pack(push, 1)
		struct {
			uint64_t timestamp;
			float gyro[3];
			float accel[3];
			float mag[3];
			float baro;
			float baro_alt;
			float baro_temp;
			float control[4];

			float actuators[8];
			float vbat;
			float adc[3];
			float local_pos[3];
			int32_t gps_pos[3];
		} sysvector = {
			.timestamp = buf.raw.timestamp,
			.gyro = {buf.raw.gyro_rad_s[0], buf.raw.gyro_rad_s[1], buf.raw.gyro_rad_s[2]},
			.accel = {buf.raw.accelerometer_m_s2[0], buf.raw.accelerometer_m_s2[1], buf.raw.accelerometer_m_s2[2]},
			.mag = {buf.raw.magnetometer_ga[0], buf.raw.magnetometer_ga[1], buf.raw.magnetometer_ga[2]},
			.baro = buf.raw.baro_pres_mbar,
			.baro_alt = buf.raw.baro_alt_meter,
			.baro_temp = buf.raw.baro_temp_celcius,
			.control = {buf.act_controls.control[0], buf.act_controls.control[1], buf.act_controls.control[2], buf.act_controls.control[3]},
			.actuators = {buf.act_outputs.output[0], buf.act_outputs.output[1], buf.act_outputs.output[2], buf.act_outputs.output[3],
					buf.act_outputs.output[4], buf.act_outputs.output[5], buf.act_outputs.output[6], buf.act_outputs.output[7]},
			.vbat = buf.raw.battery_voltage_v,
			.adc = {buf.raw.adc_voltage_v[0], buf.raw.adc_voltage_v[1], buf.raw.adc_voltage_v[2]},
			.local_pos = {buf.local_pos.x, buf.local_pos.y, buf.local_pos.z},
			.gps_pos = {buf.gps_pos.lat, buf.gps_pos.lon, buf.gps_pos.alt}
		};
		#pragma pack(pop)

		sysvector_bytes += write(sysvector_file, (const char*)&sysvector, sizeof(sysvector));

		usleep(10000);
	}

	fsync(sysvector_file);

	print_sdlog_status();

	warnx("exiting.\n");

	close(sensorfile);
	close(actuator_outputs_file);
	close(actuator_controls_file);
	fclose(gpsfile);
	fclose(blackbox_file);

	thread_running = false;

	return 0;
}

void print_sdlog_status()
{
	unsigned bytes = sysvector_bytes + sensor_combined_bytes + actuator_outputs_bytes + blackbox_file_bytes + actuator_controls_bytes;
	float mebibytes = bytes / 1024.0f / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - starttime)) / 1000000.0f;

	warnx("wrote %4.2f MiB (average %5.3f MiB/s).\n", (double)mebibytes, (double)(mebibytes / seconds));
}

/**
 * @return 0 if file exists
 */
int file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer);
}

int file_copy(const char* file_old, const char* file_new)
{
	FILE *source, *target;
	source = fopen(file_old, "r");
	int ret = 0;
 
	if( source == NULL )
	{
		warnx("failed opening input file to copy");
		return 1;
	}

	target = fopen(file_new, "w");
 
	if( target == NULL )
	{
		fclose(source);
		warnx("failed to open output file to copy");
		return 1;
	}
 
	char buf[128];
	int nread;
	while ((nread = fread(buf, 1, sizeof(buf), source)) > 0) {
		int ret = fwrite(buf, 1, nread, target);
		if (ret <= 0) {
			warnx("error writing file");
			ret = 1;
			break;
		}
	}
	fsync(fileno(target));

	fclose(source);
	fclose(target);

   return ret;
}

