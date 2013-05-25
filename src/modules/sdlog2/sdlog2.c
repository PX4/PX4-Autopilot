/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <rk3dov@gmail.com>
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
 * @file sdlog2.c
 *
 * Simple SD logger for flight data. Buffers new sensor values and
 * does the heavy SD I/O in a low-priority worker thread.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <rk3dov@gmail.com>
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <systemlib/err.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>

#include <systemlib/systemlib.h>

#include <mavlink/mavlink_log.h>

#include "sdlog2_ringbuffer.h"
#include "sdlog2_messages.h"

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */
static const int LOG_BUFFER_SIZE = 2048;
static const int MAX_WRITE_CHUNK = 1024;

static const char *mountpoint = "/fs/microsd";
int log_file = -1;
int mavlink_fd = -1;
struct sdlog2_logbuffer lb;

/* mutex / condition to synchronize threads */
pthread_mutex_t logbuffer_mutex;
pthread_cond_t logbuffer_cond;

/**
 * System state vector log buffer writing
 */
static void *sdlog2_logbuffer_write_thread(void *arg);

/**
 * Create the thread to write the system vector
 */
pthread_t sdlog2_logwriter_start(struct sdlog2_logbuffer *logbuf);

/**
 * Write a header to log file: list of message formats
 */
void sdlog2_write_formats(int fd);

/**
 * SD log management function.
 */
__EXPORT int sdlog2_main(int argc, char *argv[]);

/**
 * Mainloop of sd log deamon.
 */
int sdlog2_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int file_exist(const char *filename);

static int file_copy(const char *file_old, const char *file_new);

static void handle_command(struct vehicle_command_s *cmd);

/**
 * Print the current status.
 */
static void print_sdlog2_status(void);

/**
 * Create folder for current logging session.
 */
static int create_logfolder(char *folder_path);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: sdlog2 {start|stop|status} [-s <number of skipped lines>]\n\n");
}

unsigned long log_bytes_written = 0;
uint64_t starttime = 0;

/* logging on or off, default to true */
bool logging_enabled = true;

/**
 * The sd log deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int sdlog2_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sdlog2 already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("sdlog2",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT - 30,
					 4096,
					 sdlog2_thread_main,
					 (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			printf("\tsdlog2 is not started\n");
		}

		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			print_sdlog2_status();

		} else {
			printf("\tsdlog2 not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int create_logfolder(char *folder_path)
{
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

			/* copy parser script file */
			// TODO
			/*
			char mfile_out[100];
			sprintf(mfile_out, "%s/session%04u/run_to_plot_data.m", mountpoint, foldernumber);
			int ret = file_copy(mfile_in, mfile_out);

			if (!ret) {
				warnx("copied m file to %s", mfile_out);

			} else {
				warnx("failed copying m file from %s to\n %s", mfile_in, mfile_out);
			}
			*/

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

static void *
sdlog2_logbuffer_write_thread(void *arg)
{
	/* set name */
	prctl(PR_SET_NAME, "sdlog2 microSD I/O", 0);

	struct sdlog2_logbuffer *logbuf = (struct sdlog2_logbuffer *)arg;

	int poll_count = 0;

	void *read_ptr;
	int n = 0;

	while (!thread_should_exit) {

		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);

		/* update read pointer if needed */
		if (n > 0) {
			sdlog2_logbuffer_mark_read(&lb, n);
		}

		/* only wait if no data is available to process */
		if (sdlog2_logbuffer_is_empty(logbuf)) {
			/* blocking wait for new data at this line */
			pthread_cond_wait(&logbuffer_cond, &logbuffer_mutex);
		}

		/* only get pointer to thread-safe data, do heavy I/O a few lines down */
		n = sdlog2_logbuffer_get_ptr(logbuf, &read_ptr);

		/* continue */
		pthread_mutex_unlock(&logbuffer_mutex);

		if (n > 0) {
			/* do heavy IO here */
			if (n > MAX_WRITE_CHUNK)
				n = MAX_WRITE_CHUNK;

			n = write(log_file, read_ptr, n);

			if (n > 0) {
				log_bytes_written += n;
			}
		}

		if (poll_count % 100 == 0) {
			fsync(log_file);
		}

		poll_count++;
	}

	fsync(log_file);

	return OK;
}

pthread_t sdlog2_logwriter_start(struct sdlog2_logbuffer *logbuf)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, sdlog2_logbuffer_write_thread, logbuf);
	return thread;

	// XXX we have to destroy the attr at some point
}

void sdlog2_write_formats(int fd)
{
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER;
		struct log_format_s body;
	} log_format_packet = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	/* fill message format packet for each format and write to log */
	int i;

	for (i = 0; i < log_formats_num; i++) {
		log_format_packet.body = log_formats[i];
		write(fd, &log_format_packet, sizeof(log_format_packet));
	}

	fsync(fd);
}

int sdlog2_thread_main(int argc, char *argv[])
{
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	if (mavlink_fd < 0) {
		warnx("ERROR: Failed to open MAVLink log stream, start mavlink app first.\n");
	}

	/* log every n'th value (skip three per default) */
	int skip_value = 3;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;
	int ch;

	while ((ch = getopt(argc, argv, "s:r")) != EOF) {
		switch (ch) {
		case 's': {
				/* log only every n'th (gyro clocked) value */
				unsigned s = strtoul(optarg, NULL, 10);

				if (s < 1 || s > 250) {
					errx(1, "Wrong skip value of %d, out of range (1..250)\n", s);

				} else {
					skip_value = s;
				}
			}
			break;

		case 'r':
			/* log only on request, disable logging per default */
			logging_enabled = false;
			break;

		case '?':
			if (optopt == 'c') {
				warnx("Option -%c requires an argument.\n", optopt);

			} else if (isprint(optopt)) {
				warnx("Unknown option `-%c'.\n", optopt);

			} else {
				warnx("Unknown option character `\\x%x'.\n", optopt);
			}

		default:
			usage("unrecognized flag");
			errx(1, "exiting.");
		}
	}

	if (file_exist(mountpoint) != OK) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	char folder_path[64];

	if (create_logfolder(folder_path))
		errx(1, "unable to create logging folder, exiting.");

	/* string to hold the path to the sensorfile */
	char path_buf[64] = "";

	/* only print logging path, important to find log file later */
	warnx("logging to directory %s\n", folder_path);

	/* set up file path: e.g. /mnt/sdcard/session0001/actuator_controls0.bin */
	sprintf(path_buf, "%s/%s.bin", folder_path, "log");

	if (0 == (log_file = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC))) {
		errx(1, "opening %s failed.\n", path_buf);
	}

	/* write log messages formats */
	sdlog2_write_formats(log_file);

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 25;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];

	/* warning! using union here to save memory, elements should be used separately! */
	union {
		struct sensor_combined_s raw;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s act_controls;
		struct actuator_controls_effective_s act_controls_effective;
		struct vehicle_command_s cmd;
		struct vehicle_local_position_s local_pos;
		struct vehicle_global_position_s global_pos;
		struct vehicle_gps_position_s gps_pos;
		struct vehicle_vicon_position_s vicon_pos;
		struct optical_flow_s flow;
		struct battery_status_s batt;
		struct differential_pressure_s diff_pres;
		struct airspeed_s airspeed;
	} buf;
	memset(&buf, 0, sizeof(buf));

	struct {
		int cmd_sub;
		int sensor_sub;
		int att_sub;
		int att_sp_sub;
		int act_0_sub;
		int controls_0_sub;
		int controls_effective_0_sub;
		int local_pos_sub;
		int global_pos_sub;
		int gps_pos_sub;
		int vicon_pos_sub;
		int flow_sub;
		int batt_sub;
		int diff_pres_sub;
		int airspeed_sub;
	} subs;

	/* log message buffer: header + body */
	struct {
		LOG_PACKET_HEADER;
		union {
			struct log_TS_s log_TS;
			struct log_ATT_s log_ATT;
			struct log_RAW_s log_RAW;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
	memset(&log_msg.body, 0, sizeof(log_msg.body));

	/* --- MANAGEMENT - LOGGING COMMAND --- */
	/* subscribe to ORB for vehicle command */
	subs.cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	fds[fdsc_count].fd = subs.cmd_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GPS POSITION --- */
	/* subscribe to ORB for global position */
	subs.gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	fds[fdsc_count].fd = subs.gps_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- SENSORS RAW VALUE --- */
	/* subscribe to ORB for sensors raw */
	subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = subs.sensor_sub;
	/* do not rate limit, instead use skip counter (aliasing on rate limit) */
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
	subs.att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	fds[fdsc_count].fd = subs.att_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/** --- ACTUATOR OUTPUTS --- */
	subs.act_0_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	fds[fdsc_count].fd = subs.act_0_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL VALUE --- */
	/* subscribe to ORB for actuator control */
	subs.controls_0_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	fds[fdsc_count].fd = subs.controls_0_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL EFFECTIVE VALUE --- */
	/* subscribe to ORB for actuator control */
	subs.controls_effective_0_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE);
	fds[fdsc_count].fd = subs.controls_effective_0_sub;
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

	/* --- VICON POSITION --- */
	/* subscribe to ORB for vicon position */
	subs.vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	fds[fdsc_count].fd = subs.vicon_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- FLOW measurements --- */
	/* subscribe to ORB for flow measurements */
	subs.flow_sub = orb_subscribe(ORB_ID(optical_flow));
	fds[fdsc_count].fd = subs.flow_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- BATTERY STATUS --- */
	/* subscribe to ORB for flow measurements */
	subs.batt_sub = orb_subscribe(ORB_ID(battery_status));
	fds[fdsc_count].fd = subs.batt_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- DIFFERENTIAL PRESSURE --- */
	/* subscribe to ORB for flow measurements */
	subs.diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	fds[fdsc_count].fd = subs.diff_pres_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- AIRSPEED --- */
	/* subscribe to ORB for airspeed */
	subs.airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	fds[fdsc_count].fd = subs.airspeed_sub;
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

	/* initialize log buffer with specified size */
	sdlog2_logbuffer_init(&lb, LOG_BUFFER_SIZE);

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
	pthread_cond_init(&logbuffer_cond, NULL);

	/* start logbuffer emptying thread */
	pthread_t logwriter_pthread = sdlog2_logwriter_start(&lb);

	starttime = hrt_absolute_time();

	/* track skipping */
	int skip_count = 0;

	while (!thread_should_exit) {

		/* poll all topics */
		int poll_ret = poll(fds, 4, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* XXX this means none of our providers is giving us data - might be an error? */
		} else if (poll_ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {

			int ifds = 0;

			if (!logging_enabled) {
				usleep(100000);
				continue;
			}

			pthread_mutex_lock(&logbuffer_mutex);

			/* write time stamp message */
			log_msg.msg_type = LOG_TS_MSG;
			log_msg.body.log_TS.t = hrt_absolute_time();
			sdlog2_logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(TS));

			/* --- VEHICLE COMMAND VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy command into local buffer */
				orb_copy(ORB_ID(vehicle_command), subs.cmd_sub, &buf.cmd);
				handle_command(&buf.cmd);
			}

			/* --- GPS POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
			}

			/* --- SENSORS RAW VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.raw);
				log_msg.msg_type = LOG_RAW_MSG;
				log_msg.body.log_RAW.accX = buf.raw.accelerometer_m_s2[0];
				log_msg.body.log_RAW.accY = buf.raw.accelerometer_m_s2[1];
				log_msg.body.log_RAW.accZ = buf.raw.accelerometer_m_s2[2];
				sdlog2_logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(RAW));
			}

			/* --- ATTITUDE VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), subs.att_sub, &buf.att);
				log_msg.msg_type = LOG_ATT_MSG;
				log_msg.body.log_ATT.roll = buf.att.roll;
				log_msg.body.log_ATT.pitch = buf.att.pitch;
				log_msg.body.log_ATT.yaw = buf.att.yaw;
				sdlog2_logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(ATT));
			}

			/* --- ATTITUDE SETPOINT VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.att_sp_sub, &buf.att_sp);
				// TODO not implemented yet
			}

			/* --- ACTUATOR OUTPUTS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- ACTUATOR CONTROL VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- ACTUATOR CONTROL EFFECTIVE VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- LOCAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- GLOBAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- VICON POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- FLOW measurements --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- BATTERY STATUS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- DIFFERENTIAL PRESSURE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* --- AIRSPEED --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				// TODO not implemented yet
			}

			/* signal the other thread new data, but not yet unlock */
			if (sdlog2_logbuffer_count(&lb) > (lb.size / 2)) {
				/* only request write if several packets can be written at once */
				pthread_cond_signal(&logbuffer_cond);
			}

			/* unlock, now the writer thread may run */
			pthread_mutex_unlock(&logbuffer_mutex);
		}
	}

	print_sdlog2_status();

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	(void)pthread_join(logwriter_pthread, NULL);

	pthread_mutex_destroy(&logbuffer_mutex);
	pthread_cond_destroy(&logbuffer_cond);

	warnx("exiting.\n\n");

	thread_running = false;

	return 0;
}

void print_sdlog2_status()
{
	float mebibytes = log_bytes_written / 1024.0f / 1024.0f;
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

int file_copy(const char *file_old, const char *file_new)
{
	FILE *source, *target;
	source = fopen(file_old, "r");
	int ret = 0;

	if (source == NULL) {
		warnx("failed opening input file to copy");
		return 1;
	}

	target = fopen(file_new, "w");

	if (target == NULL) {
		fclose(source);
		warnx("failed to open output file to copy");
		return 1;
	}

	char buf[128];
	int nread;

	while ((nread = fread(buf, 1, sizeof(buf), source)) > 0) {
		ret = fwrite(buf, 1, nread, target);

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

void handle_command(struct vehicle_command_s *cmd)
{
	/* result of the command */
	uint8_t result = VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* request to set different system mode */
	switch (cmd->command) {

	case VEHICLE_CMD_PREFLIGHT_STORAGE:

		if (((int)(cmd->param3)) == 1)	{

			/* enable logging */
			mavlink_log_info(mavlink_fd, "[log] file:");
			mavlink_log_info(mavlink_fd, "logdir");
			logging_enabled = true;
		}

		if (((int)(cmd->param3)) == 0)	{

			/* disable logging */
			mavlink_log_info(mavlink_fd, "[log] stopped.");
			logging_enabled = false;
		}

		break;

	default:
		/* silently ignore */
		break;
	}

}
