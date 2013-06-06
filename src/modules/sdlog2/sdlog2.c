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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>

#include <systemlib/systemlib.h>

#include <mavlink/mavlink_log.h>

#include "logbuffer.h"
#include "sdlog2_format.h"
#include "sdlog2_messages.h"

#define LOGBUFFER_WRITE_AND_COUNT(_msg) if (logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(_msg))) { \
		log_msgs_written++; \
	} else { \
		log_msgs_skipped++; \
		/*printf("skip\n");*/ \
	}

#define LOG_ORB_SUBSCRIBE(_var, _topic) subs.##_var##_sub = orb_subscribe(ORB_ID(##_topic##)); \
		fds[fdsc_count].fd = subs.##_var##_sub; \
		fds[fdsc_count].events = POLLIN; \
		fdsc_count++;


//#define SDLOG2_DEBUG

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;			/**< Deamon status flag */
static int deamon_task;						/**< Handle of deamon task / thread */
static bool logwriter_should_exit = false;	/**< Logwriter thread exit flag */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log folders */
static const int MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static const int LOG_BUFFER_SIZE = 8192;
static const int MAX_WRITE_CHUNK = 512;
static const int MIN_BYTES_TO_WRITE = 512;

static const char *mountpoint = "/fs/microsd";
int log_file = -1;
int mavlink_fd = -1;
struct logbuffer_s lb;

/* mutex / condition to synchronize threads */
pthread_mutex_t logbuffer_mutex;
pthread_cond_t logbuffer_cond;

char folder_path[64];

/* statistics counters */
unsigned long log_bytes_written = 0;
uint64_t start_time = 0;
unsigned long log_msgs_written = 0;
unsigned long log_msgs_skipped = 0;

/* current state of logging */
bool logging_enabled = false;
/* enable logging on start (-e option) */
bool log_on_start = false;
/* enable logging when armed (-a option) */
bool log_when_armed = false;
/* delay = 1 / rate (rate defined by -r option) */
useconds_t sleep_delay = 0;

/* helper flag to track system state changes */
bool flag_system_armed = false;

pthread_t logwriter_pthread = 0;

/**
 * Log buffer writing thread. Open and close file here.
 */
static void *logwriter_thread(void *arg);

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
static void sdlog2_usage(const char *reason);

/**
 * Print the current status.
 */
static void sdlog2_status(void);

/**
 * Start logging: create new file and start log writer thread.
 */
void sdlog2_start_log();

/**
 * Stop logging: stop log writer thread and close log file.
 */
void sdlog2_stop_log();

/**
 * Write a header to log file: list of message formats.
 */
void write_formats(int fd);


static bool file_exist(const char *filename);

static int file_copy(const char *file_old, const char *file_new);

static void handle_command(struct vehicle_command_s *cmd);

static void handle_status(struct vehicle_status_s *cmd);

/**
 * Create folder for current logging session. Store folder name in 'log_folder'.
 */
static int create_logfolder();

/**
 * Select first free log file name and open it.
 */
static int open_logfile();

static void
sdlog2_usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: sdlog2 {start|stop|status} [-r <log rate>] -e -a\n"
	     "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
	     "\t-e\tEnable logging by default (if not, can be started by command)\n"
	     "\t-a\tLog only when armed (can be still overriden by command)\n");
}

/**
 * The logger deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int sdlog2_main(int argc, char *argv[])
{
	if (argc < 2)
		sdlog2_usage("missing command");

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
					 2048,
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
			sdlog2_status();

		} else {
			printf("\tsdlog2 not started\n");
		}

		exit(0);
	}

	sdlog2_usage("unrecognized command");
	exit(1);
}

int create_logfolder()
{
	/* make folder on sdcard */
	uint16_t folder_number = 1; // start with folder sess001
	int mkdir_ret;

	/* look for the next folder that does not exist */
	while (folder_number <= MAX_NO_LOGFOLDER) {
		/* set up folder path: e.g. /fs/microsd/sess001 */
		sprintf(folder_path, "%s/sess%03u", mountpoint, folder_number);
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
			folder_number++;
			continue;

		} else {
			warn("failed creating new folder");
			return -1;
		}
	}

	if (folder_number >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		warnx("all %d possible folders exist already.", MAX_NO_LOGFOLDER);
		return -1;
	}

	return 0;
}

int open_logfile()
{
	/* make folder on sdcard */
	uint16_t file_number = 1; // start with file log001

	/* string to hold the path to the log */
	char path_buf[64] = "";

	int fd = 0;

	/* look for the next file that does not exist */
	while (file_number <= MAX_NO_LOGFILE) {
		/* set up file path: e.g. /fs/microsd/sess001/log001.bin */
		sprintf(path_buf, "%s/log%03u.bin", folder_path, file_number);

		if (file_exist(path_buf)) {
			file_number++;
			continue;
		}

		fd = open(path_buf, O_CREAT | O_WRONLY | O_DSYNC);

		if (fd == 0) {
			errx(1, "opening %s failed.", path_buf);
		}

		warnx("logging to: %s", path_buf);
		mavlink_log_info(mavlink_fd, "[sdlog2] log: %s", path_buf);

		return fd;
	}

	if (file_number > MAX_NO_LOGFILE) {
		/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
		warn("all %d possible files exist already", MAX_NO_LOGFILE);
		return -1;
	}

	return 0;
}

static void *logwriter_thread(void *arg)
{
	/* set name */
	prctl(PR_SET_NAME, "sdlog2_writer", 0);

	struct logbuffer_s *logbuf = (struct logbuffer_s *)arg;

	int log_file = open_logfile();

	/* write log messages formats */
	write_formats(log_file);

	int poll_count = 0;

	void *read_ptr;
	int n = 0;
	bool should_wait = false;
	bool is_part = false;

	while (!thread_should_exit && !logwriter_should_exit) {

		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);

		/* update read pointer if needed */
		if (n > 0) {
			logbuffer_mark_read(&lb, n);
		}

		/* only wait if no data is available to process */
		if (should_wait) {
			/* blocking wait for new data at this line */
			pthread_cond_wait(&logbuffer_cond, &logbuffer_mutex);
		}

		/* only get pointer to thread-safe data, do heavy I/O a few lines down */
		int available = logbuffer_get_ptr(logbuf, &read_ptr, &is_part);

		/* continue */
		pthread_mutex_unlock(&logbuffer_mutex);

		if (available > 0) {
			/* do heavy IO here */
			if (available > MAX_WRITE_CHUNK) {
				n = MAX_WRITE_CHUNK;

			} else {
				n = available;
			}

			n = write(log_file, read_ptr, n);

			should_wait = (n == available) && !is_part;
#ifdef SDLOG2_DEBUG
			printf("%i wrote: %i of %i, is_part=%i, should_wait=%i", poll_count, n, available, (int)is_part, (int)should_wait);
#endif

			if (n < 0) {
				thread_should_exit = true;
				err(1, "error writing log file");
			}

			if (n > 0) {
				log_bytes_written += n;
			}

		} else {
			should_wait = true;
		}

		if (poll_count % 10 == 0) {
			fsync(log_file);
		}

		poll_count++;
	}

	fsync(log_file);
	close(log_file);

	return OK;
}

void sdlog2_start_log()
{
	warnx("start logging.");
	mavlink_log_info(mavlink_fd, "[sdlog2] start logging");

	/* initialize statistics counter */
	log_bytes_written = 0;
	start_time = hrt_absolute_time();
	log_msgs_written = 0;
	log_msgs_skipped = 0;

	/* initialize log buffer emptying thread */
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	logwriter_should_exit = false;
	pthread_t thread;

	/* start log buffer emptying thread */
	if (0 != pthread_create(&thread, &receiveloop_attr, logwriter_thread, &lb)) {
		errx(1, "error creating logwriter thread");
	}

	logging_enabled = true;
	// XXX we have to destroy the attr at some point
}

void sdlog2_stop_log()
{
	warnx("stop logging.");
	mavlink_log_info(mavlink_fd, "[sdlog2] stop logging");

	logging_enabled = true;
	logwriter_should_exit = true;

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	(void)pthread_join(logwriter_pthread, NULL);

	sdlog2_status();
}


void write_formats(int fd)
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
		warnx("failed to open MAVLink log stream, start mavlink app first.");
	}

	/* log every n'th value (skip three per default) */
	int skip_value = 3;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;
	int ch;

	while ((ch = getopt(argc, argv, "r:ea")) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned r = strtoul(optarg, NULL, 10);

				if (r == 0) {
					sleep_delay = 0;

				} else {
					sleep_delay = 1000000 / r;
				}
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'a':
			log_when_armed = true;
			break;

		case '?':
			if (optopt == 'c') {
				warnx("Option -%c requires an argument.", optopt);

			} else if (isprint(optopt)) {
				warnx("Unknown option `-%c'.", optopt);

			} else {
				warnx("Unknown option character `\\x%x'.", optopt);
			}

		default:
			sdlog2_usage("unrecognized flag");
			errx(1, "exiting");
		}
	}

	if (!file_exist(mountpoint)) {
		errx(1, "logging mount point %s not present, exiting.", mountpoint);
	}

	if (create_logfolder())
		errx(1, "unable to create logging folder, exiting.");

	/* only print logging path, important to find log file later */
	warnx("logging to directory: %s", folder_path);

	/* file descriptors to wait for */
	struct pollfd fds_control[2];

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 15;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];

	/* warning! using union here to save memory, elements should be used separately! */
	union {
		struct vehicle_command_s cmd;
		struct vehicle_status_s status;
		struct sensor_combined_s sensor;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s act_controls;
		struct actuator_controls_effective_s act_controls_effective;
		struct vehicle_local_position_s local_pos;
		struct vehicle_local_position_setpoint_s local_pos_sp;
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
		int status_sub;
		int sensor_sub;
		int att_sub;
		int att_sp_sub;
		int act_outputs_sub;
		int act_controls_sub;
		int act_controls_effective_sub;
		int local_pos_sub;
		int local_pos_sp_sub;
		int global_pos_sub;
		int gps_pos_sub;
		int vicon_pos_sub;
		int flow_sub;
		int batt_sub;
		int diff_pres_sub;
		int airspeed_sub;
	} subs;

	/* log message buffer: header + body */
#pragma pack(push, 1)
	struct {
		LOG_PACKET_HEADER;
		union {
			struct log_TIME_s log_TIME;
			struct log_ATT_s log_ATT;
			struct log_ATSP_s log_ATSP;
			struct log_IMU_s log_IMU;
			struct log_SENS_s log_SENS;
			struct log_LPOS_s log_LPOS;
			struct log_LPSP_s log_LPSP;
			struct log_GPS_s log_GPS;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
#pragma pack(pop)
	memset(&log_msg.body, 0, sizeof(log_msg.body));

	/* --- VEHICLE COMMAND --- */
	subs.cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	fds[fdsc_count].fd = subs.cmd_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- VEHICLE STATUS --- */
	subs.status_sub = orb_subscribe(ORB_ID(vehicle_status));
	fds[fdsc_count].fd = subs.status_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GPS POSITION --- */
	subs.gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	fds[fdsc_count].fd = subs.gps_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- SENSORS COMBINED --- */
	subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = subs.sensor_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE --- */
	subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	fds[fdsc_count].fd = subs.att_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE SETPOINT --- */
	subs.att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	fds[fdsc_count].fd = subs.att_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR OUTPUTS --- */
	subs.act_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	fds[fdsc_count].fd = subs.act_outputs_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL --- */
	subs.act_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	fds[fdsc_count].fd = subs.act_controls_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL EFFECTIVE --- */
	subs.act_controls_effective_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE);
	fds[fdsc_count].fd = subs.act_controls_effective_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- LOCAL POSITION --- */
	subs.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	fds[fdsc_count].fd = subs.local_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- LOCAL POSITION SETPOINT --- */
	subs.local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	fds[fdsc_count].fd = subs.local_pos_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GLOBAL POSITION --- */
	subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	fds[fdsc_count].fd = subs.global_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- VICON POSITION --- */
	subs.vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	fds[fdsc_count].fd = subs.vicon_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- OPTICAL FLOW --- */
	subs.flow_sub = orb_subscribe(ORB_ID(optical_flow));
	fds[fdsc_count].fd = subs.flow_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- BATTERY STATUS --- */
	subs.batt_sub = orb_subscribe(ORB_ID(battery_status));
	fds[fdsc_count].fd = subs.batt_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* WARNING: If you get the error message below,
	 * then the number of registered messages (fdsc)
	 * differs from the number of messages in the above list.
	 */
	if (fdsc_count > fdsc) {
		warn("WARNING: Not enough space for poll fds allocated. Check %s:%d.", __FILE__, __LINE__);
		fdsc_count = fdsc;
	}

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms
	 */
	const int poll_timeout = 1000;

	thread_running = true;

	/* initialize log buffer with specified size */
	logbuffer_init(&lb, LOG_BUFFER_SIZE);

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
	pthread_cond_init(&logbuffer_cond, NULL);

	/* track changes in sensor_combined topic */
	uint16_t gyro_counter = 0;
	uint16_t accelerometer_counter = 0;
	uint16_t magnetometer_counter = 0;
	uint16_t baro_counter = 0;
	uint16_t differential_pressure_counter = 0;

	/* enable logging on start if needed */
	if (log_on_start)
		sdlog2_start_log();

	while (!thread_should_exit) {
		/* decide use usleep() or blocking poll() */
		bool use_sleep = sleep_delay > 0 && logging_enabled;

		/* poll all topics if logging enabled or only management (first 2) if not */
		int poll_ret = poll(fds, logging_enabled ? fdsc_count : 2, use_sleep ? 0 : poll_timeout);

		/* handle the poll result */
		if (poll_ret < 0) {
			warnx("ERROR: poll error, stop logging.");
			thread_should_exit = true;

		} else if (poll_ret > 0) {

			/* check all data subscriptions only if logging enabled,
			 * logging_enabled can be changed while checking vehicle_command and vehicle_status */
			bool check_data = logging_enabled;
			int ifds = 0;

			/* --- VEHICLE COMMAND --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_command), subs.cmd_sub, &buf.cmd);
				handle_command(&buf.cmd);
			}

			/* --- VEHICLE STATUS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_status), subs.status_sub, &buf.status);
				if (log_when_armed) {
					handle_status(&buf.status);
				}
			}

			if (!logging_enabled || !check_data) {
				continue;
			}

			pthread_mutex_lock(&logbuffer_mutex);

			/* write time stamp message */
			log_msg.msg_type = LOG_TIME_MSG;
			log_msg.body.log_TIME.t = hrt_absolute_time();
			LOGBUFFER_WRITE_AND_COUNT(TIME);

			/* --- GPS POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);
				log_msg.msg_type = LOG_GPS_MSG;
				log_msg.body.log_GPS.gps_time = buf.gps_pos.time_gps_usec;
				log_msg.body.log_GPS.fix_type = buf.gps_pos.fix_type;
				log_msg.body.log_GPS.eph = buf.gps_pos.eph_m;
				log_msg.body.log_GPS.epv = buf.gps_pos.epv_m;
				log_msg.body.log_GPS.lat = buf.gps_pos.lat;
				log_msg.body.log_GPS.lon = buf.gps_pos.lon;
				log_msg.body.log_GPS.alt = buf.gps_pos.alt * 0.001;
				log_msg.body.log_GPS.vel_n = buf.gps_pos.vel_n_m_s;
				log_msg.body.log_GPS.vel_e = buf.gps_pos.vel_e_m_s;
				log_msg.body.log_GPS.vel_d = buf.gps_pos.vel_d_m_s;
				log_msg.body.log_GPS.cog = buf.gps_pos.cog_rad;
				LOGBUFFER_WRITE_AND_COUNT(GPS);
			}

			/* --- SENSOR COMBINED --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), subs.sensor_sub, &buf.sensor);
				bool write_IMU = false;
				bool write_SENS = false;

				if (buf.sensor.gyro_counter != gyro_counter) {
					gyro_counter = buf.sensor.gyro_counter;
					write_IMU = true;
				}

				if (buf.sensor.accelerometer_counter != accelerometer_counter) {
					accelerometer_counter = buf.sensor.accelerometer_counter;
					write_IMU = true;
				}

				if (buf.sensor.magnetometer_counter != magnetometer_counter) {
					magnetometer_counter = buf.sensor.magnetometer_counter;
					write_IMU = true;
				}

				if (buf.sensor.baro_counter != baro_counter) {
					baro_counter = buf.sensor.baro_counter;
					write_SENS = true;
				}

				if (buf.sensor.differential_pressure_counter != differential_pressure_counter) {
					differential_pressure_counter = buf.sensor.differential_pressure_counter;
					write_SENS = true;
				}

				if (write_IMU) {
					log_msg.msg_type = LOG_IMU_MSG;
					log_msg.body.log_IMU.gyro_x = buf.sensor.gyro_rad_s[0];
					log_msg.body.log_IMU.gyro_y = buf.sensor.gyro_rad_s[1];
					log_msg.body.log_IMU.gyro_z = buf.sensor.gyro_rad_s[2];
					log_msg.body.log_IMU.acc_x = buf.sensor.accelerometer_m_s2[0];
					log_msg.body.log_IMU.acc_y = buf.sensor.accelerometer_m_s2[1];
					log_msg.body.log_IMU.acc_z = buf.sensor.accelerometer_m_s2[2];
					log_msg.body.log_IMU.mag_x = buf.sensor.magnetometer_ga[0];
					log_msg.body.log_IMU.mag_y = buf.sensor.magnetometer_ga[1];
					log_msg.body.log_IMU.mag_z = buf.sensor.magnetometer_ga[2];
					LOGBUFFER_WRITE_AND_COUNT(IMU);
				}

				if (write_SENS) {
					log_msg.msg_type = LOG_SENS_MSG;
					log_msg.body.log_SENS.baro_pres = buf.sensor.baro_pres_mbar;
					log_msg.body.log_SENS.baro_alt = buf.sensor.baro_alt_meter;
					log_msg.body.log_SENS.baro_temp = buf.sensor.baro_temp_celcius;
					log_msg.body.log_SENS.diff_pres = buf.sensor.differential_pressure_pa;
					LOGBUFFER_WRITE_AND_COUNT(SENS);
				}
			}

			/* --- ATTITUDE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude), subs.att_sub, &buf.att);
				log_msg.msg_type = LOG_ATT_MSG;
				log_msg.body.log_ATT.roll = buf.att.roll;
				log_msg.body.log_ATT.pitch = buf.att.pitch;
				log_msg.body.log_ATT.yaw = buf.att.yaw;
				LOGBUFFER_WRITE_AND_COUNT(ATT);
			}

			/* --- ATTITUDE SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.att_sp_sub, &buf.att_sp);
				log_msg.msg_type = LOG_ATSP_MSG;
				log_msg.body.log_ATSP.roll_sp = buf.att_sp.roll_body;
				log_msg.body.log_ATSP.pitch_sp = buf.att_sp.pitch_body;
				log_msg.body.log_ATSP.yaw_sp = buf.att_sp.yaw_body;
				LOGBUFFER_WRITE_AND_COUNT(ATSP);
			}

			/* --- ACTUATOR OUTPUTS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_outputs_0), subs.act_outputs_sub, &buf.act_outputs);
				// TODO not implemented yet
			}

			/* --- ACTUATOR CONTROL --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.act_controls_sub, &buf.act_controls);
				// TODO not implemented yet
			}

			/* --- ACTUATOR CONTROL EFFECTIVE --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, subs.act_controls_effective_sub, &buf.act_controls_effective);
				// TODO not implemented yet
			}

			/* --- LOCAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_local_position), subs.local_pos_sub, &buf.local_pos);
				log_msg.msg_type = LOG_LPOS_MSG;
				log_msg.body.log_LPOS.x = buf.local_pos.x;
				log_msg.body.log_LPOS.y = buf.local_pos.y;
				log_msg.body.log_LPOS.z = buf.local_pos.z;
				log_msg.body.log_LPOS.vx = buf.local_pos.vx;
				log_msg.body.log_LPOS.vy = buf.local_pos.vy;
				log_msg.body.log_LPOS.vz = buf.local_pos.vz;
				log_msg.body.log_LPOS.hdg = buf.local_pos.hdg;
				log_msg.body.log_LPOS.home_lat = buf.local_pos.home_lat;
				log_msg.body.log_LPOS.home_lon = buf.local_pos.home_lon;
				log_msg.body.log_LPOS.home_alt = buf.local_pos.home_alt;
				LOGBUFFER_WRITE_AND_COUNT(LPOS);
			}

			/* --- LOCAL POSITION SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_local_position_setpoint), subs.local_pos_sp_sub, &buf.local_pos_sp);
				log_msg.msg_type = LOG_LPSP_MSG;
				log_msg.body.log_LPSP.x = buf.local_pos_sp.x;
				log_msg.body.log_LPSP.y = buf.local_pos_sp.y;
				log_msg.body.log_LPSP.z = buf.local_pos_sp.z;
				log_msg.body.log_LPSP.yaw = buf.local_pos_sp.yaw;
				LOGBUFFER_WRITE_AND_COUNT(LPSP);
			}

			/* --- GLOBAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_global_position), subs.global_pos_sub, &buf.global_pos);
				// TODO not implemented yet
			}

			/* --- VICON POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_vicon_position), subs.vicon_pos_sub, &buf.vicon_pos);
				// TODO not implemented yet
			}

			/* --- FLOW --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(optical_flow), subs.flow_sub, &buf.flow);
				// TODO not implemented yet
			}

			/* --- BATTERY STATUS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(battery_status), subs.batt_sub, &buf.batt);
				// TODO not implemented yet
			}

			/* signal the other thread new data, but not yet unlock */
			if (logbuffer_count(&lb) > MIN_BYTES_TO_WRITE) {
#ifdef SDLOG2_DEBUG
				printf("signal %i", logbuffer_count(&lb));
#endif
				/* only request write if several packets can be written at once */
				pthread_cond_signal(&logbuffer_cond);
			}

			/* unlock, now the writer thread may run */
			pthread_mutex_unlock(&logbuffer_mutex);
		}

		if (use_sleep) {
			usleep(sleep_delay);
		}
	}

	if (logging_enabled)
		sdlog2_stop_log();

	pthread_mutex_destroy(&logbuffer_mutex);
	pthread_cond_destroy(&logbuffer_cond);

	warnx("exiting.");

	thread_running = false;

	return 0;
}

void sdlog2_status()
{
	float mebibytes = log_bytes_written / 1024.0f / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - start_time)) / 1000000.0f;

	warnx("wrote %lu msgs, %4.2f MiB (average %5.3f MiB/s), skipped %lu msgs.", log_msgs_written, (double)mebibytes, (double)(mebibytes / seconds), log_msgs_skipped);
}

/**
 * @return 0 if file exists
 */
bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int file_copy(const char *file_old, const char *file_new)
{
	FILE *source, *target;
	source = fopen(file_old, "r");
	int ret = 0;

	if (source == NULL) {
		warnx("failed opening input file to copy.");
		return 1;
	}

	target = fopen(file_new, "w");

	if (target == NULL) {
		fclose(source);
		warnx("failed to open output file to copy.");
		return 1;
	}

	char buf[128];
	int nread;

	while ((nread = fread(buf, 1, sizeof(buf), source)) > 0) {
		ret = fwrite(buf, 1, nread, target);

		if (ret <= 0) {
			warnx("error writing file.");
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
	int param;

	/* request to set different system mode */
	switch (cmd->command) {

	case VEHICLE_CMD_PREFLIGHT_STORAGE:
		param = (int)(cmd->param3);

		if (param == 1)	{
			sdlog2_start_log();

		} else if (param == 0)	{
			sdlog2_stop_log();
		}

		break;

	default:
		/* silently ignore */
		break;
	}
}

void handle_status(struct vehicle_status_s *status)
{
	if (status->flag_system_armed != flag_system_armed) {
		flag_system_armed = status->flag_system_armed;

		if (flag_system_armed) {
			sdlog2_start_log();

		} else {
			sdlog2_stop_log();
		}
	}
}
