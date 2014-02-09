/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @author Anton Babushkin <anton.babushkin@me.com>
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
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/esc_status.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <version/version.h>

#include <mavlink/mavlink_log.h>

#include "logbuffer.h"
#include "sdlog2_format.h"
#include "sdlog2_messages.h"

#define LOGBUFFER_WRITE_AND_COUNT(_msg) if (logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(_msg))) { \
		log_msgs_written++; \
	} else { \
		log_msgs_skipped++; \
	}

#define LOG_ORB_SUBSCRIBE(_var, _topic) subs.##_var##_sub = orb_subscribe(ORB_ID(##_topic##)); \
	fds[fdsc_count].fd = subs.##_var##_sub; \
	fds[fdsc_count].events = POLLIN; \
	fdsc_count++;

static bool main_thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;			/**< Deamon status flag */
static int deamon_task;						/**< Handle of deamon task / thread */
static bool logwriter_should_exit = false;	/**< Logwriter thread exit flag */
static const int MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
static const int MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static const int LOG_BUFFER_SIZE_DEFAULT = 8192;
static const int MAX_WRITE_CHUNK = 512;
static const int MIN_BYTES_TO_WRITE = 512;

static const char *log_root = "/fs/microsd/log";
static int mavlink_fd = -1;
struct logbuffer_s lb;

/* mutex / condition to synchronize threads */
static pthread_mutex_t logbuffer_mutex;
static pthread_cond_t logbuffer_cond;

static char log_dir[32];

/* statistics counters */
static uint64_t start_time = 0;
static unsigned long log_bytes_written = 0;
static unsigned long log_msgs_written = 0;
static unsigned long log_msgs_skipped = 0;

/* GPS time, used for log files naming */
static uint64_t gps_time = 0;

/* current state of logging */
static bool logging_enabled = false;
/* enable logging on start (-e option) */
static bool log_on_start = false;
/* enable logging when armed (-a option) */
static bool log_when_armed = false;
/* delay = 1 / rate (rate defined by -r option) */
static useconds_t sleep_delay = 0;
/* use date/time for naming directories and files (-t option) */
static bool log_name_timestamp = false;

/* helper flag to track system state changes */
static bool flag_system_armed = false;

static pthread_t logwriter_pthread = 0;
static pthread_attr_t logwriter_attr;

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
static void sdlog2_start_log(void);

/**
 * Stop logging: stop log writer thread and close log file.
 */
static void sdlog2_stop_log(void);

/**
 * Write a header to log file: list of message formats.
 */
static int write_formats(int fd);

/**
 * Write version message to log file.
 */
static int write_version(int fd);

/**
 * Write parameters to log file.
 */
static int write_parameters(int fd);

static bool file_exist(const char *filename);

static int file_copy(const char *file_old, const char *file_new);

static void handle_command(struct vehicle_command_s *cmd);

static void handle_status(struct vehicle_status_s *cmd);

/**
 * Create dir for current logging session. Store dir name in 'log_dir'.
 */
static int create_log_dir(void);

/**
 * Select first free log file name and open it.
 */
static int open_log_file(void);

static void
sdlog2_usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	errx(1, "usage: sdlog2 {start|stop|status} [-r <log rate>] [-b <buffer size>] -e -a -t\n"
	     "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
	     "\t-b\tLog buffer size in KiB, default is 8\n"
	     "\t-e\tEnable logging by default (if not, can be started by command)\n"
	     "\t-a\tLog only when armed (can be still overriden by command)\n"
	     "\t-t\tUse date/time for naming log directories and files\n");
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
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		main_thread_should_exit = false;
		deamon_task = task_spawn_cmd("sdlog2",
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT - 30,
					     3000,
					     sdlog2_thread_main,
					     (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			warnx("not started");
		}

		main_thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			sdlog2_status();

		} else {
			warnx("not started\n");
		}

		exit(0);
	}

	sdlog2_usage("unrecognized command");
	exit(1);
}

int create_log_dir()
{
	/* create dir on sdcard if needed */
	uint16_t dir_number = 1; // start with dir sess001
	int mkdir_ret;

	if (log_name_timestamp && gps_time != 0) {
		/* use GPS date for log dir naming: e.g. /fs/microsd/2014-01-19 */
		time_t gps_time_sec = gps_time / 1000000;
		struct tm t;
		gmtime_r(&gps_time_sec, &t);
		int n = snprintf(log_dir, sizeof(log_dir), "%s/", log_root);
		strftime(log_dir + n, sizeof(log_dir) - n, "%Y-%m-%d", &t);
		mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == OK) {
			warnx("log dir created: %s", log_dir);

		} else if (errno != EEXIST) {
			warn("failed creating new dir: %s", log_dir);
			return -1;
		}

	} else {
		/* look for the next dir that does not exist */
		while (dir_number <= MAX_NO_LOGFOLDER) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			sprintf(log_dir, "%s/sess%03u", log_root, dir_number);
			mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				warnx("log dir created: %s", log_dir);
				break;

			} else if (errno != EEXIST) {
				warn("failed creating new dir: %s", log_dir);
				return -1;
			}

			/* dir exists already */
			dir_number++;
			continue;
		}

		if (dir_number >= MAX_NO_LOGFOLDER) {
			/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
			warnx("all %d possible dirs exist already", MAX_NO_LOGFOLDER);
			return -1;
		}
	}

	/* print logging path, important to find log file later */
	warnx("log dir: %s", log_dir);
	mavlink_log_info(mavlink_fd, "[sdlog2] log dir: %s", log_dir);
	return 0;
}

int open_log_file()
{
	/* string to hold the path to the log */
	char log_file_name[16] = "";
	char log_file_path[48] = "";

	if (log_name_timestamp && gps_time != 0) {
		/* use GPS time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.bin */
		time_t gps_time_sec = gps_time / 1000000;
		struct tm t;
		gmtime_r(&gps_time_sec, &t);
		strftime(log_file_name, sizeof(log_file_name), "%H_%M_%S.bin", &t);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

	} else {
		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.bin */
			snprintf(log_file_name, sizeof(log_file_name), "log%03u.bin", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			warnx("all %d possible files exist already", MAX_NO_LOGFILE);
			return -1;
		}
	}

	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);

	if (fd < 0) {
		warn("failed opening log: %s", log_file_name);
		mavlink_log_info(mavlink_fd, "[sdlog2] failed opening log: %s", log_file_name);

	} else {
		warnx("log file: %s", log_file_name);
		mavlink_log_info(mavlink_fd, "[sdlog2] log file: %s", log_file_name);
	}

	return fd;
}

static void *logwriter_thread(void *arg)
{
	/* set name */
	prctl(PR_SET_NAME, "sdlog2_writer", 0);

	int log_fd = open_log_file();

	if (log_fd < 0)
		return;

	struct logbuffer_s *logbuf = (struct logbuffer_s *)arg;

	/* write log messages formats, version and parameters */
	log_bytes_written += write_formats(log_fd);
	log_bytes_written += write_version(log_fd);
	log_bytes_written += write_parameters(log_fd);
	fsync(log_fd);

	int poll_count = 0;

	void *read_ptr;
	int n = 0;
	bool should_wait = false;
	bool is_part = false;

	while (true) {
		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);

		/* update read pointer if needed */
		if (n > 0) {
			logbuffer_mark_read(&lb, n);
		}

		/* only wait if no data is available to process */
		if (should_wait && !logwriter_should_exit) {
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

			n = write(log_fd, read_ptr, n);

			should_wait = (n == available) && !is_part;

			if (n < 0) {
				main_thread_should_exit = true;
				err(1, "error writing log file");
			}

			if (n > 0) {
				log_bytes_written += n;
			}

		} else {
			n = 0;

			/* exit only with empty buffer */
			if (main_thread_should_exit || logwriter_should_exit) {
				break;
			}

			should_wait = true;
		}

		if (++poll_count == 10) {
			fsync(log_fd);
			poll_count = 0;
		}
	}

	fsync(log_fd);
	close(log_fd);

	return;
}

void sdlog2_start_log()
{
	warnx("start logging");
	mavlink_log_info(mavlink_fd, "[sdlog2] start logging");

	/* create log dir if needed */
	if (create_log_dir() != 0) {
		mavlink_log_critical(mavlink_fd, "[sdlog2] error creating log dir");
		errx(1, "error creating log dir");
	}

	/* initialize statistics counter */
	log_bytes_written = 0;
	start_time = hrt_absolute_time();
	log_msgs_written = 0;
	log_msgs_skipped = 0;

	/* initialize log buffer emptying thread */
	pthread_attr_init(&logwriter_attr);

	struct sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&logwriter_attr, &param);

	pthread_attr_setstacksize(&logwriter_attr, 2048);

	logwriter_should_exit = false;

	/* start log buffer emptying thread */
	if (0 != pthread_create(&logwriter_pthread, &logwriter_attr, logwriter_thread, &lb)) {
		errx(1, "error creating logwriter thread");
	}

	logging_enabled = true;
}

void sdlog2_stop_log()
{
	warnx("stop logging");
	mavlink_log_info(mavlink_fd, "[sdlog2] stop logging");

	logging_enabled = false;

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	logwriter_should_exit = true;
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	int ret;

	if ((ret = pthread_join(logwriter_pthread, NULL)) != 0) {
		warnx("error joining logwriter thread: %i", ret);
	}

	logwriter_pthread = 0;
	pthread_attr_destroy(&logwriter_attr);

	sdlog2_status();
}

int write_formats(int fd)
{
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER;
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	int written = 0;

	/* fill message format packet for each format and write it */
	for (int i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		written += write(fd, &log_msg_format, sizeof(log_msg_format));
	}

	return written;
}

int write_version(int fd)
{
	/* construct version message */
	struct {
		LOG_PACKET_HEADER;
		struct log_VER_s body;
	} log_msg_VER = {
		LOG_PACKET_HEADER_INIT(LOG_VER_MSG),
	};

	/* fill version message and write it */
	strncpy(log_msg_VER.body.fw_git, FW_GIT, sizeof(log_msg_VER.body.fw_git));
	strncpy(log_msg_VER.body.arch, HW_ARCH, sizeof(log_msg_VER.body.arch));
	return write(fd, &log_msg_VER, sizeof(log_msg_VER));
}

int write_parameters(int fd)
{
	/* construct parameter message */
	struct {
		LOG_PACKET_HEADER;
		struct log_PARM_s body;
	} log_msg_PARM = {
		LOG_PACKET_HEADER_INIT(LOG_PARM_MSG),
	};

	int written = 0;
	param_t params_cnt = param_count();

	for (param_t param = 0; param < params_cnt; param++) {
		/* fill parameter message and write it */
		strncpy(log_msg_PARM.body.name, param_name(param), sizeof(log_msg_PARM.body.name));
		float value = NAN;

		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				int32_t i;
				param_get(param, &i);
				value = i;	// cast integer to float
				break;
			}

		case PARAM_TYPE_FLOAT:
			param_get(param, &value);
			break;

		default:
			break;
		}

		log_msg_PARM.body.value = value;
		written += write(fd, &log_msg_PARM, sizeof(log_msg_PARM));
	}

	return written;
}

int sdlog2_thread_main(int argc, char *argv[])
{
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	if (mavlink_fd < 0) {
		warnx("failed to open MAVLink log stream, start mavlink app first");
	}

	/* log buffer size */
	int log_buffer_size = LOG_BUFFER_SIZE_DEFAULT;

	logging_enabled = false;
	log_on_start = false;
	log_when_armed = false;
	log_name_timestamp = false;

	flag_system_armed = false;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;
	int ch;

	while ((ch = getopt(argc, argv, "r:b:eat")) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(optarg, NULL, 10);

				if (r == 0) {
					sleep_delay = 0;

				} else {
					sleep_delay = 1000000 / r;
				}
			}
			break;

		case 'b': {
				unsigned long s = strtoul(optarg, NULL, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'a':
			log_when_armed = true;
			break;

		case 't':
			log_name_timestamp = true;
			break;

		case '?':
			if (optopt == 'c') {
				warnx("option -%c requires an argument", optopt);

			} else if (isprint(optopt)) {
				warnx("unknown option `-%c'", optopt);

			} else {
				warnx("unknown option character `\\x%x'", optopt);
			}

		default:
			sdlog2_usage("unrecognized flag");
			errx(1, "exiting");
		}
	}

	gps_time = 0;

	/* create log root dir */
	int mkdir_ret = mkdir(log_root, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret != 0 && errno != EEXIST) {
		err("failed creating log root dir: %s", log_root);
	}

	/* copy conversion scripts */
	const char *converter_in = "/etc/logging/conv.zip";
	char *converter_out = malloc(64);
	snprintf(converter_out, 64, "%s/conv.zip", log_root);

	if (file_copy(converter_in, converter_out) != OK) {
		warn("unable to copy conversion scripts");
	}

	free(converter_out);

	/* initialize log buffer with specified size */
	warnx("log buffer size: %i bytes", log_buffer_size);

	if (OK != logbuffer_init(&lb, log_buffer_size)) {
		errx(1, "can't allocate log buffer, exiting");
	}

	struct vehicle_status_s buf_status;

	memset(&buf_status, 0, sizeof(buf_status));

	/* warning! using union here to save memory, elements should be used separately! */
	union {
		struct vehicle_command_s cmd;
		struct sensor_combined_s sensor;
		struct vehicle_attitude_s att;
		struct vehicle_attitude_setpoint_s att_sp;
		struct vehicle_rates_setpoint_s rates_sp;
		struct actuator_outputs_s act_outputs;
		struct actuator_controls_s act_controls;
		struct vehicle_local_position_s local_pos;
		struct vehicle_local_position_setpoint_s local_pos_sp;
		struct vehicle_global_position_s global_pos;
		struct vehicle_global_position_setpoint_s global_pos_sp;
		struct vehicle_gps_position_s gps_pos;
		struct vehicle_vicon_position_s vicon_pos;
		struct optical_flow_s flow;
		struct rc_channels_s rc;
		struct differential_pressure_s diff_pres;
		struct airspeed_s airspeed;
		struct esc_status_s esc;
		struct vehicle_global_velocity_setpoint_s global_vel_sp;
		struct battery_status_s battery;
	} buf;

	memset(&buf, 0, sizeof(buf));

	struct {
		int cmd_sub;
		int status_sub;
		int sensor_sub;
		int att_sub;
		int att_sp_sub;
		int rates_sp_sub;
		int act_outputs_sub;
		int act_controls_sub;
		int local_pos_sub;
		int local_pos_sp_sub;
		int global_pos_sub;
		int global_pos_sp_sub;
		int gps_pos_sub;
		int vicon_pos_sub;
		int flow_sub;
		int rc_sub;
		int airspeed_sub;
		int esc_sub;
		int global_vel_sp_sub;
		int battery_sub;
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
			struct log_ATTC_s log_ATTC;
			struct log_STAT_s log_STAT;
			struct log_RC_s log_RC;
			struct log_OUT0_s log_OUT0;
			struct log_AIRS_s log_AIRS;
			struct log_ARSP_s log_ARSP;
			struct log_FLOW_s log_FLOW;
			struct log_GPOS_s log_GPOS;
			struct log_GPSP_s log_GPSP;
			struct log_ESC_s log_ESC;
			struct log_GVSP_s log_GVSP;
			struct log_BATT_s log_BATT;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
#pragma pack(pop)
	memset(&log_msg.body, 0, sizeof(log_msg.body));

	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 25;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];

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

	/* --- RATES SETPOINT --- */
	subs.rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	fds[fdsc_count].fd = subs.rates_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR OUTPUTS --- */
	subs.act_outputs_sub = orb_subscribe(ORB_ID_VEHICLE_CONTROLS);
	fds[fdsc_count].fd = subs.act_outputs_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ACTUATOR CONTROL --- */
	subs.act_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	fds[fdsc_count].fd = subs.act_controls_sub;
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

	/* --- GLOBAL POSITION SETPOINT--- */
	subs.global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	fds[fdsc_count].fd = subs.global_pos_sp_sub;
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

	/* --- RC CHANNELS --- */
	subs.rc_sub = orb_subscribe(ORB_ID(rc_channels));
	fds[fdsc_count].fd = subs.rc_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- AIRSPEED --- */
	subs.airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	fds[fdsc_count].fd = subs.airspeed_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ESCs --- */
	subs.esc_sub = orb_subscribe(ORB_ID(esc_status));
	fds[fdsc_count].fd = subs.esc_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GLOBAL VELOCITY SETPOINT --- */
	subs.global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	fds[fdsc_count].fd = subs.global_vel_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- BATTERY --- */
	subs.battery_sub = orb_subscribe(ORB_ID(battery_status));
	fds[fdsc_count].fd = subs.battery_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* WARNING: If you get the error message below,
	 * then the number of registered messages (fdsc)
	 * differs from the number of messages in the above list.
	 */
	if (fdsc_count > fdsc) {
		warn("WARNING: Not enough space for poll fds allocated. Check %s:%d", __FILE__, __LINE__);
		fdsc_count = fdsc;
	}

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms
	 */
	const int poll_timeout = 1000;

	thread_running = true;

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
	if (log_on_start) {
		/* check GPS topic to get GPS time */
		if (log_name_timestamp) {
			if (OK == orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos)) {
				gps_time = buf.gps_pos.time_gps_usec;
			}
		}

		sdlog2_start_log();
	}

	while (!main_thread_should_exit) {
		/* decide use usleep() or blocking poll() */
		bool use_sleep = sleep_delay > 0 && logging_enabled;

		/* poll all topics if logging enabled or only management (first 2) if not */
		int poll_ret = poll(fds, logging_enabled ? fdsc_count : 3, use_sleep ? 0 : poll_timeout);

		/* handle the poll result */
		if (poll_ret < 0) {
			warnx("ERROR: poll error, stop logging");
			main_thread_should_exit = true;

		} else if (poll_ret > 0) {

			/* check all data subscriptions only if logging enabled,
			 * logging_enabled can be changed while checking vehicle_command and vehicle_status */
			bool check_data = logging_enabled;
			int ifds = 0;
			int handled_topics = 0;

			/* --- VEHICLE COMMAND - LOG MANAGEMENT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_command), subs.cmd_sub, &buf.cmd);
				handle_command(&buf.cmd);
				handled_topics++;
			}

			/* --- VEHICLE STATUS - LOG MANAGEMENT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_status), subs.status_sub, &buf_status);

				if (log_when_armed) {
					handle_status(&buf_status);
				}

				handled_topics++;
			}

			/* --- GPS POSITION - LOG MANAGEMENT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf.gps_pos);

				if (log_name_timestamp) {
					gps_time = buf.gps_pos.time_gps_usec;
				}

				handled_topics++;
			}

			if (!logging_enabled || !check_data || handled_topics >= poll_ret) {
				continue;
			}

			ifds = 1;	// begin from fds[1] again

			pthread_mutex_lock(&logbuffer_mutex);

			/* write time stamp message */
			log_msg.msg_type = LOG_TIME_MSG;
			log_msg.body.log_TIME.t = hrt_absolute_time();
			LOGBUFFER_WRITE_AND_COUNT(TIME);

			/* --- VEHICLE STATUS --- */
			if (fds[ifds++].revents & POLLIN) {
				// Don't orb_copy, it's already done few lines above
				log_msg.msg_type = LOG_STAT_MSG;
				log_msg.body.log_STAT.main_state = (uint8_t) buf_status.main_state;
				log_msg.body.log_STAT.navigation_state = (uint8_t) buf_status.navigation_state;
				log_msg.body.log_STAT.arming_state = (uint8_t) buf_status.arming_state;
				log_msg.body.log_STAT.battery_remaining = buf_status.battery_remaining;
				log_msg.body.log_STAT.battery_warning = (uint8_t) buf_status.battery_warning;
				log_msg.body.log_STAT.landed = (uint8_t) buf_status.condition_landed;
				LOGBUFFER_WRITE_AND_COUNT(STAT);
			}

			/* --- GPS POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				// Don't orb_copy, it's already done few lines above
				log_msg.msg_type = LOG_GPS_MSG;
				log_msg.body.log_GPS.gps_time = buf.gps_pos.time_gps_usec;
				log_msg.body.log_GPS.fix_type = buf.gps_pos.fix_type;
				log_msg.body.log_GPS.eph = buf.gps_pos.eph_m;
				log_msg.body.log_GPS.epv = buf.gps_pos.epv_m;
				log_msg.body.log_GPS.lat = buf.gps_pos.lat;
				log_msg.body.log_GPS.lon = buf.gps_pos.lon;
				log_msg.body.log_GPS.alt = buf.gps_pos.alt * 0.001f;
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
				log_msg.body.log_ATT.roll_rate = buf.att.rollspeed;
				log_msg.body.log_ATT.pitch_rate = buf.att.pitchspeed;
				log_msg.body.log_ATT.yaw_rate = buf.att.yawspeed;
				LOGBUFFER_WRITE_AND_COUNT(ATT);
			}

			/* --- ATTITUDE SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_attitude_setpoint), subs.att_sp_sub, &buf.att_sp);
				log_msg.msg_type = LOG_ATSP_MSG;
				log_msg.body.log_ATSP.roll_sp = buf.att_sp.roll_body;
				log_msg.body.log_ATSP.pitch_sp = buf.att_sp.pitch_body;
				log_msg.body.log_ATSP.yaw_sp = buf.att_sp.yaw_body;
				log_msg.body.log_ATSP.thrust_sp = buf.att_sp.thrust;
				LOGBUFFER_WRITE_AND_COUNT(ATSP);
			}

			/* --- RATES SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_rates_setpoint), subs.rates_sp_sub, &buf.rates_sp);
				log_msg.msg_type = LOG_ARSP_MSG;
				log_msg.body.log_ARSP.roll_rate_sp = buf.rates_sp.roll;
				log_msg.body.log_ARSP.pitch_rate_sp = buf.rates_sp.pitch;
				log_msg.body.log_ARSP.yaw_rate_sp = buf.rates_sp.yaw;
				LOGBUFFER_WRITE_AND_COUNT(ARSP);
			}

			/* --- ACTUATOR OUTPUTS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_outputs_0), subs.act_outputs_sub, &buf.act_outputs);
				log_msg.msg_type = LOG_OUT0_MSG;
				memcpy(log_msg.body.log_OUT0.output, buf.act_outputs.output, sizeof(log_msg.body.log_OUT0.output));
				LOGBUFFER_WRITE_AND_COUNT(OUT0);
			}

			/* --- ACTUATOR CONTROL --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.act_controls_sub, &buf.act_controls);
				log_msg.msg_type = LOG_ATTC_MSG;
				log_msg.body.log_ATTC.roll = buf.act_controls.control[0];
				log_msg.body.log_ATTC.pitch = buf.act_controls.control[1];
				log_msg.body.log_ATTC.yaw = buf.act_controls.control[2];
				log_msg.body.log_ATTC.thrust = buf.act_controls.control[3];
				LOGBUFFER_WRITE_AND_COUNT(ATTC);
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
				log_msg.body.log_LPOS.ref_lat = buf.local_pos.ref_lat;
				log_msg.body.log_LPOS.ref_lon = buf.local_pos.ref_lon;
				log_msg.body.log_LPOS.ref_alt = buf.local_pos.ref_alt;
				log_msg.body.log_LPOS.xy_flags = (buf.local_pos.xy_valid ? 1 : 0) | (buf.local_pos.v_xy_valid ? 2 : 0) | (buf.local_pos.xy_global ? 8 : 0);
				log_msg.body.log_LPOS.z_flags = (buf.local_pos.z_valid ? 1 : 0) | (buf.local_pos.v_z_valid ? 2 : 0) | (buf.local_pos.z_global ? 8 : 0);
				log_msg.body.log_LPOS.landed = buf.local_pos.landed;
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
				log_msg.msg_type = LOG_GPOS_MSG;
				log_msg.body.log_GPOS.lat = buf.global_pos.lat;
				log_msg.body.log_GPOS.lon = buf.global_pos.lon;
				log_msg.body.log_GPOS.alt = buf.global_pos.alt;
				log_msg.body.log_GPOS.vel_n = buf.global_pos.vx;
				log_msg.body.log_GPOS.vel_e = buf.global_pos.vy;
				log_msg.body.log_GPOS.vel_d = buf.global_pos.vz;
				LOGBUFFER_WRITE_AND_COUNT(GPOS);
			}

			/* --- GLOBAL POSITION SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_global_position_setpoint), subs.global_pos_sp_sub, &buf.global_pos_sp);
				log_msg.msg_type = LOG_GPSP_MSG;
				log_msg.body.log_GPSP.altitude_is_relative = buf.global_pos_sp.altitude_is_relative;
				log_msg.body.log_GPSP.lat = buf.global_pos_sp.lat;
				log_msg.body.log_GPSP.lon = buf.global_pos_sp.lon;
				log_msg.body.log_GPSP.altitude = buf.global_pos_sp.altitude;
				log_msg.body.log_GPSP.yaw = buf.global_pos_sp.yaw;
				log_msg.body.log_GPSP.loiter_radius = buf.global_pos_sp.loiter_radius;
				log_msg.body.log_GPSP.loiter_direction = buf.global_pos_sp.loiter_direction;
				log_msg.body.log_GPSP.nav_cmd = buf.global_pos_sp.nav_cmd;
				log_msg.body.log_GPSP.param1 = buf.global_pos_sp.param1;
				log_msg.body.log_GPSP.param2 = buf.global_pos_sp.param2;
				log_msg.body.log_GPSP.param3 = buf.global_pos_sp.param3;
				log_msg.body.log_GPSP.param4 = buf.global_pos_sp.param4;
				LOGBUFFER_WRITE_AND_COUNT(GPSP);
			}

			/* --- VICON POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_vicon_position), subs.vicon_pos_sub, &buf.vicon_pos);
				// TODO not implemented yet
			}

			/* --- FLOW --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(optical_flow), subs.flow_sub, &buf.flow);
				log_msg.msg_type = LOG_FLOW_MSG;
				log_msg.body.log_FLOW.flow_raw_x = buf.flow.flow_raw_x;
				log_msg.body.log_FLOW.flow_raw_y = buf.flow.flow_raw_y;
				log_msg.body.log_FLOW.flow_comp_x = buf.flow.flow_comp_x_m;
				log_msg.body.log_FLOW.flow_comp_y = buf.flow.flow_comp_y_m;
				log_msg.body.log_FLOW.distance = buf.flow.ground_distance_m;
				log_msg.body.log_FLOW.quality = buf.flow.quality;
				log_msg.body.log_FLOW.sensor_id = buf.flow.sensor_id;
				LOGBUFFER_WRITE_AND_COUNT(FLOW);
			}

			/* --- RC CHANNELS --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(rc_channels), subs.rc_sub, &buf.rc);
				log_msg.msg_type = LOG_RC_MSG;
				/* Copy only the first 8 channels of 14 */
				memcpy(log_msg.body.log_RC.channel, buf.rc.chan, sizeof(log_msg.body.log_RC.channel));
				log_msg.body.log_RC.channel_count = buf.rc.chan_count;
				LOGBUFFER_WRITE_AND_COUNT(RC);
			}

			/* --- AIRSPEED --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(airspeed), subs.airspeed_sub, &buf.airspeed);
				log_msg.msg_type = LOG_AIRS_MSG;
				log_msg.body.log_AIRS.indicated_airspeed = buf.airspeed.indicated_airspeed_m_s;
				log_msg.body.log_AIRS.true_airspeed = buf.airspeed.true_airspeed_m_s;
				LOGBUFFER_WRITE_AND_COUNT(AIRS);
			}

			/* --- ESCs --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(esc_status), subs.esc_sub, &buf.esc);

				for (uint8_t i = 0; i < buf.esc.esc_count; i++) {
					log_msg.msg_type = LOG_ESC_MSG;
					log_msg.body.log_ESC.counter = buf.esc.counter;
					log_msg.body.log_ESC.esc_count = buf.esc.esc_count;
					log_msg.body.log_ESC.esc_connectiontype = buf.esc.esc_connectiontype;
					log_msg.body.log_ESC.esc_num = i;
					log_msg.body.log_ESC.esc_address = buf.esc.esc[i].esc_address;
					log_msg.body.log_ESC.esc_version = buf.esc.esc[i].esc_version;
					log_msg.body.log_ESC.esc_voltage = buf.esc.esc[i].esc_voltage;
					log_msg.body.log_ESC.esc_current = buf.esc.esc[i].esc_current;
					log_msg.body.log_ESC.esc_rpm = buf.esc.esc[i].esc_rpm;
					log_msg.body.log_ESC.esc_temperature = buf.esc.esc[i].esc_temperature;
					log_msg.body.log_ESC.esc_setpoint = buf.esc.esc[i].esc_setpoint;
					log_msg.body.log_ESC.esc_setpoint_raw = buf.esc.esc[i].esc_setpoint_raw;
					LOGBUFFER_WRITE_AND_COUNT(ESC);
				}
			}

			/* --- GLOBAL VELOCITY SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(vehicle_global_velocity_setpoint), subs.global_vel_sp_sub, &buf.global_vel_sp);
				log_msg.msg_type = LOG_GVSP_MSG;
				log_msg.body.log_GVSP.vx = buf.global_vel_sp.vx;
				log_msg.body.log_GVSP.vy = buf.global_vel_sp.vy;
				log_msg.body.log_GVSP.vz = buf.global_vel_sp.vz;
				LOGBUFFER_WRITE_AND_COUNT(GVSP);
			}

			/* --- BATTERY --- */
			if (fds[ifds++].revents & POLLIN) {
				orb_copy(ORB_ID(battery_status), subs.battery_sub, &buf.battery);
				log_msg.msg_type = LOG_BATT_MSG;
				log_msg.body.log_BATT.voltage = buf.battery.voltage_v;
				log_msg.body.log_BATT.voltage_filtered = buf.battery.voltage_filtered_v;
				log_msg.body.log_BATT.current = buf.battery.current_a;
				log_msg.body.log_BATT.discharged = buf.battery.discharged_mah;
				LOGBUFFER_WRITE_AND_COUNT(BATT);
			}

			/* signal the other thread new data, but not yet unlock */
			if (logbuffer_count(&lb) > MIN_BYTES_TO_WRITE) {
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

	free(lb.data);

	warnx("exiting");

	thread_running = false;

	return 0;
}

void sdlog2_status()
{
	float kibibytes = log_bytes_written / 1024.0f;
	float mebibytes = kibibytes / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - start_time)) / 1000000.0f;

	warnx("wrote %lu msgs, %4.2f MiB (average %5.3f KiB/s), skipped %lu msgs", log_msgs_written, (double)mebibytes, (double)(kibibytes / seconds), log_msgs_skipped);
	mavlink_log_info(mavlink_fd, "[sdlog2] wrote %lu msgs, skipped %lu msgs", log_msgs_written, log_msgs_skipped);
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

	return OK;
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
	// TODO use flag from actuator_armed here?
	bool armed = status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR;

	if (armed != flag_system_armed) {
		flag_system_armed = armed;

		if (flag_system_armed) {
			sdlog2_start_log();

		} else {
			sdlog2_stop_log();
		}
	}
}
