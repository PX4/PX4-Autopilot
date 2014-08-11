/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_range_finder.h>

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
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/wind_estimate.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <version/version.h>

#include <mavlink/mavlink_log.h>

#include "logbuffer.h"
#include "sdlog2_format.h"
#include "sdlog2_messages.h"

/**
 * Logging rate.
 *
 * A value of -1 indicates the commandline argument
 * should be obeyed. A value of 0 sets the minimum rate,
 * any other value is interpreted as rate in Hertz. This
 * parameter is only read out before logging starts (which
 * commonly is before arming).
 *
 * @min -1
 * @max  1
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_RATE, -1);

/**
 * Enable extended logging mode.
 *
 * A value of -1 indicates the commandline argument
 * should be obeyed. A value of 0 disables extended
 * logging mode, a value of 1 enables it. This
 * parameter is only read out before logging starts
 * (which commonly is before arming).
 *
 * @min -1
 * @max  1
 * @group SD Logging
 */
PARAM_DEFINE_INT32(SDLOG_EXT, -1);

#define LOGBUFFER_WRITE_AND_COUNT(_msg) if (logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(_msg))) { \
		log_msgs_written++; \
	} else { \
		log_msgs_skipped++; \
	}

#define LOG_ORB_SUBSCRIBE(_var, _topic) subs.##_var##_sub = orb_subscribe(ORB_ID(##_topic##)); \
	fds[fdsc_count].fd = subs.##_var##_sub; \
	fds[fdsc_count].events = POLLIN; \
	fdsc_count++;

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

static bool main_thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;			/**< Deamon status flag */
static int deamon_task;						/**< Handle of deamon task / thread */
static bool logwriter_should_exit = false;	/**< Logwriter thread exit flag */
static const unsigned MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static const int LOG_BUFFER_SIZE_DEFAULT = 8192;
static const int MAX_WRITE_CHUNK = 512;
static const int MIN_BYTES_TO_WRITE = 512;

static bool _extended_logging = false;

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

static bool copy_if_updated(orb_id_t topic, int handle, void *buffer);

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

static int open_perf_file(const char* str);

static void
sdlog2_usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	errx(1, "usage: sdlog2 {start|stop|status} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
		 "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
		 "\t-b\tLog buffer size in KiB, default is 8\n"
		 "\t-e\tEnable logging by default (if not, can be started by command)\n"
		 "\t-a\tLog only when armed (can be still overriden by command)\n"
		 "\t-t\tUse date/time for naming log directories and files\n"
		 "\t-x\tExtended logging");
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
	if (argc < 2) {
		sdlog2_usage("missing command");
	}

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
	char log_file_name[32] = "";
	char log_file_path[64] = "";

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
			mavlink_log_critical(mavlink_fd, "[sdlog2] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);

	if (fd < 0) {
		warn("failed opening log: %s", log_file_name);
		mavlink_log_critical(mavlink_fd, "[sdlog2] failed opening log: %s", log_file_name);

	} else {
		warnx("log file: %s", log_file_name);
		mavlink_log_info(mavlink_fd, "[sdlog2] log file: %s", log_file_name);
	}

	return fd;
}

int open_perf_file(const char* str)
{
	/* string to hold the path to the log */
	char log_file_name[32] = "";
	char log_file_path[64] = "";

	if (log_name_timestamp && gps_time != 0) {
		/* use GPS time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.bin */
		time_t gps_time_sec = gps_time / 1000000;
		struct tm t;
		gmtime_r(&gps_time_sec, &t);
		strftime(log_file_name, sizeof(log_file_name), "perf%H_%M_%S.txt", &t);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

	} else {
		unsigned file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.bin */
			snprintf(log_file_name, sizeof(log_file_name), "perf%03u.txt", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			mavlink_log_critical(mavlink_fd, "[sdlog2] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);

	if (fd < 0) {
		warn("failed opening log: %s", log_file_name);
		mavlink_log_critical(mavlink_fd, "[sdlog2] failed opening log: %s", log_file_name);

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

	if (log_fd < 0) {
		return NULL;
	}

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

	return NULL;
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

	/* write all performance counters */
	int perf_fd = open_perf_file("preflight");
	dprintf(perf_fd, "PERFORMANCE COUNTERS PRE-FLIGHT\n\n");
	perf_print_all(perf_fd);
	close(perf_fd);

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

	/* write all performance counters */
	int perf_fd = open_perf_file("postflight");
	dprintf(perf_fd, "PERFORMANCE COUNTERS POST-FLIGHT\n\n");
	perf_print_all(perf_fd);
	close(perf_fd);

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
	for (unsigned i = 0; i < log_formats_num; i++) {
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

bool copy_if_updated(orb_id_t topic, int handle, void *buffer)
{
	bool updated;

	orb_check(handle, &updated);

	if (updated) {
		orb_copy(topic, handle, buffer);
	}

	return updated;
}

int sdlog2_thread_main(int argc, char *argv[])
{
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	if (mavlink_fd < 0) {
		warnx("failed to open MAVLink log stream, start mavlink app first");
	}

	/* delay = 1 / rate (rate defined by -r option), default log rate: 50 Hz */
	useconds_t sleep_delay = 20000;
	int log_buffer_size = LOG_BUFFER_SIZE_DEFAULT;
	logging_enabled = false;
	/* enable logging on start (-e option) */
	bool log_on_start = false;
	/* enable logging when armed (-a option) */
	bool log_when_armed = false;
	log_name_timestamp = false;

	flag_system_armed = false;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;
	int ch;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	while ((ch = getopt(argc, argv, "r:b:eatx")) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(optarg, NULL, 10);

				if (r == 0) {
					r = 1;
				}

				sleep_delay = 1000000 / r;
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

		case 'x':
			_extended_logging = true;
			break;

		case '?':
			if (optopt == 'c') {
				warnx("option -%c requires an argument", optopt);

			} else if (isprint(optopt)) {
				warnx("unknown option `-%c'", optopt);

			} else {
				warnx("unknown option character `\\x%x'", optopt);
			}
			err_flag = true;
			break;

		default:
			warnx("unrecognized flag");
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		sdlog2_usage(NULL);
	}

	gps_time = 0;

	/* interpret logging params */

	param_t log_rate_ph = param_find("SDLOG_RATE");

	if (log_rate_ph != PARAM_INVALID) {
		int32_t param_log_rate;
		param_get(log_rate_ph, &param_log_rate);

		if (param_log_rate > 0) {

			/* we can't do more than ~ 500 Hz, even with a massive buffer */
			if (param_log_rate > 500) {
				param_log_rate = 500;
			}

			sleep_delay = 1000000 / param_log_rate;
		} else if (param_log_rate == 0) {
			/* we need at minimum 10 Hz to be able to see anything */
			sleep_delay = 1000000 / 10;
		}
	}

	param_t log_ext_ph = param_find("SDLOG_EXT");

	if (log_ext_ph != PARAM_INVALID) {

		int32_t param_log_extended;
		param_get(log_ext_ph, &param_log_extended);

		if (param_log_extended > 0) {
			_extended_logging = true;
		} else if (param_log_extended == 0) {
			_extended_logging = false;
		}
		/* any other value means to ignore the parameter, so no else case */

	}

	/* create log root dir */
	int mkdir_ret = mkdir(log_root, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret != 0 && errno != EEXIST) {
		err(1, "failed creating log root dir: %s", log_root);
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

	struct vehicle_gps_position_s buf_gps_pos;

	memset(&buf_status, 0, sizeof(buf_status));

	memset(&buf_gps_pos, 0, sizeof(buf_gps_pos));

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
		struct position_setpoint_triplet_s triplet;
		struct vehicle_vicon_position_s vicon_pos;
		struct optical_flow_s flow;
		struct rc_channels_s rc;
		struct differential_pressure_s diff_pres;
		struct airspeed_s airspeed;
		struct esc_status_s esc;
		struct vehicle_global_velocity_setpoint_s global_vel_sp;
		struct battery_status_s battery;
		struct telemetry_status_s telemetry;
		struct range_finder_report range_finder;
		struct estimator_status_report estimator_status;
		struct tecs_status_s tecs_status;
		struct system_power_s system_power;
		struct servorail_status_s servorail_status;
		struct satellite_info_s sat_info;
		struct wind_estimate_s wind_estimate;
	} buf;

	memset(&buf, 0, sizeof(buf));

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
			struct log_DIST_s log_DIST;
			struct log_TEL_s log_TEL;
			struct log_EST0_s log_EST0;
			struct log_EST1_s log_EST1;
			struct log_PWR_s log_PWR;
			struct log_VICN_s log_VICN;
			struct log_GS0A_s log_GS0A;
			struct log_GS0B_s log_GS0B;
			struct log_GS1A_s log_GS1A;
			struct log_GS1B_s log_GS1B;
			struct log_TECS_s log_TECS;
			struct log_WIND_s log_WIND;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
#pragma pack(pop)
	memset(&log_msg.body, 0, sizeof(log_msg.body));

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
		int triplet_sub;
		int gps_pos_sub;
		int sat_info_sub;
		int vicon_pos_sub;
		int flow_sub;
		int rc_sub;
		int airspeed_sub;
		int esc_sub;
		int global_vel_sp_sub;
		int battery_sub;
		int telemetry_subs[TELEMETRY_STATUS_ORB_ID_NUM];
		int range_finder_sub;
		int estimator_status_sub;
		int tecs_status_sub;
		int system_power_sub;
		int servorail_status_sub;
		int wind_sub;
	} subs;

	subs.cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	subs.status_sub = orb_subscribe(ORB_ID(vehicle_status));
	subs.gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	subs.sat_info_sub = orb_subscribe(ORB_ID(satellite_info));
	subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	subs.att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	subs.rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	subs.act_outputs_sub = orb_subscribe(ORB_ID_VEHICLE_CONTROLS);
	subs.act_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	subs.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	subs.local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	subs.triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	subs.vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	subs.flow_sub = orb_subscribe(ORB_ID(optical_flow));
	subs.rc_sub = orb_subscribe(ORB_ID(rc_channels));
	subs.airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	subs.esc_sub = orb_subscribe(ORB_ID(esc_status));
	subs.global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	subs.battery_sub = orb_subscribe(ORB_ID(battery_status));
	for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
		subs.telemetry_subs[i] = orb_subscribe(telemetry_status_orb_id[i]);
	}
	subs.range_finder_sub = orb_subscribe(ORB_ID(sensor_range_finder));
	subs.estimator_status_sub = orb_subscribe(ORB_ID(estimator_status));
	subs.tecs_status_sub = orb_subscribe(ORB_ID(tecs_status));
	subs.system_power_sub = orb_subscribe(ORB_ID(system_power));
	subs.servorail_status_sub = orb_subscribe(ORB_ID(servorail_status));
	subs.wind_sub = orb_subscribe(ORB_ID(wind_estimate));
	/* we need to rate-limit wind, as we do not need the full update rate */
	orb_set_interval(subs.wind_sub, 90);

	thread_running = true;

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
	pthread_cond_init(&logbuffer_cond, NULL);

	/* track changes in sensor_combined topic */
	hrt_abstime gyro_timestamp = 0;
	hrt_abstime accelerometer_timestamp = 0;
	hrt_abstime magnetometer_timestamp = 0;
	hrt_abstime barometer_timestamp = 0;
	hrt_abstime differential_pressure_timestamp = 0;
	hrt_abstime gyro1_timestamp = 0;
	hrt_abstime accelerometer1_timestamp = 0;
	hrt_abstime magnetometer1_timestamp = 0;
	hrt_abstime gyro2_timestamp = 0;
	hrt_abstime accelerometer2_timestamp = 0;
	hrt_abstime magnetometer2_timestamp = 0;

	/* initialize calculated mean SNR */
	float snr_mean = 0.0f;

	/* enable logging on start if needed */
	if (log_on_start) {
		/* check GPS topic to get GPS time */
		if (log_name_timestamp) {
			if (!orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf_gps_pos)) {
				gps_time = buf_gps_pos.time_gps_usec;
			}
		}

		sdlog2_start_log();
	}

	while (!main_thread_should_exit) {
		usleep(sleep_delay);

		/* --- VEHICLE COMMAND - LOG MANAGEMENT --- */
		if (copy_if_updated(ORB_ID(vehicle_command), subs.cmd_sub, &buf.cmd)) {
			handle_command(&buf.cmd);
		}

		/* --- VEHICLE STATUS - LOG MANAGEMENT --- */
		bool status_updated = copy_if_updated(ORB_ID(vehicle_status), subs.status_sub, &buf_status);

		if (status_updated) {
			if (log_when_armed) {
				handle_status(&buf_status);
			}
		}

		/* --- GPS POSITION - LOG MANAGEMENT --- */
		bool gps_pos_updated = copy_if_updated(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf_gps_pos);

		if (gps_pos_updated && log_name_timestamp) {
			gps_time = buf_gps_pos.time_gps_usec;
		}

		if (!logging_enabled) {
			continue;
		}

		pthread_mutex_lock(&logbuffer_mutex);

		/* write time stamp message */
		log_msg.msg_type = LOG_TIME_MSG;
		log_msg.body.log_TIME.t = hrt_absolute_time();
		LOGBUFFER_WRITE_AND_COUNT(TIME);

		/* --- VEHICLE STATUS --- */
		if (status_updated) {
			log_msg.msg_type = LOG_STAT_MSG;
			log_msg.body.log_STAT.main_state = (uint8_t) buf_status.main_state;
			log_msg.body.log_STAT.arming_state = (uint8_t) buf_status.arming_state;
			log_msg.body.log_STAT.failsafe_state = (uint8_t) buf_status.failsafe;
			log_msg.body.log_STAT.battery_remaining = buf_status.battery_remaining;
			log_msg.body.log_STAT.battery_warning = (uint8_t) buf_status.battery_warning;
			log_msg.body.log_STAT.landed = (uint8_t) buf_status.condition_landed;
			LOGBUFFER_WRITE_AND_COUNT(STAT);
		}

		/* --- GPS POSITION - UNIT #1 --- */
		if (gps_pos_updated) {

			log_msg.msg_type = LOG_GPS_MSG;
			log_msg.body.log_GPS.gps_time = buf_gps_pos.time_gps_usec;
			log_msg.body.log_GPS.fix_type = buf_gps_pos.fix_type;
			log_msg.body.log_GPS.eph = buf_gps_pos.eph;
			log_msg.body.log_GPS.epv = buf_gps_pos.epv;
			log_msg.body.log_GPS.lat = buf_gps_pos.lat;
			log_msg.body.log_GPS.lon = buf_gps_pos.lon;
			log_msg.body.log_GPS.alt = buf_gps_pos.alt * 0.001f;
			log_msg.body.log_GPS.vel_n = buf_gps_pos.vel_n_m_s;
			log_msg.body.log_GPS.vel_e = buf_gps_pos.vel_e_m_s;
			log_msg.body.log_GPS.vel_d = buf_gps_pos.vel_d_m_s;
			log_msg.body.log_GPS.cog = buf_gps_pos.cog_rad;
			log_msg.body.log_GPS.sats = buf_gps_pos.satellites_used;
			log_msg.body.log_GPS.snr_mean = snr_mean;
			log_msg.body.log_GPS.noise_per_ms = buf_gps_pos.noise_per_ms;
			log_msg.body.log_GPS.jamming_indicator = buf_gps_pos.jamming_indicator;
			LOGBUFFER_WRITE_AND_COUNT(GPS);
		}

		/* --- SATELLITE INFO - UNIT #1 --- */
		if (_extended_logging) {

			if (copy_if_updated(ORB_ID(satellite_info), subs.sat_info_sub, &buf.sat_info)) {

				/* log the SNR of each satellite for a detailed view of signal quality */
				unsigned sat_info_count = MIN(buf.sat_info.count, sizeof(buf.sat_info.snr) / sizeof(buf.sat_info.snr[0]));
				unsigned log_max_snr = sizeof(log_msg.body.log_GS0A.satellite_snr) / sizeof(log_msg.body.log_GS0A.satellite_snr[0]);

				log_msg.msg_type = LOG_GS0A_MSG;
				memset(&log_msg.body.log_GS0A, 0, sizeof(log_msg.body.log_GS0A));
				snr_mean = 0.0f;

				/* fill set A and calculate mean SNR */
				for (unsigned i = 0; i < sat_info_count; i++) {

					snr_mean += buf.sat_info.snr[i];

					int satindex = buf.sat_info.svid[i] - 1;

					/* handles index exceeding and wraps to to arithmetic errors */
					if ((satindex >= 0) && (satindex < (int)log_max_snr)) {
						/* map satellites by their ID so that logs from two receivers can be compared */
						log_msg.body.log_GS0A.satellite_snr[satindex] = buf.sat_info.snr[i];
					}
				}
				LOGBUFFER_WRITE_AND_COUNT(GS0A);
				snr_mean /= sat_info_count;

				log_msg.msg_type = LOG_GS0B_MSG;
				memset(&log_msg.body.log_GS0B, 0, sizeof(log_msg.body.log_GS0B));

				/* fill set B */
				for (unsigned i = 0; i < sat_info_count; i++) {

					/* get second bank of satellites, thus deduct bank size from index */
					int satindex = buf.sat_info.svid[i] - 1 - log_max_snr;

					/* handles index exceeding and wraps to to arithmetic errors */
					if ((satindex >= 0) && (satindex < (int)log_max_snr)) {
						/* map satellites by their ID so that logs from two receivers can be compared */
						log_msg.body.log_GS0B.satellite_snr[satindex] = buf.sat_info.snr[i];
					}
				}
				LOGBUFFER_WRITE_AND_COUNT(GS0B);
			}
		}

		/* --- SENSOR COMBINED --- */
		if (copy_if_updated(ORB_ID(sensor_combined), subs.sensor_sub, &buf.sensor)) {
			bool write_IMU = false;
			bool write_IMU1 = false;
			bool write_IMU2 = false;
			bool write_SENS = false;

			if (buf.sensor.timestamp != gyro_timestamp) {
				gyro_timestamp = buf.sensor.timestamp;
				write_IMU = true;
			}

			if (buf.sensor.accelerometer_timestamp != accelerometer_timestamp) {
				accelerometer_timestamp = buf.sensor.accelerometer_timestamp;
				write_IMU = true;
			}

			if (buf.sensor.magnetometer_timestamp != magnetometer_timestamp) {
				magnetometer_timestamp = buf.sensor.magnetometer_timestamp;
				write_IMU = true;
			}

			if (buf.sensor.baro_timestamp != barometer_timestamp) {
				barometer_timestamp = buf.sensor.baro_timestamp;
				write_SENS = true;
			}

			if (buf.sensor.differential_pressure_timestamp != differential_pressure_timestamp) {
				differential_pressure_timestamp = buf.sensor.differential_pressure_timestamp;
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
				log_msg.body.log_SENS.diff_pres_filtered = buf.sensor.differential_pressure_filtered_pa;
				LOGBUFFER_WRITE_AND_COUNT(SENS);
			}

			if (buf.sensor.accelerometer1_timestamp != accelerometer1_timestamp) {
				accelerometer1_timestamp = buf.sensor.accelerometer1_timestamp;
				write_IMU1 = true;
			}

			if (buf.sensor.gyro1_timestamp != gyro1_timestamp) {
				gyro1_timestamp = buf.sensor.gyro1_timestamp;
				write_IMU1 = true;
			}

			if (buf.sensor.magnetometer1_timestamp != magnetometer1_timestamp) {
				magnetometer1_timestamp = buf.sensor.magnetometer1_timestamp;
				write_IMU1 = true;
			}

			if (write_IMU1) {
				log_msg.msg_type = LOG_IMU1_MSG;
				log_msg.body.log_IMU.gyro_x = buf.sensor.gyro1_rad_s[0];
				log_msg.body.log_IMU.gyro_y = buf.sensor.gyro1_rad_s[1];
				log_msg.body.log_IMU.gyro_z = buf.sensor.gyro1_rad_s[2];
				log_msg.body.log_IMU.acc_x = buf.sensor.accelerometer1_m_s2[0];
				log_msg.body.log_IMU.acc_y = buf.sensor.accelerometer1_m_s2[1];
				log_msg.body.log_IMU.acc_z = buf.sensor.accelerometer1_m_s2[2];
				log_msg.body.log_IMU.mag_x = buf.sensor.magnetometer1_ga[0];
				log_msg.body.log_IMU.mag_y = buf.sensor.magnetometer1_ga[1];
				log_msg.body.log_IMU.mag_z = buf.sensor.magnetometer1_ga[2];
				LOGBUFFER_WRITE_AND_COUNT(IMU);
			}

			if (buf.sensor.accelerometer2_timestamp != accelerometer2_timestamp) {
				accelerometer2_timestamp = buf.sensor.accelerometer2_timestamp;
				write_IMU2 = true;
			}

			if (buf.sensor.gyro2_timestamp != gyro2_timestamp) {
				gyro2_timestamp = buf.sensor.gyro2_timestamp;
				write_IMU2 = true;
			}

			if (buf.sensor.magnetometer2_timestamp != magnetometer2_timestamp) {
				magnetometer2_timestamp = buf.sensor.magnetometer2_timestamp;
				write_IMU2 = true;
			}

			if (write_IMU2) {
				log_msg.msg_type = LOG_IMU2_MSG;
				log_msg.body.log_IMU.gyro_x = buf.sensor.gyro2_rad_s[0];
				log_msg.body.log_IMU.gyro_y = buf.sensor.gyro2_rad_s[1];
				log_msg.body.log_IMU.gyro_z = buf.sensor.gyro2_rad_s[2];
				log_msg.body.log_IMU.acc_x = buf.sensor.accelerometer2_m_s2[0];
				log_msg.body.log_IMU.acc_y = buf.sensor.accelerometer2_m_s2[1];
				log_msg.body.log_IMU.acc_z = buf.sensor.accelerometer2_m_s2[2];
				log_msg.body.log_IMU.mag_x = buf.sensor.magnetometer2_ga[0];
				log_msg.body.log_IMU.mag_y = buf.sensor.magnetometer2_ga[1];
				log_msg.body.log_IMU.mag_z = buf.sensor.magnetometer2_ga[2];
				LOGBUFFER_WRITE_AND_COUNT(IMU);
			}

		}

		/* --- ATTITUDE --- */
		if (copy_if_updated(ORB_ID(vehicle_attitude), subs.att_sub, &buf.att)) {
			log_msg.msg_type = LOG_ATT_MSG;
			log_msg.body.log_ATT.roll = buf.att.roll;
			log_msg.body.log_ATT.pitch = buf.att.pitch;
			log_msg.body.log_ATT.yaw = buf.att.yaw;
			log_msg.body.log_ATT.roll_rate = buf.att.rollspeed;
			log_msg.body.log_ATT.pitch_rate = buf.att.pitchspeed;
			log_msg.body.log_ATT.yaw_rate = buf.att.yawspeed;
			log_msg.body.log_ATT.gx = buf.att.g_comp[0];
			log_msg.body.log_ATT.gy = buf.att.g_comp[1];
			log_msg.body.log_ATT.gz = buf.att.g_comp[2];
			LOGBUFFER_WRITE_AND_COUNT(ATT);
		}

		/* --- ATTITUDE SETPOINT --- */
		if (copy_if_updated(ORB_ID(vehicle_attitude_setpoint), subs.att_sp_sub, &buf.att_sp)) {
			log_msg.msg_type = LOG_ATSP_MSG;
			log_msg.body.log_ATSP.roll_sp = buf.att_sp.roll_body;
			log_msg.body.log_ATSP.pitch_sp = buf.att_sp.pitch_body;
			log_msg.body.log_ATSP.yaw_sp = buf.att_sp.yaw_body;
			log_msg.body.log_ATSP.thrust_sp = buf.att_sp.thrust;
			LOGBUFFER_WRITE_AND_COUNT(ATSP);
		}

		/* --- RATES SETPOINT --- */
		if (copy_if_updated(ORB_ID(vehicle_rates_setpoint), subs.rates_sp_sub, &buf.rates_sp)) {
			log_msg.msg_type = LOG_ARSP_MSG;
			log_msg.body.log_ARSP.roll_rate_sp = buf.rates_sp.roll;
			log_msg.body.log_ARSP.pitch_rate_sp = buf.rates_sp.pitch;
			log_msg.body.log_ARSP.yaw_rate_sp = buf.rates_sp.yaw;
			LOGBUFFER_WRITE_AND_COUNT(ARSP);
		}

		/* --- ACTUATOR OUTPUTS --- */
		if (copy_if_updated(ORB_ID(actuator_outputs_0), subs.act_outputs_sub, &buf.act_outputs)) {
			log_msg.msg_type = LOG_OUT0_MSG;
			memcpy(log_msg.body.log_OUT0.output, buf.act_outputs.output, sizeof(log_msg.body.log_OUT0.output));
			LOGBUFFER_WRITE_AND_COUNT(OUT0);
		}

		/* --- ACTUATOR CONTROL --- */
		if (copy_if_updated(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, subs.act_controls_sub, &buf.act_controls)) {
			log_msg.msg_type = LOG_ATTC_MSG;
			log_msg.body.log_ATTC.roll = buf.act_controls.control[0];
			log_msg.body.log_ATTC.pitch = buf.act_controls.control[1];
			log_msg.body.log_ATTC.yaw = buf.act_controls.control[2];
			log_msg.body.log_ATTC.thrust = buf.act_controls.control[3];
			LOGBUFFER_WRITE_AND_COUNT(ATTC);
		}

		/* --- LOCAL POSITION --- */
		if (copy_if_updated(ORB_ID(vehicle_local_position), subs.local_pos_sub, &buf.local_pos)) {
			log_msg.msg_type = LOG_LPOS_MSG;
			log_msg.body.log_LPOS.x = buf.local_pos.x;
			log_msg.body.log_LPOS.y = buf.local_pos.y;
			log_msg.body.log_LPOS.z = buf.local_pos.z;
			log_msg.body.log_LPOS.ground_dist = buf.local_pos.dist_bottom;
			log_msg.body.log_LPOS.ground_dist_rate = buf.local_pos.dist_bottom_rate;
			log_msg.body.log_LPOS.vx = buf.local_pos.vx;
			log_msg.body.log_LPOS.vy = buf.local_pos.vy;
			log_msg.body.log_LPOS.vz = buf.local_pos.vz;
			log_msg.body.log_LPOS.ref_lat = buf.local_pos.ref_lat * 1e7;
			log_msg.body.log_LPOS.ref_lon = buf.local_pos.ref_lon * 1e7;
			log_msg.body.log_LPOS.ref_alt = buf.local_pos.ref_alt;
			log_msg.body.log_LPOS.pos_flags = (buf.local_pos.xy_valid ? 1 : 0) |
											  (buf.local_pos.z_valid ? 2 : 0) |
											  (buf.local_pos.v_xy_valid ? 4 : 0) |
											  (buf.local_pos.v_z_valid ? 8 : 0) |
											  (buf.local_pos.xy_global ? 16 : 0) |
											  (buf.local_pos.z_global ? 32 : 0);
			log_msg.body.log_LPOS.landed = buf.local_pos.landed;
			log_msg.body.log_LPOS.ground_dist_flags = (buf.local_pos.dist_bottom_valid ? 1 : 0);
			log_msg.body.log_LPOS.eph = buf.local_pos.eph;
			log_msg.body.log_LPOS.epv = buf.local_pos.epv;
			LOGBUFFER_WRITE_AND_COUNT(LPOS);
		}

		/* --- LOCAL POSITION SETPOINT --- */
		if (copy_if_updated(ORB_ID(vehicle_local_position_setpoint), subs.local_pos_sp_sub, &buf.local_pos_sp)) {
			log_msg.msg_type = LOG_LPSP_MSG;
			log_msg.body.log_LPSP.x = buf.local_pos_sp.x;
			log_msg.body.log_LPSP.y = buf.local_pos_sp.y;
			log_msg.body.log_LPSP.z = buf.local_pos_sp.z;
			log_msg.body.log_LPSP.yaw = buf.local_pos_sp.yaw;
			LOGBUFFER_WRITE_AND_COUNT(LPSP);
		}

		/* --- GLOBAL POSITION --- */
		if (copy_if_updated(ORB_ID(vehicle_global_position), subs.global_pos_sub, &buf.global_pos)) {
			log_msg.msg_type = LOG_GPOS_MSG;
			log_msg.body.log_GPOS.lat = buf.global_pos.lat * 1e7;
			log_msg.body.log_GPOS.lon = buf.global_pos.lon * 1e7;
			log_msg.body.log_GPOS.alt = buf.global_pos.alt;
			log_msg.body.log_GPOS.vel_n = buf.global_pos.vel_n;
			log_msg.body.log_GPOS.vel_e = buf.global_pos.vel_e;
			log_msg.body.log_GPOS.vel_d = buf.global_pos.vel_d;
			log_msg.body.log_GPOS.eph = buf.global_pos.eph;
			log_msg.body.log_GPOS.epv = buf.global_pos.epv;
			LOGBUFFER_WRITE_AND_COUNT(GPOS);
		}

		/* --- GLOBAL POSITION SETPOINT --- */
		if (copy_if_updated(ORB_ID(position_setpoint_triplet), subs.triplet_sub, &buf.triplet)) {

			if (buf.triplet.current.valid) {
				log_msg.msg_type = LOG_GPSP_MSG;
				log_msg.body.log_GPSP.nav_state = buf.triplet.nav_state;
				log_msg.body.log_GPSP.lat = (int32_t)(buf.triplet.current.lat * 1e7d);
				log_msg.body.log_GPSP.lon = (int32_t)(buf.triplet.current.lon * 1e7d);
				log_msg.body.log_GPSP.alt = buf.triplet.current.alt;
				log_msg.body.log_GPSP.yaw = buf.triplet.current.yaw;
				log_msg.body.log_GPSP.type = buf.triplet.current.type;
				log_msg.body.log_GPSP.loiter_radius = buf.triplet.current.loiter_radius;
				log_msg.body.log_GPSP.loiter_direction = buf.triplet.current.loiter_direction;
				log_msg.body.log_GPSP.pitch_min = buf.triplet.current.pitch_min;
				LOGBUFFER_WRITE_AND_COUNT(GPSP);
			}
		}

		/* --- VICON POSITION --- */
		if (copy_if_updated(ORB_ID(vehicle_vicon_position), subs.vicon_pos_sub, &buf.vicon_pos)) {
			log_msg.msg_type = LOG_VICN_MSG;
			log_msg.body.log_VICN.x = buf.vicon_pos.x;
			log_msg.body.log_VICN.y = buf.vicon_pos.y;
			log_msg.body.log_VICN.z = buf.vicon_pos.z;
			log_msg.body.log_VICN.pitch = buf.vicon_pos.pitch;
			log_msg.body.log_VICN.roll = buf.vicon_pos.roll;
			log_msg.body.log_VICN.yaw = buf.vicon_pos.yaw;
			LOGBUFFER_WRITE_AND_COUNT(VICN);
		}

		/* --- FLOW --- */
		if (copy_if_updated(ORB_ID(optical_flow), subs.flow_sub, &buf.flow)) {
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
		if (copy_if_updated(ORB_ID(rc_channels), subs.rc_sub, &buf.rc)) {
			log_msg.msg_type = LOG_RC_MSG;
			/* Copy only the first 8 channels of 14 */
			memcpy(log_msg.body.log_RC.channel, buf.rc.channels, sizeof(log_msg.body.log_RC.channel));
			log_msg.body.log_RC.channel_count = buf.rc.channel_count;
			log_msg.body.log_RC.signal_lost = buf.rc.signal_lost;
			LOGBUFFER_WRITE_AND_COUNT(RC);
		}

		/* --- AIRSPEED --- */
		if (copy_if_updated(ORB_ID(airspeed), subs.airspeed_sub, &buf.airspeed)) {
			log_msg.msg_type = LOG_AIRS_MSG;
			log_msg.body.log_AIRS.indicated_airspeed = buf.airspeed.indicated_airspeed_m_s;
			log_msg.body.log_AIRS.true_airspeed = buf.airspeed.true_airspeed_m_s;
			log_msg.body.log_AIRS.air_temperature_celsius = buf.airspeed.air_temperature_celsius;
			LOGBUFFER_WRITE_AND_COUNT(AIRS);
		}

		/* --- ESCs --- */
		if (copy_if_updated(ORB_ID(esc_status), subs.esc_sub, &buf.esc)) {
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
		if (copy_if_updated(ORB_ID(vehicle_global_velocity_setpoint), subs.global_vel_sp_sub, &buf.global_vel_sp)) {
			log_msg.msg_type = LOG_GVSP_MSG;
			log_msg.body.log_GVSP.vx = buf.global_vel_sp.vx;
			log_msg.body.log_GVSP.vy = buf.global_vel_sp.vy;
			log_msg.body.log_GVSP.vz = buf.global_vel_sp.vz;
			LOGBUFFER_WRITE_AND_COUNT(GVSP);
		}

		/* --- BATTERY --- */
		if (copy_if_updated(ORB_ID(battery_status), subs.battery_sub, &buf.battery)) {
			log_msg.msg_type = LOG_BATT_MSG;
			log_msg.body.log_BATT.voltage = buf.battery.voltage_v;
			log_msg.body.log_BATT.voltage_filtered = buf.battery.voltage_filtered_v;
			log_msg.body.log_BATT.current = buf.battery.current_a;
			log_msg.body.log_BATT.discharged = buf.battery.discharged_mah;
			LOGBUFFER_WRITE_AND_COUNT(BATT);
		}

		/* --- SYSTEM POWER RAILS --- */
		if (copy_if_updated(ORB_ID(system_power), subs.system_power_sub, &buf.system_power)) {
			log_msg.msg_type = LOG_PWR_MSG;
			log_msg.body.log_PWR.peripherals_5v = buf.system_power.voltage5V_v;
			log_msg.body.log_PWR.usb_ok = buf.system_power.usb_connected;
			log_msg.body.log_PWR.brick_ok = buf.system_power.brick_valid;
			log_msg.body.log_PWR.servo_ok = buf.system_power.servo_valid;
			log_msg.body.log_PWR.low_power_rail_overcurrent = buf.system_power.periph_5V_OC;
			log_msg.body.log_PWR.high_power_rail_overcurrent = buf.system_power.hipower_5V_OC;

			/* copy servo rail status topic here too */
			orb_copy(ORB_ID(servorail_status), subs.servorail_status_sub, &buf.servorail_status);
			log_msg.body.log_PWR.servo_rail_5v = buf.servorail_status.voltage_v;
			log_msg.body.log_PWR.servo_rssi = buf.servorail_status.rssi_v;

			LOGBUFFER_WRITE_AND_COUNT(PWR);
		}

		/* --- TELEMETRY --- */
		for (int i = 0; i < TELEMETRY_STATUS_ORB_ID_NUM; i++) {
			if (copy_if_updated(telemetry_status_orb_id[i], subs.telemetry_subs[i], &buf.telemetry)) {
				log_msg.msg_type = LOG_TEL0_MSG + i;
				log_msg.body.log_TEL.rssi = buf.telemetry.rssi;
				log_msg.body.log_TEL.remote_rssi = buf.telemetry.remote_rssi;
				log_msg.body.log_TEL.noise = buf.telemetry.noise;
				log_msg.body.log_TEL.remote_noise = buf.telemetry.remote_noise;
				log_msg.body.log_TEL.rxerrors = buf.telemetry.rxerrors;
				log_msg.body.log_TEL.fixed = buf.telemetry.fixed;
				log_msg.body.log_TEL.txbuf = buf.telemetry.txbuf;
				log_msg.body.log_TEL.heartbeat_time = buf.telemetry.heartbeat_time;
				LOGBUFFER_WRITE_AND_COUNT(TEL);
			}
		}

		/* --- BOTTOM DISTANCE --- */
		if (copy_if_updated(ORB_ID(sensor_range_finder), subs.range_finder_sub, &buf.range_finder)) {
			log_msg.msg_type = LOG_DIST_MSG;
			log_msg.body.log_DIST.bottom = buf.range_finder.distance;
			log_msg.body.log_DIST.bottom_rate = 0.0f;
			log_msg.body.log_DIST.flags = (buf.range_finder.valid ? 1 : 0);
			LOGBUFFER_WRITE_AND_COUNT(DIST);
		}

		/* --- ESTIMATOR STATUS --- */
		if (copy_if_updated(ORB_ID(estimator_status), subs.estimator_status_sub, &buf.estimator_status)) {
			log_msg.msg_type = LOG_EST0_MSG;
			unsigned maxcopy0 = (sizeof(buf.estimator_status.states) < sizeof(log_msg.body.log_EST0.s)) ? sizeof(buf.estimator_status.states) : sizeof(log_msg.body.log_EST0.s);
			memset(&(log_msg.body.log_EST0.s), 0, sizeof(log_msg.body.log_EST0.s));
			memcpy(&(log_msg.body.log_EST0.s), buf.estimator_status.states, maxcopy0);
			log_msg.body.log_EST0.n_states = buf.estimator_status.n_states;
			log_msg.body.log_EST0.nan_flags = buf.estimator_status.nan_flags;
			log_msg.body.log_EST0.health_flags = buf.estimator_status.health_flags;
			log_msg.body.log_EST0.timeout_flags = buf.estimator_status.timeout_flags;
			LOGBUFFER_WRITE_AND_COUNT(EST0);

			log_msg.msg_type = LOG_EST1_MSG;
			unsigned maxcopy1 = ((sizeof(buf.estimator_status.states) - maxcopy0) < sizeof(log_msg.body.log_EST1.s)) ? (sizeof(buf.estimator_status.states) - maxcopy0) : sizeof(log_msg.body.log_EST1.s);
			memset(&(log_msg.body.log_EST1.s), 0, sizeof(log_msg.body.log_EST1.s));
			memcpy(&(log_msg.body.log_EST1.s), buf.estimator_status.states + maxcopy0, maxcopy1);
			LOGBUFFER_WRITE_AND_COUNT(EST1);
		}

		/* --- TECS STATUS --- */
		if (copy_if_updated(ORB_ID(tecs_status), subs.tecs_status_sub, &buf.tecs_status)) {
			log_msg.msg_type = LOG_TECS_MSG;
			log_msg.body.log_TECS.altitudeSp = buf.tecs_status.altitudeSp;
			log_msg.body.log_TECS.altitudeFiltered = buf.tecs_status.altitude_filtered;
			log_msg.body.log_TECS.flightPathAngleSp = buf.tecs_status.flightPathAngleSp;
			log_msg.body.log_TECS.flightPathAngle = buf.tecs_status.flightPathAngle;
			log_msg.body.log_TECS.flightPathAngleFiltered = buf.tecs_status.flightPathAngleFiltered;
			log_msg.body.log_TECS.airspeedSp = buf.tecs_status.airspeedSp;
			log_msg.body.log_TECS.airspeedFiltered = buf.tecs_status.airspeed_filtered;
			log_msg.body.log_TECS.airspeedDerivativeSp = buf.tecs_status.airspeedDerivativeSp;
			log_msg.body.log_TECS.airspeedDerivative = buf.tecs_status.airspeedDerivative;
			log_msg.body.log_TECS.totalEnergyRateSp = buf.tecs_status.totalEnergyRateSp;
			log_msg.body.log_TECS.totalEnergyRate = buf.tecs_status.totalEnergyRate;
			log_msg.body.log_TECS.energyDistributionRateSp = buf.tecs_status.energyDistributionRateSp;
			log_msg.body.log_TECS.energyDistributionRate = buf.tecs_status.energyDistributionRate;
			log_msg.body.log_TECS.mode = (uint8_t)buf.tecs_status.mode;
			LOGBUFFER_WRITE_AND_COUNT(TECS);
		}

		/* --- WIND ESTIMATE --- */
		if (copy_if_updated(ORB_ID(wind_estimate), subs.wind_sub, &buf.wind_estimate)) {
			log_msg.msg_type = LOG_WIND_MSG;
			log_msg.body.log_WIND.x = buf.wind_estimate.windspeed_north;
			log_msg.body.log_WIND.y = buf.wind_estimate.windspeed_east;
			log_msg.body.log_WIND.cov_x = buf.wind_estimate.covariance_north;
			log_msg.body.log_WIND.cov_y = buf.wind_estimate.covariance_east;
			LOGBUFFER_WRITE_AND_COUNT(WIND);
		}

		/* signal the other thread new data, but not yet unlock */
		if (logbuffer_count(&lb) > MIN_BYTES_TO_WRITE) {
			/* only request write if several packets can be written at once */
			pthread_cond_signal(&logbuffer_cond);
		}

		/* unlock, now the writer thread may run */
		pthread_mutex_unlock(&logbuffer_mutex);
	}

	if (logging_enabled) {
		sdlog2_stop_log();
	}

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
	warnx("extended logging: %s", (_extended_logging) ? "ON" : "OFF");
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
