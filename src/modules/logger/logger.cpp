/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>
#include "logger.h"
#include "messages.h"
#include "watchdog.h"

#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/uORBTopics.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <drivers/drv_hrt.h>
#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_sem.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <systemlib/mavlink_log.h>
#include <replay/definitions.hpp>
#include <version/version.h>

#if defined(__PX4_DARWIN)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

typedef decltype(statfs::f_bavail) px4_statfs_buf_f_bavail_t;

#define GPS_EPOCH_SECS ((time_t)1234567890ULL)

//#define DBGPRINT //write status output every few seconds

#if defined(DBGPRINT)
// needed for mallinfo
#if defined(__PX4_POSIX) && !defined(__PX4_DARWIN)
#include <malloc.h>
#endif /* __PX4_POSIX */

// struct mallinfo not available on OSX?
#if defined(__PX4_DARWIN)
#undef DBGPRINT
#endif /* defined(__PX4_DARWIN) */
#endif /* defined(DBGPRINT) */

using namespace px4::logger;


struct timer_callback_data_s {
	px4_sem_t semaphore;

	watchdog_data_t watchdog_data;
	volatile bool watchdog_triggered = false;
};

/* This is used to schedule work for the logger (periodic scan for updated topics) */
static void timer_callback(void *arg)
{
	/* Note: we are in IRQ context here (on NuttX) */

	timer_callback_data_s *data = (timer_callback_data_s *)arg;

	if (watchdog_update(data->watchdog_data)) {
		data->watchdog_triggered = true;
	}

	/* check the value of the semaphore: if the logger cannot keep up with running it's main loop as fast
	 * as the timer_callback here increases the semaphore count, the counter would increase unbounded,
	 * leading to an overflow at some point. This case we want to avoid here, so we check the current
	 * value against a (somewhat arbitrary) threshold, and avoid calling sem_post() if it's exceeded.
	 * (it's not a problem if the threshold is a bit too large, it just means the logger will do
	 * multiple iterations at once, the next time it's scheduled). */
	int semaphore_value;

	if (px4_sem_getvalue(&data->semaphore, &semaphore_value) == 0 && semaphore_value > 1) {
		return;
	}

	px4_sem_post(&data->semaphore);

}


int logger_main(int argc, char *argv[])
{
	// logger currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Logger only works on little endian!\n");
		return 1;
	}

	return Logger::main(argc, argv);
}

namespace px4
{
namespace logger
{

int Logger::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("logger not running");
		return 1;
	}

	if (!strcmp(argv[0], "on")) {
		get_instance()->set_arm_override(true);
		return 0;
	}

	if (!strcmp(argv[0], "off")) {
		get_instance()->set_arm_override(false);
		return 0;
	}

	return print_usage("unknown command");
}

int Logger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
System logger which logs a configurable set of uORB topics and system printf messages
(`PX4_WARN` and `PX4_ERR`) to ULog files. These can be used for system and flight performance evaluation,
tuning, replay and crash analysis.

It supports 2 backends:
- Files: write ULog files to the file system (SD card)
- MAVLink: stream ULog data via MAVLink to a client (the client must support this)

Both backends can be enabled and used at the same time.

### Implementation
The implementation uses two threads:
- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
  data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size. It should be large to avoid dropouts.

### Examples
Typical usage to start logging immediately:
$ logger start -e -t

Or if already running:
$ logger on
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("logger", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('m', "all", "file|mavlink|all", "Backend mode", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Enable logging right after start until disarm (otherwise only when armed)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Log until shutdown (implies -e)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Use date/time for naming log directories and files", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 280, 0, 8000, "Log rate in Hz, 0 means unlimited rate", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 12, 4, 10000, "Log buffer size in KiB", true);
	PRINT_MODULE_USAGE_PARAM_INT('q', 14, 1, 100, "uORB queue size for mavlink mode", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "<topic_name>",
					 "Poll on a topic instead of running with fixed rate (Log rate and topic intervals are ignored if this is set)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("on", "start logging now, override arming (logger must be running)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("off", "stop logging now, override arming (logger must be running)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Logger::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("logger",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_LOG_CAPTURE,
				      3600,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int Logger::print_status()
{
	PX4_INFO("Running in mode: %s", configured_backend_mode());

	bool is_logging = false;
	if (_writer.is_started(LogWriter::BackendFile)) {
		PX4_INFO("File Logging Running");
		print_statistics();
		is_logging = true;
	}

	if (_writer.is_started(LogWriter::BackendMavlink)) {
		PX4_INFO("Mavlink Logging Running");
		is_logging = true;
	}

	if (!is_logging) {
		PX4_INFO("Not logging");
	}

	return 0;
}

void Logger::print_statistics()
{
	if (!_writer.is_started(LogWriter::BackendFile)) { //currently only statistics for file logging
		return;
	}

	/* this is only for the file backend */
	float kibibytes = _writer.get_total_written_file() / 1024.0f;
	float mebibytes = kibibytes / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - _start_time_file)) / 1000000.0f;

	PX4_INFO("Log file: %s/%s", _log_dir, _log_file_name);
	PX4_INFO("Wrote %4.2f MiB (avg %5.2f KiB/s)", (double)mebibytes, (double)(kibibytes / seconds));
	PX4_INFO("Since last status: dropouts: %zu (max len: %.3f s), max used buffer: %zu / %zu B",
		 _write_dropouts, (double)_max_dropout_duration, _high_water, _writer.get_buffer_size_file());
	_high_water = 0;
	_write_dropouts = 0;
	_max_dropout_duration = 0.f;
}

Logger *Logger::instantiate(int argc, char *argv[])
{
	uint32_t log_interval = 3500;
	int log_buffer_size = 12 * 1024;
	bool log_on_start = false;
	bool log_until_shutdown = false;
	bool error_flag = false;
	bool log_name_timestamp = false;
	unsigned int queue_size = 14; //TODO: we might be able to reduce this if mavlink polled on the topic and/or
	// topic sizes get reduced
	LogWriter::Backend backend = LogWriter::BackendAll;
	const char *poll_topic = nullptr;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "r:b:etfm:q:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, nullptr, 10);

				if (r <= 0) {
					r = 1e6;
				}

				log_interval = 1e6 / r;
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, nullptr, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case 't':
			log_name_timestamp = true;
			break;

		case 'f':
			log_on_start = true;
			log_until_shutdown = true;
			break;

		case 'm':
			if (!strcmp(myoptarg, "file")) {
				backend = LogWriter::BackendFile;

			} else if (!strcmp(myoptarg, "mavlink")) {
				backend = LogWriter::BackendMavlink;

			} else if (!strcmp(myoptarg, "all")) {
				backend = LogWriter::BackendAll;

			} else {
				PX4_ERR("unknown mode: %s", myoptarg);
				error_flag = true;
			}

			break;

		case 'p':
			poll_topic = myoptarg;
			break;

		case 'q':
			queue_size = strtoul(myoptarg, nullptr, 10);

			if (queue_size == 0) {
				queue_size = 1;
			}

			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Logger *logger = new Logger(backend, log_buffer_size, log_interval, poll_topic, log_on_start,
				    log_until_shutdown, log_name_timestamp, queue_size);

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
	struct mallinfo alloc_info = mallinfo();
	PX4_INFO("largest free chunk: %d bytes", alloc_info.mxordblk);
	PX4_INFO("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

	if (logger == nullptr) {
		PX4_ERR("alloc failed");

	} else {
#ifndef __PX4_NUTTX
		//check for replay mode
		const char *logfile = getenv(px4::replay::ENV_FILENAME);

		if (logfile) {
			logger->setReplayFile(logfile);
		}

#endif /* __PX4_NUTTX */

	}

	return logger;
}


Logger::Logger(LogWriter::Backend backend, size_t buffer_size, uint32_t log_interval, const char *poll_topic_name,
	       bool log_on_start, bool log_until_shutdown, bool log_name_timestamp, unsigned int queue_size) :
	_arm_override(false),
	_log_on_start(log_on_start),
	_log_until_shutdown(log_until_shutdown),
	_log_name_timestamp(log_name_timestamp),
	_writer(backend, buffer_size, queue_size),
	_log_interval(log_interval)
{
	_log_utc_offset = param_find("SDLOG_UTC_OFFSET");
	_log_dirs_max = param_find("SDLOG_DIRS_MAX");
	_sdlog_profile_handle = param_find("SDLOG_PROFILE");

	if (poll_topic_name) {
		const orb_metadata *const*topics = orb_get_topics();

		for (size_t i = 0; i < orb_topics_count(); i++) {
			if (strcmp(poll_topic_name, topics[i]->o_name) == 0) {
				_polling_topic_meta = topics[i];
				break;
			}
		}

		if (!_polling_topic_meta) {
			PX4_ERR("Failed to find topic %s", poll_topic_name);
		}
	}
}

Logger::~Logger()
{
	if (_replay_file_name) {
		free(_replay_file_name);
	}

	if (_msg_buffer) {
		delete[](_msg_buffer);
	}
}

bool Logger::request_stop_static()
{
	if (is_running()) {
		get_instance()->request_stop();
		return false;
	}

	return true;
}

LoggerSubscription* Logger::add_topic(const orb_metadata *topic)
{
	LoggerSubscription *subscription = nullptr;
	size_t fields_len = strlen(topic->o_fields) + strlen(topic->o_name) + 1; //1 for ':'

	if (fields_len > sizeof(ulog_message_format_s::format)) {
		PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
			 sizeof(ulog_message_format_s::format));

		return nullptr;
	}

	int fd = -1;
	// Only subscribe to the topic now if it's published. If published later on, we'll dynamically
	// add the subscription then
	if (orb_exists(topic, 0) == 0) {
		fd = orb_subscribe(topic);

		if (fd < 0) {
			PX4_WARN("logger: %s subscribe failed (%i)", topic->o_name, errno);
			return nullptr;
		}
	} else {
		PX4_DEBUG("Topic %s does not exist. Not subscribing (yet)", topic->o_name);
	}

	if (_subscriptions.push_back(LoggerSubscription(fd, topic))) {
		subscription = &_subscriptions[_subscriptions.size() - 1];
	} else {
		PX4_WARN("logger: failed to add topic. Too many subscriptions");
		if (fd >= 0) {
			orb_unsubscribe(fd);
		}
	}

	return subscription;
}

bool Logger::add_topic(const char *name, unsigned interval)
{
	const orb_metadata *const*topics = orb_get_topics();
	LoggerSubscription *subscription = nullptr;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			bool already_added = false;

			// check if already added: if so, only update the interval
			for (size_t j = 0; j < _subscriptions.size(); ++j) {
				if (_subscriptions[j].metadata == topics[i]) {
					PX4_DEBUG("logging topic %s, interval: %i, already added, only setting interval",
						  topics[i]->o_name, interval);
					subscription = &_subscriptions[j];
					already_added = true;
					break;
				}
			}

			if (!already_added) {
				subscription = add_topic(topics[i]);
				PX4_DEBUG("logging topic: %s, interval: %i", topics[i]->o_name, interval);
				break;
			}
		}
	}

	// if we poll on a topic, we don't use the interval and let the polled topic define the maximum interval
	if (_polling_topic_meta) {
		interval = 0;
	}

	if (subscription) {
		if (subscription->fd[0] >= 0) {
			orb_set_interval(subscription->fd[0], interval);
		} else {
			// store the interval: use a value < 0 to know it's not a valid fd
			subscription->fd[0] = -interval - 1;
		}
	}

	return subscription;
}

bool Logger::copy_if_updated_multi(LoggerSubscription &sub, int multi_instance, void *buffer, bool try_to_subscribe)
{
	bool updated = false;
	int &handle = sub.fd[multi_instance];

	if (handle < 0 && try_to_subscribe) {

		if (try_to_subscribe_topic(sub, multi_instance)) {

			write_add_logged_msg(sub, multi_instance);

			/* copy first data */
			if (orb_copy(sub.metadata, handle, buffer) == PX4_OK) {
				updated = true;
			}
		}

	} else if (handle >= 0) {
		orb_check(handle, &updated);

		if (updated) {
			orb_copy(sub.metadata, handle, buffer);
		}
	}

	return updated;
}

bool Logger::try_to_subscribe_topic(LoggerSubscription &sub, int multi_instance)
{
	bool ret = false;
	if (OK == orb_exists(sub.metadata, multi_instance)) {

		unsigned int interval;

		if (multi_instance == 0) {
			// the first instance and no subscription yet: this means we stored the negative interval as fd
			interval = (unsigned int) (-(sub.fd[0] + 1));
		} else {
			// set to the same interval as the first instance
			if (orb_get_interval(sub.fd[0], &interval) != 0) {
				interval = 0;
			}
		}

		int &handle = sub.fd[multi_instance];
		handle = orb_subscribe_multi(sub.metadata, multi_instance);

		if (handle >= 0) {
			PX4_DEBUG("subscribed to instance %d of topic %s", multi_instance, sub.metadata->o_name);
			if (interval > 0) {
				orb_set_interval(handle, interval);
			}
			ret = true;
		} else {
			PX4_ERR("orb_subscribe_multi %s failed (%i)", sub.metadata->o_name, errno);
		}
	}
	return ret;
}

void Logger::add_default_topics()
{
	// Note: try to avoid setting the interval where possible, as it increases RAM usage
	add_topic("actuator_controls_0", 100);
	add_topic("actuator_controls_1", 100);
	add_topic("actuator_outputs", 100);
	add_topic("airspeed", 200);
	add_topic("att_pos_mocap", 50);
	add_topic("battery_status", 500);
	add_topic("camera_capture");
	add_topic("camera_trigger");
	add_topic("cpuload");
	add_topic("distance_sensor", 100);
	add_topic("ekf2_innovations", 200);
	add_topic("esc_status", 250);
	add_topic("estimator_status", 200);
	add_topic("home_position");
	add_topic("input_rc", 200);
	add_topic("landing_target_pose");
	add_topic("manual_control_setpoint", 200);
	add_topic("mission");
	add_topic("mission_result");
	add_topic("optical_flow", 50);
	add_topic("position_setpoint_triplet", 200);
	add_topic("rate_ctrl_status", 30);
	add_topic("sensor_combined", 100);
	add_topic("sensor_preflight", 200);
	add_topic("system_power", 500);
	add_topic("tecs_status", 200);
	add_topic("vehicle_attitude", 30);
	add_topic("vehicle_attitude_setpoint", 100);
	add_topic("vehicle_command");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_rates_setpoint", 30);
	add_topic("vehicle_status", 200);
	add_topic("vehicle_status_flags");
	add_topic("vehicle_trajectory_waypoint");
	add_topic("vehicle_trajectory_waypoint_desired");
	add_topic("vehicle_vision_attitude");
	add_topic("vehicle_vision_position");
	add_topic("vtol_vehicle_status", 200);
	add_topic("wind_estimate", 200);

#ifdef CONFIG_ARCH_BOARD_SITL
	add_topic("actuator_armed");
	add_topic("actuator_controls_virtual_fw");
	add_topic("actuator_controls_virtual_mc");
	add_topic("commander_state");
	add_topic("fw_pos_ctrl_status");
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_topic("multirotor_motor_limits");
	add_topic("offboard_control_mode");
	add_topic("time_offset");
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 100);
	add_topic("vehicle_roi");
#endif
}

void Logger::add_high_rate_topics()
{
	// maximum rate to analyze fast maneuvers (e.g. for racing)
	add_topic("actuator_controls_0");
	add_topic("actuator_outputs");
	add_topic("manual_control_setpoint");
	add_topic("rate_ctrl_status");
	add_topic("sensor_combined");
	add_topic("vehicle_attitude");
	add_topic("vehicle_attitude_setpoint");
	add_topic("vehicle_rates_setpoint");
}

void Logger::add_debug_topics()
{
	add_topic("debug_key_value");
	add_topic("debug_value");
	add_topic("debug_vect");
}

void Logger::add_estimator_replay_topics()
{
	// for estimator replay (need to be at full rate)
	add_topic("ekf2_timestamps");
	add_topic("ekf_gps_position");

	// current EKF2 subscriptions
	add_topic("airspeed");
	add_topic("distance_sensor");
	add_topic("optical_flow");
	add_topic("sensor_combined");
	add_topic("sensor_selection");
	add_topic("vehicle_air_data");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_magnetometer");
	add_topic("vehicle_status");
	add_topic("vehicle_vision_attitude");
	add_topic("vehicle_vision_position");
}

void Logger::add_thermal_calibration_topics()
{
	add_topic("sensor_accel", 100);
	add_topic("sensor_baro", 100);
	add_topic("sensor_gyro", 100);
}

void Logger::add_sensor_comparison_topics()
{
	add_topic("sensor_accel", 100);
	add_topic("sensor_baro", 100);
	add_topic("sensor_gyro", 100);
	add_topic("sensor_mag", 100);
}

void Logger::add_system_identification_topics()
{
	// for system id need to log imu and controls at full rate
	add_topic("actuator_controls_0");
	add_topic("actuator_controls_1");
	add_topic("sensor_combined");
}

int Logger::add_topics_from_file(const char *fname)
{
	FILE		*fp;
	char		line[80];
	char		topic_name[80];
	unsigned	interval;
	int			ntopics = 0;

	/* open the topic list file */
	fp = fopen(fname, "r");

	if (fp == nullptr) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	for (;;) {

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == nullptr) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// read line with format: <topic_name>[, <interval>]
		interval = 0;
		int nfields = sscanf(line, "%s %u", topic_name, &interval);

		if (nfields > 0) {
			int name_len = strlen(topic_name);

			if (name_len > 0 && topic_name[name_len - 1] == ',') {
				topic_name[name_len - 1] = '\0';
			}

			/* add topic with specified interval */
			if (add_topic(topic_name, interval)) {
				ntopics++;

			} else {
				PX4_ERR("Failed to add topic %s", topic_name);
			}
		}
	}

	fclose(fp);
	return ntopics;
}

const char *Logger::configured_backend_mode() const
{
	switch (_writer.backend()) {
	case LogWriter::BackendFile: return "file";

	case LogWriter::BackendMavlink: return "mavlink";

	case LogWriter::BackendAll: return "all";

	default: return "several";
	}
}

void Logger::run()
{
#ifdef DBGPRINT
	struct mallinfo alloc_info = {};
#endif /* DBGPRINT */

	PX4_INFO("logger started (mode=%s)", configured_backend_mode());

	if (_writer.backend() & LogWriter::BackendFile) {
		int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == 0) {
			PX4_INFO("log root dir created: %s", LOG_ROOT);

		} else if (errno != EEXIST) {
			PX4_ERR("failed creating log root dir: %s", LOG_ROOT);

			if ((_writer.backend() & ~LogWriter::BackendFile) == 0) {
				return;
			}
		}

		if (check_free_space() == 1) {
			return;
		}
	}

	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	uORB::Subscription<parameter_update_s> parameter_update_sub(ORB_ID(parameter_update));
	int log_message_sub = orb_subscribe(ORB_ID(log_message));
	orb_set_interval(log_message_sub, 20);

	int ntopics = add_topics_from_file(PX4_ROOTFSDIR "/fs/microsd/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {

		// get the logging profile
		SDLogProfileMask sdlog_profile = SDLogProfileMask::DEFAULT;

		if (_sdlog_profile_handle != PARAM_INVALID) {
			param_get(_sdlog_profile_handle, (int32_t*)&sdlog_profile);
		}
		if ((int32_t)sdlog_profile == 0) {
			PX4_WARN("No logging profile selected. Using default set");
			sdlog_profile = SDLogProfileMask::DEFAULT;
		}

		// load appropriate topics for profile
		// the order matters: if several profiles add the same topic, the logging rate of the last one will be used
		if (sdlog_profile & SDLogProfileMask::DEFAULT) {
			add_default_topics();
		}

		if (sdlog_profile & SDLogProfileMask::ESTIMATOR_REPLAY) {
			add_estimator_replay_topics();
		}

		if (sdlog_profile & SDLogProfileMask::THERMAL_CALIBRATION) {
			add_thermal_calibration_topics();
		}

		if (sdlog_profile & SDLogProfileMask::SYSTEM_IDENTIFICATION) {
			add_system_identification_topics();
		}

		if (sdlog_profile & SDLogProfileMask::HIGH_RATE) {
			add_high_rate_topics();
		}

		if (sdlog_profile & SDLogProfileMask::DEBUG_TOPICS) {
			add_debug_topics();
		}

		if (sdlog_profile & SDLogProfileMask::SENSOR_COMPARISON) {
			add_sensor_comparison_topics();
		}

	}

	int vehicle_command_sub = -1;
	orb_advert_t vehicle_command_ack_pub = nullptr;

	if (_writer.backend() & LogWriter::BackendMavlink) {
		vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	}

	//all topics added. Get required message buffer size
	int max_msg_size = 0;
	int ret = 0;

	for (const auto &subscription : _subscriptions) {
		//use o_size, because that's what orb_copy will use
		if (subscription.metadata->o_size > max_msg_size) {
			max_msg_size = subscription.metadata->o_size;
		}
	}

	max_msg_size += sizeof(ulog_message_data_header_s);

	if (sizeof(ulog_message_logging_s) > (size_t)max_msg_size) {
		max_msg_size = sizeof(ulog_message_logging_s);
	}

	if (_polling_topic_meta && _polling_topic_meta->o_size > max_msg_size) {
		max_msg_size = _polling_topic_meta->o_size;
	}

	if (max_msg_size > _msg_buffer_len) {
		if (_msg_buffer) {
			delete[](_msg_buffer);
		}

		_msg_buffer_len = max_msg_size;
		_msg_buffer = new uint8_t[_msg_buffer_len];

		if (!_msg_buffer) {
			PX4_ERR("failed to alloc message buffer");
			return;
		}
	}


	if (!_writer.init()) {
		PX4_ERR("writer init failed");
		return;
	}

#ifdef DBGPRINT
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;
#endif /* DBGPRINT */

	px4_register_shutdown_hook(&Logger::request_stop_static);

	// we start logging immediately
	// the case where we wait with logging until vehicle is armed is handled below
	if (_log_on_start) {
		start_log_file();
	}

	/* init the update timer */
	struct hrt_call timer_call{};
	timer_callback_data_s timer_callback_data;
	px4_sem_init(&timer_callback_data.semaphore, 0, 0);
	/* timer_semaphore use case is a signal */
	px4_sem_setprotocol(&timer_callback_data.semaphore, SEM_PRIO_NONE);

	int polling_topic_sub = -1;

	if (_polling_topic_meta) {
		polling_topic_sub = orb_subscribe(_polling_topic_meta);

		if (polling_topic_sub < 0) {
			PX4_ERR("Failed to subscribe (%i)", errno);
		}

	} else {

		if (_writer.backend() & LogWriter::BackendFile) {

			const pid_t pid_self = getpid();
			const pthread_t writer_thread = _writer.thread_id_file();

			// sched_note_start is already called from pthread_create and task_create,
			// which means we can expect to find the tasks in system_load.tasks, as required in watchdog_initialize
			watchdog_initialize(pid_self, writer_thread, timer_callback_data.watchdog_data);
		}

		hrt_call_every(&timer_call, _log_interval, _log_interval, timer_callback, &timer_callback_data);
	}

	// check for new subscription data
	hrt_abstime next_subscribe_check = 0;
	int next_subscribe_topic_index = -1; // this is used to distribute the checks over time

	while (!should_exit()) {

		// Start/stop logging when system arm/disarm
		bool vehicle_status_updated;
		ret = orb_check(vehicle_status_sub, &vehicle_status_updated);

		if (ret == 0 && vehicle_status_updated) {
			vehicle_status_s vehicle_status;
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
			bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) || _arm_override;

			if (_was_armed != armed && !_log_until_shutdown) {
				_was_armed = armed;

				if (armed) {

					if (_should_stop_file_log) { // happens on quick arming after disarm
						_should_stop_file_log = false;
						stop_log_file();
					}

					start_log_file();

#ifdef DBGPRINT
					timer_start = hrt_absolute_time();
					total_bytes = 0;
#endif /* DBGPRINT */

				} else {
					// delayed stop: we measure the process loads and then stop
					initialize_load_output(PrintLoadReason::Postflight);
					_should_stop_file_log = true;
				}
			}
		}

		/* check for logging command from MAVLink */
		if (vehicle_command_sub != -1) {
			bool command_updated = false;
			ret = orb_check(vehicle_command_sub, &command_updated);

			if (ret == 0 && command_updated) {
				vehicle_command_s command;
				orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &command);

				if (command.command == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {
					if ((int)(command.param1 + 0.5f) != 0) {
						ack_vehicle_command(vehicle_command_ack_pub, &command,
								    vehicle_command_s::VEHICLE_CMD_RESULT_UNSUPPORTED);

					} else if (can_start_mavlink_log()) {
						ack_vehicle_command(vehicle_command_ack_pub, &command,
								    vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
						start_log_mavlink();

					} else {
						ack_vehicle_command(vehicle_command_ack_pub, &command,
								    vehicle_command_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
					}

				} else if (command.command == vehicle_command_s::VEHICLE_CMD_LOGGING_STOP) {
					stop_log_mavlink();
					ack_vehicle_command(vehicle_command_ack_pub, &command,
							    vehicle_command_s::VEHICLE_CMD_RESULT_ACCEPTED);
				}
			}
		}


		if (timer_callback_data.watchdog_triggered) {
			timer_callback_data.watchdog_triggered = false;
			initialize_load_output(PrintLoadReason::Watchdog);
		}


		const hrt_abstime loop_time = hrt_absolute_time();

		if (_writer.is_started()) {

			/* check if we need to output the process load */
			if (_next_load_print != 0 && loop_time >= _next_load_print) {
				_next_load_print = 0;

				if (_should_stop_file_log) {
					_should_stop_file_log = false;
					write_load_output();
					stop_log_file();
					continue; // skip to next loop iteration

				} else {
					write_load_output();
				}
			}

			bool data_written = false;

			/* Check if parameters have changed */
			if (!_should_stop_file_log) { // do not record param changes after disarming
				if (parameter_update_sub.update()) {
					write_changed_parameters();
				}
			}

			/* wait for lock on log buffer */
			_writer.lock();

			int sub_idx = 0;

			for (LoggerSubscription &sub : _subscriptions) {
				/* each message consists of a header followed by an orb data object
				 */
				size_t msg_size = sizeof(ulog_message_data_header_s) + sub.metadata->o_size_no_padding;

				/* if this topic has been updated, copy the new data into the message buffer
				 * and write a message to the log
				 */
				for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
					if (copy_if_updated_multi(sub, instance, _msg_buffer + sizeof(ulog_message_data_header_s),
								  sub_idx == next_subscribe_topic_index)) {

						uint16_t write_msg_size = static_cast<uint16_t>(msg_size - ULOG_MSG_HEADER_LEN);
						//write one byte after another (necessary because of alignment)
						_msg_buffer[0] = (uint8_t)write_msg_size;
						_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
						_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::DATA);
						uint16_t write_msg_id = sub.msg_ids[instance];
						_msg_buffer[3] = (uint8_t)write_msg_id;
						_msg_buffer[4] = (uint8_t)(write_msg_id >> 8);

						//PX4_INFO("topic: %s, size = %zu, out_size = %zu", sub.metadata->o_name, sub.metadata->o_size, msg_size);

						if (write_message(_msg_buffer, msg_size)) {

#ifdef DBGPRINT
							total_bytes += msg_size;
#endif /* DBGPRINT */

							data_written = true;

						} else {
							break;	// Write buffer overflow, skip this record
						}
					}
				}

				++sub_idx;
			}

			//check for new logging message(s)
			bool log_message_updated = false;
			ret = orb_check(log_message_sub, &log_message_updated);

			if (ret == 0 && log_message_updated) {
				log_message_s log_message;
				orb_copy(ORB_ID(log_message), log_message_sub, &log_message);
				const char *message = (const char *)log_message.text;
				int message_len = strlen(message);

				if (message_len > 0) {
					uint16_t write_msg_size = sizeof(ulog_message_logging_s) - sizeof(ulog_message_logging_s::message)
								  - ULOG_MSG_HEADER_LEN + message_len;
					_msg_buffer[0] = (uint8_t)write_msg_size;
					_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
					_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::LOGGING);
					_msg_buffer[3] = log_message.severity + '0';
					memcpy(_msg_buffer + 4, &log_message.timestamp, sizeof(ulog_message_logging_s::timestamp));
					strncpy((char *)(_msg_buffer + 12), message, sizeof(ulog_message_logging_s::message));

					write_message(_msg_buffer, write_msg_size + ULOG_MSG_HEADER_LEN);
				}
			}

			if (!_dropout_start && _writer.get_buffer_fill_count_file() > _high_water) {
				_high_water = _writer.get_buffer_fill_count_file();
			}

			/* release the log buffer */
			_writer.unlock();

			/* notify the writer thread if data is available */
			if (data_written) {
				_writer.notify();
			}

			/* subscription update */
			if (next_subscribe_topic_index != -1) {
				if (++next_subscribe_topic_index >= (int)_subscriptions.size()) {
					next_subscribe_topic_index = -1;
					next_subscribe_check = loop_time + TRY_SUBSCRIBE_INTERVAL;
				}

			} else if (loop_time > next_subscribe_check) {
				next_subscribe_topic_index = 0;
			}

#ifdef DBGPRINT
			double deltat = (double)(hrt_absolute_time() - timer_start)  * 1e-6;

			if (deltat > 4.0) {
				alloc_info = mallinfo();
				double throughput = total_bytes / deltat;
				PX4_INFO("%8.1f kB/s, %zu highWater,  %d dropouts, %5.3f sec max, free heap: %d",
					 throughput / 1.e3, _high_water, _write_dropouts, (double)_max_dropout_duration,
					 alloc_info.fordblks);

				_high_water = 0;
				_max_dropout_duration = 0.f;
				total_bytes = 0;
				timer_start = hrt_absolute_time();
			}

#endif /* DBGPRINT */

		} else { // not logging

			// try to subscribe to new topics, even if we don't log, so that:
			// - we avoid subscribing to many topics at once, when logging starts
			// - we'll get the data immediately once we start logging (no need to wait for the next subscribe timeout)
			if (next_subscribe_topic_index != -1) {
				for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
					if (_subscriptions[next_subscribe_topic_index].fd[instance] < 0) {
						try_to_subscribe_topic(_subscriptions[next_subscribe_topic_index], instance);
					}
				}
				if (++next_subscribe_topic_index >= (int)_subscriptions.size()) {
					next_subscribe_topic_index = -1;
					next_subscribe_check = loop_time + TRY_SUBSCRIBE_INTERVAL;
				}

			} else if (loop_time > next_subscribe_check) {
				next_subscribe_topic_index = 0;
			}
		}

		// wait for next loop iteration...
		if (polling_topic_sub >= 0) {
			px4_pollfd_struct_t fds[1];
			fds[0].fd = polling_topic_sub;
			fds[0].events = POLLIN;
			int pret = px4_poll(fds, 1, 1000);

			if (pret < 0) {
				PX4_ERR("poll failed (%i)", pret);

			} else if (pret != 0) {
				if (fds[0].revents & POLLIN) {
					// need to to an orb_copy so that poll will not return immediately
					orb_copy(_polling_topic_meta, polling_topic_sub, _msg_buffer);
				}
			}

		} else {
			/*
			 * We wait on the semaphore, which periodically gets updated by a high-resolution timer.
			 * The simpler alternative would be:
			 *   usleep(max(300, _log_interval - elapsed_time_since_loop_start));
			 * And on linux this is quite accurate as well, but under NuttX it is not accurate,
			 * because usleep() has only a granularity of CONFIG_MSEC_PER_TICK (=1ms).
			 */
			while (px4_sem_wait(&timer_callback_data.semaphore) != 0);
		}
	}

	stop_log_file();

	hrt_cancel(&timer_call);
	px4_sem_destroy(&timer_callback_data.semaphore);

	// stop the writer thread
	_writer.thread_stop();

	//unsubscribe
	for (LoggerSubscription &sub : _subscriptions) {
		for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
			if (sub.fd[instance] >= 0) {
				orb_unsubscribe(sub.fd[instance]);
				sub.fd[instance] = -1;
			}
		}
	}

	if (polling_topic_sub >= 0) {
		orb_unsubscribe(polling_topic_sub);
	}

	if (_mavlink_log_pub) {
		orb_unadvertise(_mavlink_log_pub);
		_mavlink_log_pub = nullptr;
	}

	if (vehicle_command_ack_pub) {
		orb_unadvertise(vehicle_command_ack_pub);
	}

	if (vehicle_command_sub != -1) {
		orb_unsubscribe(vehicle_command_sub);
	}

	px4_unregister_shutdown_hook(&Logger::request_stop_static);
}

bool Logger::write_message(void *ptr, size_t size)
{
	if (_writer.write_message(ptr, size, _dropout_start) != -1) {

		if (_dropout_start) {
			float dropout_duration = (float)(hrt_elapsed_time(&_dropout_start) / 1000) / 1.e3f;

			if (dropout_duration > _max_dropout_duration) {
				_max_dropout_duration = dropout_duration;
			}

			_dropout_start = 0;
		}

		return true;
	}

	if (!_dropout_start) {
		_dropout_start = hrt_absolute_time();
		++_write_dropouts;
		_high_water = 0;
	}

	return false;
}

int Logger::create_log_dir(tm *tt)
{
	/* create dir on sdcard if needed */
	int mkdir_ret;

	if (tt) {
		int n = snprintf(_log_dir, sizeof(_log_dir), "%s/", LOG_ROOT);

		if (n >= (int)sizeof(_log_dir)) {
			PX4_ERR("log path too long");
			return -1;
		}

		strftime(_log_dir + n, sizeof(_log_dir) - n, "%Y-%m-%d", tt);
		mkdir_ret = mkdir(_log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret != OK && errno != EEXIST) {
			PX4_ERR("failed creating new dir: %s", _log_dir);
			return -1;
		}

	} else {
		uint16_t dir_number = _sess_dir_index;

		/* look for the next dir that does not exist */
		while (!_has_log_dir) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			int n = snprintf(_log_dir, sizeof(_log_dir), "%s/sess%03u", LOG_ROOT, dir_number);

			if (n >= (int)sizeof(_log_dir)) {
				PX4_ERR("log path too long (%i)", n);
				return -1;
			}

			mkdir_ret = mkdir(_log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				PX4_DEBUG("log dir created: %s", _log_dir);
				_has_log_dir = true;

			} else if (errno != EEXIST) {
				PX4_ERR("failed creating new dir: %s (%i)", _log_dir, errno);
				return -1;
			}

			/* dir exists already */
			dir_number++;
		}

		_has_log_dir = true;
	}

	return 0;
}

bool Logger::file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int Logger::get_log_file_name(char *file_name, size_t file_name_size)
{
	tm tt = {};
	bool time_ok = false;

	if (_log_name_timestamp) {
		/* use RTC time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.ulg */
		time_ok = get_log_time(&tt, false);
	}

	const char *replay_suffix = "";

	if (_replay_file_name) {
		replay_suffix = "_replayed";
	}


	if (time_ok) {
		if (create_log_dir(&tt)) {
			return -1;
		}

		char log_file_name_time[16] = "";
		strftime(log_file_name_time, sizeof(log_file_name_time), "%H_%M_%S", &tt);
		snprintf(_log_file_name, sizeof(_log_file_name), "%s%s.ulg", log_file_name_time, replay_suffix);
		snprintf(file_name, file_name_size, "%s/%s", _log_dir, _log_file_name);

	} else {
		if (create_log_dir(nullptr)) {
			return -1;
		}

		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.ulg */
			snprintf(_log_file_name, sizeof(_log_file_name), "log%03u%s.ulg", file_number, replay_suffix);
			snprintf(file_name, file_name_size, "%s/%s", _log_dir, _log_file_name);

			if (!file_exist(file_name)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			return -1;
		}
	}


	return 0;
}

void Logger::setReplayFile(const char *file_name)
{
	if (_replay_file_name) {
		free(_replay_file_name);
	}

	_replay_file_name = strdup(file_name);
}

bool Logger::get_log_time(struct tm *tt, bool boot_time)
{
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	if (vehicle_gps_position_sub < 0) {
		return false;
	}

	/* Get the latest GPS publication */
	vehicle_gps_position_s gps_pos;
	time_t utc_time_sec;
	bool use_clock_time = true;

	if (orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps_pos) == 0) {
		utc_time_sec = gps_pos.time_utc_usec / 1e6;

		if (gps_pos.fix_type >= 2 && utc_time_sec >= GPS_EPOCH_SECS) {
			use_clock_time = false;
		}
	}

	orb_unsubscribe(vehicle_gps_position_sub);

	if (use_clock_time) {
		/* take clock time if there's no fix (yet) */
		struct timespec ts = {};
		px4_clock_gettime(CLOCK_REALTIME, &ts);
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);

		if (utc_time_sec < GPS_EPOCH_SECS) {
			return false;
		}
	}

	/* strip the time elapsed since boot */
	if (boot_time) {
		utc_time_sec -= hrt_absolute_time() / 1e6;
	}

	int32_t utc_offset = 0;

	if (_log_utc_offset != PARAM_INVALID) {
		param_get(_log_utc_offset, &utc_offset);
	}

	/* apply utc offset */
	utc_time_sec += utc_offset * 60;

	return gmtime_r(&utc_time_sec, tt) != nullptr;
}

void Logger::start_log_file()
{
	if (_writer.is_started(LogWriter::BackendFile) || (_writer.backend() & LogWriter::BackendFile) == 0) {
		return;
	}

	PX4_INFO("Start file log");

	char file_name[LOG_DIR_LEN] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		PX4_ERR("logger: failed to get log file name");
		return;
	}

	/* print logging path, important to find log file later */
	mavlink_log_info(&_mavlink_log_pub, "[logger] file: %s", file_name);

	_writer.start_log_file(file_name);
	_writer.select_write_backend(LogWriter::BackendFile);
	_writer.set_need_reliable_transfer(true);
	write_header();
	write_version();
	write_formats();
	write_parameters();
	write_perf_data(true);
	write_all_add_logged_msg();
	_writer.set_need_reliable_transfer(false);
	_writer.unselect_write_backend();
	_writer.notify();

	/* reset performance counters to get in-flight min and max values in post flight log */
	perf_reset_all();

	_start_time_file = hrt_absolute_time();

	initialize_load_output(PrintLoadReason::Preflight);
}

void Logger::stop_log_file()
{
	if (!_writer.is_started(LogWriter::BackendFile)) {
		return;
	}

	_writer.set_need_reliable_transfer(true);
	write_perf_data(false);
	_writer.set_need_reliable_transfer(false);
	_writer.stop_log_file();
}

void Logger::start_log_mavlink()
{
	if (!can_start_mavlink_log()) {
		return;
	}

	PX4_INFO("Start mavlink log");

	_writer.start_log_mavlink();
	_writer.select_write_backend(LogWriter::BackendMavlink);
	_writer.set_need_reliable_transfer(true);
	write_header();
	write_version();
	write_formats();
	write_parameters();
	write_perf_data(true);
	write_all_add_logged_msg();
	_writer.set_need_reliable_transfer(false);
	_writer.unselect_write_backend();
	_writer.notify();

	initialize_load_output(PrintLoadReason::Preflight);
}

void Logger::stop_log_mavlink()
{
	// don't write perf data since a client does not expect more data after a stop command
	PX4_INFO("Stop mavlink log");
	_writer.stop_log_mavlink();
}

struct perf_callback_data_t {
	Logger *logger;
	int counter;
	bool preflight;
	char *buffer;
};

void Logger::perf_iterate_callback(perf_counter_t handle, void *user)
{
	perf_callback_data_t *callback_data = (perf_callback_data_t *)user;
	const int buffer_length = 256;
	char buffer[buffer_length];
	const char *perf_name;

	perf_print_counter_buffer(buffer, buffer_length, handle);

	if (callback_data->preflight) {
		perf_name = "perf_counter_preflight";

	} else {
		perf_name = "perf_counter_postflight";
	}

	callback_data->logger->write_info_multiple(perf_name, buffer, callback_data->counter != 0);
	++callback_data->counter;
}

void Logger::write_perf_data(bool preflight)
{
	perf_callback_data_t callback_data = {};
	callback_data.logger = this;
	callback_data.counter = 0;
	callback_data.preflight = preflight;

	// write the perf counters
	perf_iterate_all(perf_iterate_callback, &callback_data);
}


void Logger::print_load_callback(void *user)
{
	perf_callback_data_t *callback_data = (perf_callback_data_t *)user;
	const char *perf_name;

	if (!callback_data->buffer) {
		return;
	}

	switch (callback_data->logger->_print_load_reason) {
		case PrintLoadReason::Preflight:
			perf_name = "perf_top_preflight";
			break;
		case PrintLoadReason::Postflight:
			perf_name = "perf_top_postflight";
			break;
		case PrintLoadReason::Watchdog:
			perf_name = "perf_top_watchdog";
			break;
		default:
			perf_name = "perf_top";
			break;
	}

	callback_data->logger->write_info_multiple(perf_name, callback_data->buffer, callback_data->counter != 0);
	++callback_data->counter;
}

void Logger::initialize_load_output(PrintLoadReason reason)
{
	perf_callback_data_t callback_data;
	callback_data.logger = this;
	callback_data.counter = 0;
	callback_data.buffer = nullptr;
	char buffer[140];
	hrt_abstime curr_time = hrt_absolute_time();
	init_print_load_s(curr_time, &_load);
	// this will not yet print anything
	print_load_buffer(curr_time, buffer, sizeof(buffer), print_load_callback, &callback_data, &_load);
	_next_load_print = curr_time + 1000000;
	_print_load_reason = reason;
}

void Logger::write_load_output()
{
	if (_print_load_reason == PrintLoadReason::Watchdog) {
		PX4_ERR("Writing watchdog data"); // this is just that we see it easily in the log
	}
	perf_callback_data_t callback_data = {};
	char buffer[140];
	callback_data.logger = this;
	callback_data.counter = 0;
	callback_data.buffer = buffer;
	hrt_abstime curr_time = hrt_absolute_time();
	_writer.set_need_reliable_transfer(true);
	// TODO: maybe we should restrict the output to a selected backend (eg. when file logging is running
	// and mavlink log is started, this will be added to the file as well)
	print_load_buffer(curr_time, buffer, sizeof(buffer), print_load_callback, &callback_data, &_load);
	_writer.set_need_reliable_transfer(false);
}

void Logger::write_formats()
{
	_writer.lock();
	ulog_message_format_s msg = {};
	const orb_metadata *const*topics = orb_get_topics();

	//write all known formats
	for (size_t i = 0; i < orb_topics_count(); i++) {
		int format_len = snprintf(msg.format, sizeof(msg.format), "%s:%s", topics[i]->o_name, topics[i]->o_fields);
		size_t msg_size = sizeof(msg) - sizeof(msg.format) + format_len;
		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_message(&msg, msg_size);
	}

	_writer.unlock();
}

void Logger::write_all_add_logged_msg()
{
	_writer.lock();

	for (LoggerSubscription &sub : _subscriptions) {
		for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; ++instance) {
			if (sub.fd[instance] >= 0) {
				write_add_logged_msg(sub, instance);
			}
		}
	}

	_writer.unlock();
}

void Logger::write_add_logged_msg(LoggerSubscription &subscription, int instance)
{
	ulog_message_add_logged_s msg;

	if (subscription.msg_ids[instance] == (uint16_t) - 1) {
		subscription.msg_ids[instance] = _next_topic_id++;
	}

	msg.msg_id = subscription.msg_ids[instance];
	msg.multi_id = instance;

	int message_name_len = strlen(subscription.metadata->o_name);

	memcpy(msg.message_name, subscription.metadata->o_name, message_name_len);

	size_t msg_size = sizeof(msg) - sizeof(msg.message_name) + message_name_len;
	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	bool prev_reliable = _writer.need_reliable_transfer();
	_writer.set_need_reliable_transfer(true);
	write_message(&msg, msg_size);
	_writer.set_need_reliable_transfer(prev_reliable);
}

/* write info message */
void Logger::write_info(const char *name, const char *value)
{
	_writer.lock();
	ulog_message_info_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_message(buffer, msg_size);
	}

	_writer.unlock();
}

void Logger::write_info_multiple(const char *name, const char *value, bool is_continued)
{
	_writer.lock();
	ulog_message_info_multiple_header_s msg;
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO_MULTIPLE);
	msg.is_continued = is_continued;

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_message(buffer, msg_size);
	}

	_writer.unlock();
}

void Logger::write_info(const char *name, int32_t value)
{
	write_info_template<int32_t>(name, value, "int32_t");
}

void Logger::write_info(const char *name, uint32_t value)
{
	write_info_template<uint32_t>(name, value, "uint32_t");
}


template<typename T>
void Logger::write_info_template(const char *name, T value, const char *type_str)
{
	_writer.lock();
	ulog_message_info_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);
	msg.msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, name);
	size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

	/* copy string value directly to buffer */
	memcpy(&buffer[msg_size], &value, sizeof(T));
	msg_size += sizeof(T);

	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_message(buffer, msg_size);

	_writer.unlock();
}

void Logger::write_header()
{
	ulog_file_header_s header = {};
	header.magic[0] = 'U';
	header.magic[1] = 'L';
	header.magic[2] = 'o';
	header.magic[3] = 'g';
	header.magic[4] = 0x01;
	header.magic[5] = 0x12;
	header.magic[6] = 0x35;
	header.magic[7] = 0x01; //file version 1
	header.timestamp = hrt_absolute_time();
	_writer.lock();
	write_message(&header, sizeof(header));

	// write the Flags message: this MUST be written right after the ulog header
	ulog_message_flag_bits_s flag_bits{};

	flag_bits.msg_size = sizeof(flag_bits) - ULOG_MSG_HEADER_LEN;
	flag_bits.msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	write_message(&flag_bits, sizeof(flag_bits));

	_writer.unlock();
}

/* write version info messages */
void Logger::write_version()
{
	write_info("ver_sw", px4_firmware_version_string());
	write_info("ver_sw_release", px4_firmware_version());
	uint32_t vendor_version = px4_firmware_vendor_version();

	if (vendor_version > 0) {
		write_info("ver_vendor_sw_release", vendor_version);
	}

	write_info("ver_hw", px4_board_name());
	const char *board_sub_type = px4_board_sub_type();

	if (board_sub_type && board_sub_type[0]) {
		write_info("ver_hw_subtype", board_sub_type);
	}
	write_info("sys_name", "PX4");
	write_info("sys_os_name", px4_os_name());
	const char *os_version = px4_os_version_string();

	const char *git_branch = px4_firmware_git_branch();

	if (git_branch && git_branch[0]) {
		write_info("ver_sw_branch", git_branch);
	}

	if (os_version) {
		write_info("sys_os_ver", os_version);
	}

	write_info("sys_os_ver_release", px4_os_version());
	write_info("sys_toolchain", px4_toolchain_name());
	write_info("sys_toolchain_ver", px4_toolchain_version());

	const char* ecl_version = px4_ecl_lib_version_string();

	if (ecl_version && ecl_version[0]) {
		write_info("sys_lib_ecl_ver", ecl_version);
	}

	char revision = 'U';
	const char *chip_name = nullptr;

	if (board_mcu_version(&revision, &chip_name, nullptr) >= 0) {
		char mcu_ver[64];
		snprintf(mcu_ver, sizeof(mcu_ver), "%s, rev. %c", chip_name, revision);
		write_info("sys_mcu", mcu_ver);
	}

#ifndef BOARD_HAS_NO_UUID
	/* write the UUID if enabled */
	param_t write_uuid_param = param_find("SDLOG_UUID");

	if (write_uuid_param != PARAM_INVALID) {
		int32_t write_uuid;
		param_get(write_uuid_param, &write_uuid);

		if (write_uuid == 1) {
			char uuid_string[PX4_CPU_UUID_WORD32_FORMAT_SIZE];
			board_get_uuid32_formated(uuid_string, sizeof(uuid_string), "%08X", NULL);
			write_info("sys_uuid", uuid_string);
		}
	}
#endif /* BOARD_HAS_NO_UUID */

	int32_t utc_offset = 0;

	if (_log_utc_offset != PARAM_INVALID) {
		param_get(_log_utc_offset, &utc_offset);
		write_info("time_ref_utc", utc_offset * 60);
	}

	if (_replay_file_name) {
		write_info("replay", _replay_file_name);
	}
}

void Logger::write_parameters()
{
	_writer.lock();
	ulog_message_parameter_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// skip over all parameters which are not invalid and not used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// save parameters which are valid AND used
		if (param != PARAM_INVALID) {
			// get parameter type and size
			const char *type_str;
			param_type_t type = param_type(param);
			size_t value_size = 0;

			switch (type) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			// format parameter key (type and name)
			msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

			// copy parameter value directly to buffer
			switch (type) {
			case PARAM_TYPE_INT32:
				param_get(param, (int32_t*)&buffer[msg_size]);
				break;

			case PARAM_TYPE_FLOAT:
				param_get(param, (float*)&buffer[msg_size]);
				break;

			default:
				continue;
			}
			msg_size += value_size;

			msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

			write_message(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters()
{
	_writer.lock();
	ulog_message_parameter_header_s msg = {};
	uint8_t *buffer = reinterpret_cast<uint8_t *>(&msg);

	msg.msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// skip over all parameters which are not invalid and not used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// log parameters which are valid AND used AND unsaved
		if ((param != PARAM_INVALID) && param_value_unsaved(param)) {

			// get parameter type and size
			const char *type_str;
			param_type_t type = param_type(param);
			size_t value_size = 0;

			switch (type) {
			case PARAM_TYPE_INT32:
				type_str = "int32_t";
				value_size = sizeof(int32_t);
				break;

			case PARAM_TYPE_FLOAT:
				type_str = "float";
				value_size = sizeof(float);
				break;

			default:
				continue;
			}

			// format parameter key (type and name)
			msg.key_len = snprintf(msg.key, sizeof(msg.key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(msg) - sizeof(msg.key) + msg.key_len;

			// copy parameter value directly to buffer
			switch (type) {
			case PARAM_TYPE_INT32:
				param_get(param, (int32_t*)&buffer[msg_size]);
				break;

			case PARAM_TYPE_FLOAT:
				param_get(param, (float*)&buffer[msg_size]);
				break;

			default:
				continue;
			}
			msg_size += value_size;

			// msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size
			msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

			write_message(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

int Logger::check_free_space()
{
	struct statfs statfs_buf;

	int32_t max_log_dirs_to_keep = 0;

	if (_log_dirs_max != PARAM_INVALID) {
		param_get(_log_dirs_max, &max_log_dirs_to_keep);
	}

	if (max_log_dirs_to_keep == 0) {
		max_log_dirs_to_keep = INT32_MAX;
	}

	// remove old logs if the free space falls below a threshold
	do {
		if (statfs(LOG_ROOT, &statfs_buf) != 0) {
			return PX4_ERROR;
		}

		DIR *dp = opendir(LOG_ROOT);

		if (dp == nullptr) {
			break; // ignore if we cannot access the log directory
		}

		struct dirent *result = nullptr;

		int num_sess = 0, num_dates = 0;

		// There are 2 directory naming schemes: sess<i> or <year>-<month>-<day>.
		// For both we find the oldest and then remove the one which has more directories.
		int year_min = 10000, month_min = 99, day_min = 99, sess_idx_min = 99999999, sess_idx_max = 0;

		while ((result = readdir(dp))) {
			int year, month, day, sess_idx;

			if (sscanf(result->d_name, "sess%d", &sess_idx) == 1) {
				++num_sess;

				if (sess_idx > sess_idx_max) {
					sess_idx_max = sess_idx;
				}

				if (sess_idx < sess_idx_min) {
					sess_idx_min = sess_idx;
				}

			} else if (sscanf(result->d_name, "%d-%d-%d", &year, &month, &day) == 3) {
				++num_dates;

				if (year < year_min) {
					year_min = year;
					month_min = month;
					day_min = day;

				} else if (year == year_min) {
					if (month < month_min) {
						month_min = month;
						day_min = day;

					} else if (month == month_min) {
						if (day < day_min) {
							day_min = day;
						}
					}
				}
			}
		}

		closedir(dp);

		_sess_dir_index = sess_idx_max + 1;


		uint64_t min_free_bytes = 300ULL * 1024ULL * 1024ULL;
		uint64_t total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;

		if (total_bytes / 10 < min_free_bytes) { // reduce the minimum if it's larger than 10% of the disk size
			min_free_bytes = total_bytes / 10;
		}

		if (num_sess + num_dates <= max_log_dirs_to_keep &&
		    statfs_buf.f_bavail >= (px4_statfs_buf_f_bavail_t)(min_free_bytes / statfs_buf.f_bsize)) {
			break; // enough free space and limit not reached
		}

		if (num_sess == 0 && num_dates == 0) {
			break; // nothing to delete
		}

		char directory_to_delete[LOG_DIR_LEN];
		int n;

		if (num_sess >= num_dates) {
			n = snprintf(directory_to_delete, sizeof(directory_to_delete), "%s/sess%03u", LOG_ROOT, sess_idx_min);

		} else {
			n = snprintf(directory_to_delete, sizeof(directory_to_delete), "%s/%04u-%02u-%02u", LOG_ROOT, year_min, month_min,
				     day_min);
		}

		if (n >= (int)sizeof(directory_to_delete)) {
			PX4_ERR("log path too long (%i)", n);
			break;
		}

		PX4_INFO("removing log directory %s to get more space (left=%u MiB)", directory_to_delete,
			 (unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize / 1024U / 1024U));

		if (remove_directory(directory_to_delete)) {
			PX4_ERR("Failed to delete directory");
			break;
		}

	} while (true);


	/* use a threshold of 50 MiB: if below, do not start logging */
	if (statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(50 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_log_critical(&_mavlink_log_pub,
				     "[logger] Not logging; SD almost full: %u MiB",
				     (unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize / 1024U / 1024U));
		return 1;
	}

	return PX4_OK;
}

int Logger::remove_directory(const char *dir)
{
	DIR *d = opendir(dir);
	size_t dir_len = strlen(dir);
	struct dirent *p;
	int ret = 0;

	if (!d) {
		return -1;
	}

	while (!ret && (p = readdir(d))) {
		int ret2 = -1;
		char *buf;
		size_t len;

		if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, "..")) {
			continue;
		}

		len = dir_len + strlen(p->d_name) + 2;
		buf = new char[len];

		if (buf) {
			struct stat statbuf;

			snprintf(buf, len, "%s/%s", dir, p->d_name);

			if (!stat(buf, &statbuf)) {
				if (S_ISDIR(statbuf.st_mode)) {
					ret2 = remove_directory(buf);

				} else {
					ret2 = unlink(buf);
				}
			}

			delete[] buf;
		}

		ret = ret2;
	}

	closedir(d);

	if (!ret) {
		ret = rmdir(dir);
	}

	return ret;
}

void Logger::ack_vehicle_command(orb_advert_t &vehicle_command_ack_pub, vehicle_command_s *cmd, uint32_t result)
{
	vehicle_command_ack_s vehicle_command_ack = {};
	vehicle_command_ack.timestamp = hrt_absolute_time();
	vehicle_command_ack.command = cmd->command;
	vehicle_command_ack.result = (uint8_t)result;
	vehicle_command_ack.target_system = cmd->source_system;
	vehicle_command_ack.target_component = cmd->source_component;

	if (vehicle_command_ack_pub == nullptr) {
		vehicle_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &vehicle_command_ack, vehicle_command_ack_s::ORB_QUEUE_LENGTH);
	} else {
		orb_publish(ORB_ID(vehicle_command_ack), vehicle_command_ack_pub, &vehicle_command_ack);
	}
}

} // namespace logger
} // namespace px4
