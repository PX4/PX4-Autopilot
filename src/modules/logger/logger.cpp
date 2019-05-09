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
#include <uORB/topics/vehicle_command_ack.h>

#include <drivers/drv_hrt.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <px4_sem.h>
#include <px4_shutdown.h>
#include <px4_tasks.h>
#include <systemlib/mavlink_log.h>
#include <replay/definitions.hpp>
#include <version/version.h>

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

constexpr const char *Logger::LOG_ROOT[(int)LogType::Count];

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

The file backend supports 2 types of log files: full (the normal log) and a mission
log. The mission log is a reduced ulog file and can be used for example for geotagging or
vehicle management. It can be enabled and configured via SDLOG_MISSION parameter.
The normal log is always a superset of the mission log.

### Implementation
The implementation uses two threads:
- The main thread, running at a fixed rate (or polling on a topic if started with -p) and checking for
  data updates
- The writer thread, writing data to the file

In between there is a write buffer with configurable size (and another fixed-size buffer for
the mission log). It should be large to avoid dropouts.

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
				      3700,
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
	if (_writer.is_started(LogType::Full, LogWriter::BackendFile)) {
		PX4_INFO("Full File Logging Running:");
		print_statistics(LogType::Full);
		is_logging = true;
	}

	if (_writer.is_started(LogType::Mission, LogWriter::BackendFile)) {
		PX4_INFO("Mission File Logging Running:");
		print_statistics(LogType::Mission);
		is_logging = true;
	}

	if (_writer.is_started(LogType::Full, LogWriter::BackendMavlink)) {
		PX4_INFO("Mavlink Logging Running (Full log)");
		is_logging = true;
	}

	if (!is_logging) {
		PX4_INFO("Not logging");
	}

	return 0;
}

void Logger::print_statistics(LogType type)
{
	if (!_writer.is_started(type, LogWriter::BackendFile)) { //currently only statistics for file logging
		return;
	}
	Statistics &stats = _statistics[(int)type];

	/* this is only for the file backend */
	float kibibytes = _writer.get_total_written_file(type) / 1024.0f;
	float mebibytes = kibibytes / 1024.0f;
	float seconds = ((float)(hrt_absolute_time() - stats.start_time_file)) / 1000000.0f;

	PX4_INFO("Log file: %s/%s/%s", LOG_ROOT[(int)type], _file_name[(int)type].log_dir, _file_name[(int)type].log_file_name);
	if (mebibytes < 0.1f) {
		PX4_INFO("Wrote %4.2f KiB (avg %5.2f KiB/s)", (double)kibibytes, (double)(kibibytes / seconds));
	} else {
		PX4_INFO("Wrote %4.2f MiB (avg %5.2f KiB/s)", (double)mebibytes, (double)(kibibytes / seconds));
	}
	PX4_INFO("Since last status: dropouts: %zu (max len: %.3f s), max used buffer: %zu / %zu B",
		 stats.write_dropouts, (double)stats.max_dropout_duration, stats.high_water, _writer.get_buffer_size_file(type));
	stats.high_water = 0;
	stats.write_dropouts = 0;
	stats.max_dropout_duration = 0.f;
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
	_mission_log = param_find("SDLOG_MISSION");

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

bool Logger::copy_if_updated_multi(int sub_idx, int multi_instance, void *buffer, bool try_to_subscribe)
{
	bool updated = false;
	LoggerSubscription &sub = _subscriptions[sub_idx];
	int &handle = sub.fd[multi_instance];

	if (handle < 0 && try_to_subscribe) {

		if (try_to_subscribe_topic(sub, multi_instance)) {

			write_add_logged_msg(LogType::Full, sub, multi_instance);
			if (sub_idx < _num_mission_subs) {
				write_add_logged_msg(LogType::Mission, sub, multi_instance);
			}

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
	add_topic("battery_status", 500);
	add_topic("camera_capture");
	add_topic("camera_trigger");
	add_topic("camera_trigger_secondary");
	add_topic("cpuload");
	add_topic("distance_sensor", 100);
	add_topic("ekf2_innovations", 200);
	//add_topic("ekf_gps_drift");
	add_topic("esc_status", 250);
	add_topic("estimator_status", 200);
	add_topic("home_position");
	add_topic("input_rc", 200);
	add_topic("manual_control_setpoint", 200);
	//add_topic("mission");
	//add_topic("mission_result");
	add_topic("optical_flow", 50);
	add_topic("position_setpoint_triplet", 200);
	//add_topic("radio_status");
	add_topic("rate_ctrl_status", 30);
	add_topic("sensor_combined", 100);
	add_topic("sensor_preflight", 200);
	add_topic("system_power", 500);
	add_topic("tecs_status", 200);
	add_topic("trajectory_setpoint", 200);
	add_topic("telemetry_status");
	add_topic("vehicle_air_data", 200);
	add_topic("vehicle_attitude", 30);
	add_topic("vehicle_attitude_setpoint", 100);
	add_topic("vehicle_command");
	add_topic("vehicle_global_position", 200);
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 100);
	add_topic("vehicle_magnetometer", 200);
	add_topic("vehicle_rates_setpoint", 30);
	add_topic("vehicle_status", 200);
	add_topic("vehicle_status_flags");
	add_topic("vtol_vehicle_status", 200);
	add_topic("wind_estimate", 200);

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
	add_topic("actuator_controls_virtual_fw");
	add_topic("actuator_controls_virtual_mc");
	add_topic("fw_virtual_attitude_setpoint");
	add_topic("mc_virtual_attitude_setpoint");
	add_topic("multirotor_motor_limits");
	add_topic("position_controller_status");
	add_topic("offboard_control_mode");
	add_topic("time_offset");
	add_topic("vehicle_attitude_groundtruth", 10);
	add_topic("vehicle_global_position_groundtruth", 100);
	add_topic("vehicle_local_position_groundtruth", 100);
	add_topic("vehicle_roi");
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
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
	add_topic("debug_array");
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
	add_topic("vehicle_visual_odometry");
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

void Logger::add_vision_and_avoidance_topics()
{
	add_topic("collision_constraints");
	add_topic("obstacle_distance");
	add_topic("vehicle_mocap_odometry", 30);
	add_topic("vehicle_trajectory_waypoint", 200);
	add_topic("vehicle_trajectory_waypoint_desired", 200);
	add_topic("vehicle_visual_odometry", 30);
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

void Logger::initialize_mission_topics(MissionLogType type)
{
	if (type == MissionLogType::Complete) {
		add_mission_topic("camera_capture");
		add_mission_topic("mission_result");
		add_mission_topic("vehicle_global_position", 1000);
		add_mission_topic("vehicle_status", 1000);
	} else if (type == MissionLogType::Geotagging) {
		add_mission_topic("camera_capture");
	}
}

void Logger::initialize_configured_topics()
{
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

	if (sdlog_profile & SDLogProfileMask::VISION_AND_AVOIDANCE) {
		add_vision_and_avoidance_topics();
	}
}


void Logger::add_mission_topic(const char* name, unsigned interval)
{
	if (_num_mission_subs >= MAX_MISSION_TOPICS_NUM) {
		PX4_ERR("Max num mission topics exceeded");
		return;
	}
	if(add_topic(name, interval)) {
		_mission_subscriptions[_num_mission_subs].min_delta_ms = interval;
		_mission_subscriptions[_num_mission_subs].next_write_time = 0;
		++_num_mission_subs;
	}
}

void Logger::run()
{
	PX4_INFO("logger started (mode=%s)", configured_backend_mode());

	if (_writer.backend() & LogWriter::BackendFile) {
		int mkdir_ret = mkdir(LOG_ROOT[(int)LogType::Full], S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == 0) {
			PX4_INFO("log root dir created: %s", LOG_ROOT[(int)LogType::Full]);

		} else if (errno != EEXIST) {
			PX4_ERR("failed creating log root dir: %s (%i)", LOG_ROOT[(int)LogType::Full], errno);

			if ((_writer.backend() & ~LogWriter::BackendFile) == 0) {
				return;
			}
		}

		int32_t max_log_dirs_to_keep = 0;

		if (_log_dirs_max != PARAM_INVALID) {
			param_get(_log_dirs_max, &max_log_dirs_to_keep);
		}

		if (util::check_free_space(LOG_ROOT[(int)LogType::Full], max_log_dirs_to_keep, _mavlink_log_pub,
				_file_name[(int)LogType::Full].sess_dir_index) == 1) {
			return;
		}
	}

	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	uORB::Subscription<parameter_update_s> parameter_update_sub(ORB_ID(parameter_update));
	int log_message_sub = orb_subscribe(ORB_ID(log_message));
	orb_set_interval(log_message_sub, 20);

	// mission log topics if enabled (must be added first)
	int32_t mission_log_mode = 0;

	if (_mission_log != PARAM_INVALID) {
		param_get(_mission_log, &mission_log_mode);
		initialize_mission_topics((MissionLogType)mission_log_mode);

		if (_num_mission_subs > 0) {
			int mkdir_ret = mkdir(LOG_ROOT[(int)LogType::Mission], S_IRWXU | S_IRWXG | S_IRWXO);
			if (mkdir_ret != 0 && errno != EEXIST) {
				PX4_ERR("failed creating log root dir: %s (%i)", LOG_ROOT[(int)LogType::Mission], errno);
			}
		}
	}

	int ntopics = add_topics_from_file(PX4_STORAGEDIR "/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		initialize_configured_topics();
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

	/* debug stats */
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;

	px4_register_shutdown_hook(&Logger::request_stop_static);

	// we start logging immediately
	// the case where we wait with logging until vehicle is armed is handled below
	if (_log_on_start) {
		start_log_file(LogType::Full);
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
		const bool logging_started = check_arming_state(vehicle_status_sub, (MissionLogType)mission_log_mode);
		if (logging_started) {
#ifdef DBGPRINT
			timer_start = hrt_absolute_time();
			total_bytes = 0;
#endif /* DBGPRINT */
		}

		/* check for logging command from MAVLink (start/stop streaming) */
		if (vehicle_command_sub >= 0) {
			handle_vehicle_command_update(vehicle_command_sub, vehicle_command_ack_pub);
		}


		if (timer_callback_data.watchdog_triggered) {
			timer_callback_data.watchdog_triggered = false;
			initialize_load_output(PrintLoadReason::Watchdog);
		}


		const hrt_abstime loop_time = hrt_absolute_time();

		if (_writer.is_started(LogType::Full)) { // mission log only runs when full log is also started

			/* check if we need to output the process load */
			if (_next_load_print != 0 && loop_time >= _next_load_print) {
				_next_load_print = 0;
				write_load_output();

				if (_should_stop_file_log) {
					_should_stop_file_log = false;
					stop_log_file(LogType::Full);
					continue; // skip to next loop iteration
				}
			}

			bool data_written = false;

			/* Check if parameters have changed */
			if (!_should_stop_file_log) { // do not record param changes after disarming
				if (parameter_update_sub.update()) {
					write_changed_parameters(LogType::Full);
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
					if (copy_if_updated_multi(sub_idx, instance, _msg_buffer + sizeof(ulog_message_data_header_s),
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

						// full log
						if (write_message(LogType::Full, _msg_buffer, msg_size)) {

#ifdef DBGPRINT
							total_bytes += msg_size;
#endif /* DBGPRINT */

							data_written = true;
						}

						// mission log
						if (sub_idx < _num_mission_subs) {
							if (_writer.is_started(LogType::Mission)) {
								if (_mission_subscriptions[sub_idx].next_write_time < (loop_time / 100000)) {
									unsigned delta_time = _mission_subscriptions[sub_idx].min_delta_ms;
									if (delta_time > 0) {
										_mission_subscriptions[sub_idx].next_write_time = (loop_time / 100000) + delta_time / 100;
									}
									if (write_message(LogType::Mission, _msg_buffer, msg_size)) {
										data_written = true;
									}
								}
							}
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

					write_message(LogType::Full, _msg_buffer, write_msg_size + ULOG_MSG_HEADER_LEN);
				}
			}

			// update buffer statistics
			for (int i = 0; i < (int)LogType::Count; ++i) {
				if (!_statistics[i].dropout_start && _writer.get_buffer_fill_count_file((LogType)i) > _statistics[i].high_water) {
					_statistics[i].high_water = _writer.get_buffer_fill_count_file((LogType)i);
				}
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

			debug_print_buffer(total_bytes, timer_start);

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
					// need to to an orb_copy so that the next poll will not return immediately
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

	stop_log_file(LogType::Full);
	stop_log_file(LogType::Mission);

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

void Logger::debug_print_buffer(uint32_t &total_bytes, hrt_abstime &timer_start)
{
#ifdef DBGPRINT
	double deltat = (double)(hrt_absolute_time() - timer_start)  * 1e-6;

	if (deltat > 4.0) {
		struct mallinfo alloc_info = mallinfo();
		double throughput = total_bytes / deltat;
		PX4_INFO("%8.1f kB/s, %zu highWater,  %d dropouts, %5.3f sec max, free heap: %d",
				throughput / 1.e3, _statistics[0].high_water, _statistics[0].write_dropouts,
				(double)_statistics[0].max_dropout_duration, alloc_info.fordblks);

		_statistics[0].high_water = 0;
		_statistics[0].max_dropout_duration = 0.f;
		total_bytes = 0;
		timer_start = hrt_absolute_time();
	}
#endif /* DBGPRINT */
}

bool Logger::check_arming_state(int vehicle_status_sub, MissionLogType mission_log_type)
{
	bool vehicle_status_updated;
	int ret = orb_check(vehicle_status_sub, &vehicle_status_updated);
	bool bret = false;

	if (ret == 0 && vehicle_status_updated) {
		vehicle_status_s vehicle_status;
		orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
		bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) || _arm_override;

		if (_was_armed != armed && !_log_until_shutdown) {
			_was_armed = armed;

			if (armed) {

				if (_should_stop_file_log) { // happens on quick arming after disarm
					_should_stop_file_log = false;
					stop_log_file(LogType::Full);
				}

				start_log_file(LogType::Full);
				bret = true;

				if (mission_log_type != MissionLogType::Disabled) {
					start_log_file(LogType::Mission);
				}

			} else {
				// delayed stop: we measure the process loads and then stop
				initialize_load_output(PrintLoadReason::Postflight);
				_should_stop_file_log = true;

				if (mission_log_type != MissionLogType::Disabled) {
					stop_log_file(LogType::Mission);
				}
			}
		}
	}
	return bret;
}

void Logger::handle_vehicle_command_update(int vehicle_command_sub, orb_advert_t &vehicle_command_ack_pub)
{
	bool command_updated = false;
	int ret = orb_check(vehicle_command_sub, &command_updated);

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

bool Logger::write_message(LogType type, void *ptr, size_t size)
{
	Statistics &stats = _statistics[(int)type];
	if (_writer.write_message(type, ptr, size, stats.dropout_start) != -1) {

		if (stats.dropout_start) {
			float dropout_duration = (float)(hrt_elapsed_time(&stats.dropout_start) / 1000) / 1.e3f;

			if (dropout_duration > stats.max_dropout_duration) {
				stats.max_dropout_duration = dropout_duration;
			}

			stats.dropout_start = 0;
		}

		return true;
	}

	if (!stats.dropout_start) {
		stats.dropout_start = hrt_absolute_time();
		++stats.write_dropouts;
		stats.high_water = 0;
	}

	return false;
}

int Logger::create_log_dir(LogType type, tm *tt, char *log_dir, int log_dir_len)
{
	LogFileName& file_name = _file_name[(int)type];

	/* create dir on sdcard if needed */
	int n = snprintf(log_dir, log_dir_len, "%s/", LOG_ROOT[(int)type]);

	if (n >= log_dir_len) {
		PX4_ERR("log path too long");
		return -1;
	}

	if (tt) {
		strftime(file_name.log_dir, sizeof(LogFileName::log_dir), "%Y-%m-%d", tt);
		strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
		int mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret != OK && errno != EEXIST) {
			PX4_ERR("failed creating new dir: %s", log_dir);
			return -1;
		}

	} else {
		uint16_t dir_number = file_name.sess_dir_index;

		if (file_name.has_log_dir) {
			strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
		}

		/* look for the next dir that does not exist */
		while (!file_name.has_log_dir) {
			/* format log dir: e.g. /fs/microsd/log/sess001 */
			int n2 = snprintf(file_name.log_dir, sizeof(LogFileName::log_dir), "sess%03u", dir_number);

			if (n2 >= (int)sizeof(LogFileName::log_dir)) {
				PX4_ERR("log path too long (%i)", n);
				return -1;
			}

			strncpy(log_dir + n, file_name.log_dir, log_dir_len - n);
			int mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				PX4_DEBUG("log dir created: %s", log_dir);
				file_name.has_log_dir = true;

			} else if (errno != EEXIST) {
				PX4_ERR("failed creating new dir: %s (%i)", log_dir, errno);
				return -1;
			}

			/* dir exists already */
			dir_number++;
		}

		file_name.has_log_dir = true;
	}

	return strlen(log_dir);
}

int Logger::get_log_file_name(LogType type, char *file_name, size_t file_name_size)
{
	tm tt = {};
	bool time_ok = false;

	if (_log_name_timestamp) {
		int32_t utc_offset = 0;

		if (_log_utc_offset != PARAM_INVALID) {
			param_get(_log_utc_offset, &utc_offset);
		}

		/* use RTC time for log file naming, e.g. /fs/microsd/log/2014-01-19/19_37_52.ulg */
		time_ok = util::get_log_time(&tt, utc_offset * 60, false);
	}

	const char *replay_suffix = "";

	if (_replay_file_name) {
		replay_suffix = "_replayed";
	}

	char *log_file_name = _file_name[(int)type].log_file_name;

	if (time_ok) {
		int n = create_log_dir(type, &tt, file_name, file_name_size);
		if (n < 0) {
			return -1;
		}

		char log_file_name_time[16] = "";
		strftime(log_file_name_time, sizeof(log_file_name_time), "%H_%M_%S", &tt);
		snprintf(log_file_name, sizeof(LogFileName::log_file_name), "%s%s.ulg", log_file_name_time, replay_suffix);
		snprintf(file_name + n, file_name_size - n, "/%s", log_file_name);

	} else {
		int n = create_log_dir(type, nullptr, file_name, file_name_size);
		if (n < 0) {
			return -1;
		}

		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/log/sess001/log001.ulg */
			snprintf(log_file_name, sizeof(LogFileName::log_file_name), "log%03u%s.ulg", file_number, replay_suffix);
			snprintf(file_name + n, file_name_size - n, "/%s", log_file_name);

			if (!util::file_exist(file_name)) {
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

void Logger::start_log_file(LogType type)
{
	if (_writer.is_started(type, LogWriter::BackendFile) || (_writer.backend() & LogWriter::BackendFile) == 0) {
		return;
	}

	PX4_INFO("Start file log (type: %s)", log_type_str(type));

	char file_name[LOG_DIR_LEN] = "";

	if (get_log_file_name(type, file_name, sizeof(file_name))) {
		PX4_ERR("failed to get log file name");
		return;
	}

	if (type == LogType::Full) {
		/* print logging path, important to find log file later */
		mavlink_log_info(&_mavlink_log_pub, "[logger] file: %s", file_name);
	}

	_writer.start_log_file(type, file_name);
	_writer.select_write_backend(LogWriter::BackendFile);
	_writer.set_need_reliable_transfer(true);
	write_header(type);
	write_version(type);
	write_formats(type);
	if (type == LogType::Full) {
		write_parameters(type);
		write_perf_data(true);
	}
	write_all_add_logged_msg(type);
	_writer.set_need_reliable_transfer(false);
	_writer.unselect_write_backend();
	_writer.notify();

	if (type == LogType::Full) {
		/* reset performance counters to get in-flight min and max values in post flight log */
		perf_reset_all();

		initialize_load_output(PrintLoadReason::Preflight);
	}

	_statistics[(int)type].start_time_file = hrt_absolute_time();

}

void Logger::stop_log_file(LogType type)
{
	if (!_writer.is_started(type, LogWriter::BackendFile)) {
		return;
	}

	if (type == LogType::Full) {
		_writer.set_need_reliable_transfer(true);
		write_perf_data(false);
		_writer.set_need_reliable_transfer(false);
	}
	_writer.stop_log_file(type);
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
	write_header(LogType::Full);
	write_version(LogType::Full);
	write_formats(LogType::Full);
	write_parameters(LogType::Full);
	write_perf_data(true);
	write_all_add_logged_msg(LogType::Full);
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

	callback_data->logger->write_info_multiple(LogType::Full, perf_name, buffer, callback_data->counter != 0);
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

	callback_data->logger->write_info_multiple(LogType::Full, perf_name, callback_data->buffer, callback_data->counter != 0);
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

void Logger::write_format(LogType type, const orb_metadata &meta, WrittenFormats &written_formats, ulog_message_format_s& msg, int level)
{
	if (level > 3) {
		// precaution: limit recursion level. If we land here it's either a bug or nested topic definitions. In the
		// latter case, increase the maximum level.
		PX4_ERR("max recursion level reached (%i)", level);
		return;
	}

	// Write the current format (we don't need to check if we already added it to written_formats)
	int format_len = snprintf(msg.format, sizeof(msg.format), "%s:%s", meta.o_name, meta.o_fields);
	size_t msg_size = sizeof(msg) - sizeof(msg.format) + format_len;
	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_message(type, &msg, msg_size);

	if (!written_formats.push_back(&meta)) {
		PX4_ERR("Array too small");
	}

	// Now go through the fields and check for nested type usages.
	// o_fields looks like this for example: "uint64_t timestamp;uint8_t[5] array;"
	const char* fmt = meta.o_fields;
	while (fmt && *fmt) {
		// extract the type name
		char type_name[64];
		const char *space = strchr(fmt, ' ');
		if (!space) {
			PX4_ERR("invalid format %s", fmt);
			break;
		}
		const char *array_start = strchr(fmt, '['); // check for an array

		int type_length;
		if (array_start && array_start < space) {
			type_length = array_start - fmt;
		} else {
			type_length = space - fmt;
		}
		if (type_length >= (int)sizeof(type_name)) {
			PX4_ERR("buf len too small");
			break;
		}
		memcpy(type_name, fmt, type_length);
		type_name[type_length] = '\0';

		// ignore built-in types
		if (strcmp(type_name, "int8_t") != 0 &&
				strcmp(type_name, "uint8_t") != 0 &&
				strcmp(type_name, "int16_t") != 0 &&
				strcmp(type_name, "uint16_t") != 0 &&
				strcmp(type_name, "int16_t") != 0 &&
				strcmp(type_name, "uint16_t") != 0 &&
				strcmp(type_name, "int32_t") != 0 &&
				strcmp(type_name, "uint32_t") != 0 &&
				strcmp(type_name, "int64_t") != 0 &&
				strcmp(type_name, "uint64_t") != 0 &&
				strcmp(type_name, "float") != 0 &&
				strcmp(type_name, "double") != 0 &&
				strcmp(type_name, "bool") != 0) {

			// find orb meta for type
			const orb_metadata *const*topics = orb_get_topics();
			const orb_metadata *found_topic = nullptr;
			for (size_t i = 0; i < orb_topics_count(); i++) {
				if (strcmp(topics[i]->o_name, type_name) == 0) {
					found_topic = topics[i];
				}
			}
			if (found_topic) {
				// check if we already wrote the format
				for (const auto& written_format: written_formats) {
					if (written_format == found_topic) {
						found_topic = nullptr;
						break;
					}
				}
				if (found_topic) {
					write_format(type, *found_topic, written_formats, msg, level+1);
				}
			} else {
				PX4_ERR("No definition for topic %s found", fmt);
			}
		}

		fmt = strchr(fmt, ';');
		if (fmt) { ++fmt; }
	}
}

void Logger::write_formats(LogType type)
{
	_writer.lock();

	// both of these are large and thus we need to be careful in terms of stack size requirements
	ulog_message_format_s msg;
	WrittenFormats written_formats;

	// write all subscribed formats
	size_t sub_count = _subscriptions.size();
	if (type == LogType::Mission) {
		sub_count = _num_mission_subs;
	}
	for (size_t i = 0; i < sub_count; ++i) {
		const LoggerSubscription &sub = _subscriptions[i];
		write_format(type, *sub.metadata, written_formats, msg);
	}

	_writer.unlock();
}

void Logger::write_all_add_logged_msg(LogType type)
{
	_writer.lock();

	size_t sub_count = _subscriptions.size();
	if (type == LogType::Mission) {
		sub_count = _num_mission_subs;
	}
	for (size_t i = 0; i < sub_count; ++i) {
		LoggerSubscription &sub = _subscriptions[i];
		for (int instance = 0; instance < ORB_MULTI_MAX_INSTANCES; ++instance) {
			if (sub.fd[instance] >= 0) {
				write_add_logged_msg(type, sub, instance);
			}
		}
	}

	_writer.unlock();
}

void Logger::write_add_logged_msg(LogType type, LoggerSubscription &subscription, int instance)
{
	ulog_message_add_logged_s msg;

	if (subscription.msg_ids[instance] == (uint8_t) - 1) {
		if (_next_topic_id == (uint8_t) - 1) {
			// if we land here an uint8 is too small -> switch to uint16
			PX4_ERR("limit for _next_topic_id reached");
			return;
		}
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
	write_message(type, &msg, msg_size);
	_writer.set_need_reliable_transfer(prev_reliable);
}

void Logger::write_info(LogType type, const char *name, const char *value)
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

		write_message(type, buffer, msg_size);
	}

	_writer.unlock();
}

void Logger::write_info_multiple(LogType type, const char *name, const char *value, bool is_continued)
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

		write_message(type, buffer, msg_size);
	}

	_writer.unlock();
}

void Logger::write_info(LogType type, const char *name, int32_t value)
{
	write_info_template<int32_t>(type, name, value, "int32_t");
}

void Logger::write_info(LogType type, const char *name, uint32_t value)
{
	write_info_template<uint32_t>(type, name, value, "uint32_t");
}


template<typename T>
void Logger::write_info_template(LogType type, const char *name, T value, const char *type_str)
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

	write_message(type, buffer, msg_size);

	_writer.unlock();
}

void Logger::write_header(LogType type)
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
	write_message(type, &header, sizeof(header));

	// write the Flags message: this MUST be written right after the ulog header
	ulog_message_flag_bits_s flag_bits{};

	flag_bits.msg_size = sizeof(flag_bits) - ULOG_MSG_HEADER_LEN;
	flag_bits.msg_type = static_cast<uint8_t>(ULogMessageType::FLAG_BITS);

	write_message(type, &flag_bits, sizeof(flag_bits));

	_writer.unlock();
}

void Logger::write_version(LogType type)
{
	write_info(type, "ver_sw", px4_firmware_version_string());
	write_info(type, "ver_sw_release", px4_firmware_version());
	uint32_t vendor_version = px4_firmware_vendor_version();

	if (vendor_version > 0) {
		write_info(type, "ver_vendor_sw_release", vendor_version);
	}

	write_info(type, "ver_hw", px4_board_name());
	const char *board_sub_type = px4_board_sub_type();

	if (board_sub_type && board_sub_type[0]) {
		write_info(type, "ver_hw_subtype", board_sub_type);
	}
	write_info(type, "sys_name", "PX4");
	write_info(type, "sys_os_name", px4_os_name());
	const char *os_version = px4_os_version_string();

	const char *git_branch = px4_firmware_git_branch();

	if (git_branch && git_branch[0]) {
		write_info(type, "ver_sw_branch", git_branch);
	}

	if (os_version) {
		write_info(type, "sys_os_ver", os_version);
	}

	write_info(type, "sys_os_ver_release", px4_os_version());
	write_info(type, "sys_toolchain", px4_toolchain_name());
	write_info(type, "sys_toolchain_ver", px4_toolchain_version());

	const char* ecl_version = px4_ecl_lib_version_string();

	if (ecl_version && ecl_version[0]) {
		write_info(type, "sys_lib_ecl_ver", ecl_version);
	}

	char revision = 'U';
	const char *chip_name = nullptr;

	if (board_mcu_version(&revision, &chip_name, nullptr) >= 0) {
		char mcu_ver[64];
		snprintf(mcu_ver, sizeof(mcu_ver), "%s, rev. %c", chip_name, revision);
		write_info(type, "sys_mcu", mcu_ver);
	}

#ifndef BOARD_HAS_NO_UUID
	/* write the UUID if enabled */
	param_t write_uuid_param = param_find("SDLOG_UUID");

	if (write_uuid_param != PARAM_INVALID) {
		int32_t write_uuid;
		param_get(write_uuid_param, &write_uuid);

		if (write_uuid == 1) {
			char px4_uuid_string[PX4_GUID_FORMAT_SIZE];
			board_get_px4_guid_formated(px4_uuid_string, sizeof(px4_uuid_string));
			write_info(type, "sys_uuid", px4_uuid_string);
		}
	}
#endif /* BOARD_HAS_NO_UUID */

	int32_t utc_offset = 0;

	if (_log_utc_offset != PARAM_INVALID) {
		param_get(_log_utc_offset, &utc_offset);
		write_info(type, "time_ref_utc", utc_offset * 60);
	}

	if (_replay_file_name) {
		write_info(type, "replay", _replay_file_name);
	}

	if (type == LogType::Mission) {
		write_info(type, "log_type", "mission");
	}
}

void Logger::write_parameters(LogType type)
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
			param_type_t ptype = param_type(param);
			size_t value_size = 0;

			switch (ptype) {
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
			switch (ptype) {
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

			write_message(type, buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters(LogType type)
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
			param_type_t ptype = param_type(param);
			size_t value_size = 0;

			switch (ptype) {
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
			switch (ptype) {
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

			write_message(type, buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
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
