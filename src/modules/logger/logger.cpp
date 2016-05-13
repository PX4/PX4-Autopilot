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

#include "logger.h"

#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/uORBTopics.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <systemlib/mavlink_log.h>

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

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

int logger_main(int argc, char *argv[])
{
	// logger currently assumes little endian
	int num = 1;

	if (*(char *)&num != 1) {
		PX4_ERR("Logger only works on little endian!\n");
		return 1;
	}

	if (argc < 2) {
		PX4_INFO("usage: logger {start|stop|status}");
		return 1;
	}

	//Check if thread exited, but the object has not been destroyed yet (happens in case of an error)
	if (logger_task == -1 && logger_ptr) {
		delete logger_ptr;
		logger_ptr = nullptr;
	}

	if (!strcmp(argv[1], "start")) {

		if (logger_ptr != nullptr) {
			PX4_INFO("already running");
			return 1;
		}

		if (OK != Logger::start((char *const *)argv)) {
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (logger_ptr == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		delete logger_ptr;
		logger_ptr = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (logger_ptr) {
			logger_ptr->status();
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	Logger::usage("unrecognized command");
	return 1;
}

namespace px4
{
namespace logger
{

void Logger::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_INFO("usage: logger {start|stop|status} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
		 "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
		 "\t-b\tLog buffer size in KiB, default is 12\n"
		 "\t-e\tEnable logging right after start until disarm (otherwise only when armed)\n"
		 "\t-f\tLog until shutdown (implies -e)\n"
		 "\t-t\tUse date/time for naming log directories and files");
}

int Logger::start(char *const *argv)
{
	ASSERT(logger_task == -1);

	/* start the task */
	logger_task = px4_task_spawn_cmd("logger",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 3200,
					 (px4_main_t)&Logger::run_trampoline,
					 (char *const *)argv);

	if (logger_task < 0) {
		logger_task = -1;
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

void Logger::status()
{
	if (!_enabled) {
		PX4_INFO("Running, but not logging");

	} else {
		PX4_INFO("Running");

		float kibibytes = _writer.get_total_written() / 1024.0f;
		float mebibytes = kibibytes / 1024.0f;
		float seconds = ((float)(hrt_absolute_time() - _start_time)) / 1000000.0f;

		PX4_INFO("Wrote %4.2f MiB (avg %5.2f KiB/s)", (double)mebibytes, (double)(kibibytes / seconds));
		PX4_INFO("Since last status: dropouts: %zu (max len: %.3f s), max used buffer: %zu / %zu B",
			 _write_dropouts, (double)_max_dropout_duration, _high_water, _writer.get_buffer_size());
		_high_water = 0;
		_write_dropouts = 0;
		_max_dropout_duration = 0.f;
	}
}

void Logger::run_trampoline(int argc, char *argv[])
{
	unsigned log_interval = 3500;
	int log_buffer_size = 12 * 1024;
	bool log_on_start = false;
	bool log_until_shutdown = false;
	bool error_flag = false;
	bool log_name_timestamp = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "r:b:etf", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, NULL, 10);

				if (r <= 0) {
					r = 1;
				}

				log_interval = 1e6 / r;
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, NULL, 10);

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
		logger_task = -1;
		return;
	}

	logger_ptr = new Logger(log_buffer_size, log_interval, log_on_start,
				log_until_shutdown, log_name_timestamp);

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
	struct mallinfo alloc_info = mallinfo();
	warnx("largest free chunk: %d bytes", alloc_info.mxordblk);
	warnx("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

	if (logger_ptr == nullptr) {
		PX4_WARN("alloc failed");

	} else {
		logger_ptr->run();
	}

	logger_task = -1;
}

enum class MessageType : uint8_t {
	FORMAT = 'F',
	DATA = 'D',
	INFO = 'I',
	PARAMETER = 'P',
};

/* declare message data structs with byte alignment (no padding) */
#pragma pack(push, 1)
struct message_format_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::FORMAT);
	uint16_t msg_size;

	uint8_t msg_id;
	uint16_t format_len;
	char format[2096];
};

struct message_data_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::DATA);
	uint16_t msg_size;

	uint8_t msg_id;
	uint8_t multi_id;
};

struct message_info_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::INFO);
	uint16_t msg_size;

	uint8_t key_len;
	char key[255];
};

struct message_parameter_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	uint16_t msg_size;

	uint8_t key_len;
	char key[255];
};
#pragma pack(pop)


static constexpr size_t MAX_DATA_SIZE = 740;

Logger::Logger(size_t buffer_size, unsigned log_interval, bool log_on_start,
	       bool log_until_shutdown, bool log_name_timestamp) :
	_log_on_start(log_on_start),
	_log_until_shutdown(log_until_shutdown),
	_log_name_timestamp(log_name_timestamp),
	_writer(buffer_size),
	_log_interval(log_interval)
{
	_log_utc_offset = param_find("SDLOG_UTC_OFFSET");
}

Logger::~Logger()
{
	if (logger_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned int i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 200) {
				px4_task_delete(logger_task);
				logger_task = -1;
				break;
			}
		} while (logger_task != -1);
	}
}

int Logger::add_topic(const orb_metadata *topic)
{
	int fd = -1;

	if (topic->o_size > MAX_DATA_SIZE) {
		PX4_WARN("skip topic %s, data size is too large: %zu (max is %zu)", topic->o_name, topic->o_size, MAX_DATA_SIZE);
		return -1;

	}

	size_t fields_len = strlen(topic->o_fields);

	if (fields_len > sizeof(message_format_s::format)) {
		PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
			 sizeof(message_format_s::format));

		return -1;
	}

	fd = orb_subscribe(topic);

	if (fd < 0) {
		PX4_WARN("logger: orb_subscribe failed");
		return -1;
	}

	if (!_subscriptions.push_back(LoggerSubscription(fd, topic))) {
		PX4_WARN("logger: failed to add topic. Too many subscriptions");
		orb_unsubscribe(fd);
		fd = -1;
	}

	return fd;
}

int Logger::add_topic(const char *name, unsigned interval = 0)
{
	const orb_metadata **topics = orb_get_topics();
	int fd = -1;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(name, topics[i]->o_name) == 0) {
			fd = add_topic(topics[i]);
			PX4_INFO("logging topic: %zu, %s", i, topics[i]->o_name);
			break;
		}
	}

	if (fd >= 0 && interval != 0) {
		orb_set_interval(fd, interval);
	}

	return fd;
}

bool Logger::copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer,
				   uint64_t *time_last_checked)
{
	bool updated = false;

	// only try to subscribe to topic if this is the first time
	// after that just check after a certain interval to avoid high cpu usage
	if (*handle < 0 && (*time_last_checked == 0 || hrt_elapsed_time(time_last_checked) > TRY_SUBSCRIBE_INTERVAL)) {
		//if (multi_instance == 1) warnx("checking instance 1 of topic %s", topic->o_name);
		*time_last_checked = hrt_absolute_time();

		if (OK == orb_exists(topic, multi_instance)) {
			*handle = orb_subscribe_multi(topic, multi_instance);

			//warnx("subscribed to instance %d of topic %s", multi_instance, topic->o_name);

			/* copy first data */
			if (*handle >= 0) {
				orb_copy(topic, *handle, buffer);
				updated = true;
			}
		}

	} else if (*handle >= 0) {
		orb_check(*handle, &updated);

		if (updated) {
			orb_copy(topic, *handle, buffer);
		}
	}

	return updated;
}

void Logger::run()
{
#ifdef DBGPRINT
	struct mallinfo alloc_info = {};
#endif /* DBGPRINT */

	PX4_INFO("logger started");

	int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret == 0) {
		PX4_INFO("log root dir created: %s", LOG_ROOT);

	} else if (errno != EEXIST) {
		PX4_ERR("failed creating log root dir: %s", LOG_ROOT);
		return;
	}

	if (check_free_space() == 1) {
		return;
	}

	uORB::Subscription<vehicle_status_s> vehicle_status_sub(ORB_ID(vehicle_status));
	uORB::Subscription<parameter_update_s> parameter_update_sub(ORB_ID(parameter_update));


	add_topic("sensor_gyro", 0);
	add_topic("sensor_accel", 0);
	add_topic("vehicle_rates_setpoint", 10);
	add_topic("vehicle_attitude_setpoint", 10);
	add_topic("vehicle_attitude", 0);
	add_topic("actuator_outputs", 50);
	add_topic("battery_status", 100);
	add_topic("vehicle_command", 100);
	add_topic("actuator_controls", 10);
	add_topic("vehicle_local_position_setpoint", 200);
	add_topic("rc_channels", 20);
//	add_topic("ekf2_innovations", 20);
	add_topic("commander_state", 100);
	add_topic("vehicle_local_position", 200);
	add_topic("vehicle_global_position", 200);
	add_topic("system_power", 100);
	add_topic("servorail_status", 200);
	add_topic("mc_att_ctrl_status", 50);
//	add_topic("control_state");
//	add_topic("estimator_status");
	add_topic("vehicle_status", 200);

	if (!_writer.init()) {
		PX4_ERR("logger: init of writer failed");
		return;
	}

	int ret = _writer.thread_start(_writer_thread);

	if (ret) {
		PX4_ERR("logger: failed to create writer thread (%i)", ret);
		return;
	}

	_task_should_exit = false;

#ifdef DBGPRINT
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;
#endif /* DBGPRINT */

	// we start logging immediately
	// the case where we wait with logging until vehicle is armed is handled below
	if (_log_on_start) {
		start_log();
	}

	/* every log_interval usec, check for orb updates */
	while (!_task_should_exit) {

		// Start/stop logging when system arm/disarm
		if (vehicle_status_sub.check_updated()) {
			vehicle_status_sub.update();
			bool armed = (vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) ||
				     (vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);

			if (_was_armed != armed && !_log_until_shutdown) {
				_was_armed = armed;
				if (armed) {
					start_log();

#ifdef DBGPRINT
					timer_start = hrt_absolute_time();
					total_bytes = 0;
#endif /* DBGPRINT */

				} else {
					stop_log();
				}
			}
		}

		if (_enabled) {

			bool data_written = false;

			/* Check if parameters have changed */
			// this needs to change to a timestamped record to record a history of parameter changes
			if (parameter_update_sub.check_updated()) {
				warnx("parameter update");
				parameter_update_sub.update();
				write_changed_parameters();
			}

			// Write data messages for normal subscriptions
			int msg_id = 0;

			/* wait for lock on log buffer */
			_writer.lock();

			for (LoggerSubscription &sub : _subscriptions) {
				/* each message consists of a header followed by an orb data object
				 */
				size_t msg_size = sizeof(message_data_header_s) + sub.metadata->o_size;
				//TODO: use sub.metadata->o_size_no_padding
				uint8_t buffer[msg_size];

				/* if this topic has been updated, copy the new data into the message buffer
				 * and write a message to the log
				 */
				for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
					if (copy_if_updated_multi(sub.metadata, instance, &sub.fd[instance], buffer + sizeof(message_data_header_s),
								  &sub.time_tried_subscribe)) {

						message_data_header_s *header = reinterpret_cast<message_data_header_s *>(buffer);
						header->msg_type = static_cast<uint8_t>(MessageType::DATA);
						header->msg_size = static_cast<uint16_t>(msg_size - 3);
						header->msg_id = msg_id;
						header->multi_id = 0x80 + instance;	// Non multi, active

						//PX4_INFO("topic: %s, size = %zu, out_size = %zu", sub.metadata->o_name, sub.metadata->o_size, msg_size);

						if (_writer.write(buffer, msg_size)) {

#ifdef DBGPRINT
							total_bytes += msg_size;
#endif /* DBGPRINT */

							if (_dropout_start) {
								float dropout_duration = (float)(hrt_elapsed_time(&_dropout_start) / 1000) / 1.e3f;

								if (dropout_duration > _max_dropout_duration) {
									_max_dropout_duration = dropout_duration;
								}

								_dropout_start = 0;
							}

							data_written = true;

						} else {

							if (!_dropout_start) {
								_dropout_start = hrt_absolute_time();
								++_write_dropouts;
								_high_water = 0;
							}

							break;	// Write buffer overflow, skip this record
						}
					}
				}

				msg_id++;
			}

			if (!_dropout_start && _writer.get_buffer_fill_count() > _high_water) {
				_high_water = _writer.get_buffer_fill_count();
			}

			/* release the log buffer */
			_writer.unlock();

			/* notify the writer thread if data is available */
			if (data_written) {
				_writer.notify();
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

		}

		usleep(_log_interval);
	}

	// stop the writer thread
	_writer.thread_stop();

	_writer.notify();

	// wait for thread to complete
	ret = pthread_join(_writer_thread, NULL);

	if (ret) {
		PX4_WARN("join failed: %d", ret);
	}

	//unsubscribe
	for (LoggerSubscription &sub : _subscriptions) {
		for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
			if (sub.fd[instance] != -1) {
				orb_unsubscribe(sub.fd[instance]);
				sub.fd[instance] = -1;
			}
		}
	}

	if (_mavlink_log_pub) {
		orb_unadvertise(_mavlink_log_pub);
		_mavlink_log_pub = nullptr;
	}
}

int Logger::create_log_dir(tm *tt)
{
	/* create dir on sdcard if needed */
	int mkdir_ret;

	if (tt) {
		int n = snprintf(_log_dir, sizeof(_log_dir), "%s/", LOG_ROOT);
		strftime(_log_dir + n, sizeof(_log_dir) - n, "%Y-%m-%d", tt);
		mkdir_ret = mkdir(_log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret != OK && errno != EEXIST) {
			PX4_ERR("failed creating new dir: %s", _log_dir);
			return -1;
		}

	} else {
		uint16_t dir_number = 1; // start with dir sess001

		/* look for the next dir that does not exist */
		while (!_has_log_dir && dir_number <= MAX_NO_LOGFOLDER) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			sprintf(_log_dir, "%s/sess%03u", LOG_ROOT, dir_number);
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

		if (dir_number >= MAX_NO_LOGFOLDER) {
			/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
			PX4_ERR("All %d possible dirs exist already", MAX_NO_LOGFOLDER);
			return -1;
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
	tm tt;
	bool time_ok = false;

	if (_log_name_timestamp) {
		/* use RTC time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.ulg */
		time_ok = get_log_time(&tt, false);
	}

	if (time_ok) {
		if (create_log_dir(&tt)) {
			return -1;
		}

		char log_file_name[64] = "";
		strftime(log_file_name, sizeof(log_file_name), "%H_%M_%S.ulg", &tt);
		snprintf(file_name, file_name_size, "%s/%s", _log_dir, log_file_name);

	} else {
		if (create_log_dir(nullptr)) {
			return -1;
		}

		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.ulg */
			snprintf(file_name, file_name_size, "%s/log%03u.ulg", _log_dir, file_number);

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

bool Logger::get_log_time(struct tm *tt, bool boot_time)
{
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	if (vehicle_gps_position_sub < 0) {
		return false;
	}

	/* Get the latest GPS publication */
	vehicle_gps_position_s gps_pos;

	if (orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps_pos) < 0) {
		orb_unsubscribe(vehicle_gps_position_sub);
		return false;
	}

	orb_unsubscribe(vehicle_gps_position_sub);
	time_t utc_time_sec = gps_pos.time_utc_usec / 1e6;

	if (gps_pos.fix_type < 2 || utc_time_sec < GPS_EPOCH_SECS) {
		return false;
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

void Logger::start_log()
{
	if (_enabled) {
		return;
	}

	PX4_INFO("start log");

	char file_name[64] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		PX4_ERR("logger: failed to get log file name");
		return;
	}

	/* print logging path, important to find log file later */
	mavlink_log_info(&_mavlink_log_pub, "[logger] file: %s", file_name);

	_writer.start_log(file_name);
	write_version();
	write_formats();
	write_parameters();
	_writer.notify();
	_enabled = true;
	_start_time = hrt_absolute_time();
}

void Logger::stop_log()
{
	if (!_enabled) {
		return;
	}

	_enabled = false;
	_writer.stop_log();
}

bool Logger::write_wait(void *ptr, size_t size)
{
	while (!_writer.write(ptr, size)) {
		_writer.unlock();
		_writer.notify();
		usleep(_log_interval);
		_writer.lock();
	}

	return true;
}

void Logger::write_formats()
{
	_writer.lock();
	message_format_s msg;

	int msg_id = 0;

	for (LoggerSubscription &sub : _subscriptions) {
		msg.msg_id = msg_id;
		msg.format_len = snprintf(msg.format, sizeof(msg.format), "%s", sub.metadata->o_fields);
		size_t msg_size = sizeof(msg) - sizeof(msg.format) + msg.format_len;
		msg.msg_size = msg_size - 3;

		write_wait(&msg, msg_size);

		msg_id++;
	}

	_writer.unlock();
}

/* write info message */
void Logger::write_info(const char *name, const char *value)
{
	_writer.lock();
	uint8_t buffer[sizeof(message_info_header_s)];
	message_info_header_s *msg = reinterpret_cast<message_info_header_s *>(buffer);
	msg->msg_type = static_cast<uint8_t>(MessageType::INFO);

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg->key_len = snprintf(msg->key, sizeof(msg->key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(*msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg->msg_size = msg_size - 3;

		write_wait(buffer, msg_size);
	}

	_writer.unlock();
}

/* write version info messages */
void Logger::write_version()
{
	write_info("ver_sw", PX4_GIT_VERSION_STR);
	write_info("ver_hw", HW_ARCH);
}

void Logger::write_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(message_parameter_header_s) + sizeof(param_value_u)];
	message_parameter_header_s *msg = reinterpret_cast<message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// get next parameter which is invalid OR used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// save parameters which are valid AND used
		if (param != PARAM_INVALID) {
			/* get parameter type and size */
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

			/* format parameter key (type and name) */
			msg->key_len = snprintf(msg->key, sizeof(msg->key), "%s %s", type_str, param_name(param));
			size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

			/* copy parameter value directly to buffer */
			param_get(param, &buffer[msg_size]);
			msg_size += value_size;

			msg->msg_size = msg_size - 3;

			write_wait(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(message_parameter_header_s) + sizeof(param_value_u)];
	message_parameter_header_s *msg = reinterpret_cast<message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		// get next parameter which is invalid OR used
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

		// log parameters which are valid AND used AND unsaved
		if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
			warnx("logging change to parameter %s", param_name(param));

			/* get parameter type and size */
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

			/* format parameter key (type and name) */
			msg->key_len = snprintf(msg->key, sizeof(msg->key), "%s %s ", type_str, param_name(param));
			size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

			/* copy parameter value directly to buffer */
			param_get(param, &buffer[msg_size]);
			msg_size += value_size;

			/* msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size */
			msg->msg_size = msg_size - 3;

			write_wait(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

int Logger::check_free_space()
{
	/* use statfs to determine the number of blocks left */
	struct statfs statfs_buf;

	if (statfs(LOG_ROOT, &statfs_buf) != 0) {
		return PX4_ERROR;
	}

	/* use a threshold of 50 MiB */
	if (statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(50 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_and_console_log_critical(&_mavlink_log_pub,
						 "[logger] Not logging; SD almost full: %u MiB",
						 (unsigned int)(statfs_buf.f_bavail / 1024U * statfs_buf.f_bsize / 1024U));
		return 1;
	}

	return PX4_OK;
}

}
}
