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
#include "messages.h"

#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/uORBTopics.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>

#include <drivers/drv_hrt.h>
#include <px4_includes.h>
#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_sem.h>
#include <systemlib/mavlink_log.h>
#include <replay/definitions.hpp>

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

static Logger *logger_ptr = nullptr;
static int logger_task = -1;
static pthread_t writer_thread;

/* This is used to schedule work for the logger (periodic scan for updated topics) */
static void timer_callback(void *arg)
{
	px4_sem_t *semaphore = (px4_sem_t *)arg;
	px4_sem_post(semaphore);
}


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

	if (!strcmp(argv[1], "on")) {
		if (logger_ptr != nullptr) {
			logger_ptr->set_arm_override(true);
			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "off")) {
		if (logger_ptr != nullptr) {
			logger_ptr->set_arm_override(false);
			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		if (logger_ptr == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		logger_ptr->print_statistics();
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

	PX4_INFO("usage: logger {start|stop|on|off|status} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
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
		print_statistics();
	}
}
void Logger::print_statistics()
{
	if (!_enabled) {
		return;
	}

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

void Logger::run_trampoline(int argc, char *argv[])
{
	uint32_t log_interval = 3500;
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
					r = 1e6;
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
	PX4_INFO("largest free chunk: %d bytes", alloc_info.mxordblk);
	PX4_INFO("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

	if (logger_ptr == nullptr) {
		PX4_ERR("alloc failed");

	} else {
		//check for replay mode
		const char *logfile = getenv(px4::replay::ENV_FILENAME);

		if (logfile) {
			logger_ptr->setReplayFile(logfile);
		}

		logger_ptr->run();
	}

	logger_task = -1;
}


Logger::Logger(size_t buffer_size, uint32_t log_interval, bool log_on_start,
	       bool log_until_shutdown, bool log_name_timestamp) :
	_arm_override(false),
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

	if (_replay_file_name) {
		free(_replay_file_name);
	}

	if (_msg_buffer) {
		delete[](_msg_buffer);
	}
}

int Logger::add_topic(const orb_metadata *topic)
{
	int fd = -1;
	size_t fields_len = strlen(topic->o_fields) + strlen(topic->o_name) + 1; //1 for ':'

	if (fields_len > sizeof(ulog_message_format_s::format)) {
		PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
			 sizeof(ulog_message_format_s::format));

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
			PX4_DEBUG("logging topic: %s, interval: %i", topics[i]->o_name, interval);
			break;
		}
	}

	if (fd >= 0 && interval != 0) {
		orb_set_interval(fd, interval);
	}

	return fd;
}

bool Logger::copy_if_updated_multi(LoggerSubscription &sub, int multi_instance, void *buffer)
{
	bool updated = false;
	int &handle = sub.fd[multi_instance];

	// only try to subscribe to topic if this is the first time
	// after that just check after a certain interval to avoid high cpu usage
	if (handle < 0 && (sub.time_tried_subscribe == 0 ||
			   hrt_elapsed_time(&sub.time_tried_subscribe) > TRY_SUBSCRIBE_INTERVAL)) {
		sub.time_tried_subscribe = hrt_absolute_time();

		if (OK == orb_exists(sub.metadata, multi_instance)) {
			handle = orb_subscribe_multi(sub.metadata, multi_instance);

			//PX4_INFO("subscribed to instance %d of topic %s", multi_instance, topic->o_name);

			/* copy first data */
			if (handle >= 0) {
				write_add_logged_msg(sub, multi_instance);

				/* set to the same interval as the first instance */
				unsigned int interval;

				if (orb_get_interval(sub.fd[0], &interval) == 0 && interval > 0) {
					orb_set_interval(handle, interval);
				}

				orb_copy(sub.metadata, handle, buffer);
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

void Logger::add_default_topics()
{
	add_topic("vehicle_attitude", 10);
	add_topic("actuator_outputs", 50);
	add_topic("telemetry_status", 50);
	add_topic("vehicle_command");
	add_topic("vehicle_status", 200);
	add_topic("vtol_vehicle_status", 100);
	add_topic("commander_state", 100);
	add_topic("satellite_info", 1000);
	add_topic("vehicle_attitude_setpoint", 20);
	add_topic("vehicle_rates_setpoint", 10);
	add_topic("actuator_controls", 20);
	add_topic("actuator_controls_0", 20);
	add_topic("actuator_controls_1", 20);
	add_topic("vehicle_local_position", 100);
	add_topic("vehicle_local_position_setpoint", 50);
	add_topic("vehicle_global_position", 100);
	add_topic("vehicle_global_velocity_setpoint", 100);
	add_topic("battery_status", 300);
	add_topic("system_power", 300);
	add_topic("position_setpoint_triplet", 10);
	add_topic("att_pos_mocap", 50);
	add_topic("vision_position_estimate", 50);
	add_topic("optical_flow", 50);
	add_topic("rc_channels");
	add_topic("airspeed", 50);
	add_topic("distance_sensor", 20);
	add_topic("esc_status", 20);
	add_topic("estimator_status", 50); //this one is large
	add_topic("ekf2_innovations", 20);
	add_topic("tecs_status", 20);
	add_topic("wind_estimate", 100);
	add_topic("control_state", 20);
	add_topic("camera_trigger");
	add_topic("cpuload");
	add_topic("gps_dump"); //this will only be published if GPS_DUMP_COMM is set

	/* for estimator replay (need to be at full rate) */
	add_topic("sensor_combined");
	add_topic("vehicle_gps_position");
	add_topic("vehicle_land_detected");
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

	if (fp == NULL) {
		return -1;
	}

	/* call add_topic for each topic line in the file */
	// format is TOPIC_NAME, [interval]
	for (;;) {

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {
			break;
		}

		/* skip comment lines */
		if ((strlen(line) < 2) || (line[0] == '#')) {
			continue;
		}

		// default interval to zero
		interval = 0;
		int nfields = sscanf(line, "%s, %u", topic_name, &interval);

		if (nfields > 0) {
			/* add topic with specified interval */
			add_topic(topic_name, interval);
			ntopics++;
		}
	}

	fclose(fp);
	return ntopics;
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
	uORB::Subscription<mavlink_log_s> mavlink_log_sub(ORB_ID(mavlink_log));

	int ntopics = add_topics_from_file(PX4_ROOTFSDIR "/fs/microsd/etc/logging/logger_topics.txt");

	if (ntopics > 0) {
		PX4_INFO("logging %d topics from logger_topics.txt", ntopics);

	} else {
		add_default_topics();
	}

	//all topics added. Get required message buffer size
	int max_msg_size = 0;

	for (const auto &subscription : _subscriptions) {
		//use o_size, because that's what orb_copy will use
		if (subscription.metadata->o_size > max_msg_size) {
			max_msg_size = subscription.metadata->o_size;
		}
	}

	max_msg_size += sizeof(ulog_message_data_header_s);

	if (sizeof(ulog_message_logging_s) > max_msg_size) {
		max_msg_size = sizeof(ulog_message_logging_s);
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
		PX4_ERR("init of writer failed (alloc failed)");
		return;
	}

	int ret = _writer.thread_start(writer_thread);

	if (ret) {
		PX4_ERR("failed to create writer thread (%i)", ret);
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

	/* init the update timer */
	struct hrt_call timer_call;
	px4_sem_t timer_semaphore;
	px4_sem_init(&timer_semaphore, 0, 0);
	hrt_call_every(&timer_call, _log_interval, _log_interval, timer_callback, &timer_semaphore);


	while (!_task_should_exit) {

		// Start/stop logging when system arm/disarm
		if (vehicle_status_sub.check_updated()) {
			vehicle_status_sub.update();
			bool armed = (vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) ||
				     (vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) ||
				     _arm_override;

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
				parameter_update_sub.update();
				write_changed_parameters();
			}

			/* wait for lock on log buffer */
			_writer.lock();

			for (LoggerSubscription &sub : _subscriptions) {
				/* each message consists of a header followed by an orb data object
				 */
				size_t msg_size = sizeof(ulog_message_data_header_s) + sub.metadata->o_size_no_padding;

				/* if this topic has been updated, copy the new data into the message buffer
				 * and write a message to the log
				 */
				for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
					if (copy_if_updated_multi(sub, instance, _msg_buffer + sizeof(ulog_message_data_header_s))) {

						uint16_t write_msg_size = static_cast<uint16_t>(msg_size - ULOG_MSG_HEADER_LEN);
						//write one byte after another (necessary because of alignment)
						_msg_buffer[0] = (uint8_t)write_msg_size;
						_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
						_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::DATA);
						uint16_t write_msg_id = sub.msg_ids[instance];
						_msg_buffer[3] = (uint8_t)write_msg_id;
						_msg_buffer[4] = (uint8_t)(write_msg_id >> 8);

						//PX4_INFO("topic: %s, size = %zu, out_size = %zu", sub.metadata->o_name, sub.metadata->o_size, msg_size);

						if (write(_msg_buffer, msg_size)) {

#ifdef DBGPRINT
							total_bytes += msg_size;
#endif /* DBGPRINT */

							data_written = true;

						} else {
							break;	// Write buffer overflow, skip this record
						}
					}
				}
			}

			//check for new mavlink log message
			if (mavlink_log_sub.check_updated()) {
				mavlink_log_sub.update();
				const char *message = (const char *)mavlink_log_sub.get().text;
				int message_len = strlen(message);

				if (message_len > 0) {
					uint16_t write_msg_size = sizeof(ulog_message_logging_s) - sizeof(ulog_message_logging_s::message)
								  - ULOG_MSG_HEADER_LEN + message_len;
					_msg_buffer[0] = (uint8_t)write_msg_size;
					_msg_buffer[1] = (uint8_t)(write_msg_size >> 8);
					_msg_buffer[2] = static_cast<uint8_t>(ULogMessageType::LOGGING);
					_msg_buffer[3] = mavlink_log_sub.get().severity + '0';
					memcpy(_msg_buffer + 4, &mavlink_log_sub.get().timestamp, sizeof(ulog_message_logging_s::timestamp));
					strncpy((char *)(_msg_buffer + 12), message, sizeof(ulog_message_logging_s::message));

					write(_msg_buffer, write_msg_size + ULOG_MSG_HEADER_LEN);
				}
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

		/*
		 * We wait on the semaphore, which periodically gets updated by a high-resolution timer.
		 * The simpler alternative would be:
		 *   usleep(max(300, _log_interval - elapsed_time_since_loop_start));
		 * And on linux this is quite accurate as well, but under NuttX it is not accurate,
		 * because usleep() has only a granularity of CONFIG_MSEC_PER_TICK (=1ms).
		 */
		while (px4_sem_wait(&timer_semaphore) != 0);
	}

	hrt_cancel(&timer_call);
	px4_sem_destroy(&timer_semaphore);

	// stop the writer thread
	_writer.thread_stop();

	_writer.notify();

	// wait for thread to complete
	ret = pthread_join(writer_thread, NULL);

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

bool Logger::write(void *ptr, size_t size)
{
	if (_writer.write(ptr, size, _dropout_start)) {

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

		if (n >= sizeof(_log_dir)) {
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
		uint16_t dir_number = 1; // start with dir sess001

		/* look for the next dir that does not exist */
		while (!_has_log_dir && dir_number <= MAX_NO_LOGFOLDER) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			int n = snprintf(_log_dir, sizeof(_log_dir), "%s/sess%03u", LOG_ROOT, dir_number);

			if (n >= sizeof(_log_dir)) {
				PX4_ERR("log path too long");
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

	const char *replay_suffix = "";

	if (_replay_file_name) {
		replay_suffix = "_replayed";
	}


	if (time_ok) {
		if (create_log_dir(&tt)) {
			return -1;
		}

		char log_file_name[64] = "";
		strftime(log_file_name, sizeof(log_file_name), "%H_%M_%S", &tt);
		snprintf(file_name, file_name_size, "%s/%s%s.ulg", _log_dir, log_file_name, replay_suffix);

	} else {
		if (create_log_dir(nullptr)) {
			return -1;
		}

		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.ulg */
			snprintf(file_name, file_name_size, "%s/log%03u%s.ulg", _log_dir, file_number, replay_suffix);

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
		orb_unsubscribe(vehicle_gps_position_sub);

		if (gps_pos.fix_type >= 2 && utc_time_sec >= GPS_EPOCH_SECS) {
			use_clock_time = false;
		}
	}

	if (use_clock_time) {
		/* take clock time if there's no fix (yet) */
		struct timespec ts;
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

void Logger::start_log()
{
	if (_enabled) {
		return;
	}

	PX4_INFO("start log");

	char file_name[LOG_DIR_LEN] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		PX4_ERR("logger: failed to get log file name");
		return;
	}

	/* print logging path, important to find log file later */
	mavlink_log_info(&_mavlink_log_pub, "[logger] file: %s", file_name);
	_next_topic_id = 0;

	_writer.start_log(file_name);
	write_header();
	write_version();
	write_formats();
	write_parameters();
	write_all_add_logged_msg();
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
	ulog_message_format_s msg;
	const orb_metadata **topics = orb_get_topics();

	//write all known formats
	for (size_t i = 0; i < orb_topics_count(); i++) {
		int format_len = snprintf(msg.format, sizeof(msg.format), "%s:%s", topics[i]->o_name, topics[i]->o_fields);
		size_t msg_size = sizeof(msg) - sizeof(msg.format) + format_len;
		msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_wait(&msg, msg_size);
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

	msg.multi_id = instance;
	subscription.msg_ids[instance] = _next_topic_id;
	msg.msg_id = _next_topic_id;

	int message_name_len = strlen(subscription.metadata->o_name);

	memcpy(msg.message_name, subscription.metadata->o_name, message_name_len);

	size_t msg_size = sizeof(msg) - sizeof(msg.message_name) + message_name_len;
	msg.msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_wait(&msg, msg_size);

	++_next_topic_id;
}

/* write info message */
void Logger::write_info(const char *name, const char *value)
{
	_writer.lock();
	uint8_t buffer[sizeof(ulog_message_info_header_s)];
	ulog_message_info_header_s *msg = reinterpret_cast<ulog_message_info_header_s *>(buffer);
	msg->msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	size_t vlen = strlen(value);
	msg->key_len = snprintf(msg->key, sizeof(msg->key), "char[%zu] %s", vlen, name);
	size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(*msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg->msg_size = msg_size - ULOG_MSG_HEADER_LEN;

		write_wait(buffer, msg_size);
	}

	_writer.unlock();
}
void Logger::write_info(const char *name, int32_t value)
{
	_writer.lock();
	uint8_t buffer[sizeof(ulog_message_info_header_s)];
	ulog_message_info_header_s *msg = reinterpret_cast<ulog_message_info_header_s *>(buffer);
	msg->msg_type = static_cast<uint8_t>(ULogMessageType::INFO);

	/* construct format key (type and name) */
	msg->key_len = snprintf(msg->key, sizeof(msg->key), "int32_t %s", name);
	size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

	/* copy string value directly to buffer */
	memcpy(&buffer[msg_size], &value, sizeof(int32_t));
	msg_size += sizeof(int32_t);

	msg->msg_size = msg_size - ULOG_MSG_HEADER_LEN;

	write_wait(buffer, msg_size);

	_writer.unlock();
}

void Logger::write_header()
{
	ulog_file_header_s header;
	header.magic[0] = 'U';
	header.magic[1] = 'L';
	header.magic[2] = 'o';
	header.magic[3] = 'g';
	header.magic[4] = 0x01;
	header.magic[5] = 0x12;
	header.magic[6] = 0x35;
	header.magic[7] = 0x00; //file version 0
	header.timestamp = hrt_absolute_time();
	_writer.lock();
	write_wait(&header, sizeof(header));
	_writer.unlock();
}

/* write version info messages */
void Logger::write_version()
{
	write_info("ver_sw", PX4_GIT_VERSION_STR);
	write_info("ver_hw", HW_ARCH);
	write_info("sys_name", "PX4");
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
	uint8_t buffer[sizeof(ulog_message_parameter_header_s) + sizeof(param_value_u)];
	ulog_message_parameter_header_s *msg = reinterpret_cast<ulog_message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
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

			msg->msg_size = msg_size - ULOG_MSG_HEADER_LEN;

			write_wait(buffer, msg_size);
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

void Logger::write_changed_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(ulog_message_parameter_header_s) + sizeof(param_value_u)];
	ulog_message_parameter_header_s *msg = reinterpret_cast<ulog_message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(ULogMessageType::PARAMETER);
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

			/* msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size */
			msg->msg_size = msg_size - ULOG_MSG_HEADER_LEN;

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
