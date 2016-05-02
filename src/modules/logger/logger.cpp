#include "logger.h"

#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include <uORB/uORBTopics.h>
#include <px4_getopt.h>

#define DBGPRINT

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
	if (argc < 2) {
		PX4_INFO("usage: logger {start|stop|status}");
		return 1;
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
		 "\t-b\tLog buffer size in KiB, default is 8\n"
		 "\t-e\tEnable logging by default (if not, can be started by command)\n"
		 "\t-a\tLog only when armed (can be still overriden by command)\n"
		 "\t-t\tUse date/time for naming log directories and files\n"
		 "\t-x\tExtended logging");
}

int Logger::start(char *const *argv)
{
	ASSERT(logger_task == -1);

	/* start the task */
	logger_task = px4_task_spawn_cmd("logger",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 3100,
					 (px4_main_t)&Logger::run_trampoline,
					 (char *const *)argv);

	if (logger_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

void Logger::status()
{
	if (!_enabled) {
		PX4_INFO("running, but not logging");

	} else {
		PX4_INFO("running");

//		float kibibytes = _writer.get_total_written() / 1024.0f;
//		float mebibytes = kibibytes / 1024.0f;
//		float seconds = ((float)(hrt_absolute_time() - _start_time)) / 1000000.0f;
//
//		PX4_WARN("wrote %lu msgs, %4.2f MiB (average %5.3f KiB/s), skipped %lu msgs", log_msgs_written, (double)mebibytes, (double)(kibibytes / seconds), log_msgs_skipped);
//		mavlink_log_info(&mavlink_log_pub, "[blackbox] wrote %lu msgs, skipped %lu msgs", log_msgs_written, log_msgs_skipped);
	}
}

void Logger::run_trampoline(int argc, char *argv[])
{
	unsigned log_interval = 3500;
	int log_buffer_size = 12 * 1024;
	bool log_on_start = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "r:b:eatx", &myoptind, &myoptarg)) != EOF) {
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

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
	struct mallinfo alloc_info = mallinfo();
	warnx("largest free chunk: %d bytes", alloc_info.mxordblk);
	warnx("allocating %d bytes for log_buffer", log_buffer_size);
#endif /* DBGPRINT */

	logger_ptr = new Logger(log_buffer_size, log_interval, log_on_start);

	if (logger_ptr->_log_buffer == nullptr) {
		PX4_WARN("log buffer malloc failed");

	} else {

#if defined(DBGPRINT) && defined(__PX4_NUTTX)
		alloc_info = mallinfo();
		warnx("remaining free heap: %d bytes", alloc_info.fordblks);
#endif /* DBGPRINT */

		logger_ptr->run();
	}
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

// currently unused
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

Logger::Logger(size_t buffer_size, unsigned log_interval, bool log_on_start) :
	_log_on_start(log_on_start),
	_writer((_log_buffer = new uint8_t[buffer_size]), buffer_size),
	_log_interval(log_interval)
{
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

	delete [] _log_buffer;
	logger_ptr = nullptr;
}

int Logger::add_topic(const orb_metadata *topic)
{
	int fd = -1;

	if (topic->o_size > MAX_DATA_SIZE) {
		PX4_WARN("skip topic %s, data size is too large: %zu (max is %zu)", topic->o_name, topic->o_size, MAX_DATA_SIZE);

	} else {
		size_t fields_len = strlen(topic->o_fields);

		if (fields_len > sizeof(message_format_s::format)) {
			PX4_WARN("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
				 sizeof(message_format_s::format));

		} else {
			fd = orb_subscribe(topic);
			_subscriptions.push_back(LoggerSubscription(fd, topic));
		}
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
			printf("logging topic: %zu, %s\n", i, topics[i]->o_name);
			break;
		}
	}

	if ((fd > 0) && (interval != 0)) {
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

	PX4_WARN("logger started");

	int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret == 0) {
		PX4_WARN("log root dir created: %s", LOG_ROOT);

	} else if (errno != EEXIST) {
		PX4_WARN("failed creating log root dir: %s", LOG_ROOT);
		return;
	}

	add_topic("sensor_gyro", 10);
	add_topic("sensor_accel", 10);
	add_topic("vehicle_rates_setpoint", 10);
	add_topic("vehicle_attitude_setpoint", 10);
	add_topic("vehicle_attitude", 10);
	add_topic("actuator_outputs", 50);
	add_topic("battery_status", 100);
	add_topic("vehicle_command", 100);
	add_topic("actuator_controls", 10);
	add_topic("vehicle_local_position_setpoint", 30);
	add_topic("rc_channels", 100);
//	add_topic("ekf2_innovations", 20);
	add_topic("commander_state", 100);
	add_topic("vehicle_local_position", 10);
	add_topic("vehicle_global_position", 10);
	add_topic("system_power", 100);
	add_topic("servorail_status", 100);
	add_topic("mc_att_ctrl_status", 50);
	add_topic("control_state");
//	add_topic("estimator_status");
	add_topic("vehicle_status", 100);

	int ret = _writer.thread_start(_writer_thread);

	if (ret) {
		PX4_ERR("logger: failed to create writer thread (%i)", ret);
		return;
	}

	_task_should_exit = false;

#ifdef DBGPRINT
	hrt_abstime	dropout_start = 0;
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;
	uint16_t	dropout_count = 0;
	size_t 		highWater = 0;
	size_t		available = 0;
	double		max_drop_len = 0;
#endif /* DBGPRINT */

	// we start logging immediately
	// the case where we wait with logging until vehicle is armed is handled below
	if (_log_on_start) {
		start_log();
	}

	/* every log_interval usec, check for orb updates */
	while (!_task_should_exit) {

		// Start/stop logging when system arm/disarm
		if (_vehicle_status_sub.check_updated()) {
			_vehicle_status_sub.update();
			bool armed = (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) ||
				     (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);

			if (_enabled != armed && !_log_on_start) {
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
			/* wait for lock on log buffer */
			_writer.lock();

			bool data_written = false;

			/* Check if parameters have changed */
			// this needs to change to a timestamped record to record a history of parameter changes
			if (_parameter_update_sub.check_updated()) {
				warnx("parameter update");
				_parameter_update_sub.update();
				write_changed_parameters();
			}

			// Write data messages for normal subscriptions
			int msg_id = 0;

			for (LoggerSubscription &sub : _subscriptions) {
				/* each message consists of a header followed by an orb data object
				 * The size of the data object is given by orb_metadata.o_size
				 */
				size_t msg_size = sizeof(message_data_header_s) + sub.metadata->o_size;
				uint8_t buffer[msg_size];

				/* if this topic has been updated, copy the new data into the message buffer
				 * and write a message to the log
				 */
				//orb_check(sub.fd, &updated);	// check whether a non-multi topic has been updated
				/* this works for both single and multi-instances */
				for (uint8_t instance = 0; instance < ORB_MULTI_MAX_INSTANCES; instance++) {
					if (copy_if_updated_multi(sub.metadata, instance, &sub.fd[instance], buffer + sizeof(message_data_header_s),
								  &sub.time_tried_subscribe)) {

						//uint64_t timestamp;
						//memcpy(&timestamp, buffer + sizeof(message_data_header_s), sizeof(timestamp));
						//warnx("topic: %s, instance: %d, timestamp: %llu",
						//		sub.metadata->o_name, instance, timestamp);

						/* copy the current topic data into the buffer after the header */
						//orb_copy(sub.metadata, sub.fd, buffer + sizeof(message_data_header_s));

						/* fill the message header struct in-place at the front of the buffer,
						 * accessing the unaligned (packed) structure properly
						 */
						message_data_header_s *header = reinterpret_cast<message_data_header_s *>(buffer);
						header->msg_type = static_cast<uint8_t>(MessageType::DATA);
						header->msg_size = static_cast<uint16_t>(msg_size - 3);
						header->msg_id = msg_id;
						header->multi_id = 0x80 + instance;	// Non multi, active

#ifdef DBGPRINT
						//warnx("subscription %s updated: %d, size: %d", sub.metadata->o_name, updated, msg_size);
						hrt_abstime trytime = hrt_absolute_time();

						if (_writer._count > highWater) {
							highWater = _writer._count;
						}

#endif /* DBGPRINT */

						if (_writer.write(buffer, msg_size)) {

#ifdef DBGPRINT

							// successful write: note end of dropout if dropout_start != 0
							if (dropout_start != 0) {
								double drop_len = (double)(trytime - dropout_start)  * 1e-6;

								if (drop_len > max_drop_len) { max_drop_len = drop_len; }

								PX4_WARN("dropout length: %5.3f seconds", drop_len);
								dropout_start = 0;
								highWater = 0;
							}

							total_bytes += msg_size;
#endif /* DBGPRINT */

							data_written = true;

						} else {

#ifdef DBGPRINT

							if (dropout_start == 0)	{
								available = _writer._count;
								PX4_WARN("dropout, available: %d/%d", available, _writer._buffer_size);
								dropout_start = trytime;
								dropout_count++;
							}

#endif /* DBGPRINT */

							break;	// Write buffer overflow, skip this record
						}
					}
				}

				msg_id++;
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
				PX4_INFO("%8.1e Kbytes/sec, %d highWater,  %d dropouts, %5.3f sec max, free heap: %d",
					 throughput / 1e3, highWater, dropout_count, max_drop_len, alloc_info.fordblks);

				total_bytes = 0;
				highWater = 0,
				dropout_count = 0;
				max_drop_len = 0;
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

	logger_task = -1;
}

int Logger::create_log_dir()
{
	/* create dir on sdcard if needed */
	uint16_t dir_number = 1; // start with dir sess001
	int mkdir_ret;

	/* look for the next dir that does not exist */
	while (dir_number <= MAX_NO_LOGFOLDER) {
		/* format log dir: e.g. /fs/microsd/sess001 */
		sprintf(_log_dir, "%s/sess%03u", LOG_ROOT, dir_number);
		mkdir_ret = mkdir(_log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if (mkdir_ret == 0) {
			PX4_INFO("log dir created: %s", _log_dir);
			break;

		} else if (errno != EEXIST) {
			PX4_WARN("failed creating new dir: %s", _log_dir);
			return -1;
		}

		/* dir exists already */
		dir_number++;
		continue;
	}

	if (dir_number >= MAX_NO_LOGFOLDER) {
		/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
		PX4_WARN("all %d possible dirs exist already", MAX_NO_LOGFOLDER);
		return -1;
	}

	/* print logging path, important to find log file later */
	//mavlink_and_console_log_info(mavlink_fd, "[sdlog2] log dir: %s", log_dir);
	return 0;
}

bool Logger::file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int Logger::get_log_file_name(char *file_name, size_t file_name_size)
{
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
		//mavlink_and_console_log_critical(mavlink_fd, "[sdlog2] ERR: max files %d", MAX_NO_LOGFILE);
		return -1;
	}

	return 0;
}

void Logger::start_log()
{
	PX4_WARN("start log");

	if (create_log_dir()) {
		return;
	}

	char file_name[64] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		return;
	}

	_writer.start_log(file_name);
	write_version();
	write_formats();
	write_parameters();
	_enabled = true;
}

void Logger::stop_log()
{
	_enabled = false;
	_writer.stop_log();
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

		while (!_writer.write(&msg, msg_size)) {
			_writer.unlock();
			_writer.notify();
			usleep(_log_interval);	// Wait if buffer is full, don't skip FORMAT messages
			_writer.lock();
		}

		msg_id++;
	}

	_writer.unlock();
	_writer.notify();
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
	msg->key_len = snprintf(msg->key, sizeof(msg->key), "char[%u] %s", (unsigned)vlen, name);
	size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

	/* copy string value directly to buffer */
	if (vlen < (sizeof(*msg) - msg_size)) {
		memcpy(&buffer[msg_size], value, vlen);
		msg_size += vlen;

		msg->msg_size = msg_size - 3;

		/* write message */
		while (!_writer.write(buffer, msg_size)) {
			/* wait if buffer is full, don't skip INFO messages */
			_writer.unlock();
			_writer.notify();
			usleep(_log_interval);
			_writer.lock();
		}
	}
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

			/* write message */
			while (!_writer.write(buffer, msg_size)) {
				/* wait if buffer is full, don't skip PARAMETER messages */
				_writer.unlock();
				_writer.notify();
				usleep(_log_interval);
				_writer.lock();
			}
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

			/* format parameter key (type and name and timestamp) */
			msg->key_len = snprintf(msg->key, sizeof(msg->key), "%s %s ", type_str, param_name(param));
			size_t msg_size = sizeof(*msg) - sizeof(msg->key) + msg->key_len;

			/* copy parameter value directly to buffer */
			param_get(param, &buffer[msg_size]);
			msg_size += value_size;

			/* msg_size is now 1 (msg_type) + 2 (msg_size) + 1 (key_len) + key_len + value_size */
			msg->msg_size = msg_size - 3;

			/* write message */
			while (!_writer.write(buffer, msg_size)) {
				/* wait if buffer is full, don't skip PARAMETER messages */
				_writer.unlock();
				_writer.notify();
				usleep(_log_interval);
				_writer.lock();
			}
		}
	} while ((param != PARAM_INVALID) && (param_idx < (int) param_count()));

	_writer.unlock();
	_writer.notify();
}

}
}
