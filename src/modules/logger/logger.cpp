#include "logger.h"
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <uORB/uORBTopics.h>

#define DBGPRINT

using namespace px4::logger;

int logger_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: logger {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (logger_ptr != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != Logger::start()) {
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (logger_ptr == nullptr) {
			warnx("not running");
			return 1;
		}

		warnx("deleting logger");
		delete logger_ptr;
		logger_ptr = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (logger_ptr) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

namespace px4
{
namespace logger
{

void Logger::usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	warnx("usage: logger {start|stop|status|on|off} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
	      "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
	      "\t-b\tLog buffer size in KiB, default is 8\n"
	      "\t-e\tEnable logging by default (if not, can be started by command)\n"
	      "\t-a\tLog only when armed (can be still overriden by command)\n"
	      "\t-t\tUse date/time for naming log directories and files\n"
	      "\t-x\tExtended logging");
}

int Logger::start()
{
	ASSERT(logger_task == -1);

	/* start the task */
	logger_task = px4_task_spawn_cmd("logger",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 3100,
					 (px4_main_t)&Logger::run_trampoline,
					 nullptr);

	if (logger_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void Logger::run_trampoline(int argc, char *argv[])
{
	logger_ptr = new Logger(32768, 8000);

	if (logger_ptr == nullptr) {
		warnx("alloc failed");

	} else {
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
	//uint64_t timestamp;	// this field already included in each data struct
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

Logger::Logger(size_t buffer_size, unsigned log_interval) :
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
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(logger_task);
				break;
			}
		} while (logger_task != -1);
	}

	logger_ptr = nullptr;
}

void Logger::add_topic(const orb_metadata *topic)
{
	if (topic->o_size > MAX_DATA_SIZE) {
		warn("skip topic %s, data size is too large: %zu (max is %zu)", topic->o_name, topic->o_size, MAX_DATA_SIZE);
		return;
	}

	size_t fields_len = strlen(topic->o_fields);


	if (fields_len > sizeof(message_format_s::format)) {
		warn("skip topic %s, format string is too large: %zu (max is %zu)", topic->o_name, fields_len,
		     sizeof(message_format_s::format));
		return;
	}

	_subscriptions.push_back(LoggerSubscription(orb_subscribe(topic), topic));
}

void Logger::add_all_topics()
{
	const orb_metadata **topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		add_topic(topics[i]);
	}
}

void Logger::run()
{
	struct	mallinfo alloc_info;

	warnx("started");

	// There doesn't seem to be a way to poll topics in array _subscriptions like this:
//	int vehicle_status_fd = orb_subscribe(ORB_ID(vehicle_status));

//	px4_pollfd_struct_t fds[1] = {};
//	fds[0].fd = vehicle_status_fd;
//	fds[0].events = POLLIN;

//	if (_subscriptions.size() == 0) {
//		warnx("log all topics");
//		add_all_topics();
//	}
	add_topic(ORB_ID(vehicle_status));

	const orb_metadata **topics = orb_get_topics();
	add_topic(topics[3]);	// vehicle_attitude
//	add_topic(topics[5]);	// estimator_status
	add_topic(topics[68]);	// sensor_accel
//	add_topic(topics[64]);	// sensor_combined
	add_topic(topics[39]);	// sensor_baro
	add_topic(topics[53]);	// sensor_gyro

	int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret == 0) {
		warnx("log root dir created: %s", LOG_ROOT);

	} else if (errno != EEXIST) {
		warn("failed creating log root dir: %s", LOG_ROOT);
		return;
	}

	_writer.thread_start();

	_task_should_exit = false;

#ifdef DBGPRINT
	hrt_abstime	dropout_start = 0;
	hrt_abstime	timer_start = 0;
	uint32_t	total_bytes = 0;
	uint16_t	dropout_count = 0;
	size_t 		highWater = 0;
	size_t		available = 0;
	double		max_drop_len = 0;
#endif

	while (!_task_should_exit) {

//		px4_poll(fds, 1, 1000);

		// Start/stop logging when system arm/disarm
		if (_vehicle_status_sub.check_updated()) {
			_vehicle_status_sub.update();
			bool armed = (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED) ||
				     (_vehicle_status_sub.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);

			if (_enabled != armed) {
				if (armed) {
					start_log();

#ifdef DBGPRINT
					timer_start = hrt_absolute_time();
					total_bytes = 0;
#endif

				} else {
					stop_log();
				}
			}
		}

		if (_enabled) {
			/* wait for lock on log buffer */
			_writer.lock();

			bool data_written = false;

			// Write data messages for normal subscriptions
			int msg_id = 0;

			for (LoggerSubscription &sub : _subscriptions) {
				/* each message consists of a header followed by an orb data object
				 * The size of the data object is given by orb_metadata.o_size
				 */
				size_t msg_size = sizeof(message_data_header_s) + sub.metadata->o_size;
				uint8_t buffer[msg_size];
				bool updated = false;
				orb_check(sub.fd, &updated);

				if (updated) {
					/* copy the current topic data into the buffer after the header */
					orb_copy(sub.metadata, sub.fd, buffer + sizeof(message_data_header_s));

					/* fill the message header struct in-place at the front of the buffer,
					 * accessing the unaligned (packed) structure properly
					 */
					message_data_header_s *header = reinterpret_cast<message_data_header_s *>(buffer);
					header->msg_type = static_cast<uint8_t>(MessageType::DATA);
					/* the ORB metadata object has 2 unused trailing bytes? */
					header->msg_size = static_cast<uint16_t>(msg_size - 2);
					header->msg_id = msg_id;
					header->multi_id = 0x80;	// Non multi, active

#ifdef DBGPRINT
					//warnx("subscription %s updated: %d, size: %d", sub.metadata->o_name, updated, msg_size);
					hrt_abstime trytime = hrt_absolute_time();
					if (_writer._count > highWater) {
						highWater = _writer._count;
					}
#endif

					if (_writer.write(buffer, msg_size)) {
#ifdef DBGPRINT
						// successful write: note end of dropout if dropout_start != 0
						if (dropout_start != 0) {
							double drop_len = (double)(trytime - dropout_start)  * 1e-6;
							if (drop_len > max_drop_len) max_drop_len = drop_len;
							warnx("dropout length: %5.3f seconds", drop_len);
							dropout_start = 0;
							highWater = 0;
						}

#endif
						data_written = true;
#ifdef DBGPRINT
						total_bytes += msg_size;
#endif

					} else {
#ifdef DBGPRINT

						if (dropout_start == 0)	{
							available = _writer._count;
							warnx("dropout, available: %d/%d", available, _writer._buffer_size);
							dropout_start = trytime;
							dropout_count++;
						}

#endif
						break;	// Write buffer overflow, skip this record
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

			if (deltat > 10.0) {
				alloc_info = mallinfo();
				double throughput = total_bytes / deltat;
				warnx("%8.1e Kbytes/sec, %d highWater,  %d dropouts, %5.3f sec max, free heap: %d",
						throughput / 1e3, highWater, dropout_count, max_drop_len, alloc_info.fordblks);

				total_bytes = 0;
				highWater = 0,
				dropout_count = 0;
				max_drop_len = 0;
				timer_start = hrt_absolute_time();
			}

#endif
		}

		/* sleep only 2msec in order to catch every 250Hz orb update
		 * polling would be more efficient
		 */
		usleep(2000);
//		usleep(_log_interval);
	}
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
			warnx("log dir created: %s", _log_dir);
			break;

		} else if (errno != EEXIST) {
			warn("failed creating new dir: %s", _log_dir);
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
	warnx("start log");

	if (create_log_dir()) {
		return;
	}

	char file_name[64] = "";

	if (get_log_file_name(file_name, sizeof(file_name))) {
		return;
	}

	_writer.start_log(file_name);
	write_formats();
	write_parameters();
	_enabled = true;
}

void Logger::stop_log()
{
	warnx("stop log");
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
		msg.msg_size = msg_size - 2;

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

void Logger::write_parameters()
{
	_writer.lock();
	uint8_t buffer[sizeof(message_parameter_header_s) + sizeof(param_value_u)];
	message_parameter_header_s *msg = reinterpret_cast<message_parameter_header_s *>(buffer);

	msg->msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	int param_idx = 0;
	param_t param = 0;

	do {
		do {
			param = param_for_index(param_idx);
			++param_idx;
		} while (param != PARAM_INVALID && !param_used(param));

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

			msg->msg_size = msg_size - 2;

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
