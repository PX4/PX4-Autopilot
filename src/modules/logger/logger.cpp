#include "logger.h"
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <uORB/uORBTopics.h>

using namespace px4::logger;

int logger_main(int argc, char *argv[]) {
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

namespace px4 {
namespace logger {

void Logger::usage(const char *reason) {
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

int Logger::start() {
	ASSERT(logger_task == -1);

	/* start the task */
	logger_task = px4_task_spawn_cmd("logger",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (px4_main_t)&Logger::run_trampoline,
				       nullptr);

	if (logger_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void Logger::run_trampoline(int argc, char *argv[]) {
	logger_ptr = new Logger(32768, 10000);
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

#pragma pack(push, 1)
struct message_format_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::FORMAT);
	uint8_t msg_size;

	uint8_t msg_id;
	uint8_t format_len;
	char format[254];
};

struct message_data_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::DATA);
	uint8_t msg_size;

	uint8_t msg_id;
	uint8_t multi_id;
	//uint64_t timestamp;	// this field already included in each data struct
};

struct message_info_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::INFO);
	uint8_t msg_size;

	uint8_t key_len;
	char key[255];
};

struct message_parameter_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::PARAMETER);
	uint8_t msg_size;

	uint8_t key_len;
	char key[255];
};
#pragma pack(pop)


static constexpr size_t MAX_DATA_SIZE = 255 - 2;

Logger::Logger(size_t buffer_size, unsigned log_interval) :
		_writer((_log_buffer = new uint8_t[buffer_size]), buffer_size),
		_log_interval(log_interval) {
}

Logger::~Logger() {
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

void Logger::add_topic(const orb_metadata *topic) {
	if (topic->o_size > MAX_DATA_SIZE) {
		warn("skip topic %s, data size is too large: %i (max is %i)", topic->o_name, topic->o_size, MAX_DATA_SIZE);
		return;
	}
	size_t fields_len = strlen(topic->o_fields);
	if (fields_len > sizeof(message_format_s::format)) {
		warn("skip topic %s, format string is too large: %i (max is %i)", topic->o_name, fields_len, sizeof(message_format_s::format));
		return;
	}
	_subscriptions.push_back(LoggerSubscription(orb_subscribe(topic), topic));
}

void Logger::add_all_topics() {
	const orb_metadata **topics = orb_get_topics();
	for (size_t i = 0; i < orb_topics_count(); i++) {
		add_topic(topics[i]);
	}
}

void Logger::run() {
	warnx("started");

	if (_subscriptions.size() == 0) {
		warnx("log all topics");
		add_all_topics();
	}

	int mkdir_ret = mkdir(LOG_ROOT, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret == 0) {
		warnx("log root dir created: %s", LOG_ROOT);

	} else if (errno != EEXIST) {
		warn("failed creating log root dir: %s", LOG_ROOT);
		return;
	}

	_writer.thread_start();

	_task_should_exit = false;
	while (!_task_should_exit) {
		// Start/stop logging when system arm/disarm
		if (_vehicle_status_sub.check_updated()) {
			bool armed = (_vehicle_status_sub.arming_state == vehicle_status_s::ARMING_STATE_ARMED) || (_vehicle_status_sub.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR);
			if (_enabled != armed) {
				if (armed) {
					start_log();
				} else {
					stop_log();
				}
			}
		}

		if (_enabled) {
			_writer.lock();

			bool data_written = false;

			// Write data messages for normal subscriptions
			int msg_id = 0;
			for (LoggerSubscription &sub : _subscriptions) {
				size_t msg_size = sizeof(message_data_header_s) + sub.metadata->o_size;
				uint8_t buffer[msg_size];
				bool updated = false;
				orb_check(sub.fd, &updated);
				if (updated) {
					orb_copy(sub.metadata, sub.fd, buffer + sizeof(message_data_header_s));
					message_data_header_s *header = reinterpret_cast<message_data_header_s *>(buffer);
					header->msg_type = static_cast<uint8_t>(MessageType::DATA);
					header->msg_size = static_cast<uint8_t>(msg_size - 2);
					header->msg_id = msg_id;
					header->multi_id = 0x80;	// Non multi, active

					if (_writer.write(buffer, msg_size)) {
						data_written = true;
					} else {
						break;	// Write buffer overflow
					}
				}
				msg_id++;
			}

			_writer.unlock();
			if (data_written) {
				_writer.notify();
			}
		}

		usleep(_log_interval);
	}
}

int Logger::create_log_dir() {
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

bool Logger::file_exist(const char *filename) {
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int Logger::get_log_file_name(char *file_name, size_t file_name_size) {
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

void Logger::start_log() {
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

void Logger::stop_log() {
	warnx("stop log");
	_enabled = false;
	_writer.stop_log();
}

void Logger::write_formats() {
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

void Logger::write_parameters() {
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
