#include "logger.h"
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include <uORB/topics/sensor_combined.h>

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
		logger_ptr->add_topic(ORB_ID(sensor_combined));
		logger_ptr->run();
	}
}

enum class MessageType : uint8_t {
	FORMAT = 'F',
	TIME = 'T',
	DATA = 'D',
	MULTIDATA = 'M'
};

#pragma pack(push, 1)
struct message_format_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::FORMAT);
	uint8_t msg_id;
	uint8_t size;
	uint8_t format_len;
	char format[256];
};

struct message_time_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::TIME);
	uint64_t timestamp;
};

struct message_data_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::DATA);
	uint8_t msg_id;
};

struct message_multidata_header_s {
	uint8_t msg_type = static_cast<uint8_t>(MessageType::MULTIDATA);
	uint8_t msg_id;
	uint8_t multi_id;
};
#pragma pack(pop)

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
	_subscriptions.push_back(LoggerSubscription(orb_subscribe(topic), topic));
}

void Logger::add_all_topics() {
	// TODO
}

void Logger::run() {
	warnx("started");

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

			_timestamp_written = false;	// Flag to track if timestamp was already written on this update

			const hrt_abstime t = hrt_absolute_time();

			// Write data messages for normal subscriptions
			int msg_id = 0;
			for (LoggerSubscription &sub : _subscriptions) {
				size_t msg_size = sizeof(message_data_header_s) + sub.metadata->o_size;
				uint8_t buffer[msg_size];
				bool updated = false;
				orb_check(sub.fd, &updated);
				if (updated) {
					orb_copy(sub.metadata, sub.fd, buffer + sizeof(message_data_header_s));
					// Write timestamp only before data messages to avoid writing timestamps without data messages
					if (!write_time(t)) {
						break;
					}

					message_data_header_s *header = reinterpret_cast<message_data_header_s *>(buffer);
					header->msg_type = static_cast<uint8_t>(MessageType::DATA);
					header->msg_id = msg_id;

					if (!_writer.write(buffer, msg_size)) {
						break;
					}
				}
				msg_id++;
			}

			_writer.unlock();
			if (_timestamp_written) {
				_writer.notify();
			}
		}

		usleep(_log_interval);
	}
}

bool Logger::write_time(const hrt_abstime &t) {
	if (!_timestamp_written) {
		message_time_s msg;
		msg.timestamp = t;
		if (_writer.write(&msg, sizeof(msg))) {
			_timestamp_written = true;
			return true;
		} else {
			return false;
		}
	} else {
		return true;
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
		/* format log file path: e.g. /fs/microsd/sess001/log001.px4log */
		snprintf(file_name, file_name_size, "%s/log%03u.px4log", _log_dir, file_number);

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
	_writer.lock();
	write_formats();
	_writer.unlock();
	_writer.notify();
	_enabled = true;
}

void Logger::stop_log() {
	warnx("stop log");
	_enabled = false;
	_writer.stop_log();
}

void Logger::write_formats() {
	message_format_s msg;

	int msg_id = 0;
	for (LoggerSubscription &sub : _subscriptions) {
		msg.msg_id = msg_id;
		msg.size = sub.metadata->o_size;
		msg.format_len = snprintf(msg.format, sizeof(msg.format), "%s:", sub.metadata->o_name /* TODO , sub.metadata->fields*/);
		size_t msg_size = sizeof(msg) - sizeof(msg.format) + msg.format_len;
		while (!_writer.write(&msg, msg_size)) {
			usleep(_log_interval);	// Wait if buffer is full, don't skip FORMAT messages
		}
		msg_id++;
	}
}

}
}
