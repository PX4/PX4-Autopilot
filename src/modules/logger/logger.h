#pragma once

#include "log_writer.h"
#include "array.h"
#include <px4.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

namespace px4 {
namespace logger {

struct LoggerSubscription {
	int fd = -1;
	const orb_metadata *metadata = nullptr;

	LoggerSubscription() {}

	LoggerSubscription(int fd_, const orb_metadata *metadata_) :
			fd(fd_), metadata(metadata_) {}
};

class Logger {
public:
	Logger(size_t buffer_size, unsigned log_interval);

	~Logger();

	void add_topic(const orb_metadata *topic);

	void add_all_topics();

	static int start();

	static void usage(const char *reason);

private:
	static void run_trampoline(int argc, char *argv[]);

	void run();

	int create_log_dir();

	static bool file_exist(const char *filename);

	int get_log_file_name(char *file_name, size_t file_name_size);

	void start_log();

	void stop_log();

	void write_formats();

	void write_parameters();

	static constexpr size_t 	MAX_TOPICS_NUM = 32;
	static constexpr size_t 	MAX_TOPICS_GROUPS_NUM = 32;
	static constexpr uint8_t 	MAGIC_NUM = 0x77;
	static constexpr unsigned	MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
	static constexpr unsigned	MAX_NO_LOGFILE = 999;	/**< Maximum number of log files */
	static constexpr const char *		LOG_ROOT = PX4_ROOTFSDIR"/fs/microsd/log";

	bool						_task_should_exit = true;
	uint8_t *					_log_buffer;
	char 						_log_dir[64];
	uORB::Subscription<vehicle_status_s>	_vehicle_status_sub {ORB_ID(vehicle_status)};
	bool						_enabled = false;
    Array<LoggerSubscription, MAX_TOPICS_NUM>	_subscriptions;
    LogWriter					_writer;
    uint32_t					_log_interval;
};

Logger *logger_ptr;
int		logger_task = -1;

}
}
