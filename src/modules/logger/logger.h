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

#pragma once

#include "log_writer.h"
#include "array.h"
#include <px4.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <version/version.h>
#include <systemlib/git_version.h>
#include <systemlib/param/param.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

#define TRY_SUBSCRIBE_INTERVAL 1000*1000	// interval in microseconds at which we try to subscribe to a topic
// if we haven't succeeded before

#ifdef __PX4_NUTTX
#define LOG_DIR_LEN 64
#else
#define LOG_DIR_LEN 256
#endif

namespace px4
{
namespace logger
{

struct LoggerSubscription {
	int fd[ORB_MULTI_MAX_INSTANCES];
	uint16_t msg_ids[ORB_MULTI_MAX_INSTANCES];
	uint64_t time_tried_subscribe;	// captures the time at which we checked last time if this instance existed
	const orb_metadata *metadata = nullptr;

	LoggerSubscription() {}

	LoggerSubscription(int fd_, const orb_metadata *metadata_) :
		metadata(metadata_)
	{
		fd[0] = fd_;
		time_tried_subscribe = 0;

		for (int i = 1; i < ORB_MULTI_MAX_INSTANCES; i++) {
			fd[i] = -1;
		}
	}
};

class Logger
{
public:
	Logger(size_t buffer_size, uint32_t log_interval, bool log_on_start,
	       bool log_until_shutdown, bool log_name_timestamp);

	~Logger();

	/**
	 * Tell the logger that we're in replay mode. This must be called
	 * before starting the logger.
	 * @param file_name file name of the used log replay file. Will be copied.
	 */
	void setReplayFile(const char *file_name);

	/**
	 * Add a topic to be logged. This must be called before start_log()
	 * (because it does not write an ADD_LOGGED_MSG message).
	 * @param name topic name
	 * @param interval limit rate if >0, otherwise log as fast as the topic is updated.
	 * @return 0 on success
	 */
	int add_topic(const char *name, unsigned interval);

	/**
	 * add a logged topic (called by add_topic() above)
	 */
	int add_topic(const orb_metadata *topic);

	static int start(char *const *argv);

	static void usage(const char *reason);

	void status();
	void print_statistics();

	void set_arm_override(bool override) { _arm_override = override; }

private:
	static void run_trampoline(int argc, char *argv[]);

	void run();

	/**
	 * Write an ADD_LOGGED_MSG to the log for a all current subscriptions and instances
	 */
	void write_all_add_logged_msg();

	/**
	 * Write an ADD_LOGGED_MSG to the log for a given subscription and instance.
	 * _writer.lock() must be held when calling this.
	 */
	void write_add_logged_msg(LoggerSubscription &subscription, int instance);

	/**
	 * Create logging directory
	 * @param tt if not null, use it for the directory name
	 * @return 0 on success
	 */
	int create_log_dir(tm *tt);

	static bool file_exist(const char *filename);

	/**
	 * Get log file name with directory (create it if necessary)
	 */
	int get_log_file_name(char *file_name, size_t file_name_size);

	/**
	 * Check if there is enough free space left on the SD Card
	 * @return 0 on success, 1 if not enough space, <0 on error
	 */
	int check_free_space();

	void start_log();

	void stop_log();

	/**
	 * write the file header with file magic and timestamp.
	 */
	void write_header();

	void write_formats();

	void write_version();

	void write_info(const char *name, const char *value);
	void write_info(const char *name, int32_t value);

	void write_parameters();

	void write_changed_parameters();

	bool copy_if_updated_multi(LoggerSubscription &sub, int multi_instance, void *buffer);

	/**
	 * Write data to the logger. Waits if buffer is full until all data is written.
	 * Must be called with _writer.lock() held.
	 */
	bool write_wait(void *ptr, size_t size);

	/**
	 * Write data to the logger and handle dropouts.
	 * Must be called with _writer.lock() held.
	 * @return true if data written, false otherwise (on overflow)
	 */
	bool write(void *ptr, size_t size);

	/**
	 * Get the time for log file name
	 * @param tt returned time
	 * @param boot_time use time when booted instead of current time
	 * @return true on success, false otherwise (eg. if no gps)
	 */
	bool get_log_time(struct tm *tt, bool boot_time = false);

	/**
	 * Parse a file containing a list of uORB topics to log, calling add_topic for each
	 * @param fname name of file
	 * @return number of topics added
	 */
	int add_topics_from_file(const char *fname);

	void add_default_topics();

	static constexpr size_t 	MAX_TOPICS_NUM = 64; /**< Maximum number of logged topics */
	static constexpr unsigned	MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
	static constexpr unsigned	MAX_NO_LOGFILE = 999;	/**< Maximum number of log files */
#ifdef __PX4_POSIX_EAGLE
	static constexpr const char	*LOG_ROOT = PX4_ROOTFSDIR"/log";
#else
	static constexpr const char 	*LOG_ROOT = PX4_ROOTFSDIR"/fs/microsd/log";
#endif

	uint8_t						*_msg_buffer = nullptr;
	int						_msg_buffer_len = 0;
	bool						_task_should_exit = true;
	char 						_log_dir[LOG_DIR_LEN];
	bool						_has_log_dir = false;
	bool						_enabled = false;
	bool						_was_armed = false;
	bool						_arm_override;


	// statistics
	hrt_abstime					_start_time; ///< Time when logging started (not the logger thread)
	hrt_abstime					_dropout_start = 0; ///< start of current dropout (0 = no dropout)
	float						_max_dropout_duration = 0.f; ///< max duration of dropout [s]
	size_t						_write_dropouts = 0; ///< failed buffer writes due to buffer overflow
	size_t						_high_water = 0; ///< maximum used write buffer

	const bool 					_log_on_start;
	const bool 					_log_until_shutdown;
	const bool					_log_name_timestamp;
	Array<LoggerSubscription, MAX_TOPICS_NUM>	_subscriptions;
	LogWriter					_writer;
	uint32_t					_log_interval;
	param_t						_log_utc_offset;
	orb_advert_t					_mavlink_log_pub = nullptr;
	uint16_t					_next_topic_id; ///< id of next subscribed topic
	char						*_replay_file_name = nullptr;
};

} //namespace logger
} //namespace px4
