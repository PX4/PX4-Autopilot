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
#include <uORB/topics/vehicle_status.h>
#include <version/version.h>
#include <systemlib/git_version.h>

extern "C" __EXPORT int logger_main(int argc, char *argv[]);

#define TRY_SUBSCRIBE_INTERVAL 1000*1000	// interval in microseconds at which we try to subscribe to a topic
// if we haven't succeeded before

namespace px4
{
namespace logger
{

struct LoggerSubscription {
	int fd[ORB_MULTI_MAX_INSTANCES];
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
	Logger(size_t buffer_size, unsigned log_interval, bool log_on_start);

	~Logger();

	int add_topic(const orb_metadata *topic);

	int add_topic(const char *name, unsigned interval);

	static int start(char *const *argv);

	static void usage(const char *reason);

	void status();

private:
	static void run_trampoline(int argc, char *argv[]);

	void run();

	int create_log_dir();

	static bool file_exist(const char *filename);

	int get_log_file_name(char *file_name, size_t file_name_size);

	void start_log();

	void stop_log();

	void write_formats();

	void write_version();

	void write_info(const char *name, const char *value);

	void write_parameters();

	void write_changed_parameters();

	bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer, uint64_t *time_last_checked);

	/**
	 * Write data to the logger. Waits if buffer is full until all data is written.
	 * Must be called with _writer.lock() held.
	 */
	bool write_wait(void *ptr, size_t size);

	static constexpr size_t 	MAX_TOPICS_NUM = 128; /**< Maximum number of logged topics */
	static constexpr unsigned	MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
	static constexpr unsigned	MAX_NO_LOGFILE = 999;	/**< Maximum number of log files */
#ifdef __PX4_POSIX_EAGLE
	static constexpr const char	*LOG_ROOT = PX4_ROOTFSDIR"/log";
#else
	static constexpr const char 	*LOG_ROOT = PX4_ROOTFSDIR"/fs/microsd/log";
#endif

	bool						_task_should_exit = true;
	char 						_log_dir[64];
	bool						_enabled = false;

	// statistics
	hrt_abstime					_start_time; ///< Time when logging started (not the logger thread)
	hrt_abstime					_dropout_start = 0; ///< start of current dropout (0 = no dropout)
	float						_max_dropout_duration = 0.f; ///< max duration of dropout [s]
	size_t						_write_dropouts = 0; ///< failed buffer writes due to buffer overflow
	size_t						_high_water = 0; ///< maximum used write buffer

	bool 						_log_on_start;
	Array<LoggerSubscription, MAX_TOPICS_NUM>	_subscriptions;
	LogWriter					_writer;
	uint32_t					_log_interval;
};

Logger *logger_ptr = nullptr;
int		logger_task = -1;
pthread_t _writer_thread;

}
}
