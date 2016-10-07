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

#include "log_writer_file.h"

namespace px4
{
namespace logger
{

/**
 * @class LogWriter
 * Manages starting, stopping & writing of logged data using the configured backend.
 */
class LogWriter
{
public:

	/** bitfield to specify a backend */
	typedef uint8_t Backend;
	static constexpr Backend BackendFile = 1 << 0;
	static constexpr Backend BackendMavlink = 1 << 1;
	static constexpr Backend BackendAll = BackendFile | BackendMavlink;

	LogWriter(Backend backend, size_t file_buffer_size, unsigned int queue_size);
	~LogWriter();

	bool init();

	Backend backend() const { return _backend; }

	/** stop all running threads and wait for them to exit */
	void thread_stop();

	void start_log_file(const char *filename);

	void stop_log_file();

	/**
	 * whether logging is currently active or not (any of the selected backends).
	 */
	bool is_started() const;

	/**
	 * whether logging is currently active or not for a specific backend.
	 */
	bool is_started(Backend backend) const;

	/**
	 * Write data to be logged. The caller must call lock() before calling this.
	 * @param dropout_start timestamp when lastest dropout occured. 0 if no dropout at the moment.
	 * @return true on success, false if not enough space in the buffer left
	 */
	bool write(void *ptr, size_t size, uint64_t dropout_start = 0);


	/* file logging methods */

	void lock()
	{
		if (_log_writer_file) { _log_writer_file->lock(); }
	}

	void unlock()
	{
		if (_log_writer_file) { _log_writer_file->unlock(); }
	}

	void notify()
	{
		if (_log_writer_file) { _log_writer_file->notify(); }
	}

	size_t get_total_written_file() const
	{
		if (_log_writer_file) { return _log_writer_file->get_total_written(); }

		return 0;
	}

	size_t get_buffer_size_file() const
	{
		if (_log_writer_file) { return _log_writer_file->get_buffer_size(); }

		return 0;
	}

	size_t get_buffer_fill_count_file() const
	{
		if (_log_writer_file) { return _log_writer_file->get_buffer_fill_count(); }

		return 0;
	}

private:

	LogWriterFile *_log_writer_file = nullptr;
	const Backend _backend;
};


}
}
