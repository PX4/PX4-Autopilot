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
#include "log_writer_mavlink.h"

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

	LogWriter(Backend configured_backend, size_t file_buffer_size);
	~LogWriter();

	bool init();

	Backend backend() const { return _backend; }

	/** stop all running threads and wait for them to exit */
	void thread_stop();

	void start_log_file(LogType type, const char *filename);

	void stop_log_file(LogType type);

	void start_log_mavlink();

	void stop_log_mavlink();

	/**
	 * whether logging is currently active or not (any of the selected backends).
	 */
	bool is_started(LogType type) const;

	/**
	 * whether logging is currently active or not for a specific backend.
	 */
	bool is_started(LogType type, Backend query_backend) const;

	/**
	 * Write a single ulog message (including header). The caller must call lock() before calling this.
	 * @param dropout_start timestamp when lastest dropout occured. 0 if no dropout at the moment.
	 * @return 0 on success (or if no logging started),
	 *         -1 if not enough space in the buffer left (file backend), -2 mavlink backend failed
	 *  add type -> pass through, but not to mavlink if mission log
	 */
	int write_message(LogType type, void *ptr, size_t size, uint64_t dropout_start = 0);

	/**
	 * Select a backend, so that future calls to write_message() only write to the selected
	 * sel_backend, until unselect_write_backend() is called.
	 * @param backend
	 */
	void select_write_backend(Backend sel_backend);
	void unselect_write_backend() { select_write_backend(BackendAll); }

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

	size_t get_total_written_file(LogType type) const
	{
		if (_log_writer_file) { return _log_writer_file->get_total_written(type); }

		return 0;
	}

	size_t get_buffer_size_file(LogType type) const
	{
		if (_log_writer_file) { return _log_writer_file->get_buffer_size(type); }

		return 0;
	}

	size_t get_buffer_fill_count_file(LogType type) const
	{
		if (_log_writer_file) { return _log_writer_file->get_buffer_fill_count(type); }

		return 0;
	}

	pthread_t thread_id_file() const
	{
		if (_log_writer_file) { return _log_writer_file->thread_id(); }

		return (pthread_t)0;
	}


	/**
	 * Indicate to the underlying backend whether future write_message() calls need a reliable
	 * transfer. Needed for header integrity.
	 */
	void set_need_reliable_transfer(bool need_reliable)
	{
		if (_log_writer_file) { _log_writer_file->set_need_reliable_transfer(need_reliable); }

		if (_log_writer_mavlink) { _log_writer_mavlink->set_need_reliable_transfer(need_reliable); }
	}

	bool need_reliable_transfer() const
	{
		if (_log_writer_file) { return _log_writer_file->need_reliable_transfer(); }

		if (_log_writer_mavlink) { return _log_writer_mavlink->need_reliable_transfer(); }

		return false;
	}

#if defined(PX4_CRYPTO)
	void set_encryption_parameters(px4_crypto_algorithm_t algorithm, uint8_t key_idx,  uint8_t exchange_key_idx)
	{
		if (_log_writer_file) { _log_writer_file->set_encryption_parameters(algorithm, key_idx, exchange_key_idx); }
	}
#endif
private:

	LogWriterFile *_log_writer_file = nullptr;
	LogWriterMavlink *_log_writer_mavlink = nullptr;

	LogWriterFile *_log_writer_file_for_write =
		nullptr; ///< pointer that is used for writing, to temporarily select write backends
	LogWriterMavlink *_log_writer_mavlink_for_write = nullptr;

	const Backend _backend;
};


}
}
