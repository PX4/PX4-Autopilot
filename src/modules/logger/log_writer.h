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

#include <px4.h>
#include <stdint.h>
#include <pthread.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>

namespace px4
{
namespace logger
{

class LogWriter
{
public:
	LogWriter(size_t buffer_size);
	~LogWriter();

	bool init();

	/**
	 * start the thread
	 * @param thread will be set to the created thread on success
	 * @return 0 on success, error number otherwise (@see pthread_create)
	 */
	int thread_start(pthread_t &thread);

	void thread_stop();

	void start_log(const char *filename);

	void stop_log();

	/**
	 * Write data to be logged. The caller must call lock() before calling this.
	 * @param dropout_start timestamp when lastest dropout occured. 0 if no dropout at the moment.
	 * @return true on success, false if not enough space in the buffer left
	 */
	bool write(void *ptr, size_t size, uint64_t dropout_start = 0);

	void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	void notify()
	{
		pthread_cond_broadcast(&_cv);
	}

	size_t get_total_written() const
	{
		return _total_written;
	}

	size_t get_buffer_size() const
	{
		return _buffer_size;
	}

	size_t get_buffer_fill_count() const
	{
		return _count;
	}

private:
	static void *run_helper(void *);

	void run();

	size_t get_read_ptr(void **ptr, bool *is_part);

	void mark_read(size_t n)
	{
		_count -= n;
	}

	/**
	 * Write to the buffer but assuming there is enough space
	 */
	inline void write_no_check(void *ptr, size_t size);

	/* 512 didn't seem to work properly, 4096 should match the FAT cluster size */
	static constexpr size_t	_min_write_chunk = 4096;

	int			_fd = -1;
	uint8_t 	*_buffer = nullptr;
	const size_t	_buffer_size;
	size_t			_head = 0; ///< next position to write to
	size_t			_count = 0; ///< number of bytes in _buffer to be written
	size_t		_total_written = 0;
	bool		_should_run = false;
	bool		_running = false;
	bool 		_exit_thread = false;
	pthread_mutex_t		_mtx;
	pthread_cond_t		_cv;
	perf_counter_t _perf_write;
	perf_counter_t _perf_fsync;
};

}
}
