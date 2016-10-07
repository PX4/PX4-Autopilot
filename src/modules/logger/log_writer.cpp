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

#include "log_writer.h"

namespace px4
{
namespace logger
{

LogWriter::LogWriter(Backend backend, size_t file_buffer_size)
	: _backend(backend)
{
	if (backend & BackendFile) {
		_log_writer_file = new LogWriterFile(file_buffer_size);

		if (!_log_writer_file) {
			PX4_ERR("LogWriterFile allocation failed");
		}
	}
}

bool LogWriter::init()
{
	if (_log_writer_file) {
		if (!_log_writer_file->init()) {
			return false;
		}
	}

	return true;
}

LogWriter::~LogWriter()
{
	if (_log_writer_file) {
		delete(_log_writer_file);
	}
}

bool LogWriter::is_started() const
{
	bool ret = false;

	if (_log_writer_file) {
		ret = _log_writer_file->is_started();
	}

	return ret;
}

bool LogWriter::is_started_file() const
{
	if (_log_writer_file) {
		return _log_writer_file->is_started();
	}

	return false;
}

void LogWriter::start_log_file(const char *filename)
{
	if (_log_writer_file) {
		_log_writer_file->start_log(filename);
	}
}

void LogWriter::stop_log_file()
{
	if (_log_writer_file) {
		_log_writer_file->stop_log();
	}
}

int LogWriter::thread_start(pthread_t &thread)
{
	if (_log_writer_file) {
		return _log_writer_file->thread_start(thread);
	}

	return 0;
}

void LogWriter::thread_stop()
{
	if (_log_writer_file) {
		_log_writer_file->thread_stop();
	}
}

bool LogWriter::write(void *ptr, size_t size, uint64_t dropout_start)
{
	if (_log_writer_file) {
		return _log_writer_file->write(ptr, size, dropout_start);
	}

	return true;
}

}
}
