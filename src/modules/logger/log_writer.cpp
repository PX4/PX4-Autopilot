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

LogWriter::LogWriter(Backend configured_backend, size_t file_buffer_size)
	: _backend(configured_backend)
{
	if (configured_backend & BackendFile) {
		_log_writer_file_for_write = _log_writer_file = new LogWriterFile(file_buffer_size);

		if (!_log_writer_file) {
			PX4_ERR("LogWriterFile allocation failed");
		}
	}

	if (configured_backend & BackendMavlink) {
		_log_writer_mavlink_for_write = _log_writer_mavlink = new LogWriterMavlink();

		if (!_log_writer_mavlink) {
			PX4_ERR("LogWriterMavlink allocation failed");
		}
	}
}

bool LogWriter::init()
{
	if (_log_writer_file) {
		if (!_log_writer_file->init()) {
			PX4_ERR("alloc failed");
			return false;
		}

		int ret = _log_writer_file->thread_start();

		if (ret) {
			PX4_ERR("failed to create writer thread (%i)", ret);
			return false;
		}
	}

	if (_log_writer_mavlink) {
		if (!_log_writer_mavlink->init()) {
			PX4_ERR("mavlink init failed");
			return false;
		}
	}

	return true;
}

LogWriter::~LogWriter()
{
	if (_log_writer_file) {
		delete (_log_writer_file);
	}

	if (_log_writer_mavlink) {
		delete (_log_writer_mavlink);
	}
}

bool LogWriter::is_started(LogType type) const
{
	bool ret = false;

	if (_log_writer_file) {
		ret = _log_writer_file->is_started(type);
	}

	if (_log_writer_mavlink && type == LogType::Full) {
		ret = ret || _log_writer_mavlink->is_started();
	}

	return ret;
}

bool LogWriter::is_started(LogType type, Backend query_backend) const
{
	if (query_backend == BackendFile && _log_writer_file) {
		return _log_writer_file->is_started(type);
	}

	if (query_backend == BackendMavlink && _log_writer_mavlink && type == LogType::Full) {
		return _log_writer_mavlink->is_started();
	}

	return false;
}

bool LogWriter::start_log_file(LogType type, const char *filename)
{
	if (_log_writer_file) {
		return _log_writer_file->start_log(type, filename);
	}

	return false;
}

void LogWriter::stop_log_file(LogType type)
{
	if (_log_writer_file) {
		_log_writer_file->stop_log(type);
	}
}

void LogWriter::start_log_mavlink()
{
	if (_log_writer_mavlink) {
		_log_writer_mavlink->start_log();
	}
}

void LogWriter::stop_log_mavlink()
{
	if (_log_writer_mavlink) {
		_log_writer_mavlink->stop_log();
	}
}

void LogWriter::thread_stop()
{
	if (_log_writer_file) {
		_log_writer_file->thread_stop();
	}
}

int LogWriter::write_message(LogType type, void *ptr, size_t size, uint64_t dropout_start)
{
	int ret_file = 0, ret_mavlink = 0;

	if (_log_writer_file_for_write) {
		ret_file = _log_writer_file_for_write->write_message(type, ptr, size, dropout_start);
	}

	if (_log_writer_mavlink_for_write && type == LogType::Full) {
		ret_mavlink = _log_writer_mavlink_for_write->write_message(ptr, size);
	}

	// file backend errors takes precedence
	if (ret_file != 0) {
		return ret_file;
	}

	return ret_mavlink;
}

void LogWriter::select_write_backend(Backend sel_backend)
{
	if (sel_backend & BackendFile) {
		_log_writer_file_for_write = _log_writer_file;

	} else {
		_log_writer_file_for_write = nullptr;
	}

	if (sel_backend & BackendMavlink) {
		_log_writer_mavlink_for_write = _log_writer_mavlink;

	} else {
		_log_writer_mavlink_for_write = nullptr;
	}
}

bool LogWriter::had_file_write_error() const
{
	if (_log_writer_file) {
		return _log_writer_file->had_write_error();
	}

	return false;
}

}
}
