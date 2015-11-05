#include "log_writer.h"
#include <fcntl.h>
#include <string.h>

namespace px4 {
namespace logger {

LogWriter::LogWriter(uint8_t *buffer, size_t buffer_size) :
		_buffer(buffer),
		_buffer_size(buffer_size),
		_min_write_chunk(_buffer_size / 8) {
	pthread_mutex_init(&_mtx, nullptr);
	pthread_cond_init(&_cv, nullptr);
}

void LogWriter::start_log(const char *filename) {
	::strncpy(_filename, filename, sizeof(_filename));
	// Clear buffer and counters
	_head = 0;
	_count = 0;
	_total_written = 0;
	_should_run = true;
	notify();
}

void LogWriter::stop_log() {
	_should_run = false;
	notify();
}

pthread_t LogWriter::thread_start() {
	pthread_attr_t thr_attr;
	pthread_attr_init(&thr_attr);

	sched_param param;
	/* low priority, as this is expensive disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 40;
	(void)pthread_attr_setschedparam(&thr_attr, &param);

	pthread_attr_setstacksize(&thr_attr, 2048);

	pthread_t thr;
	if (0 != pthread_create(&thr, &thr_attr, &LogWriter::run_helper, this)) {
		warnx("error creating logwriter thread");
	}
	return thr;
}

void * LogWriter::run_helper(void *context) {
	prctl(PR_SET_NAME, "log_writer", 0);
	reinterpret_cast<LogWriter *>(context)->run();
	return nullptr;
}

void LogWriter::run() {
	while (true) {
		// Outer endless loop, start new file each time
		// _filename must be set before setting _should_run = true

		// Wait for _should_run flag
		while (true) {
			bool start = false;
			pthread_mutex_lock(&_mtx);
			pthread_cond_wait(&_cv, &_mtx);
			start = _should_run;
			pthread_mutex_unlock(&_mtx);
			if (start) {
				break;
			}
		}

		int poll_count = 0;
		int written = 0;

		_fd = ::open(_filename, O_CREAT | O_WRONLY | O_DSYNC, PX4_O_MODE_666);
		if (_fd < 0) {
			warn("can't open log file %s", _filename);
			_should_run = false;
			continue;
		}
		warnx("started, log file: %s", _filename);

		_should_run = true;

		while (true) {
			size_t available = 0;
			void *read_ptr = nullptr;
			bool is_part = false;

			pthread_mutex_lock(&_mtx);
			mark_read(written);
			written = 0;
			while (true) {
				available = get_read_ptr(&read_ptr, &is_part);
				if ((available > _min_write_chunk) || is_part || !_should_run) {
					break;
				}
				pthread_cond_wait(&_cv, &_mtx);
			}
			pthread_mutex_unlock(&_mtx);

			if (available > 0) {
				written = ::write(_fd, read_ptr, available);
				if (++poll_count >= 10) {
					::fsync(_fd);
					poll_count = 0;
				}

				if (written < 0) {
					warn("error writing log file");
					_should_run = false;
					break;
				}

				_total_written += written;
			}

			if (!_should_run && written == static_cast<int>(available) && !is_part) {
				// Stop only when all data written
				break;
			}
		}

		_head = 0;
		_count = 0;

		int res = ::close(_fd);
		if (res) {
			warn("error closing log file");
		}

		warnx("stopped, bytes written: %i", _total_written);
	}
}

bool LogWriter::write(void *ptr, size_t size) {
	// Bytes available to write
	size_t available = _buffer_size - _count;

	if (size > available) {
		// buffer overflow
		return false;
	}

	size_t n = _buffer_size - _head;	// bytes to end of the buffer

	uint8_t *buffer_c = reinterpret_cast<uint8_t *>(ptr);

	if (size > n) {
		// Message goes over the end of the buffer
		memcpy(&(_buffer[_head]), buffer_c, n);
		_head = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	size_t p = size - n;	// number of bytes to write
	memcpy(&(_buffer[_head]), &(buffer_c[n]), p);
	_head = (_head + p) % _buffer_size;
	_count += size;
	return true;
}

size_t LogWriter::get_read_ptr(void **ptr, bool *is_part) {
	// bytes available to read
	int read_ptr = _head - _count;
	if (read_ptr < 0) {
		read_ptr += _buffer_size;
		*ptr = &_buffer[read_ptr];
		*is_part = true;
		return _buffer_size - read_ptr;
	} else {
		*ptr = &_buffer[read_ptr];
		*is_part = false;
		return _count;
	}
}

}
}
