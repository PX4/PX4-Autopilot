/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/console_buffer.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#ifdef BOARD_ENABLE_CONSOLE_BUFFER
#ifndef BOARD_CONSOLE_BUFFER_SIZE
# define BOARD_CONSOLE_BUFFER_SIZE (1024*10) // default buffer size
#endif

class ConsoleBuffer
{
public:

	void write(const char *buffer, size_t len);

	void print(bool follow);

	int size();

	int read(char *buffer, int buffer_length, int *offset);

	px4_sem_t _lock;
	px4_sem_t _initialized;

private:
	void lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void unlock() { px4_sem_post(&_lock); }

	char _buffer[BOARD_CONSOLE_BUFFER_SIZE];
	int _head{0};
	int _tail{0};
};

void ConsoleBuffer::print(bool follow)
{
	const int buffer_length = 512;
	char buffer[buffer_length];
	int offset = -1;

	do {
		// print the full buffer or what's newly added
		int total_size_read = 0;

		do {
			int read_size = read(buffer, buffer_length, &offset);

			if (read_size <= 0) {
				break;
			}

			if (::write(STDERR_FILENO, buffer, read_size) < 0) {
				break;
			}

			if (read_size < buffer_length) {
				break;
			}

			total_size_read += read_size;
		} while (total_size_read < BOARD_CONSOLE_BUFFER_SIZE);


		if (follow) {
			usleep(10000);
		}
	} while (follow);
}

void ConsoleBuffer::write(const char *buffer, size_t len)
{
	lock();

	for (size_t i = 0; i < len; ++i) {
		_buffer[_tail] = buffer[i];
		_tail = (_tail + 1) % BOARD_CONSOLE_BUFFER_SIZE;

		if (_tail == _head) {
			_head = (_head + 1) % BOARD_CONSOLE_BUFFER_SIZE;
		}
	}

	unlock();
}

int ConsoleBuffer::size()
{
	lock();
	int size;

	if (_head <= _tail) {
		size = _tail - _head;

	} else {
		size = BOARD_CONSOLE_BUFFER_SIZE - (_head - _tail);
	}

	unlock();
	return size;
}

int ConsoleBuffer::read(char *buffer, int buffer_length, int *offset)
{
	lock();

	if (*offset == -1) {
		*offset = _head;
	}

	int size = 0;

	if (*offset < _tail) {
		size = _tail - *offset;

		if (size > buffer_length) {
			size = buffer_length;
		}

		memcpy(buffer, _buffer + *offset, size);

	} else if (_tail < *offset) {
		size = BOARD_CONSOLE_BUFFER_SIZE - *offset;

		if (size > buffer_length) {
			size = buffer_length;
		}

		memcpy(buffer, _buffer + *offset, size);
		buffer += size;
		buffer_length -= size;

		int size_secondary = _tail;

		if (size_secondary > buffer_length) {
			size_secondary = buffer_length;
		}

		if (size_secondary > 0) {
			memcpy(buffer, _buffer, size_secondary);
			size += size_secondary;
		}
	}

	unlock();
	*offset = (*offset + size) % BOARD_CONSOLE_BUFFER_SIZE;
	return size;
}

static ConsoleBuffer g_console_buffer;

int px4_console_buffer_thread(int argc, char *argv[])
{
	int ret = PX4_OK;
	int orig_stdout_fd = dup(STDOUT_FILENO);

	// Check if the console_buf named pipe already exists. If so delete it.
	struct stat stats;

	if (stat(CONSOLE_BUFFER_DEVICE, &stats) < 0) {

		if (errno != ENOENT) {
			perror("Console buffer stat failed");
			ret = PX4_ERROR;
			goto out;
		}

	} else  if (unlink(CONSOLE_BUFFER_DEVICE) < 0) {

		perror("Console buffer unlink failed");
		ret = PX4_ERROR;
		goto out;
	}

	// Create console buffer named pipe
	if ((mkfifo(CONSOLE_BUFFER_DEVICE, 0666)) < 0) {
		perror("Console buffer could not be created");
		ret = PX4_ERROR;
		goto out;
	}

	// Open NONBLOCK so it returns even though the write side is not yet open
	int buffered_stdout;

	if ((buffered_stdout = open(CONSOLE_BUFFER_DEVICE, O_RDONLY | O_NONBLOCK)) < 0) {
		perror("Failed to open console buffer buffered_stdout");
		ret = PX4_ERROR;
		goto out;
	}

	fcntl(buffered_stdout, F_SETFL, fcntl(buffered_stdout, F_GETFL) & ~O_NONBLOCK);

	int console_buf;

	if ((console_buf = open(CONSOLE_BUFFER_DEVICE, O_WRONLY)) < 0) {
		perror("Failed to open console buffer pipe in");
		ret = PX4_ERROR;
		goto out;
	}

	// Set all stdout to go to the console buffer pipe
	if (dup2(console_buf, STDOUT_FILENO) < 0) {
		perror("Console buf dup2 failed");
		ret = PX4_ERROR;
		goto out;

	}

	close(console_buf);

out:
	// Signal initalization finished
	px4_sem_post(&g_console_buffer._initialized);

	if (ret == PX4_ERROR) {
		return PX4_ERROR;
	}

	// Check for input sent to the console buffer pipe. Save to the local console buffer
	// as well as output it to the local terminal
	while (1) {

		char buffer[512];
		size_t len;

		if ((len = read(buffered_stdout, buffer, sizeof(buffer))) <= 0) {
			break; //EOF or ERROR
		}

		// Send all the stdout data to the local terminal as well as the console buffer
		g_console_buffer.write(buffer, len);

		if (write(orig_stdout_fd, buffer, len) <= 0) {
			perror("run_console_buf write stdout");
			break;
		}
	}

	// Restore stdout and close the named pipe
	dup2(orig_stdout_fd, STDOUT_FILENO);
	close(orig_stdout_fd);
	close(buffered_stdout);
	unlink(CONSOLE_BUFFER_DEVICE);

	return ret;
}

px4_task_t _task;

int px4_console_buffer_init()
{
	px4_sem_init(&g_console_buffer._lock, 0, 1);
	px4_sem_init(&g_console_buffer._initialized, 0, 0);

	// Thread priority set one higher than shell thread to capture entire command in console buffer
	// before being executed
	_task = px4_task_spawn_cmd("console_buffer",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_DEFAULT + 1,
				   8192,
				   &px4_console_buffer_thread,
				   nullptr);

	if (_task < 0) {
		PX4_ERR("Failed to create console_buffer task");
		return -1;
	}

	// ensure thread is initialized before returning
	while (px4_sem_wait(&g_console_buffer._initialized) != 0);

	return 0;
}

int px4_console_buffer_size()
{
	return g_console_buffer.size();
}

int px4_console_buffer_read(char *buffer, int buffer_length, int *offset)
{
	return g_console_buffer.read(buffer, buffer_length, offset);
}

void px4_console_buffer_print(bool follow)
{
	g_console_buffer.print(follow);
}

#endif /* BOARD_ENABLE_CONSOLE_BUFFER */
