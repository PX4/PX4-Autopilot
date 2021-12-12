/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/sem.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>

#ifdef BOARD_ENABLE_CONSOLE_BUFFER
#ifndef BOARD_CONSOLE_BUFFER_SIZE
# define BOARD_CONSOLE_BUFFER_SIZE (1024*4) // default buffer size
#endif

static ssize_t console_buffer_write(struct file *filep, const char *buffer, size_t buflen);


class ConsoleBuffer
{
public:

	void write(const char *buffer, size_t len);

	void print(bool follow);

	int size();

	int read(char *buffer, int buffer_length, int *offset);

private:
	void		lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void		unlock() { px4_sem_post(&_lock); }

	char _buffer[BOARD_CONSOLE_BUFFER_SIZE];
	int _head{0};
	int _tail{0};
	px4_sem_t _lock = SEM_INITIALIZER(1);
};

void ConsoleBuffer::print(bool follow)
{
	// print to stdout, but with a buffer in between to avoid nested locking and potential dead-locks
	// (which could happen in combination with the MAVLink shell: dmesg writing to the pipe waiting
	// mavlink to read, while mavlink calls printf, waiting for the console lock)
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

			::write(1, buffer, read_size);

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
	lock(); // same rule as for printf: this cannot be used from IRQ handlers

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


void px4_console_buffer_print(bool follow)
{
	g_console_buffer.print(follow);
}

ssize_t console_buffer_write(struct file *filep, const char *buffer, size_t len)
{
	g_console_buffer.write(buffer, len);

	// stderr still points to our original console and is available from everywhere, so we output to that.
	// We could use up_putc() as well, but it is considerably less efficient.
	// The drawback here is that a module writing to stderr bypasses the console buffer.
	write(2, buffer, len);
	fsync(2);

	return len;
}

static const struct file_operations g_console_buffer_fops = {
	NULL,         /* open */
	NULL,         /* close */
	NULL,         /* read */
	console_buffer_write, /* write */
	NULL,         /* seek */
	NULL          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, NULL        /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
	, NULL        /* unlink */
#endif
};

int px4_console_buffer_init()
{
	return register_driver(CONSOLE_BUFFER_DEVICE, &g_console_buffer_fops, 0666, NULL);
}

int px4_console_buffer_size()
{
	return g_console_buffer.size();
}

int px4_console_buffer_read(char *buffer, int buffer_length, int *offset)
{
	return g_console_buffer.read(buffer, buffer_length, offset);
}

#endif /* BOARD_ENABLE_CONSOLE_BUFFER */
