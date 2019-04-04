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

#include <px4_config.h>
#include <px4_console_buffer.h>
#include <px4_defines.h>
#include <px4_sem.h>
#include <pthread.h>

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

private:
	void		lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void		unlock() { px4_sem_post(&_lock); }

	char _buffer[BOARD_CONSOLE_BUFFER_SIZE];
	int _head{0};
	int _tail{0};
	px4_sem_t _lock = SEM_INITIALIZER(1);
	bool _is_printing{false};
	pthread_t _printing_task;
};

void ConsoleBuffer::print(bool follow)
{
	lock();
	_printing_task = pthread_self();
	int i = _head;

	while (true) {
		_is_printing = true;

		if (i < _tail) {
			::write(1, _buffer + i, _tail - i);

		} else if (_tail < i) {
			::write(1, _buffer + i, BOARD_CONSOLE_BUFFER_SIZE - i);
			::write(1, _buffer, _tail);
		}

		i = _tail;

		_is_printing = false;

		if (follow) {
			unlock();
			usleep(10000);
			lock();

		} else {
			break;
		}
	}

	unlock();
}

void ConsoleBuffer::write(const char *buffer, size_t len)
{
	if (_is_printing && pthread_self() == _printing_task) { // avoid adding to the buffer while we are printing it
		return;
	}

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

#else

int px4_console_buffer_init()
{
	return 0;
}
#endif /* BOARD_ENABLE_CONSOLE_BUFFER */
