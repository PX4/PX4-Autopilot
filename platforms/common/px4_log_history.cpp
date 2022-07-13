/****************************************************************************
 *
 * Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/log_history.h>

#if defined(BOARD_ENABLE_LOG_HISTORY)
#include <string.h>

void LogHistory::print(FILE *out)
{
	const int buffer_length = 512;
	char buffer[buffer_length];
	int offset = -1;

	int total_size_read = 0;

	do {
		int read_size = read(buffer, buffer_length, &offset);

		if (read_size <= 0) {
			break;
		}

		fwrite(buffer, 1, read_size, out);

		if (read_size < buffer_length) {
			break;
		}

		total_size_read += read_size;
	} while (total_size_read < BOARD_LOG_HISTORY_SIZE);
}

// Log a null terminated string to the circular history buffer.
void LogHistory::write(const char *buffer)
{
	lock(); // same rule as for printf: this cannot be used from IRQ handlers

	int i = 0;

	while (buffer[i] != '\0') {

		_log_history[_tail] = buffer[i++];
		_tail = (_tail + 1) % BOARD_LOG_HISTORY_SIZE;

		if (_tail == _head) {
			_head = (_head + 1) % BOARD_LOG_HISTORY_SIZE;
		}
	}

	unlock();
}

int LogHistory::size()
{
	lock();
	int size;

	if (_head <= _tail) {
		size = _tail - _head;

	} else {
		size = BOARD_LOG_HISTORY_SIZE - (_head - _tail);
	}

	unlock();
	return size;
}

int LogHistory::read(char *buffer, int buffer_length, int *offset)
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

		memcpy(buffer, _log_history + *offset, size);

	} else if (_tail < *offset) {
		size = BOARD_LOG_HISTORY_SIZE - *offset;

		if (size > buffer_length) {
			size = buffer_length;
		}

		memcpy(buffer, _log_history + *offset, size);
		buffer += size;
		buffer_length -= size;

		int size_secondary = _tail;

		if (size_secondary > buffer_length) {
			size_secondary = buffer_length;
		}

		if (size_secondary > 0) {
			memcpy(buffer, _log_history, size_secondary);
			size += size_secondary;
		}
	}

	unlock();
	*offset = (*offset + size) % BOARD_LOG_HISTORY_SIZE;
	return size;
}

#endif
