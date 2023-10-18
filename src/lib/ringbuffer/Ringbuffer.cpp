/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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




#include "Ringbuffer.hpp"

#include <mathlib/mathlib.h>
#include <assert.h>
#include <string.h>


Ringbuffer::~Ringbuffer()
{
	deallocate();
}

bool Ringbuffer::allocate(size_t buffer_size)
{
	assert(_ringbuffer == nullptr);

	_size = buffer_size;
	_ringbuffer = new uint8_t[_size];
	return _ringbuffer != nullptr;
}

void Ringbuffer::deallocate()
{
	delete[] _ringbuffer;
	_ringbuffer = nullptr;
	_size = 0;
}

size_t Ringbuffer::space_available() const
{
	if (_start > _end) {
		return _start - _end - 1;

	} else {
		return _start - _end - 1 + _size;
	}
}

size_t Ringbuffer::space_used() const
{
	if (_start <= _end) {
		return _end - _start;

	} else {
		// Potential wrap around.
		return _end - _start + _size;
	}
}


bool Ringbuffer::push_back(const uint8_t *buf, size_t buf_len)
{
	if (buf_len == 0 || buf == nullptr) {
		// Nothing to add, we better don't try.
		return false;
	}

	if (_start > _end) {
		// Add after end up to start, no wrap around.

		// Leave one byte free so that start don't end up the same
		// which signals empty.
		const size_t available = _start - _end - 1;

		if (available < buf_len) {
			return false;
		}

		memcpy(&_ringbuffer[_end], buf, buf_len);
		_end += buf_len;

	} else {
		// Add after end, maybe wrap around.
		const size_t available = _start - _end - 1 + _size;

		if (available < buf_len) {
			return false;
		}

		const size_t remaining_packet_len = _size - _end;

		if (buf_len > remaining_packet_len) {
			memcpy(&_ringbuffer[_end], buf, remaining_packet_len);
			_end = 0;

			memcpy(&_ringbuffer[_end], buf + remaining_packet_len, buf_len - remaining_packet_len);
			_end += buf_len - remaining_packet_len;

		} else {
			memcpy(&_ringbuffer[_end], buf, buf_len);
			_end += buf_len;
		}
	}

	return true;
}

size_t Ringbuffer::pop_front(uint8_t *buf, size_t buf_max_len)
{
	if (buf == nullptr) {
		// User needs to supply a valid pointer.
		return 0;
	}

	if (_start == _end) {
		// Empty
		return 0;
	}

	if (_start < _end) {

		// No wrap around.
		size_t to_copy_len = math::min(_end - _start, buf_max_len);

		memcpy(buf, &_ringbuffer[_start], to_copy_len);
		_start += to_copy_len;

		return to_copy_len;

	} else {
		// Potential wrap around.
		size_t to_copy_len = _end - _start + _size;

		if (to_copy_len > buf_max_len) {
			to_copy_len = buf_max_len;
		}

		const size_t remaining_buf_len = _size - _start;

		if (to_copy_len > remaining_buf_len) {

			memcpy(buf, &_ringbuffer[_start], remaining_buf_len);
			_start = 0;
			memcpy(buf + remaining_buf_len, &_ringbuffer[_start], to_copy_len - remaining_buf_len);
			_start += to_copy_len - remaining_buf_len;

		} else {
			memcpy(buf, &_ringbuffer[_start], to_copy_len);
			_start += to_copy_len;
		}

		return to_copy_len;
	}
}
