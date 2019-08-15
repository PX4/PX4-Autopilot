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

#include <stdlib.h>

namespace px4
{

template <class T, size_t N>
class Array
{

public:

	bool push_back(const T &x)
	{
		if (_size == N) {
			_overflow = true;
			return false;

		} else {
			_items[_size] = x;
			++_size;
			return true;
		}
	}

	void remove(unsigned idx)
	{
		if (idx < _size) {
			--_size;

			for (unsigned i = idx; i < _size; ++i) {
				_items[i] = _items[i + 1];
			}
		}
	}

	void erase(T *item)
	{
		if (item - _items < static_cast<int>(_size)) {
			--_size;

			for (T *it = item; it != &_items[_size]; ++it) {
				*it = *(it + 1);
			}
		}
	}

	T &operator[](size_t n) { return _items[n]; }
	const T &operator[](size_t n) const { return _items[n]; }

	T &at(size_t n) { return _items[n]; }
	const T &at(size_t n) const { return _items[n]; }

	size_t size() const { return _size; }
	size_t max_size() const { return N; }
	size_t capacity() const { return N; }

	bool empty() const { return _size == 0; }

	bool is_overflowed() { return _overflow; }

	T *begin() { return &_items[0]; }
	T *end() { return &_items[_size]; }

	const T *begin() const { return &_items[0]; }
	const T *end() const { return &_items[_size]; }

private:
	T        _items[N];
	size_t      _size{0};
	bool        _overflow{false};
};

}
