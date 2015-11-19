/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file RingBuffer.h
 * @author Roman Bapst <bapstroman@gmail.com>
 * Template RingBuffer.
 */

template <typename data_type>
class RingBuffer
{
public:
	RingBuffer() {_buffer = NULL;};
	~RingBuffer() {delete _buffer;}

	bool allocate(unsigned size)
	{
		if (size == 0) {
			return false;
		}

		_buffer = new data_type[size];

		if (_buffer == NULL) {
			return false;
		}

		_size = size;
		return true;
	}

	inline void push(data_type sample)
	{
		_buffer[_head] = sample;
		_head = (_head + 1) % _size;

		// move tail if we overwrite it
		if (_head == _tail && _size > 1) {
			_tail = (_tail + 1) % _size;
		}
	}

	inline data_type get_oldest()
	{
		return _buffer[_tail];
	}

	inline bool pop_first_older_than(uint64_t timestamp, data_type &sample)
	{
		// start looking from oldest data of buffer
		for (unsigned i = 0; i < _size; i++) {
			unsigned index = (_tail + i) % _size;

			if (timestamp >= _buffer[index].time_us) {
				sample = _buffer[_tail + i];
				// now we can set the tail to the item
				// which comes after the one we removed
				_tail = (_tail + i + 1);
				return true;
			}

			if (index == _head) {
				// we have reached the head and haven't found anything
				return false;
			}
		}

		return false;
	}

	data_type &operator[](unsigned index)
	{
		return _buffer[index];
	}

private:
	data_type *_buffer;
	unsigned _head, _tail, _size;
};