/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file ringbuffer.h
 *
 * A simple ringbuffer template.
 */

#pragma once

template<typename T>
class RingBuffer {
public:
	RingBuffer(unsigned size);
	virtual ~RingBuffer();

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(T &val);

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(const T &val);

	/**
	 * Get an item from the buffer.
	 *
	 * @param val		Item that was gotten
	 * @return		true if an item was got, false if the buffer was empty.
	 */
	bool			get(T &val);

	/**
	 * Get an item from the buffer (scalars only).
	 *
	 * @return		The value that was fetched, or zero if the buffer was
	 *			empty.
	 */
	T			get(void);

	/*
	 * Get the number of slots free in the buffer.
	 *
	 * @return		The number of items that can be put into the buffer before
	 *			it becomes full.
	 */
	unsigned		space(void);

	/*
	 * Get the number of items in the buffer.
	 *
	 * @return		The number of items that can be got from the buffer before
	 *			it becomes empty.
	 */
	unsigned		count(void);

	/*
	 * Returns true if the buffer is empty.
	 */
	bool			empty() { return _tail == _head; }

	/*
	 * Returns true if the buffer is full.
	 */
	bool			full() { return _next(_head) == _tail; }

	/*
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	unsigned		size() { return (_buf != nullptr) ? _size : 0; }

	/*
	 * Empties the buffer.
	 */
	void			flush() { _head = _tail = _size; }

private:
	T			*const _buf;	
	const unsigned		_size;
	volatile unsigned	_head;	/**< insertion point */
	volatile unsigned	_tail;	/**< removal point */

	unsigned		_next(unsigned index);
};

template <typename T>
RingBuffer<T>::RingBuffer(unsigned with_size) :
	_buf(new T[with_size + 1]),
	_size(with_size),
 	_head(with_size),
 	_tail(with_size)
{}

template <typename T>
RingBuffer<T>::~RingBuffer()
{
	if (_buf != nullptr)
		delete[] _buf;
}

template <typename T>
bool RingBuffer<T>::put(T &val) 
{
	unsigned next = _next(_head);
	if (next != _tail) {
		_buf[_head] = val;
		_head = next;
		return true;
	} else {
		return false;
	}
}

template <typename T>
bool RingBuffer<T>::put(const T &val) 
{
	unsigned next = _next(_head);
	if (next != _tail) {
		_buf[_head] = val;
		_head = next;
		return true;
	} else {
		return false;
	}
}

template <typename T>
bool RingBuffer<T>::get(T &val) 
{
	if (_tail != _head) {
		val = _buf[_tail];
		_tail = _next(_tail);
		return true;
	} else {
		return false;
	}
}

template <typename T>
T RingBuffer<T>::get(void) 
{
	T val;
	return get(val) ? val : 0;
}

template <typename T>
unsigned RingBuffer<T>::space(void) 
{
	return (_tail >= _head) ? (_size - (_tail - _head)) : (_head - _tail - 1);
}

template <typename T>
unsigned RingBuffer<T>::count(void) 
{
	return _size - space();
}

template <typename T>
unsigned RingBuffer<T>::_next(unsigned index) 
{
	return (0 == index) ? _size : (index - 1);
}
