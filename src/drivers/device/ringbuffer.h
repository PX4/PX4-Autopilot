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
	 * Put an item into the buffer if there is space.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(const T &val);

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool			force(T &val);

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool			force(const T &val);

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
	 * @return		The value that was fetched. If the buffer is empty, 
	 *			returns zero.
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
	bool			empty();

	/*
	 * Returns true if the buffer is full.
	 */
	bool			full();

	/*
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	unsigned		size();

	/*
	 * Empties the buffer.
	 */
	void			flush();

	/*
	 * resize the buffer. This is unsafe to be called while
	 * a producer or consuming is running. Caller is responsible
	 * for any locking needed
	 *
	 * @param new_size	new size for buffer
	 * @return		true if the resize succeeds, false if
	 * 			not (allocation error)
	 */
	bool			resize(unsigned new_size);

	/*
	 * printf() some info on the buffer
	 */
	void			print_info(const char *name);

private:
	T			*_buf;	
	unsigned		_size;
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
bool RingBuffer<T>::empty()
{
	return _tail == _head; 
}

template <typename T>
bool RingBuffer<T>::full()
{
	return _next(_head) == _tail; 
}

template <typename T>
unsigned RingBuffer<T>::size()
{
	return (_buf != nullptr) ? _size : 0; 
}

template <typename T>
void RingBuffer<T>::flush()
{
	T junk;
	while (!empty())
		get(junk); 
}

template <typename T>
unsigned RingBuffer<T>::_next(unsigned index)
{
	return (0 == index) ? _size : (index - 1); 
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
bool RingBuffer<T>::force(T &val)
{
	bool overwrote = false;

	for (;;) {
		if (put(val))
			break;
		T junk;
		get(junk);
		overwrote = true;
	}
	return overwrote;
}

template <typename T>
bool RingBuffer<T>::force(const T &val)
{
	bool overwrote = false;

	for (;;) {
		if (put(val))
			break;
		T junk;
		get(junk);
		overwrote = true;
	}
	return overwrote;
}

template <typename T>
bool RingBuffer<T>::get(T &val) 
{
	if (_tail != _head) {
		unsigned candidate;
		unsigned next;
		do {
			/* decide which element we think we're going to read */
			candidate = _tail;

			/* and what the corresponding next index will be */
			next = _next(candidate);

			/* go ahead and read from this index */
			val = _buf[candidate];

			/* if the tail pointer didn't change, we got our item */
		} while (!__sync_bool_compare_and_swap(&_tail, candidate, next));

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
	unsigned tail, head;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		head = _head;
		tail = _tail;
	} while (head != _head);

	return (tail >= head) ? (_size - (tail - head)) : (head - tail - 1);
}

template <typename T>
unsigned RingBuffer<T>::count(void) 
{
	/*
	 * Note that due to the conservative nature of space(), this may
	 * over-estimate the number of items in the buffer.
	 */
	return _size - space();
}

template <typename T>
bool RingBuffer<T>::resize(unsigned new_size) 
{
	T *old_buffer;
	T *new_buffer = new T[new_size + 1];
	if (new_buffer == nullptr) {
		return false;
	}
	old_buffer = _buf;
	_buf = new_buffer;
	_size = new_size;
	_head = new_size;
	_tail = new_size;
	delete[] old_buffer;
	return true;
}

template <typename T>
void RingBuffer<T>::print_info(const char *name) 
{
	printf("%s	%u (%u/%u @ %p)\n",
	       name, _size, _head, _tail, _buf);
}
