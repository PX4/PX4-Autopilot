/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * A flexible ringbuffer class.
 */

#pragma once

class RingBuffer {
public:
	RingBuffer(unsigned ring_size, size_t entry_size);
	virtual ~RingBuffer();

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(const void *val, size_t val_size = 0);

	bool			put(int8_t val);
	bool			put(uint8_t val);
	bool			put(int16_t val);
	bool			put(uint16_t val);
	bool			put(int32_t val);
	bool			put(uint32_t val);
	bool			put(int64_t val);
	bool			put(uint64_t val);
	bool			put(float val);
	bool			put(double val);

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool			force(const void *val, size_t val_size = 0);

	bool			force(int8_t val);
	bool			force(uint8_t val);
	bool			force(int16_t val);
	bool			force(uint16_t val);
	bool			force(int32_t val);
	bool			force(uint32_t val);
	bool			force(int64_t val);
	bool			force(uint64_t val);
	bool			force(float val);
	bool			force(double val);

	/**
	 * Get an item from the buffer.
	 *
	 * @param val		Item that was gotten
	 * @return		true if an item was got, false if the buffer was empty.
	 */
	bool			get(void *val, size_t val_size = 0);

	bool			get(int8_t &val);
	bool			get(uint8_t &val);
	bool			get(int16_t &val);
	bool			get(uint16_t &val);
	bool			get(int32_t &val);
	bool			get(uint32_t &val);
	bool			get(int64_t &val);
	bool			get(uint64_t &val);
	bool			get(float &val);
	bool			get(double &val);

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
	unsigned		_num_items;
	const size_t		_item_size;
	char			*_buf;	
	volatile unsigned	_head;	/**< insertion point in _item_size units */
	volatile unsigned	_tail;	/**< removal point in _item_size units */

	unsigned		_next(unsigned index);

	/* we don't want this class to be copied */
	RingBuffer(const RingBuffer&);
	RingBuffer operator=(const RingBuffer&);
};

RingBuffer::RingBuffer(unsigned num_items, size_t item_size) :
	_num_items(num_items),
	_item_size(item_size),
	_buf(new char[(_num_items+1) * item_size]),
 	_head(_num_items),
 	_tail(_num_items)
{}

RingBuffer::~RingBuffer()
{
	if (_buf != nullptr)
		delete[] _buf;
}

unsigned
RingBuffer::_next(unsigned index)
{
	return (0 == index) ? _num_items : (index - 1); 
}

bool
RingBuffer::empty()
{
	return _tail == _head; 
}

bool
RingBuffer::full()
{
	return _next(_head) == _tail; 
}

unsigned
RingBuffer::size()
{
	return (_buf != nullptr) ? _num_items : 0; 
}

void
RingBuffer::flush()
{
	while (!empty())
		get(NULL);
}

bool
RingBuffer::put(const void *val, size_t val_size) 
{
	unsigned next = _next(_head);
	if (next != _tail) {
		if ((val_size == 0) || (val_size > _item_size))
			val_size = _item_size;
		memcpy(&_buf[_head * _item_size], val, val_size);
		_head = next;
		return true;
	} else {
		return false;
	}
}

bool
RingBuffer::put(int8_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint8_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int16_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint16_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int32_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint32_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int64_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint64_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(float val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(double val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::force(const void *val, size_t val_size)
{
	bool overwrote = false;

	for (;;) {
		if (put(val, val_size))
			break;
		get(NULL);
		overwrote = true;
	}
	return overwrote;
}

bool
RingBuffer::force(int8_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint8_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int16_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint16_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int32_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint32_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int64_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint64_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(float val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(double val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::get(void *val, size_t val_size) 
{
	if (_tail != _head) {
		unsigned candidate;
		unsigned next;

		if ((val_size == 0) || (val_size > _item_size))
			val_size = _item_size;

		do {
			/* decide which element we think we're going to read */
			candidate = _tail;

			/* and what the corresponding next index will be */
			next = _next(candidate);

			/* go ahead and read from this index */
			if (val != NULL)
				memcpy(val, &_buf[candidate * _item_size], val_size);

			/* if the tail pointer didn't change, we got our item */
		} while (!__sync_bool_compare_and_swap(&_tail, candidate, next));

		return true;
	} else {
		return false;
	}
}

bool
RingBuffer::get(int8_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint8_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int16_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint16_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int32_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint32_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int64_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint64_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(float &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(double &val)
{
	return get(&val, sizeof(val));
}

unsigned
RingBuffer::space(void) 
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

	return (tail >= head) ? (_num_items - (tail - head)) : (head - tail - 1);
}

unsigned
RingBuffer::count(void) 
{
	/*
	 * Note that due to the conservative nature of space(), this may
	 * over-estimate the number of items in the buffer.
	 */
	return _num_items - space();
}

bool
RingBuffer::resize(unsigned new_size) 
{
	char *old_buffer;
	char *new_buffer = new char [(new_size+1) * _item_size];
	if (new_buffer == nullptr) {
		return false;
	}
	old_buffer = _buf;
	_buf = new_buffer;
	_num_items = new_size;
	_head = new_size;
	_tail = new_size;
	delete[] old_buffer;
	return true;
}

void
RingBuffer::print_info(const char *name) 
{
	printf("%s	%u/%u (%u/%u @ %p)\n",
	       name, 
	       _num_items, 
	       _num_items * _item_size, 
	       _head, 
	       _tail, 
	       _buf);
}
