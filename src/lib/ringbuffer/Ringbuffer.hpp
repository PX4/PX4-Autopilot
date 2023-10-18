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



#pragma once

#include <stdint.h>
#include <pthread.h>


// FIFO ringbuffer implementation.
//
// The ringbuffer can store up 1 byte less than allocated as
// start and end marker need to be one byte apart when the buffer
// is full, otherwise it would suddenly be empty.
//
// The buffer is not thread-safe.

class Ringbuffer
{
public:
	/* @brief Constructor
	 *
	 * @note Does not allocate automatically.
	 */
	Ringbuffer() = default;

	/*
	 * @brief Destructor
	 *
	 * Automatically calls deallocate.
	 */
	~Ringbuffer();

	/* @brief Allocate ringbuffer
	 *
	 * @param buffer_size Number of bytes to allocate on heap.
	 *
	 * @returns false if allocation fails.
	 */
	bool allocate(size_t buffer_size);

	/*
	 * @brief Deallocate ringbuffer
	 *
	 * @note only required to deallocate and reallocate again.
	 */
	void deallocate();

	/*
	 * @brief Space available to copy bytes into
	 *
	 * @returns number of free bytes.
	 */
	size_t space_available() const;

	/*
	 * @brief Space used to copy data from
	 *
	 * @returns number of used bytes.
	 */
	size_t space_used() const;

	/*
	 * @brief Copy data into ringbuffer
	 *
	 * @param buf Pointer to buffer to copy from.
	 * @param buf_len Number of bytes to copy.
	 *
	 * @returns true if packet could be copied into buffer.
	 */
	bool push_back(const uint8_t *buf, size_t buf_len);

	/*
	 * @brief Get data from ringbuffer
	 *
	 * @param buf Pointer to buffer where data can be copied into.
	 * @param max_buf_len Max number of bytes to copy.
	 *
	 * @returns 0 if buffer is empty.
	 */
	size_t pop_front(uint8_t *buf, size_t max_buf_len);

private:
	size_t _size {0};
	uint8_t *_ringbuffer {nullptr};
	size_t _start{0};
	size_t _end{0};
};
