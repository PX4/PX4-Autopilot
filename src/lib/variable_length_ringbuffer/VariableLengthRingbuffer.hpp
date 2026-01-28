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
#include <lib/ringbuffer/Ringbuffer.hpp>


// FIFO ringbuffer implementation for packets of variable length.
//
// The variable length is implemented using a 4 byte header
// containing a the length.
//
// The buffer is not thread-safe.

class VariableLengthRingbuffer
{
public:
	/* @brief Constructor
	 *
	 * @note Does not allocate automatically.
	 */
	VariableLengthRingbuffer() = default;

	/*
	 * @brief Destructor
	 *
	 * Automatically calls deallocate.
	 */
	~VariableLengthRingbuffer();

	/* @brief Allocate ringbuffer
	 *
	 * @note The variable length requires 4 bytes
	 * of overhead per packet.
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
	 * @brief Copy packet into ringbuffer
	 *
	 * @param packet Pointer to packet to copy from.
	 * @param packet_len Length of packet.
	 *
	 * @returns true if packet could be copied into buffer.
	 */
	bool push_back(const uint8_t *packet, size_t packet_len);

	/*
	 * @brief Get packet from ringbuffer
	 *
	 * @note max_buf_len needs to be bigger equal to any pushed packet.
	 *
	 * @param buf Pointer to where next packet can be copied into.
	 * @param max_buf_len Max size of buf
	 *
	 * @returns 0 if packet is bigger than max_len or buffer is empty.
	 */
	size_t pop_front(uint8_t *buf, size_t max_buf_len);

private:
	struct Header {
		uint32_t len;
	};

	Ringbuffer _ringbuffer {};
};
