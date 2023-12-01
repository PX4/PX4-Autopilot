/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
* @file QueueBuffer.cpp
*
* A very lightweight QueueBuffer implemtnation
*
* @author Chris Seto <chris1seto@gmail.com>
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "QueueBuffer.hpp"

void QueueBuffer_Init(QueueBuffer_t *const q, uint8_t *buffer, const uint32_t buffer_size)
{
	q->buffer = buffer;
	q->buffer_size = buffer_size;

	q->head = 0;
	q->tail = 0;
	q->count = 0;
}

uint32_t QueueBuffer_Count(const QueueBuffer_t *q)
{
	return q->count;
}

void QueueBuffer_Append(QueueBuffer_t *const q, const uint8_t x)
{
	// Append the data and make the new tail index
	q->buffer[q->tail] = x;
	q->tail = ((q->tail + 1) % q->buffer_size);

	// Check if we can expand the count any more
	if (q->count < q->buffer_size) {
		q->count++;
	}
}

bool QueueBuffer_AppendBuffer(QueueBuffer_t *const q, const uint8_t *x, const uint32_t append_size)
{
	uint32_t buffer_end_size;

	buffer_end_size = q->buffer_size - q->tail;

	// Check if we can even put this buffer into the queu
	if (q->count + append_size > q->buffer_size) {
		return false;
	}

	// Check if there is enough space on the end of the queue to put the entire buffer
	if (buffer_end_size >= append_size) {
		// We can write the entire append buffer in one shot
		memcpy((void *)(q->buffer + q->tail), (void *)x, append_size);

	} else {
		memcpy((void *)(q->buffer + q->tail), (void *)x, buffer_end_size);
		memcpy((void *)(q->buffer), (void *)(x + buffer_end_size), append_size - buffer_end_size);
	}

	// Append to the tail
	q->tail = ((q->tail + append_size) % q->buffer_size);
	q->count += append_size;

	return true;
}

bool QueueBuffer_IsEmpty(const QueueBuffer_t *q)
{
	return (q->count == 0);
}

bool QueueBuffer_Get(QueueBuffer_t *const q, uint8_t *const x)
{
	if (q->count == 0) {
		return false;
	}

	*x = q->buffer[q->head];
	q->head = ((q->head + 1) % q->buffer_size);
	q->count--;

	return true;
}

void QueueBuffer_Dequeue(QueueBuffer_t *const q, const uint32_t n)
{
	if (n > q->count) {
		return;
	}

	q->count -= n;
	q->head = (q->head + n) % q->buffer_size;
}

bool QueueBuffer_Peek(const QueueBuffer_t *q, const uint32_t index, uint8_t *const x)
{
	if (index >= q->count) {
		return false;
	}

	*x = q->buffer[(q->head + index) % q->buffer_size];
	return true;
}

bool QueueBuffer_PeekBuffer(const QueueBuffer_t *q, const uint32_t index, uint8_t *buffer, const uint32_t size)
{
	uint32_t copy_start;

	copy_start = q->head + index;

	// Check to see if this amount of sizegth exists at the index
	if (index + size > q->count) {
		return false;
	}

	// Check if we can do this in a single shot
	if (copy_start + size <= q->buffer_size) {
		memcpy((void *)buffer, (void *)(q->buffer + copy_start), size);

	} else {
		// Double shot copy
		uint32_t copy1Size = (q->buffer_size - copy_start);
		memcpy((void *)buffer, (void *)(q->buffer + copy_start), copy1Size);
		memcpy((void *)(buffer + copy1Size), (void *)(q->buffer), (size - copy1Size));
	}

	return true;
}
