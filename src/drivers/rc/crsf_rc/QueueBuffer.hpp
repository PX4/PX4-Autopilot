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
 * @file QueueBuffer.hpp
 *
 * A very lightweight QueueBuffer implemtnation
 *
 * @author Chris Seto <chris1seto@gmail.com>
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint8_t *buffer;
	uint32_t buffer_size;
	uint32_t count;
	uint32_t head;
	uint32_t tail;
} QueueBuffer_t;

void QueueBuffer_Init(QueueBuffer_t *const q, uint8_t *buffer, const uint32_t buffer_size);
uint32_t QueueBuffer_Count(const QueueBuffer_t *q);
void QueueBuffer_Append(QueueBuffer_t *const q, const uint8_t x);
bool QueueBuffer_AppendBuffer(QueueBuffer_t *const q, const uint8_t *x, const uint32_t append_size);
bool QueueBuffer_IsEmpty(const QueueBuffer_t *q);
bool QueueBuffer_Get(QueueBuffer_t *const q, uint8_t *const x);
void QueueBuffer_Dequeue(QueueBuffer_t *const q, const uint32_t n);
bool QueueBuffer_Peek(const QueueBuffer_t *q, const uint32_t index, uint8_t *const x);
bool QueueBuffer_PeekBuffer(const QueueBuffer_t *q, const uint32_t index, uint8_t *buffer, const uint32_t size);
