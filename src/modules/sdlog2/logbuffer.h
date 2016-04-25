/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file logbuffer.h
 *
 * Ring FIFO buffer for binary log data.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef SDLOG2_RINGBUFFER_H_
#define SDLOG2_RINGBUFFER_H_

#include <stdbool.h>
#include <systemlib/perf_counter.h>

struct logbuffer_s {
	// pointers and size are in bytes
	int write_ptr;
	int read_ptr;
	int size;
	char *data;
	perf_counter_t perf_dropped;
};

int logbuffer_init(struct logbuffer_s *lb, int size);

int logbuffer_count(struct logbuffer_s *lb);

int logbuffer_is_empty(struct logbuffer_s *lb);

bool logbuffer_write(struct logbuffer_s *lb, void *ptr, int size);

int logbuffer_get_ptr(struct logbuffer_s *lb, void **ptr, bool *is_part);

void logbuffer_mark_read(struct logbuffer_s *lb, int n);

void logbuffer_free(struct logbuffer_s *lb);

void logbuffer_reset(struct logbuffer_s *lb);

#endif
