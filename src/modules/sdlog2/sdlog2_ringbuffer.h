/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <rk3dov@gmail.com>
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
 * @file sdlog2_ringbuffer.h
 *
 * Ring FIFO buffer for binary data.
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 */

#ifndef SDLOG2_RINGBUFFER_H_
#define SDLOG2_RINGBUFFER_H_

struct sdlog2_logbuffer {
	// all pointers are in bytes
	int write_ptr;
	int read_ptr;
	int size;
	char *data;
};

void sdlog2_logbuffer_init(struct sdlog2_logbuffer *lb, int size);

int sdlog2_logbuffer_free(struct sdlog2_logbuffer *lb);

int sdlog2_logbuffer_count(struct sdlog2_logbuffer *lb);

int sdlog2_logbuffer_is_empty(struct sdlog2_logbuffer *lb);

void sdlog2_logbuffer_write(struct sdlog2_logbuffer *lb, void *ptr, int size);

int sdlog2_logbuffer_get_ptr(struct sdlog2_logbuffer *lb, void **ptr);

void sdlog2_logbuffer_mark_read(struct sdlog2_logbuffer *lb, int n);

#endif
