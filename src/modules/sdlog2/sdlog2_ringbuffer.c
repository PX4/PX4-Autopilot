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
 * @file sdlog2_ringbuffer.c
 *
 * Ring FIFO buffer for binary data.
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 */

#include <string.h>
#include <stdlib.h>

#include "sdlog2_ringbuffer.h"

void sdlog2_logbuffer_init(struct sdlog2_logbuffer *lb, int size)
{
	lb->size  = size;
	lb->write_ptr = 0;
	lb->read_ptr = 0;
	lb->data = malloc(lb->size);
}

int sdlog2_logbuffer_free(struct sdlog2_logbuffer *lb)
{
	int n = lb->read_ptr - lb->write_ptr - 1;

	if (n < 0) {
		n += lb->size;
	}

	return n;
}

int sdlog2_logbuffer_count(struct sdlog2_logbuffer *lb)
{
	int n = lb->write_ptr - lb->read_ptr;

	if (n < 0) {
		n += lb->size;
	}

	return n;
}

int sdlog2_logbuffer_is_empty(struct sdlog2_logbuffer *lb)
{
	return lb->read_ptr == lb->write_ptr;
}

void sdlog2_logbuffer_write(struct sdlog2_logbuffer *lb, void *ptr, int size)
{
	// bytes available to write
	int available = lb->read_ptr - lb->write_ptr - 1;

	if (available < 0)
		available += lb->size;

	if (size > available) {
		// buffer overflow
		return;
	}

	char *c = (char *) ptr;
	int n = lb->size - lb->write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(lb->data[lb->write_ptr]), c, n);
		lb->write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(lb->data[lb->write_ptr]), &(c[n]), p);
	lb->write_ptr = (lb->write_ptr + p) % lb->size;
}

int sdlog2_logbuffer_get_ptr(struct sdlog2_logbuffer *lb, void **ptr)
{
	// bytes available to read
	int available = lb->write_ptr - lb->read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, write all available bytes
		n = available;

	} else {
		// read pointer is after write pointer, write bytes from read_ptr to end
		n = lb->size - lb->read_ptr;
	}

	*ptr = &(lb->data[lb->read_ptr]);
	return n;
}

void sdlog2_logbuffer_mark_read(struct sdlog2_logbuffer *lb, int n)
{
	lb->read_ptr = (lb->read_ptr + n) % lb->size;
}
