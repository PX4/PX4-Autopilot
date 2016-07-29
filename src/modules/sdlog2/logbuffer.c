/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file logbuffer.c
 *
 * Ring FIFO buffer for binary log data.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_defines.h>
#include <string.h>
#include <stdlib.h>

#include "logbuffer.h"

int logbuffer_init(struct logbuffer_s *lb, int size)
{
	lb->size  = size;
	lb->write_ptr = 0;
	lb->read_ptr = 0;
	lb->data = NULL;
	lb->perf_dropped = perf_alloc(PC_COUNT, "sd drop");
	return PX4_OK;
}

int logbuffer_count(struct logbuffer_s *lb)
{
	int n = lb->write_ptr - lb->read_ptr;

	if (n < 0) {
		n += lb->size;
	}

	return n;
}

int logbuffer_is_empty(struct logbuffer_s *lb)
{
	return lb->read_ptr == lb->write_ptr;
}

bool logbuffer_write(struct logbuffer_s *lb, void *ptr, int size)
{
	// allocate buffer if not yet present
	if (lb->data == NULL) {
		lb->data = malloc(lb->size);
	}

	// allocation failed, bail out
	if (lb->data == NULL) {
		return false;
	}

	// bytes available to write
	int available = lb->read_ptr - lb->write_ptr - 1;

	if (available < 0) {
		available += lb->size;
	}

	if (size > available) {
		// buffer overflow
		perf_count(lb->perf_dropped);
		return false;
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
	return true;
}

int logbuffer_get_ptr(struct logbuffer_s *lb, void **ptr, bool *is_part)
{
	// bytes available to read
	int available = lb->write_ptr - lb->read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = lb->size - lb->read_ptr;
		*is_part = lb->write_ptr > 0;
	}

	*ptr = &(lb->data[lb->read_ptr]);
	return n;
}

void logbuffer_mark_read(struct logbuffer_s *lb, int n)
{
	lb->read_ptr = (lb->read_ptr + n) % lb->size;
}

void logbuffer_free(struct logbuffer_s *lb)
{
	if (lb->data) {
		free(lb->data);
		lb->write_ptr = 0;
		lb->read_ptr = 0;
		lb->data = NULL;
		perf_free(lb->perf_dropped);
	}
}

void logbuffer_reset(struct logbuffer_s *lb)
{
	// Keep the buffer but reset the pointers.
	lb->write_ptr = 0;
	lb->read_ptr = 0;
}
