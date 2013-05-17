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
 * @file sdlog_log.c
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <string.h>
#include <stdlib.h>

#include "sdlog_ringbuffer.h"

void sdlog_logbuffer_init(struct sdlog_logbuffer *lb, int size)
{
	lb->size  = size;
	lb->start = 0;
	lb->count = 0;
	lb->elems = (struct sdlog_sysvector *)calloc(lb->size, sizeof(struct sdlog_sysvector));
}

int sdlog_logbuffer_is_full(struct sdlog_logbuffer *lb)
{
	return lb->count == (int)lb->size;
}

int sdlog_logbuffer_is_empty(struct sdlog_logbuffer *lb)
{
	return lb->count == 0;
}


// XXX make these functions thread-safe
void sdlog_logbuffer_write(struct sdlog_logbuffer *lb, const struct sdlog_sysvector *elem)
{
	int end = (lb->start + lb->count) % lb->size;
	memcpy(&(lb->elems[end]), elem, sizeof(struct sdlog_sysvector));

	if (sdlog_logbuffer_is_full(lb)) {
		lb->start = (lb->start + 1) % lb->size; /* full, overwrite */

	} else {
		++lb->count;
	}
}

int sdlog_logbuffer_read(struct sdlog_logbuffer *lb, struct sdlog_sysvector *elem)
{
	if (!sdlog_logbuffer_is_empty(lb)) {
		memcpy(elem, &(lb->elems[lb->start]), sizeof(struct sdlog_sysvector));
		lb->start = (lb->start + 1) % lb->size;
		--lb->count;
		return 0;

	} else {
		return 1;
	}
}
