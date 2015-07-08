/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mavlink_log.c
 * MAVLink text logging.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <px4_posix.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include <mavlink/mavlink_log.h>

__EXPORT void mavlink_logbuffer_init(struct mavlink_logbuffer *lb, int size)
{
	lb->size  = size;
	lb->start = 0;
	lb->count = 0;
	lb->elems = calloc(lb->size, sizeof(struct mavlink_logmessage));
}

__EXPORT void mavlink_logbuffer_destroy(struct mavlink_logbuffer *lb)
{
	lb->size  = 0;
	lb->start = 0;
	lb->count = 0;
	free(lb->elems);
}

__EXPORT int mavlink_logbuffer_is_full(struct mavlink_logbuffer *lb)
{
	return lb->count == (int)lb->size;
}

__EXPORT int mavlink_logbuffer_is_empty(struct mavlink_logbuffer *lb)
{
	return lb->count == 0;
}

__EXPORT void mavlink_logbuffer_write(struct mavlink_logbuffer *lb, const struct mavlink_logmessage *elem)
{
	int end = (lb->start + lb->count) % lb->size;
	memcpy(&(lb->elems[end]), elem, sizeof(struct mavlink_logmessage));

	if (mavlink_logbuffer_is_full(lb)) {
		lb->start = (lb->start + 1) % lb->size; /* full, overwrite */

	} else {
		++lb->count;
	}
}

__EXPORT int mavlink_logbuffer_read(struct mavlink_logbuffer *lb, struct mavlink_logmessage *elem)
{
	if (!mavlink_logbuffer_is_empty(lb)) {
		memcpy(elem, &(lb->elems[lb->start]), sizeof(struct mavlink_logmessage));
		lb->start = (lb->start + 1) % lb->size;
		--lb->count;

		return 0;

	} else {
		return 1;
	}
}

__EXPORT void mavlink_logbuffer_vasprintf(struct mavlink_logbuffer *lb, int severity, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int end = (lb->start + lb->count) % lb->size;
	lb->elems[end].severity = severity;
	vsnprintf(lb->elems[end].text, sizeof(lb->elems[0].text), fmt, ap);
	va_end(ap);

	/* increase count */
	if (mavlink_logbuffer_is_full(lb)) {
		lb->start = (lb->start + 1) % lb->size; /* full, overwrite */

	} else {
		++lb->count;
	}
}

__EXPORT void mavlink_vasprintf(int _fd, int severity, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	char text[MAVLINK_LOG_MAXLEN + 1];
	vsnprintf(text, sizeof(text), fmt, ap);
	va_end(ap);
	px4_ioctl(_fd, severity, (unsigned long)&text[0]);
}
