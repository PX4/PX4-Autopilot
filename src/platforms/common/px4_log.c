/****************************************************************************
 *
 * Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

#include <px4_log.h>

#include <stdlib.h>
#include <string.h>

#if defined(__PX4_POSIX) && !defined(__PX4_CYGWIN)
#include <execinfo.h>
#endif

#include <uORB/uORB.h>
#include <uORB/topics/log_message.h>
#include <drivers/drv_hrt.h>

static orb_advert_t orb_log_message_pub = NULL;

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = { "INFO", "DEBUG", "WARN", "ERROR", "PANIC" };
__EXPORT const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] =
{ PX4_ANSI_COLOR_RESET, PX4_ANSI_COLOR_GREEN, PX4_ANSI_COLOR_YELLOW, PX4_ANSI_COLOR_RED, PX4_ANSI_COLOR_RED };


void px4_log_initialize(void)
{
	ASSERT(orb_log_message_pub == NULL);

	/* we need to advertise with a valid message */
	struct log_message_s log_message;
	log_message.timestamp = hrt_absolute_time();
	log_message.severity = 6; //info
	strcpy((char *)log_message.text, "initialized uORB logging");

#if !defined(PARAM_NO_ORB)
	orb_log_message_pub = orb_advertise_queue(ORB_ID(log_message), &log_message, 2);
#endif /* !PARAM_NO_ORB */

	if (!orb_log_message_pub) {
		PX4_ERR("failed to advertise log_message");
	}
}

void px4_backtrace()
{
#if defined(__PX4_POSIX) && !defined(__PX4_CYGWIN)
	void *buffer[10];
	char **callstack;
	int bt_size;
	int idx;

	bt_size = backtrace(buffer, 10);
	callstack = backtrace_symbols(buffer, bt_size);

	PX4_INFO("Backtrace: %d", bt_size);

	for (idx = 0; idx < bt_size; idx++) {
		PX4_INFO("%s", callstack[idx]);
	}

	free(callstack);
#endif
}

__EXPORT void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...)
{
	PX4_LOG_COLOR_START
	printf(__px4__log_level_fmt __px4__log_level_arg(level));
	PX4_LOG_COLOR_MODULE
	printf(__px4__log_modulename_pfmt, moduleName);
	PX4_LOG_COLOR_MESSAGE
	va_list argptr;
	va_start(argptr, fmt);
	vprintf(fmt, argptr);
	va_end(argptr);
	PX4_LOG_COLOR_END
	printf("\n");

	/* publish an orb log message */
	if (level >= _PX4_LOG_LEVEL_WARN && orb_log_message_pub) { //only publish important messages

		struct log_message_s log_message;
		const unsigned max_length = sizeof(log_message.text);
		log_message.timestamp = hrt_absolute_time();

		const uint8_t log_level_table[] = {
			6, /* _PX4_LOG_LEVEL_ALWAYS */
			7, /* _PX4_LOG_LEVEL_DEBUG */
			4, /* _PX4_LOG_LEVEL_WARN */
			3, /* _PX4_LOG_LEVEL_ERROR */
			0  /* _PX4_LOG_LEVEL_PANIC */
		};
		log_message.severity = log_level_table[level];

		unsigned pos = 0;

		pos += snprintf((char *)log_message.text + pos, max_length - pos, __px4__log_modulename_pfmt, moduleName);
		va_start(argptr, fmt);
		pos += vsnprintf((char *)log_message.text + pos, max_length - pos, fmt, argptr);
		va_end(argptr);
		log_message.text[max_length - 1] = 0; //ensure 0-termination

#if !defined(PARAM_NO_ORB)
		orb_publish(ORB_ID(log_message), orb_log_message_pub, &log_message);
#endif /* !PARAM_NO_ORB */
	}
}
