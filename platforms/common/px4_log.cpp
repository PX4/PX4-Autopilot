/****************************************************************************
 *
 * Copyright (C) 2017-2018 PX4 Development Team. All rights reserved.
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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef MODULE_NAME
#define MODULE_NAME "log"
#endif

#include <px4_platform_common/log.h>
#if defined(__PX4_POSIX)
#include <px4_daemon/server_io.h>
#endif

#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/log_message.h>
#include <drivers/drv_hrt.h>

static orb_advert_t orb_log_message_pub = nullptr;

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = { "DEBUG", "INFO", "WARN", "ERROR", "PANIC" };

#define PX4_ANSI_COLOR_RED     "\x1b[31m"
#define PX4_ANSI_COLOR_GREEN   "\x1b[32m"
#define PX4_ANSI_COLOR_YELLOW  "\x1b[33m"
#define PX4_ANSI_COLOR_BLUE    "\x1b[34m"
#define PX4_ANSI_COLOR_MAGENTA "\x1b[35m"
#define PX4_ANSI_COLOR_CYAN    "\x1b[36m"
#define PX4_ANSI_COLOR_GRAY    "\x1B[37m"
#define PX4_ANSI_COLOR_RESET   "\x1b[0m"

static constexpr const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] {
	PX4_ANSI_COLOR_GREEN,  // DEBUG
	PX4_ANSI_COLOR_RESET,  // INFO
	PX4_ANSI_COLOR_YELLOW, // WARN
	PX4_ANSI_COLOR_RED,    // ERROR
	PX4_ANSI_COLOR_RED     // PANIC
};

void px4_log_initialize(void)
{
	// we need to advertise with a valid message
	log_message_s log_message{};
	log_message.severity = 6; // info
	strcpy((char *)log_message.text, "initialized uORB logging");
	log_message.timestamp = hrt_absolute_time();
	orb_log_message_pub = orb_advertise_queue(ORB_ID(log_message), &log_message, log_message_s::ORB_QUEUE_LENGTH);
}

__EXPORT void px4_log_modulename(int level, const char *module_name, const char *fmt, ...)
{
	static constexpr ssize_t max_length = sizeof(log_message_s::text);

	FILE *out = stdout;

#if defined(PX4_LOG_COLORIZED_OUTPUT)
	bool use_color = true;
#endif // PX4_LOG_COLORIZED_OUTPUT

#if defined(__PX4_POSIX)
	bool isatty_ = false;
	out = get_stdout(&isatty_);

#if defined(PX4_LOG_COLORIZED_OUTPUT)
	use_color = isatty_;
#endif // PX4_LOG_COLORIZED_OUTPUT
#endif // PX4_POSIX

	if (level >= _PX4_LOG_LEVEL_INFO) {
		char buf[max_length + 1]; // same length as log_message_s::text, but add newline
		ssize_t pos = 0;

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += sprintf(buf + pos, "%s", __px4_log_level_color[level]);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), __px4__log_level_fmt, __px4_log_level_str[level]);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), PX4_ANSI_COLOR_GRAY);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), __px4__log_modulename_pfmt, module_name);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", __px4_log_level_color[level]);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		va_list argptr;
		va_start(argptr, fmt);
		pos += vsnprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), fmt, argptr);
		va_end(argptr);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			// alway reset color
			const ssize_t sz = math::min(pos, max_length - (ssize_t)strlen(PX4_ANSI_COLOR_RESET) - (ssize_t)1);
			pos += sprintf(buf + sz, "%s\n", PX4_ANSI_COLOR_RESET);

		} else
#endif // PX4_LOG_COLORIZED_OUTPUT
		{
			pos += sprintf(buf + math::min(pos, max_length - (ssize_t)1), "\n");
		}

		// ensure NULL termination (buffer is max_length + 1)
		buf[max_length] = 0;

		fputs(buf, out);

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
		// Without flushing it's tricky to see stdout output when PX4 is started by
		// a script like for the MAVSDK tests.
		fflush(out);
#endif // CONFIG_ARCH_BOARD_PX4_SITL
	}

	/* publish an orb log message */
	if (level >= _PX4_LOG_LEVEL_INFO && orb_log_message_pub) { //publish all messages

		log_message_s log_message;

		const uint8_t log_level_table[] = {
			7, /* _PX4_LOG_LEVEL_DEBUG */
			6, /* _PX4_LOG_LEVEL_INFO */
			4, /* _PX4_LOG_LEVEL_WARN */
			3, /* _PX4_LOG_LEVEL_ERROR */
			0  /* _PX4_LOG_LEVEL_PANIC */
		};
		log_message.severity = log_level_table[level];

		ssize_t pos = snprintf((char *)log_message.text, max_length, __px4__log_modulename_pfmt, module_name);

		va_list argptr;
		va_start(argptr, fmt);
		pos += vsnprintf((char *)log_message.text + pos, math::max(max_length - pos, (ssize_t)0), fmt, argptr);
		va_end(argptr);
		log_message.text[max_length - 1] = 0; //ensure 0-termination
		log_message.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(log_message), orb_log_message_pub, &log_message);
	}
}

__EXPORT void px4_log_raw(int level, const char *fmt, ...)
{
	FILE *out = stdout;

#ifdef __PX4_POSIX
	bool use_color = true;
	out = get_stdout(&use_color);
#endif

	if (level >= _PX4_LOG_LEVEL_INFO) {
		static constexpr ssize_t max_length = sizeof(log_message_s::text);
		char buf[max_length + 1]; // same length as log_message_s::text, but add newline
		ssize_t pos = 0;

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += sprintf(buf + pos, "%s", __px4_log_level_color[level]);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		va_list argptr;
		va_start(argptr, fmt);
		pos += vsnprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), fmt, argptr);
		va_end(argptr);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			// alway reset color
			const ssize_t sz = math::min(pos, max_length - (ssize_t)strlen(PX4_ANSI_COLOR_RESET));
			pos += sprintf(buf + sz, "%s", PX4_ANSI_COLOR_RESET);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		if (pos > max_length) {
			// preserve newline if necessary
			if (fmt[strlen(fmt) - 1] == '\n') {
				buf[max_length - 1] = '\n';
			}
		}

		// ensure NULL termination (buffer is max_length + 1)
		buf[max_length] = 0;

		fputs(buf, out);
	}
}
