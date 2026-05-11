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
#include <ctype.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(__PX4_WINDOWS)
#include <io.h>
#include <windows.h>
#endif

#ifndef MODULE_NAME
#define MODULE_NAME "log"
#endif

#include <px4_platform_common/log.h>
#include <px4_platform_common/log_history.h>
#if defined(__PX4_POSIX)
#include <px4_daemon/server_io.h>
#endif

#include <lib/mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/log_message.h>
#include <drivers/drv_hrt.h>

#if defined(BOARD_ENABLE_LOG_HISTORY)
static LogHistory g_log_history;
#endif

static orb_advert_t orb_log_message_pub = nullptr;

#if defined(PX4_LOG_COLORIZED_OUTPUT)
static bool px4_force_color_enabled()
{
	const char *force_color = getenv("PX4_FORCE_COLOR");
	return force_color && strcmp(force_color, "1") == 0;
}

static void px4_copy_without_ansi(char *dst, size_t dst_size, const char *src)
{
	if (dst_size == 0) {
		return;
	}

	size_t out = 0;

	for (size_t in = 0; src[in] != '\0' && out + 1 < dst_size;) {
		if (src[in] == '\x1b' && src[in + 1] == '[') {
			size_t end = in + 2;

			while (src[end] != '\0' && (src[end] < '@' || src[end] > '~')) {
				end++;
			}

			if (src[end] != '\0') {
				in = end + 1;
				continue;
			}
		}

		dst[out++] = src[in++];
	}

	dst[out] = '\0';
}

#if defined(__PX4_WINDOWS)
static bool px4_is_running_under_wine()
{
	if (HMODULE ntdll = GetModuleHandleA("ntdll.dll")) {
		return GetProcAddress(ntdll, "wine_get_version") != nullptr;
	}

	return false;
}

static bool px4_terminal_likely_supports_ansi()
{
	const char *term = getenv("TERM");

	if (term && term[0] != '\0' && strcmp(term, "dumb") != 0) {
		return true;
	}

	const char *colorterm = getenv("COLORTERM");

	if (colorterm && colorterm[0] != '\0') {
		return true;
	}

#if defined(__PX4_WINDOWS)
	const char *ansicon = getenv("ANSICON");
	const char *conemu_ansi = getenv("ConEmuANSI");
	const char *wt_session = getenv("WT_SESSION");
	const char *term_program = getenv("TERM_PROGRAM");

	if ((ansicon && ansicon[0] != '\0')
	    || (wt_session && wt_session[0] != '\0')
	    || (term_program && term_program[0] != '\0')) {
		return true;
	}

	if (conemu_ansi && strcmp(conemu_ansi, "ON") == 0) {
		return true;
	}

#endif

	return false;
}
#endif

static bool px4_should_use_color_output(bool isatty_)
{
	bool use_color = isatty_;

	const char *no_color = getenv("NO_COLOR");

	if (no_color && no_color[0] != '\0') {
		return false;
	}

	if (px4_force_color_enabled()) {
		return true;
	}

#if defined(__PX4_WINDOWS)

	// Under Wine, PX4's Windows build usually writes through Wine's console
	// bridge to a host terminal. Keep POSIX's tty/TERM gating; the emit
	// path below writes complete SGR buffers directly to the host fd.
	if (px4_is_running_under_wine()) {
		return use_color && px4_terminal_likely_supports_ansi();
	}

#endif

	return use_color;
}
#endif

__EXPORT const char *__px4_log_level_str[_PX4_LOG_LEVEL_PANIC + 1] = { "DEBUG", "INFO", "WARN", "ERROR", "PANIC" };

static constexpr const char *__px4_log_level_color[_PX4_LOG_LEVEL_PANIC + 1] {
	PX4_ANSI_COLOR_GREEN,  // DEBUG
	PX4_ANSI_COLOR_RESET,  // INFO
	PX4_ANSI_COLOR_YELLOW, // WARN
	PX4_ANSI_COLOR_RED,    // ERROR
	PX4_ANSI_COLOR_RED     // PANIC
};

static int px4_log_level_from_text(const char *level_start, size_t level_len)
{
	for (int level = _PX4_LOG_LEVEL_DEBUG; level <= _PX4_LOG_LEVEL_PANIC; ++level) {
		const char *level_text = __px4_log_level_str[level];

		if (strlen(level_text) == level_len && strncmp(level_start, level_text, level_len) == 0) {
			return level;
		}
	}

	return -1;
}

__EXPORT int px4_log_modulename_from_text(const char *line)
{
	if (line == nullptr) {
		return 0;
	}

	const char *p = line;

	while (*p == ' ' || *p == '\t') {
		++p;
	}

	const char *level_start = p;

	while (*p >= 'A' && *p <= 'Z') {
		++p;
	}

	if (p == level_start || (*p != ' ' && *p != '\t')) {
		return 0;
	}

	const int level = px4_log_level_from_text(level_start, static_cast<size_t>(p - level_start));

	if (level < _PX4_LOG_LEVEL_INFO || level > _PX4_LOG_LEVEL_PANIC) {
		return 0;
	}

	while (*p == ' ' || *p == '\t') {
		++p;
	}

	if (*p != '[') {
		return 0;
	}

	const char *module_start = ++p;

	while (*p != '\0' && *p != ']' && *p != '\r' && *p != '\n') {
		++p;
	}

	if (*p != ']' || p == module_start) {
		return 0;
	}

	const size_t module_len = static_cast<size_t>(p - module_start);
	char module_name[64] {};

	if (module_len >= sizeof(module_name)) {
		return 0;
	}

	memcpy(module_name, module_start, module_len);
	module_name[module_len] = '\0';

	++p;

	while (*p == ' ' || *p == '\t') {
		++p;
	}

	size_t message_len = strlen(p);

	while (message_len > 0 && (p[message_len - 1] == '\r' || p[message_len - 1] == '\n')) {
		--message_len;
	}

	if (message_len > static_cast<size_t>(INT_MAX)) {
		message_len = static_cast<size_t>(INT_MAX);
	}

	px4_log_modulename(level, module_name, "%.*s", static_cast<int>(message_len), p);
	return 1;
}

__EXPORT void px4_log_write_text(FILE *out, const char *data, size_t length)
{
	if (out == nullptr) {
		out = stdout;
	}

	if (data == nullptr || length == 0) {
		return;
	}

	size_t start = 0;

	while (start < length) {
		size_t end = start;

		while (end < length && data[end] != '\n') {
			++end;
		}

		const bool has_newline = end < length;
		const size_t chunk_len = end - start + (has_newline ? 1 : 0);
		bool handled = false;

		if (has_newline) {
			const size_t line_len = end - start;
			char line_buf[sizeof(log_message_s::text) + 128] {};

			if (line_len < sizeof(line_buf)) {
				memcpy(line_buf, data + start, line_len);
				line_buf[line_len] = '\0';
				handled = px4_log_modulename_from_text(line_buf) != 0;
			}
		}

		if (!handled) {
			(void)fwrite(data + start, 1, chunk_len, out);
		}

		start += chunk_len;
	}

	fflush(out);
}

#if defined(__PX4_WINDOWS) && defined(PX4_LOG_COLORIZED_OUTPUT)
static bool px4_windows_stdout_supports_vt(FILE *out)
{
	if (out != stdout || _isatty(_fileno(stdout)) == 0) {
		return false;
	}

	static int cached_result = -1;

	if (cached_result >= 0) {
		return cached_result != 0;
	}

	HANDLE stdout_handle = GetStdHandle(STD_OUTPUT_HANDLE);

	if (stdout_handle == INVALID_HANDLE_VALUE || stdout_handle == nullptr) {
		cached_result = 0;
		return false;
	}

	DWORD mode = 0;

	if (!GetConsoleMode(stdout_handle, &mode)) {
		cached_result = 0;
		return false;
	}

#ifdef ENABLE_VIRTUAL_TERMINAL_PROCESSING

	if ((mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING) == 0) {
		if (!SetConsoleMode(stdout_handle, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {
			cached_result = 0;
			return false;
		}
	}

#endif

	cached_result = 1;
	return true;
}

#if defined(__x86_64__) && defined(__GNUC__)
static long long px4_linux_syscall3(long long num, long long a, long long b, long long c)
{
	long long ret;
	__asm__ volatile(
		"syscall"
		: "=a"(ret)
		: "0"(num), "D"(a), "S"(b), "d"(c)
		: "rcx", "r11", "memory"
	);
	return ret;
}
#endif

static bool px4_wine_write_stdout_direct(const char *buffer, size_t length)
{
#if defined(__x86_64__) && defined(__GNUC__)

	if (!px4_is_running_under_wine() || length == 0) {
		return false;
	}

	constexpr long long LX_SYS_write = 1;
	size_t written = 0;

	while (written < length) {
		const long long ret = px4_linux_syscall3(LX_SYS_write, 1,
				      reinterpret_cast<long long>(buffer + written),
				      static_cast<long long>(length - written));

		if (ret <= 0) {
			return false;
		}

		written += static_cast<size_t>(ret);
	}

	return true;
#else
	(void)buffer;
	(void)length;
	return false;
#endif
}

static bool px4_windows_should_emit_ansi(FILE *out, bool use_color)
{
	if (!use_color) {
		return false;
	}

	if (px4_force_color_enabled()) {
		if (px4_is_running_under_wine()) {
			return out == stdout;
		}

		return true;
	}

	if (px4_is_running_under_wine()) {
		return out == stdout;
	}

	return px4_windows_stdout_supports_vt(out);
}

static bool px4_get_windows_console(FILE *out, HANDLE *handle, WORD *default_attributes)
{
	if (out != stdout || _isatty(_fileno(stdout)) == 0) {
		return false;
	}

	HANDLE stdout_handle = GetStdHandle(STD_OUTPUT_HANDLE);

	if (stdout_handle == INVALID_HANDLE_VALUE || stdout_handle == nullptr) {
		return false;
	}

	CONSOLE_SCREEN_BUFFER_INFO info{};

	if (!GetConsoleScreenBufferInfo(stdout_handle, &info)) {
		return false;
	}

	if (handle) {
		*handle = stdout_handle;
	}

	if (default_attributes) {
		*default_attributes = info.wAttributes;
	}

	return true;
}

static WORD px4_windows_level_attributes(int level, WORD default_attributes)
{
	const WORD background = default_attributes & 0xF0;

	switch (level) {
	case _PX4_LOG_LEVEL_DEBUG:
		return background | FOREGROUND_GREEN | FOREGROUND_INTENSITY;

	case _PX4_LOG_LEVEL_WARN:
		return background | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY;

	case _PX4_LOG_LEVEL_ERROR:
	case _PX4_LOG_LEVEL_PANIC:
		return background | FOREGROUND_RED | FOREGROUND_INTENSITY;

	case _PX4_LOG_LEVEL_INFO:
	default:
		return default_attributes;
	}
}

static WORD px4_windows_module_attributes(WORD default_attributes)
{
	const WORD background = default_attributes & 0xF0;
	return background | FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE;
}

static bool px4_windows_write_modulename_log(FILE *out, int level, const char *module_name, const char *message)
{
	HANDLE console_handle = nullptr;
	WORD default_attributes = 0;

	if (!px4_get_windows_console(out, &console_handle, &default_attributes)) {
		return false;
	}

	const WORD level_attributes = px4_windows_level_attributes(level, default_attributes);
	const WORD module_attributes = px4_windows_module_attributes(default_attributes);

	SetConsoleTextAttribute(console_handle, level_attributes);
	fprintf(out, __px4__log_level_fmt, __px4_log_level_str[level]);
	SetConsoleTextAttribute(console_handle, module_attributes);
	fprintf(out, __px4__log_modulename_pfmt, module_name);
	SetConsoleTextAttribute(console_handle, level == _PX4_LOG_LEVEL_INFO ? default_attributes : level_attributes);
	fprintf(out, "%s\n", message);
	SetConsoleTextAttribute(console_handle, default_attributes);
	return true;
}

static bool px4_windows_write_raw_log(FILE *out, int level, const char *message)
{
	HANDLE console_handle = nullptr;
	WORD default_attributes = 0;

	if (!px4_get_windows_console(out, &console_handle, &default_attributes)) {
		return false;
	}

	SetConsoleTextAttribute(console_handle, px4_windows_level_attributes(level, default_attributes));
	fputs(message, out);
	SetConsoleTextAttribute(console_handle, default_attributes);
	return true;
}
#endif

void px4_log_initialize(void)
{
	// we need to advertise with a valid message
	log_message_s log_message{};
	log_message.severity = 6; // info
	strcpy((char *)log_message.text, "initialized uORB logging");
	log_message.timestamp = hrt_absolute_time();
	orb_log_message_pub = orb_advertise(ORB_ID(log_message), &log_message);
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
	use_color = px4_should_use_color_output(isatty_);
#endif // PX4_LOG_COLORIZED_OUTPUT
#endif // PX4_POSIX

	if (level >= _PX4_LOG_LEVEL_INFO) {
		char message_buf[max_length + 1] {};
		char buf[max_length + 1]; // same length as log_message_s::text, but add newline
		ssize_t pos = 0;
		bool wrote_to_stdout = false;

		va_list argptr;
		va_start(argptr, fmt);
		vsnprintf(message_buf, max_length, fmt, argptr);
		va_end(argptr);

#if defined(PX4_LOG_COLORIZED_OUTPUT)
		char plain_message_buf[max_length + 1] {};
		px4_copy_without_ansi(plain_message_buf, sizeof(plain_message_buf), message_buf);
		const char *plain_message = plain_message_buf;
		const char *output_message = use_color ? message_buf : plain_message;
		const char *history_message = plain_message;
#else
		const char *output_message = message_buf;
		const char *history_message = message_buf;
#endif
		(void)history_message;

#if defined(__PX4_WINDOWS) && defined(PX4_LOG_COLORIZED_OUTPUT)
		const bool use_ansi_color = px4_windows_should_emit_ansi(out, use_color);

		if (use_color && !use_ansi_color
		    && px4_windows_write_modulename_log(out, level, module_name, plain_message)) {
			snprintf(buf, max_length, __px4__log_level_fmt, __px4_log_level_str[level]);
			pos = strlen(buf);
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), __px4__log_modulename_pfmt, module_name);
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s\n", plain_message);
			buf[max_length] = 0;
			wrote_to_stdout = true;
		}

		use_color = use_ansi_color;
		output_message = use_color ? message_buf : plain_message;
#endif

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", __px4_log_level_color[level]);
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

		pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", output_message);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			// always reset color
			const ssize_t sz = math::min(pos, max_length - (ssize_t)strlen(PX4_ANSI_COLOR_RESET) - (ssize_t)1);
			pos += snprintf(buf + sz, math::max(max_length - sz, (ssize_t)0), "%s\n", PX4_ANSI_COLOR_RESET);

		} else
#endif // PX4_LOG_COLORIZED_OUTPUT
		{
			pos += snprintf(buf + math::min(pos, max_length - (ssize_t)1), 2, "\n");
		}

		// ensure NULL termination (buffer is max_length + 1)
		buf[max_length] = 0;

#if defined(__PX4_WINDOWS) && defined(PX4_LOG_COLORIZED_OUTPUT)

		if (!wrote_to_stdout && use_color && px4_is_running_under_wine()) {
			fflush(out);
			wrote_to_stdout = px4_wine_write_stdout_direct(buf, strlen(buf));
		}

#endif

		if (!wrote_to_stdout) {
			fputs(buf, out);
		}

#if defined(BOARD_ENABLE_LOG_HISTORY)

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		// No color formatting for log history
		if (use_color) {
			pos = snprintf(buf, max_length, __px4__log_level_fmt, __px4_log_level_str[level]);
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), __px4__log_modulename_pfmt, module_name);
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", history_message);
			pos += snprintf(buf + math::min(pos, max_length - (ssize_t)1), 2, "\n");
			buf[max_length] = 0; // ensure NULL termination
		}

#endif
		g_log_history.write(buf);
#endif // BOARD_ENABLE_LOG_HISTORY

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
#if defined(PX4_LOG_COLORIZED_OUTPUT)
	use_color = px4_should_use_color_output(use_color);
#endif
#endif

	if (level >= _PX4_LOG_LEVEL_INFO) {
		static constexpr ssize_t max_length = sizeof(log_message_s::text);
		char message_buf[max_length + 1] {};
		char buf[max_length + 1]; // same length as log_message_s::text, but add newline
		ssize_t pos = 0;
		bool wrote_to_stdout = false;

		va_list argptr;
		va_start(argptr, fmt);
		vsnprintf(message_buf, max_length, fmt, argptr);
		va_end(argptr);

#if defined(PX4_LOG_COLORIZED_OUTPUT)
		char plain_message_buf[max_length + 1] {};
		px4_copy_without_ansi(plain_message_buf, sizeof(plain_message_buf), message_buf);
		const char *plain_message = plain_message_buf;
		const char *output_message = use_color ? message_buf : plain_message;
		const char *history_message = plain_message;
#else
		const char *output_message = message_buf;
		const char *history_message = message_buf;
#endif
		(void)history_message;

#if defined(__PX4_WINDOWS) && defined(PX4_LOG_COLORIZED_OUTPUT)
		const bool use_ansi_color = px4_windows_should_emit_ansi(out, use_color);

		if (use_color && !use_ansi_color
		    && px4_windows_write_raw_log(out, level, plain_message)) {
			snprintf(buf, max_length, "%s", plain_message);
			buf[max_length] = 0;
			wrote_to_stdout = true;
		}

		use_color = use_ansi_color;
		output_message = use_color ? message_buf : plain_message;
#endif

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", __px4_log_level_color[level]);
		}

#endif // PX4_LOG_COLORIZED_OUTPUT

		pos += snprintf(buf + pos, math::max(max_length - pos, (ssize_t)0), "%s", output_message);

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		if (use_color) {
			// alway reset color
			const ssize_t sz = math::min(pos, max_length - (ssize_t)strlen(PX4_ANSI_COLOR_RESET));
			pos += snprintf(buf + sz, math::max(max_length - sz, (ssize_t)0), "%s", PX4_ANSI_COLOR_RESET);
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

#if defined(__PX4_WINDOWS) && defined(PX4_LOG_COLORIZED_OUTPUT)

		if (!wrote_to_stdout && use_color && px4_is_running_under_wine()) {
			fflush(out);
			wrote_to_stdout = px4_wine_write_stdout_direct(buf, strlen(buf));
		}

#endif

		if (!wrote_to_stdout) {
			fputs(buf, out);
		}

#if defined(BOARD_ENABLE_LOG_HISTORY)

#if defined(PX4_LOG_COLORIZED_OUTPUT)

		// No color formatting for log history
		if (use_color) {
			pos = snprintf(buf, max_length, "%s", history_message);

			if (pos > max_length) {
				// preserve newline if necessary
				if (fmt[strlen(fmt) - 1] == '\n') {
					buf[max_length - 1] = '\n';
				}
			}

			buf[max_length] = 0; // ensure NULL termination
		}

#endif
		g_log_history.write(buf);
#endif // BOARD_ENABLE_LOG_HISTORY
	}
}

__EXPORT void px4_log_history(FILE *out)
{

#if defined(BOARD_ENABLE_LOG_HISTORY)

	g_log_history.print(out);
#endif // BOARD_ENABLE_LOG_HISTORY
}
