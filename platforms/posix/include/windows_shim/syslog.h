/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file syslog.h
 *
 * MinGW has no <syslog.h>. Most PX4 call sites only include the header
 * (to satisfy printf-style macros through other logging layers) rather
 * than actually calling syslog(), so we provide a header with the
 * standard priority levels and a syslog() that forwards to stderr.
 */
#pragma once

#include <stdarg.h>
#include <stdio.h>

#define LOG_KERN    (0 << 3)
#define LOG_USER    (1 << 3)
#define LOG_MAIL    (2 << 3)
#define LOG_DAEMON  (3 << 3)
#define LOG_AUTH    (4 << 3)
#define LOG_SYSLOG  (5 << 3)
#define LOG_LPR     (6 << 3)
#define LOG_NEWS    (7 << 3)
#define LOG_UUCP    (8 << 3)
#define LOG_CRON    (9 << 3)
#define LOG_AUTHPRIV (10 << 3)
#define LOG_FTP     (11 << 3)
#define LOG_LOCAL0  (16 << 3)
#define LOG_LOCAL1  (17 << 3)
#define LOG_LOCAL2  (18 << 3)
#define LOG_LOCAL3  (19 << 3)
#define LOG_LOCAL4  (20 << 3)
#define LOG_LOCAL5  (21 << 3)
#define LOG_LOCAL6  (22 << 3)
#define LOG_LOCAL7  (23 << 3)

#define LOG_EMERG   0
#define LOG_ALERT   1
#define LOG_CRIT    2
#define LOG_ERR     3
#define LOG_WARNING 4
#define LOG_NOTICE  5
#define LOG_INFO    6
#define LOG_DEBUG   7

#define LOG_PID     0x01
#define LOG_CONS    0x02
#define LOG_NDELAY  0x08
#define LOG_NOWAIT  0x10
#define LOG_PERROR  0x20

#define LOG_PRIMASK 0x07
#define LOG_FACMASK 0x03f8
#define LOG_PRI(p)  ((p) & LOG_PRIMASK)
#define LOG_FAC(p)  (((p) & LOG_FACMASK) >> 3)
#define LOG_MAKEPRI(fac, pri) ((fac) | (pri))
#define LOG_MASK(pri) (1 << (pri))
#define LOG_UPTO(pri) ((1 << ((pri) + 1)) - 1)

#ifdef __cplusplus
extern "C" {
#endif

static int px4_syslog_mask = LOG_UPTO(LOG_DEBUG);

/** @brief Accept syslog identity/options; no persistent Windows sink is opened. */
static inline void openlog(const char *ident, int option, int facility)
{
	(void)ident; (void)option; (void)facility;
}

/** @brief Close the syslog sink; no-op for the stderr-backed shim. */
static inline void closelog() {}

/**
 * @brief Set the active syslog priority mask.
 *
 * @return Previous mask, matching POSIX setlogmask().
 */
static inline int setlogmask(int maskpri)
{
	const int old_mask = px4_syslog_mask;

	if (maskpri != 0) {
		px4_syslog_mask = maskpri;
	}

	return old_mask;
}

/** @brief Write a masked syslog message to stderr with a trailing newline. */
static inline void vsyslog(int priority, const char *fmt, va_list ap)
{
	if ((LOG_MASK(LOG_PRI(priority)) & px4_syslog_mask) == 0) {
		return;
	}

	vfprintf(stderr, fmt, ap);
	fputc('\n', stderr);
}

/** @brief Variadic syslog wrapper around vsyslog(). */
static inline void syslog(int priority, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsyslog(priority, fmt, ap);
	va_end(ap);
}

#ifdef __cplusplus
}
#endif
