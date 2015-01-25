/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file err.c
 *
 * Simple error/warning functions, heavily inspired by the BSD functions of
 * the same names.
 */

#include <nuttx/config.h>

#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "err.h"

#define NOCODE		1000	/* larger than maximum errno */

#if CONFIG_NFILE_STREAMS > 0
#  include <stdio.h>
#elif defined(CONFIG_ARCH_LOWPUTC)
#  include <debug.h>
extern int lib_lowvprintf(const char *fmt, va_list ap);
#else
#  warning Cannot output without one of CONFIG_NFILE_STREAMS or CONFIG_ARCH_LOWPUTC
#endif

const char *
getprogname(void)
{
#if CONFIG_TASK_NAME_SIZE > 0
	FAR struct tcb_s	*thisproc = sched_self();

	return thisproc->name;
#else
	return "app";
#endif
}

static void
warnerr_core(int errcode, const char *fmt, va_list args)
{
#if CONFIG_NFILE_STREAMS > 0
	fprintf(stderr, "%s: ", getprogname());
	vfprintf(stderr, fmt, args);

	/* convenience as many parts of NuttX use negative errno */
	if (errcode < 0)
		errcode = -errcode;

	if (errcode < NOCODE)
		fprintf(stderr, ": %s", strerror(errcode));

	fprintf(stderr, "\n");
#elif CONFIG_ARCH_LOWPUTC
	lowsyslog("%s: ", getprogname());
	lowvsyslog(fmt, args);

	/* convenience as many parts of NuttX use negative errno */
	if (errcode < 0)
		errcode = -errcode;

	if (errcode < NOCODE)
		lowsyslog(": %s", strerror(errcode));

	lowsyslog("\n");
#endif
}

void
err(int exitcode, const char *fmt, ...)
{
	va_list	args;

	va_start(args, fmt);
	verr(exitcode, fmt, args);
}

void
verr(int exitcode, const char *fmt, va_list args)
{
	warnerr_core(errno, fmt, args);
	exit(exitcode);
}

void
errc(int exitcode, int errcode, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	verrc(exitcode, errcode, fmt, args);
}

void
verrc(int exitcode, int errcode, const char *fmt, va_list args)
{
	warnerr_core(errcode, fmt, args);
	exit(exitcode);
}

void
errx(int exitcode, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	verrx(exitcode, fmt, args);
}

void
verrx(int exitcode, const char *fmt, va_list args)
{
	warnerr_core(NOCODE, fmt, args);
	exit(exitcode);
}

void
warn(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vwarn(fmt, args);
	va_end(args);
}

void
vwarn(const char *fmt, va_list args)
{
	warnerr_core(errno, fmt, args);
}

void
warnc(int errcode, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vwarnc(errcode, fmt, args);
	va_end(args);
}

void
vwarnc(int errcode, const char *fmt, va_list args)
{
	warnerr_core(errcode, fmt, args);
}

void
warnx(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vwarnx(fmt, args);
	va_end(args);
}

void
vwarnx(const char *fmt, va_list args)
{
	warnerr_core(NOCODE, fmt, args);
}
