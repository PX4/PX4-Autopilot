/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file err.h
 *
 * Simple error/warning functions, heavily inspired by the BSD functions of
 * the same names.
 *
 * The err() and warn() family of functions display a formatted error
 * message on the standard error output.  In all cases, the last
 * component of the program name, a colon character, and a space are
 * output.  If the fmt argument is not NULL, the printf(3)-like formatted
 * error message is output.  The output is terminated by a newline
 * character.
 *
 * The err(), errc(), verr(), verrc(), warn(), warnc(), vwarn(), and
 * vwarnc() functions append an error message obtained from strerror(3)
 * based on a supplied error code value or the global variable errno,
 * preceded by another colon and space unless the fmt argument is NULL.
 *
 * In the case of the errc(), verrc(), warnc(), and vwarnc() functions,
 * the code argument is used to look up the error message.
 *
 * The err(), verr(), warn(), and vwarn() functions use the global
 * variable errno to look up the error message.
 *
 * The errx() and warnx() functions do not append an error message.
 *
 * The err(), verr(), errc(), verrc(), errx(), and verrx() functions do
 * not return, but exit with the value of the argument eval.
 *
 */

#ifndef _SYSTEMLIB_ERR_H
#define _SYSTEMLIB_ERR_H

#include <px4_platform_common/log.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>

__BEGIN_DECLS


#ifdef __PX4_NUTTX
#define EXIT(eval) exit(eval)
#else
#define EXIT(eval) px4_task_exit(eval)
#endif


#define err(eval, ...)		do { \
		PX4_ERR(__VA_ARGS__); \
		PX4_ERR("Task exited with errno=%i\n", errno); \
		EXIT(eval); \
	} while(0)

#define errx(eval, ...)		do { \
		PX4_ERR(__VA_ARGS__); \
		EXIT(eval); \
	} while(0)

#define warn(...) 		PX4_WARN(__VA_ARGS__)
#define warnx(...) 		PX4_WARN(__VA_ARGS__)

__END_DECLS

#endif
