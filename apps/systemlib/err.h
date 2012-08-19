/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 */

#ifndef _SYSTEMLIB_ERR_H
#define _SYSTEMLIB_ERR_H

#include <stdarg.h>

__BEGIN_DECLS

__EXPORT const char *getprogname(void);

__EXPORT void	err(int, const char *, ...) __attribute__((noreturn,format(printf,2, 3)));
__EXPORT void	verr(int, const char *, va_list) __attribute__((noreturn,format(printf,2, 0)));
__EXPORT void	errc(int, int, const char *, ...) __attribute__((noreturn,format(printf,3, 4)));
__EXPORT void	verrc(int, int, const char *, va_list) __attribute__((noreturn,format(printf,3, 0)));
__EXPORT void	errx(int, const char *, ...) __attribute__((noreturn,format(printf,2, 3)));
__EXPORT void	verrx(int, const char *, va_list) __attribute__((noreturn,format(printf,2, 0)));
__EXPORT void	warn(const char *, ...)  __attribute__((format(printf,1, 2)));
__EXPORT void	vwarn(const char *, va_list)  __attribute__((format(printf,1, 0)));
__EXPORT void	warnc(int, const char *, ...)  __attribute__((format(printf,2, 3)));
__EXPORT void	vwarnc(int, const char *, va_list)  __attribute__((format(printf,2, 0)));
__EXPORT void	warnx(const char *, ...)  __attribute__((format(printf,1, 2)));
__EXPORT void	vwarnx(const char *, va_list)  __attribute__((format(printf,1, 0)));

__END_DECLS

#endif
