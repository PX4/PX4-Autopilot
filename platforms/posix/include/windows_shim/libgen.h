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
 * @file libgen.h
 *
 * basename()/dirname() helpers for Windows builds.
 */

#pragma once

#ifdef _WIN32

#include <string.h>

/* This header is consumed from both C and C++ translation units. Provide a
 * fall-back macro so the inline shim bodies below can spell `nullptr` while
 * remaining valid in C, where `nullptr` is reserved before C23. */
#if !defined(__cplusplus) && !defined(nullptr)
#  define nullptr NULL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return the final path component in place.
 *
 * Accepts both POSIX '/' and Windows '\\' separators. The returned pointer
 * aliases @p path or points to a static "." literal for empty paths.
 */
static inline char *basename(char *path)
{
	if (!path || !*path) {
		return (char *)".";
	}

	char *last = path;

	for (char *p = path; *p; ++p) {
		if (*p == '/' || *p == '\\') {
			last = p + 1;
		}
	}

	return *last ? last : (char *)".";
}

/**
 * @brief Replace the final path separator with NUL and return the directory.
 *
 * Accepts both POSIX '/' and Windows '\\' separators. The returned pointer
 * aliases @p path or points to a static "." literal when no directory exists.
 */
static inline char *dirname(char *path)
{
	if (!path || !*path) {
		return (char *)".";
	}

	char *last = nullptr;

	for (char *p = path; *p; ++p) {
		if (*p == '/' || *p == '\\') {
			last = p;
		}
	}

	if (!last) {
		return (char *)".";
	}

	if (last == path) {
		last[1] = '\0';
		return path;
	}

	*last = '\0';
	return path;
}

#ifdef __cplusplus
}
#endif

#endif /* _WIN32 */
