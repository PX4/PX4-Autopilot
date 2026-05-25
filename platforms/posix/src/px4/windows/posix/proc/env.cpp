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
 * @file env.cpp
 *
 * POSIX environment manipulation: setenv(3) and unsetenv(3),
 * routed through MSVCRT _putenv_s.
 *
 * _putenv_s updates the process environment visible to the CRT and child
 * processes launched through the CRT. That matches PX4's usage: startup
 * scripts and module code exchange small process-wide settings, not per-thread
 * environment snapshots.
 */

#include "px4_windows_internal.h"

extern "C" int setenv(const char *name, const char *value, int overwrite)
{
	if (!name || name[0] == '\0' || strchr(name, '=') != nullptr || !value) {
		errno = EINVAL;
		return -1;
	}

	if (!overwrite && getenv(name) != nullptr) {
		/* POSIX says setenv(..., overwrite=0) leaves an existing value intact. */
		return 0;
	}

	return _putenv_s(name, value) == 0 ? 0 : (errno = EINVAL, -1);
}

extern "C" int unsetenv(const char *name)
{
	if (!name || name[0] == '\0' || strchr(name, '=') != nullptr) {
		errno = EINVAL;
		return -1;
	}

	return _putenv_s(name, "") == 0 ? 0 : (errno = EINVAL, -1);
}
