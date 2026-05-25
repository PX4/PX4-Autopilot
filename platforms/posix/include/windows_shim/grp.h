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
 * @file grp.h
 *
 * Windows has no /etc/group. Like pwd.h, this shim synthesizes a
 * minimal current-user group entry so POSIX consumers can compile and
 * basic lookups continue to work.
 */
#pragma once

#include <stddef.h>
#include <sys/types.h>

#ifndef _PX4_UID_T_DEFINED_SHIM
#define _PX4_UID_T_DEFINED_SHIM
typedef unsigned int uid_t;
typedef unsigned int gid_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POSIX group entry synthesized for the current Windows user.
 */
struct group {
	char  *gr_name;
	char  *gr_passwd;
	gid_t  gr_gid;
	char **gr_mem;
};

/** @brief Return a static group entry for the requested GID when available. */
struct group *getgrgid(gid_t gid);

/** @brief Return a static group entry for the requested group name. */
struct group *getgrnam(const char *name);

/**
 * @brief Reentrant GID lookup.
 *
 * @return 0 on success or not found; @p result is NULL when no entry matches.
 *         Returns an errno value when @p buf is too small or arguments are
 *         invalid.
 */
int           getgrgid_r(gid_t gid, struct group *grp, char *buf, size_t buflen, struct group **result);

/**
 * @brief Reentrant group-name lookup.
 *
 * @return 0 on success or not found; @p result is NULL when no entry matches.
 *         Returns an errno value when @p buf is too small or arguments are
 *         invalid.
 */
int           getgrnam_r(const char *name, struct group *grp, char *buf, size_t buflen, struct group **result);

#ifdef __cplusplus
}
#endif
