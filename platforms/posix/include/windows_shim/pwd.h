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
 * @file pwd.h
 *
 * Windows has no /etc/passwd. SITL consumers only ever want the current
 * user's home directory or name - we synthesise a passwd entry from
 * %USERPROFILE% / GetUserName to keep them building and (mostly) working.
 */
#pragma once

#include <stddef.h>
#include <sys/types.h>

/* MinGW's sys/types.h does not define uid_t/gid_t. */
#ifndef _PX4_UID_T_DEFINED_SHIM
#define _PX4_UID_T_DEFINED_SHIM
typedef unsigned int uid_t;
typedef unsigned int gid_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POSIX passwd entry synthesized from the current Windows user.
 */
struct passwd {
	char *pw_name;
	char *pw_passwd;
	uid_t pw_uid;
	gid_t pw_gid;
	char *pw_gecos;
	char *pw_dir;
	char *pw_shell;
};

/** @brief Return a static passwd entry for the requested UID when available. */
struct passwd *getpwuid(uid_t uid);

/** @brief Return a static passwd entry for the requested user name. */
struct passwd *getpwnam(const char *name);

/**
 * @brief Reentrant UID lookup.
 *
 * @return 0 on success or not found; @p result is NULL when no entry matches.
 *         Returns an errno value when @p buf is too small or arguments are
 *         invalid.
 */
int            getpwuid_r(uid_t uid, struct passwd *pwd, char *buf, size_t buflen, struct passwd **result);

/**
 * @brief Reentrant name lookup.
 *
 * @return 0 on success or not found; @p result is NULL when no entry matches.
 *         Returns an errno value when @p buf is too small or arguments are
 *         invalid.
 */
int            getpwnam_r(const char *name, struct passwd *pwd, char *buf, size_t buflen, struct passwd **result);

#ifdef __cplusplus
}
#endif
