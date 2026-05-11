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
 * @file user.cpp
 *
 * POSIX user / group database shims: pwd.h and grp.h. Synthesised
 * from %USERPROFILE% / GetUserName so code that calls
 * getpwuid / getpwnam works without a real /etc/passwd.
 *
 * The UID/GID values are stable placeholders. PX4 only needs a home directory,
 * user name, and successful lookups for logging/config paths; Windows account
 * SIDs are not exposed through the POSIX structs.
 */

#include "px4_windows_internal.h"

#include <array>

/* --------------------------------------------------------------------------
 * pwd/grp: synthesise an entry from %USERPROFILE% / GetUserName.
 * -------------------------------------------------------------------------- */
static thread_local char _pw_name[256];
static thread_local char _pw_dir[MAX_PATH];
static thread_local char _pw_shell[] = "cmd.exe";
static thread_local char _pw_passwd[] = "x";
static thread_local struct passwd _pw_entry;
static thread_local char _gr_name[256];
static thread_local char _gr_passwd[] = "x";
static thread_local char *_gr_mem[2] = {nullptr, nullptr};
static thread_local struct group _gr_entry;

static int append_buf_string(char **cursor, size_t *remaining, const char *src, char **dst)
{
	if (!cursor || !remaining || !dst || !src) {
		errno = EINVAL;
		return ERANGE;
	}

	const size_t len = strlen(src) + 1;

	if (*remaining < len) {
		errno = ERANGE;
		return ERANGE;
	}

	memcpy(*cursor, src, len);
	/* The *_r APIs require all pointed-to strings to live inside the caller
	 * buffer. Advance a simple bump pointer through that buffer. */
	*dst = *cursor;
	*cursor += len;
	*remaining -= len;
	return 0;
}

static int align_buf(char **cursor, size_t *remaining, size_t alignment)
{
	/* group::gr_mem is a char** stored inside the caller buffer. Keep it aligned
	 * even when the supplied buffer starts at an odd address. */
	uintptr_t ptr = reinterpret_cast<uintptr_t>(*cursor);
	const size_t misalignment = ptr % alignment;

	if (misalignment == 0) {
		return 0;
	}

	const size_t padding = alignment - misalignment;

	if (*remaining < padding) {
		errno = ERANGE;
		return ERANGE;
	}

	*cursor += padding;
	*remaining -= padding;
	return 0;
}

static struct passwd *fill_passwd()
{
	DWORD n = sizeof(_pw_name);

	if (!GetUserNameA(_pw_name, &n)) {
		/* GetUserNameA failed; fall back to a stable placeholder. Use a bounded
		 * copy rather than strcpy() so MSVC does not flag the deprecated CRT
		 * API and we cannot accidentally overflow if _pw_name shrinks later. */
		strncpy(_pw_name, "px4", sizeof(_pw_name) - 1);
		_pw_name[sizeof(_pw_name) - 1] = '\0';
	}

	const char *home = getenv("USERPROFILE");

	if (!home) { home = "C:\\"; }

	strncpy(_pw_dir, home, sizeof(_pw_dir) - 1);
	_pw_dir[sizeof(_pw_dir) - 1] = 0;

	_pw_entry.pw_name   = _pw_name;
	_pw_entry.pw_passwd = _pw_passwd;
	_pw_entry.pw_uid    = 1000;
	_pw_entry.pw_gid    = 1000;
	_pw_entry.pw_gecos  = _pw_name;
	_pw_entry.pw_dir    = _pw_dir;
	_pw_entry.pw_shell  = _pw_shell;
	return &_pw_entry;
}

static struct group *fill_group()
{
	struct passwd *pw = fill_passwd();
	strncpy(_gr_name, pw->pw_name, sizeof(_gr_name) - 1);
	_gr_name[sizeof(_gr_name) - 1] = '\0';
	_gr_mem[0] = _gr_name;
	_gr_mem[1] = nullptr;

	_gr_entry.gr_name = _gr_name;
	_gr_entry.gr_passwd = _gr_passwd;
	_gr_entry.gr_gid = 1000;
	_gr_entry.gr_mem = _gr_mem;
	return &_gr_entry;
}

static int fill_passwd_r(struct passwd *pwd, char *buf, size_t buflen, struct passwd **result)
{
	if (!pwd || !buf || !result) {
		errno = EINVAL;
		return EINVAL;
	}

	struct passwd *src = fill_passwd();

	char *cursor = buf;

	size_t remaining = buflen;

	int rc = 0;

	rc = append_buf_string(&cursor, &remaining, src->pw_name, &pwd->pw_name);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->pw_passwd, &pwd->pw_passwd);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->pw_gecos, &pwd->pw_gecos);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->pw_dir, &pwd->pw_dir);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->pw_shell, &pwd->pw_shell);

	if (rc != 0) { *result = nullptr; return rc; }

	pwd->pw_uid = src->pw_uid;
	pwd->pw_gid = src->pw_gid;
	*result = pwd;
	return 0;
}

static int fill_group_r(struct group *grp, char *buf, size_t buflen, struct group **result)
{
	if (!grp || !buf || !result) {
		errno = EINVAL;
		return EINVAL;
	}

	struct group *src = fill_group();

	char *cursor = buf;

	size_t remaining = buflen;

	int rc = 0;

	rc = align_buf(&cursor, &remaining, alignof(char *));

	if (rc != 0) { *result = nullptr; return rc; }

	if (remaining < sizeof(char *) * 2) {
		errno = ERANGE;
		*result = nullptr;
		return ERANGE;
	}

	grp->gr_mem = reinterpret_cast<char **>(cursor);
	cursor += sizeof(char *) * 2;
	remaining -= sizeof(char *) * 2;

	rc = append_buf_string(&cursor, &remaining, src->gr_name, &grp->gr_name);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->gr_passwd, &grp->gr_passwd);

	if (rc != 0) { *result = nullptr; return rc; }

	rc = append_buf_string(&cursor, &remaining, src->gr_mem[0], &grp->gr_mem[0]);

	if (rc != 0) { *result = nullptr; return rc; }

	grp->gr_mem[1] = nullptr;
	grp->gr_gid = src->gr_gid;
	*result = grp;
	return 0;
}

extern "C" struct passwd *getpwuid(uid_t) { return fill_passwd(); }
extern "C" struct passwd *getpwnam(const char *name)
{
	struct passwd *pw = fill_passwd();
	return (!name || strcmp(name, pw->pw_name) == 0) ? pw : nullptr;
}
extern "C" int getpwuid_r(uid_t uid, struct passwd *pwd, char *buf, size_t buflen, struct passwd **result)
{
	(void)uid;
	return fill_passwd_r(pwd, buf, buflen, result);
}
extern "C" int getpwnam_r(const char *name, struct passwd *pwd, char *buf, size_t buflen, struct passwd **result)
{
	struct passwd *pw = fill_passwd();

	if (name && strcmp(name, pw->pw_name) != 0) {
		*result = nullptr;
		return 0;
	}

	return fill_passwd_r(pwd, buf, buflen, result);
}
extern "C" struct group *getgrgid(gid_t gid)
{
	struct group *grp = fill_group();
	return (gid == grp->gr_gid) ? grp : nullptr;
}
extern "C" struct group *getgrnam(const char *name)
{
	struct group *grp = fill_group();
	return (!name || strcmp(name, grp->gr_name) == 0) ? grp : nullptr;
}
extern "C" int getgrgid_r(gid_t gid, struct group *grp, char *buf, size_t buflen, struct group **result)
{
	struct group *src = fill_group();

	if (gid != src->gr_gid) {
		*result = nullptr;
		return 0;
	}

	return fill_group_r(grp, buf, buflen, result);
}
extern "C" int getgrnam_r(const char *name, struct group *grp, char *buf, size_t buflen, struct group **result)
{
	struct group *src = fill_group();

	if (name && strcmp(name, src->gr_name) != 0) {
		*result = nullptr;
		return 0;
	}

	return fill_group_r(grp, buf, buflen, result);
}
