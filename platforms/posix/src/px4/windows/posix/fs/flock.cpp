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
 * @file flock.cpp
 *
 * POSIX sys/file.h: flock(2) via LockFileEx / UnlockFileEx.
 * Used by px4_daemon server-singleton enforcement.
 *
 * PX4 only needs advisory whole-file locks here. Windows exposes byte-range
 * locks, so the implementation locks the maximum byte range from offset 0 and
 * maps ERROR_LOCK_VIOLATION back to the POSIX non-blocking-lock error.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * sys/file: flock via LockFileEx.
 * -------------------------------------------------------------------------- */
extern "C" int flock(int fd, int operation)
{
	HANDLE h = (HANDLE)_get_osfhandle(fd);

	if (h == INVALID_HANDLE_VALUE) { errno = EBADF; return -1; }

	if (operation & LOCK_UN) {
		/* Match the whole-file range used for LOCK_SH/LOCK_EX below. */
		OVERLAPPED ov{}; ov.Offset = 0; ov.OffsetHigh = 0;
		return UnlockFileEx(h, 0, MAXDWORD, MAXDWORD, &ov) ? 0 : -1;
	}

	DWORD flags = 0;

	if (operation & LOCK_EX) { flags |= LOCKFILE_EXCLUSIVE_LOCK; }

	if (operation & LOCK_NB) { flags |= LOCKFILE_FAIL_IMMEDIATELY; }

	OVERLAPPED ov{}; ov.Offset = 0; ov.OffsetHigh = 0;

	if (LockFileEx(h, flags, 0, MAXDWORD, MAXDWORD, &ov)) { return 0; }

	errno = (GetLastError() == ERROR_LOCK_VIOLATION) ? EWOULDBLOCK : EIO;
	return -1;
}
