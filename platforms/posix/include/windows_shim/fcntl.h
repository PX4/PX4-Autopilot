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
 * @file fcntl.h
 *
 * MinGW-w64 ships a minimal <fcntl.h> (O_RDONLY / O_BINARY / O_CREAT /
 * etc.) but none of the POSIX fcntl() operations - F_GETFL, F_SETFL,
 * F_GETFD, F_SETFD, O_NONBLOCK - and no fcntl() function. PX4 uses
 * these only to toggle non-blocking I/O, so we satisfy the interface
 * and route the one semantically meaningful op (O_NONBLOCK) onto
 * ioctlsocket(FIONBIO) / _setmode as appropriate.
 */
#pragma once

#if defined(_MSC_VER)
#if defined(__has_include)
#  if __has_include(<../ucrt/fcntl.h>)
#    include <../ucrt/fcntl.h>
#  endif
#endif
#include <io.h>
#include <stdio.h>
#ifndef O_RDONLY
#define O_RDONLY 0x0000
#endif
#ifndef O_WRONLY
#define O_WRONLY 0x0001
#endif
#ifndef O_RDWR
#define O_RDWR   0x0002
#endif
#ifndef O_APPEND
#define O_APPEND 0x0008
#endif
#ifndef O_CREAT
#define O_CREAT  0x0100
#endif
#ifndef O_TRUNC
#define O_TRUNC  0x0200
#endif
#ifndef O_EXCL
#define O_EXCL   0x0400
#endif
#ifndef O_TEXT
#define O_TEXT   0x4000
#endif
#ifndef O_BINARY
#define O_BINARY 0x8000
#endif
#else
#include_next <fcntl.h>
#endif

#include <windows.h>
#include <winsock2.h>
#include <io.h>
#include <errno.h>
#include <stdarg.h>
#include <string.h>

/* <windows.h> (pulled in transitively by <winsock2.h>) leaks a handful
 * of all-caps macros that collide with PX4 identifiers. Undefine them
 * centrally here - this header sits at the top of posix.h's include
 * chain, so every PX4 translation unit that touches winsock goes
 * through these undefs before any C++ code uses the names. */
/* wingdi.h defines ERROR as 0, winbase.h defines IGNORE as 0, winnt.h
 * defines OPTIONAL as empty. All three clash with identifiers PX4 uses
 * (ERROR=-1 from defines.h, FailureMode::IGNORE enumerator, and
 * `OPTIONAL` function-argument annotations in third-party code). Strip
 * the Windows leak, then - to cover the case where defines.h was
 * already processed earlier in this TU - restore PX4's ERROR. */
#ifdef ERROR
#undef ERROR
#endif
#if defined(__PX4_POSIX)
#define ERROR (-1)
#endif
#ifdef IGNORE
#undef IGNORE
#endif
#ifdef OPTIONAL
#undef OPTIONAL
#endif

#ifndef F_GETFD
#define F_GETFD 1
#endif
#ifndef F_DUPFD
#define F_DUPFD 0
#endif
#ifndef F_DUPFD_CLOEXEC
#define F_DUPFD_CLOEXEC 1030
#endif
#ifndef F_SETFD
#define F_SETFD 2
#endif
#ifndef F_GETFL
#define F_GETFL 3
#endif
#ifndef F_SETFL
#define F_SETFL 4
#endif
#ifndef F_GETLK
#define F_GETLK 5
#endif
#ifndef F_SETLK
#define F_SETLK 6
#endif
#ifndef F_SETLKW
#define F_SETLKW 7
#endif
#ifndef F_RDLCK
#define F_RDLCK 0
#endif
#ifndef F_WRLCK
#define F_WRLCK 1
#endif
#ifndef F_UNLCK
#define F_UNLCK 2
#endif
#ifndef FD_CLOEXEC
#define FD_CLOEXEC 1
#endif
#ifndef AT_FDCWD
#define AT_FDCWD (-100)
#endif
#ifndef AT_SYMLINK_NOFOLLOW
#define AT_SYMLINK_NOFOLLOW 0x100
#endif
#ifndef AT_REMOVEDIR
#define AT_REMOVEDIR 0x200
#endif
#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#ifndef O_DIRECTORY
#define O_DIRECTORY 0
#endif
#ifndef O_NOFOLLOW
#define O_NOFOLLOW 0
#endif
#ifndef O_SYNC
#define O_SYNC 0
#endif
#ifndef O_DSYNC
#define O_DSYNC O_SYNC
#endif

/**
 * @brief POSIX advisory byte-range lock descriptor.
 *
 * Windows has no fcntl-style locking;
 * F_GETLK/F_SETLK are implemented below via LockFileEx/UnlockFileEx,
 * which lock a byte range of a HANDLE. PX4 uses the "whole file"
 * convention (l_start=0, l_len=0) for server singleton enforcement;
 * that maps cleanly onto a max-range LockFileEx call.
 */
#ifndef _PX4_STRUCT_FLOCK_DEFINED
#define _PX4_STRUCT_FLOCK_DEFINED
struct flock {
	short  l_type;    /* F_RDLCK / F_WRLCK / F_UNLCK */
	short  l_whence;  /* SEEK_SET / SEEK_CUR / SEEK_END */
	long   l_start;
	long   l_len;     /* 0 == lock through EOF */
	int    l_pid;
};
#endif

/* O_NONBLOCK isn't defined by MinGW's fcntl.h. Pick a high bit that
 * does not collide with the O_* flags MinGW does define. */
#ifndef O_NONBLOCK
#if defined(_MSC_VER)
#define O_NONBLOCK 0x200000
#else
#define O_NONBLOCK 0x4000
#endif
#endif

/* POSIX terminal-control flag; Windows has no controlling terminal
 * concept, so we accept the flag and ignore it. */
#ifndef O_NOCTTY
#define O_NOCTTY 0
#endif

#ifndef O_NDELAY
#define O_NDELAY O_NONBLOCK
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief POSIX fcntl() compatibility subset.
 *
 * Supported operations are the descriptor/status queries PX4 uses,
 * FD_CLOEXEC acceptance, O_NONBLOCK enablement for Winsock sockets, and
 * advisory file locking through LockFileEx.
 *
 * @return 0 or a positive query value on success, -1 with errno set on failure.
 */
static inline int fcntl(int fd, int cmd, ...)
{
	switch (cmd) {
	case F_GETFL:
	case F_GETFD:
		/* No file-descriptor flag storage on Windows CRT fds. Pretend
		 * we got back 0 - PX4 only OR's O_NONBLOCK onto the result
		 * before handing it back via F_SETFL. */
		return 0;

	case F_SETFL: {
			va_list ap;
			va_start(ap, cmd);
			int flags = va_arg(ap, int);
			va_end(ap);

			if (flags & O_NONBLOCK) {
				/* Socket fds live in a separate namespace on winsock -
				 * ioctlsocket works on SOCKET handles, which the CRT
				 * stores directly in its int fds via _open_osfhandle. */
				u_long mode = 1;

				if (ioctlsocket((SOCKET)fd, FIONBIO, &mode) == 0) {
					return 0;
				}

				errno = EBADF;
				return -1;
			}

			return 0;
		}

	case F_SETFD:
		/* FD_CLOEXEC has no Win32 analog - close-on-exec is the
		 * default when bInheritHandle is false, which is the CRT
		 * default. Accept the call so code compiles. */
		return 0;

	case F_GETLK:
	case F_SETLK:
	case F_SETLKW: {
			va_list ap;
			va_start(ap, cmd);
			struct flock *fl = va_arg(ap, struct flock *);
			va_end(ap);

			if (!fl) { errno = EINVAL; return -1; }

			HANDLE h = (HANDLE)_get_osfhandle(fd);

			if (h == INVALID_HANDLE_VALUE) { errno = EBADF; return -1; }

			/* l_len == 0 means "lock through EOF" in POSIX. Use the max
			 * range LockFileEx accepts (2^63-1) so the byte-range lock
			 * effectively covers the whole file. */
			DWORD off_lo = (DWORD)fl->l_start;
			DWORD off_hi = 0;
			DWORD len_lo = fl->l_len ? (DWORD)fl->l_len : 0xFFFFFFFFu;
			DWORD len_hi = fl->l_len ? 0 : 0x7FFFFFFFu;
			OVERLAPPED ov;
			memset(&ov, 0, sizeof(ov));
			ov.Offset = off_lo;
			ov.OffsetHigh = off_hi;

			if (cmd == F_GETLK) {
				/* Try to acquire non-blocking; if it succeeds no conflict
				 * exists, so release and report F_UNLCK. If it fails with
				 * ERROR_LOCK_VIOLATION something else holds a lock. */
				DWORD flags = LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY;

				if (LockFileEx(h, flags, 0, len_lo, len_hi, &ov)) {
					UnlockFileEx(h, 0, len_lo, len_hi, &ov);
					fl->l_type = F_UNLCK;

				} else if (GetLastError() == ERROR_LOCK_VIOLATION
					   || GetLastError() == ERROR_IO_PENDING) {
					fl->l_type = F_WRLCK;
					fl->l_pid = 0;

				} else {
					errno = EACCES;
					return -1;
				}

				return 0;
			}

			if (fl->l_type == F_UNLCK) {
				if (UnlockFileEx(h, 0, len_lo, len_hi, &ov)) { return 0; }

				errno = EACCES;
				return -1;
			}

			DWORD flags = LOCKFILE_EXCLUSIVE_LOCK;

			if (cmd == F_SETLK) { flags |= LOCKFILE_FAIL_IMMEDIATELY; }

			if (LockFileEx(h, flags, 0, len_lo, len_hi, &ov)) { return 0; }

			errno = (GetLastError() == ERROR_LOCK_VIOLATION) ? EAGAIN : EACCES;
			return -1;
		}

	default:
		errno = EINVAL;
		return -1;
	}
}

#ifdef __cplusplus
}
#endif
