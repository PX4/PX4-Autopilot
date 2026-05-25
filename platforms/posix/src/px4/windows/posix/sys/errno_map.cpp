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
 * @file errno_map.cpp
 *
 * Translation tables from Win32 / Winsock error codes into POSIX
 * errno values. Shared by every shim that pipes through the
 * Windows API boundary.
 *
 * Prefer a coarse but stable errno over leaking Windows-specific error values.
 * Callers usually log strerror(errno) or branch on broad categories such as
 * EAGAIN, ENOENT, EACCES, and ECONNREFUSED.
 */

#include "px4_windows_internal.h"

int px4_win_error_to_errno(DWORD err)
{
	switch (err) {
	case ERROR_FILE_NOT_FOUND:
	case ERROR_PATH_NOT_FOUND:
	case ERROR_INVALID_DRIVE:
	case ERROR_BAD_PATHNAME:
		return ENOENT;

	case ERROR_ACCESS_DENIED:
	case ERROR_SHARING_VIOLATION:
	case ERROR_LOCK_VIOLATION:
		return EACCES;

	case ERROR_ALREADY_EXISTS:
	case ERROR_FILE_EXISTS:
		return EEXIST;

	case ERROR_INVALID_HANDLE:
		return EBADF;

	case ERROR_NOT_ENOUGH_MEMORY:
	case ERROR_OUTOFMEMORY:
		return ENOMEM;

	case ERROR_DIRECTORY:
	case ERROR_INVALID_NAME:
	case ERROR_INVALID_PARAMETER:
		return EINVAL;

	case ERROR_DIR_NOT_EMPTY:
		return ENOTEMPTY;

	case ERROR_NOT_SUPPORTED:
	case ERROR_CALL_NOT_IMPLEMENTED:
		return ENOTSUP;

	case ERROR_BUSY:
		return EBUSY;

	case ERROR_DISK_FULL:
		return ENOSPC;

	case ERROR_PROC_NOT_FOUND:
		return ESRCH;

	default:
		/* EIO is the safest "something at the OS boundary failed" fallback. */
		return EIO;
	}
}

int px4_wsa_error_to_errno(int err)
{
	switch (err) {
	case WSAEWOULDBLOCK: return EWOULDBLOCK;

	case WSAEINPROGRESS: return EINPROGRESS;

	case WSAEALREADY: return EALREADY;

	case WSAENOTSOCK: return ENOTSOCK;

	case WSAEDESTADDRREQ: return EDESTADDRREQ;

	case WSAEMSGSIZE: return EMSGSIZE;

	case WSAEPROTOTYPE: return EPROTOTYPE;

	case WSAENOPROTOOPT: return ENOPROTOOPT;

	case WSAEPROTONOSUPPORT: return EPROTONOSUPPORT;

	case WSAEOPNOTSUPP: return EOPNOTSUPP;

	case WSAEAFNOSUPPORT: return EAFNOSUPPORT;

	case WSAEADDRINUSE: return EADDRINUSE;

	case WSAEADDRNOTAVAIL: return EADDRNOTAVAIL;

	case WSAENETDOWN: return ENETDOWN;

	case WSAENETUNREACH: return ENETUNREACH;

	case WSAENETRESET: return ENETRESET;

	case WSAECONNABORTED: return ECONNABORTED;

	case WSAECONNRESET: return ECONNRESET;

	case WSAENOBUFS: return ENOBUFS;

	case WSAEISCONN: return EISCONN;

	case WSAENOTCONN: return ENOTCONN;

	case WSAETIMEDOUT: return ETIMEDOUT;

	case WSAECONNREFUSED: return ECONNREFUSED;

	default: return EIO;
	}
}

const char *px4_hstrerror_text(int err)
{
	switch (err) {
	case HOST_NOT_FOUND: return "Unknown host";

	case TRY_AGAIN: return "Temporary failure in name resolution";

	case NO_RECOVERY: return "Non-recoverable name server error";

	case NO_DATA: return "No address associated with name";

	default: return "Resolver error";
	}
}
