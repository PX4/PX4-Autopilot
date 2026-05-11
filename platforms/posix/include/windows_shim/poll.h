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
 * @file poll.h
 *
 * Maps POSIX poll() onto Win32 readiness primitives. WSAPoll has the
 * same struct layout and event flags as POSIX poll; it has been available
 * since Vista but only works on sockets. PX4 also routes command output
 * through CRT pipes on Windows, so pipe-backed fds are handled locally and
 * socket-only calls keep the direct WSAPoll fast path.
 */
#pragma once

#include <errno.h>
#include <io.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <winsock2.h>
#include <windows.h>

/* This header is consumed from both C and C++ translation units. Provide a
 * fall-back macro so the inline shim bodies below can spell `nullptr` while
 * remaining valid in C, where `nullptr` is reserved before C23. */
#if !defined(__cplusplus) && !defined(nullptr)
#  define nullptr NULL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* winsock2 already defines pollfd / WSAPOLLFD and the event bits when
 * _WIN32_WINNT >= 0x0600 (Vista). Re-export as the POSIX-style names. */

#ifndef POLLRDNORM
#define POLLRDNORM POLLIN
#endif
#ifndef POLLRDBAND
#define POLLRDBAND 0x0080
#endif
#ifndef POLLWRNORM
#define POLLWRNORM POLLOUT
#endif
#ifndef POLLWRBAND
#define POLLWRBAND 0x0100
#endif
#ifndef POLLMSG
#define POLLMSG 0x0400
#endif
#ifndef POLLREMOVE
#define POLLREMOVE 0x1000
#endif

typedef unsigned long nfds_t;

enum px4_windows_poll_fd_kind {
	PX4_WINDOWS_POLL_FD_IGNORED = 0,
	PX4_WINDOWS_POLL_FD_SOCKET,
	PX4_WINDOWS_POLL_FD_PIPE,
	PX4_WINDOWS_POLL_FD_CHAR,
	PX4_WINDOWS_POLL_FD_DISK,
	PX4_WINDOWS_POLL_FD_INVALID
};

static inline int px4_windows_poll_fd_ignored(SOCKET fd)
{
	return (intptr_t)fd < 0;
}

static inline int px4_windows_poll_errno_from_wsa(int wsa_error)
{
	switch (wsa_error) {
	case WSAEINTR:
		return EINTR;

	case WSAEINVAL:
		return EINVAL;

	case WSAENOBUFS:
		return ENOMEM;

	default:
		return EBADF;
	}
}

static inline enum px4_windows_poll_fd_kind px4_windows_poll_classify_fd(SOCKET fd, HANDLE *handle, DWORD *file_type)
{
	int socket_type = 0;
	int socket_type_len = sizeof(socket_type);

	if (px4_windows_poll_fd_ignored(fd)) {
		if (handle != NULL) {
			*handle = INVALID_HANDLE_VALUE;
		}

		if (file_type != NULL) {
			*file_type = FILE_TYPE_UNKNOWN;
		}

		return PX4_WINDOWS_POLL_FD_IGNORED;
	}

	/* Winsock SOCKET values and CRT fd numbers live in different namespaces.
	 * Test for a real socket first so a small SOCKET value is never mistaken
	 * for a same-numbered CRT fd. */
	if (getsockopt(fd, SOL_SOCKET, SO_TYPE, (char *)&socket_type, &socket_type_len) == 0) {
		if (handle != NULL) {
			*handle = INVALID_HANDLE_VALUE;
		}

		if (file_type != NULL) {
			*file_type = FILE_TYPE_UNKNOWN;
		}

		return PX4_WINDOWS_POLL_FD_SOCKET;
	}

	if (fd > (SOCKET)INT_MAX) {
		if (handle != NULL) {
			*handle = INVALID_HANDLE_VALUE;
		}

		if (file_type != NULL) {
			*file_type = FILE_TYPE_UNKNOWN;
		}

		return PX4_WINDOWS_POLL_FD_INVALID;
	}

	HANDLE h = (HANDLE)_get_osfhandle((int)fd);

	if (h == INVALID_HANDLE_VALUE || h == NULL) {
		if (handle != NULL) {
			*handle = INVALID_HANDLE_VALUE;
		}

		if (file_type != NULL) {
			*file_type = FILE_TYPE_UNKNOWN;
		}

		return PX4_WINDOWS_POLL_FD_INVALID;
	}

	SetLastError(ERROR_SUCCESS);
	DWORD type = GetFileType(h);
	DWORD error = GetLastError();

	if (type == FILE_TYPE_UNKNOWN && error != ERROR_SUCCESS) {
		if (handle != NULL) {
			*handle = INVALID_HANDLE_VALUE;
		}

		if (file_type != NULL) {
			*file_type = FILE_TYPE_UNKNOWN;
		}

		return PX4_WINDOWS_POLL_FD_INVALID;
	}

	if (handle != NULL) {
		*handle = h;
	}

	if (file_type != NULL) {
		*file_type = type;
	}

	switch (type) {
	case FILE_TYPE_PIPE:
		return PX4_WINDOWS_POLL_FD_PIPE;

	case FILE_TYPE_CHAR:
		return PX4_WINDOWS_POLL_FD_CHAR;

	case FILE_TYPE_DISK:
		return PX4_WINDOWS_POLL_FD_DISK;

	default:
		return PX4_WINDOWS_POLL_FD_INVALID;
	}
}

static inline short px4_windows_poll_ready_events(short requested_events, short ready_events)
{
	return (short)(requested_events & ready_events);
}

static inline short px4_windows_poll_pipe_revents(HANDLE handle, short events)
{
	const short read_events = (short)(POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI);
	const short write_events = (short)(POLLOUT | POLLWRNORM | POLLWRBAND);
	short revents = 0;
	DWORD available = 0;

	if (PeekNamedPipe(handle, NULL, 0, NULL, &available, NULL)) {
		if (available > 0) {
			revents |= px4_windows_poll_ready_events(events, read_events);
		}

	} else {
		const DWORD error = GetLastError();

		if (error == ERROR_BROKEN_PIPE || error == ERROR_HANDLE_EOF || error == ERROR_PIPE_NOT_CONNECTED) {
			revents |= POLLHUP;

			if ((events & read_events) != 0) {
				revents |= POLLIN;
			}

		} else {
			revents |= POLLERR;
		}
	}

	/* Anonymous pipes are used for PX4 command-output relay. Treat write
	 * interest as immediately ready, matching the previous shim behavior and
	 * avoiding WSAPoll on non-socket CRT fds. */
	revents |= px4_windows_poll_ready_events(events, write_events);

	return revents;
}

static inline short px4_windows_poll_char_revents(HANDLE handle, short events)
{
	const short read_events = (short)(POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI);
	const short write_events = (short)(POLLOUT | POLLWRNORM | POLLWRBAND);
	short revents = 0;
	DWORD console_mode = 0;

	if ((events & read_events) != 0) {
		if (GetConsoleMode(handle, &console_mode)) {
			DWORD pending = 0;

			if (GetNumberOfConsoleInputEvents(handle, &pending) && pending > 0) {
				revents |= px4_windows_poll_ready_events(events, read_events);
			}

		} else {
			const DWORD wait_result = WaitForSingleObject(handle, 0);

			if (wait_result == WAIT_OBJECT_0) {
				revents |= px4_windows_poll_ready_events(events, read_events);

			} else if (wait_result == WAIT_FAILED) {
				revents |= POLLERR;
			}
		}
	}

	revents |= px4_windows_poll_ready_events(events, write_events);
	return revents;
}

static inline short px4_windows_poll_disk_revents(short events)
{
	const short ready_events = (short)(POLLIN | POLLRDNORM | POLLRDBAND | POLLPRI | POLLOUT | POLLWRNORM | POLLWRBAND);
	return px4_windows_poll_ready_events(events, ready_events);
}

/**
 * @brief Poll socket descriptors and Windows CRT pipe descriptors.
 *
 * @return Number of ready descriptors, 0 on timeout, or -1 with errno set.
 */
static inline int poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	if (timeout < -1) {
		errno = EINVAL;
		return -1;
	}

	if (fds == nullptr && nfds > 0) {
		errno = EFAULT;
		return -1;
	}

	if (nfds == 0) {
		if (timeout < 0) {
			Sleep(INFINITE);

		} else if (timeout > 0) {
			Sleep((DWORD)timeout);
		}

		return 0;
	}

	int socket_only = true;
	int has_ignored_fd = false;

	for (nfds_t i = 0; i < nfds; ++i) {
		if (px4_windows_poll_fd_ignored((SOCKET)fds[i].fd)) {
			has_ignored_fd = true;
			continue;
		}

		if (px4_windows_poll_classify_fd((SOCKET)fds[i].fd, NULL, NULL) != PX4_WINDOWS_POLL_FD_SOCKET) {
			socket_only = false;
			break;
		}
	}

	if (socket_only && !has_ignored_fd) {
		const int ret = WSAPoll(fds, (ULONG)nfds, timeout);

		if (ret == SOCKET_ERROR) {
			errno = px4_windows_poll_errno_from_wsa(WSAGetLastError());
			return -1;
		}

		return ret;
	}

	struct pollfd *socket_fds = (struct pollfd *)malloc(sizeof(struct pollfd) * nfds);

	nfds_t *socket_indices = (nfds_t *)malloc(sizeof(nfds_t) * nfds);

	if (socket_fds == nullptr || socket_indices == nullptr) {
		free(socket_fds);
		free(socket_indices);
		errno = ENOMEM;
		return -1;
	}

	const ULONGLONG start_ms = GetTickCount64();

	for (;;) {
		int ready_count = 0;
		nfds_t socket_count = 0;

		for (nfds_t i = 0; i < nfds; ++i) {
			fds[i].revents = 0;

			if (px4_windows_poll_fd_ignored((SOCKET)fds[i].fd)) {
				continue;
			}

			HANDLE h = INVALID_HANDLE_VALUE;
			DWORD file_type = FILE_TYPE_UNKNOWN;
			const enum px4_windows_poll_fd_kind kind = px4_windows_poll_classify_fd((SOCKET)fds[i].fd, &h, &file_type);

			if (kind == PX4_WINDOWS_POLL_FD_PIPE) {
				fds[i].revents = px4_windows_poll_pipe_revents(h, fds[i].events);

				if (fds[i].revents != 0) {
					++ready_count;
				}

				continue;
			}

			if (kind == PX4_WINDOWS_POLL_FD_SOCKET) {
				socket_fds[socket_count] = fds[i];
				socket_indices[socket_count] = i;
				++socket_count;
				continue;
			}

			if (kind == PX4_WINDOWS_POLL_FD_CHAR) {
				fds[i].revents = px4_windows_poll_char_revents(h, fds[i].events);

			} else if (kind == PX4_WINDOWS_POLL_FD_DISK) {
				(void)file_type;
				fds[i].revents = px4_windows_poll_disk_revents(fds[i].events);

			} else if (kind == PX4_WINDOWS_POLL_FD_INVALID) {
				fds[i].revents = POLLNVAL;
			}

			if (fds[i].revents != 0) {
				++ready_count;
			}
		}

		if (socket_count > 0) {
			const int socket_ready = WSAPoll(socket_fds, (ULONG)socket_count, 0);

			if (socket_ready == SOCKET_ERROR) {
				free(socket_fds);
				free(socket_indices);
				errno = px4_windows_poll_errno_from_wsa(WSAGetLastError());
				return -1;
			}

			for (nfds_t i = 0; i < socket_count; ++i) {
				if (socket_fds[i].revents != 0) {
					fds[socket_indices[i]].revents = socket_fds[i].revents;
				}
			}

			ready_count += socket_ready;
		}

		if (ready_count > 0 || timeout == 0) {
			free(socket_fds);
			free(socket_indices);
			return ready_count;
		}

		if (timeout > 0) {
			const ULONGLONG elapsed_ms = GetTickCount64() - start_ms;

			if (elapsed_ms >= (ULONGLONG)timeout) {
				free(socket_fds);
				free(socket_indices);
				return 0;
			}
		}

		Sleep(1);
	}
}

#ifdef __cplusplus
}
#endif
