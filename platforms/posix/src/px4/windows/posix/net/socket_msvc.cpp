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
 * @file socket_msvc.cpp
 *
 * MSVC-compatible WinSock wrappers. GNU/MinGW builds use ld --wrap in
 * socket_wrap.cpp; MSVC routes call sites through macros in sys/socket.h.
 *
 * This file intentionally mirrors socket_wrap.cpp's behavior without relying
 * on GNU linker features. `PX4_WINDOWS_NO_SOCKET_MACROS` is defined before
 * including the shim headers so the wrapper bodies can call the real Winsock
 * functions instead of recursively calling themselves.
 */

#define PX4_WINDOWS_NO_SOCKET_MACROS
#include "px4_windows_internal.h"

#if defined(_MSC_VER)

namespace
{
static inline void set_wsa_errno()
{
	/* Winsock keeps its error code separate from POSIX errno. This is the one
	 * translation point for every MSVC-routed socket call below. */
	errno = px4_wsa_error_to_errno(WSAGetLastError());
}
}

extern "C" {

	SOCKET WSAAPI px4_windows_socket(int af, int type, int protocol)
	{
		const SOCKET s = socket(af, type, protocol);

		if (s == INVALID_SOCKET) {
			set_wsa_errno();
		}

		return s;
	}

	int WSAAPI px4_windows_bind(SOCKET s, const struct sockaddr *name, int namelen)
	{
		const int rc = bind(s, name, namelen);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_listen(SOCKET s, int backlog)
	{
		const int rc = listen(s, backlog);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	SOCKET WSAAPI px4_windows_accept(SOCKET s, struct sockaddr *addr, int *addrlen)
	{
		const SOCKET r = accept(s, addr, addrlen);

		if (r == INVALID_SOCKET) {
			set_wsa_errno();
		}

		return r;
	}

	int WSAAPI px4_windows_connect(SOCKET s, const struct sockaddr *name, int namelen)
	{
		const int rc = connect(s, name, namelen);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_setsockopt(SOCKET s, int level, int optname, const char *optval, int optlen)
	{
		const int rc = setsockopt(s, level, optname, optval, optlen);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_shutdown(SOCKET s, int how)
	{
		const int rc = shutdown(s, how);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_recv(SOCKET s, char *buf, int len, int flags)
	{
		const int rc = recv(s, buf, len, flags);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_send(SOCKET s, const char *buf, int len, int flags)
	{
		const int rc = send(s, buf, len, flags);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_recvfrom(SOCKET s, char *buf, int len, int flags, struct sockaddr *from, int *fromlen)
	{
		const int rc = recvfrom(s, buf, len, flags, from, fromlen);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	int WSAAPI px4_windows_sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *to, int tolen)
	{
		const int rc = sendto(s, buf, len, flags, to, tolen);

		if (rc == SOCKET_ERROR) {
			set_wsa_errno();
		}

		return rc;
	}

	char *px4_windows_strerror(int e)
	{
		switch (e) {
		case EADDRINUSE:      return const_cast<char *>("Address already in use");

		case EADDRNOTAVAIL:   return const_cast<char *>("Address not available");

		case EAFNOSUPPORT:    return const_cast<char *>("Address family not supported");

		case ENOTSOCK:        return const_cast<char *>("Not a socket");

		case EDESTADDRREQ:    return const_cast<char *>("Destination address required");

		case EMSGSIZE:        return const_cast<char *>("Message too long");

		case EPROTOTYPE:      return const_cast<char *>("Protocol wrong type for socket");

		case ENOPROTOOPT:     return const_cast<char *>("Protocol option not available");

		case EPROTONOSUPPORT: return const_cast<char *>("Protocol not supported");

		case EOPNOTSUPP:      return const_cast<char *>("Operation not supported");

		case ENETDOWN:        return const_cast<char *>("Network is down");

		case ENETUNREACH:     return const_cast<char *>("Network unreachable");

		case ENETRESET:       return const_cast<char *>("Connection aborted by network");

		case ECONNABORTED:    return const_cast<char *>("Connection aborted");

		case ECONNRESET:      return const_cast<char *>("Connection reset by peer");

		case ENOBUFS:         return const_cast<char *>("No buffer space available");

		case EISCONN:         return const_cast<char *>("Already connected");

		case ENOTCONN:        return const_cast<char *>("Not connected");

		case ETIMEDOUT:       return const_cast<char *>("Connection timed out");

		case ECONNREFUSED:    return const_cast<char *>("Connection refused");

		case EALREADY:        return const_cast<char *>("Operation already in progress");

		case EINPROGRESS:     return const_cast<char *>("Operation in progress");

		case EWOULDBLOCK:     return const_cast<char *>("Resource temporarily unavailable");

		default:              return strerror(e);
		}
	}

} // extern "C"

#endif // defined(_MSC_VER)
