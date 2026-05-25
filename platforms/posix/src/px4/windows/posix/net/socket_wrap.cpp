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
 * @file socket_wrap.cpp
 *
 * Linker `--wrap` shims that translate WSAGetLastError into POSIX errno
 * for the bare WinSock entry points PX4 calls directly (socket, bind,
 * connect, listen, accept, setsockopt, shutdown, recv/send,
 * recvfrom/sendto). Without these wrappers, code such as
 *     if (bind(fd, ...) < 0) PX4_WARN("%s", strerror(errno));
 * prints whatever stale errno some unrelated prior call (often a file
 * open returning ENOENT) happened to leave behind. WinSock keeps its
 * own thread-local error retrievable via WSAGetLastError() and never
 * touches POSIX errno.
 *
 * The --wrap=<name> flags are configured on the px4 link in
 * platforms/posix/cmake/windows.cmake. This only intercepts calls that
 * resolve to the bare `<name>` symbol; without the
 * `-DWINSOCK_API_LINKAGE=` global compile def in px4_impl_os.cmake,
 * winsock prototypes carry __declspec(dllimport) and PX4 callers
 * resolve to `__imp_<name>` (an indirect call through the import
 * table) which --wrap cannot intercept. Stripping the dllimport makes
 * MinGW emit a thunk named `<name>` in the executable; --wrap rewrites
 * references to that thunk to `__wrap_<name>`, and `__real_<name>`
 * resolves back to the original thunk. The wrappers below are the
 * only place the errno translation happens.
 *
 * Keep this file and socket_msvc.cpp behaviorally aligned. The dispatch
 * mechanism is compiler-specific, but PX4 modules should observe the same
 * return values, errno values, and strerror text on MinGW and MSVC.
 */

#include "px4_windows_internal.h"

extern "C" {

	SOCKET WSAAPI __real_socket(int af, int type, int protocol);
	int    WSAAPI __real_bind(SOCKET s, const struct sockaddr *name, int namelen);
	int    WSAAPI __real_listen(SOCKET s, int backlog);
	SOCKET WSAAPI __real_accept(SOCKET s, struct sockaddr *addr, int *addrlen);
	int    WSAAPI __real_connect(SOCKET s, const struct sockaddr *name, int namelen);
	int    WSAAPI __real_setsockopt(SOCKET s, int level, int optname, const char *optval, int optlen);
	int    WSAAPI __real_shutdown(SOCKET s, int how);
	int    WSAAPI __real_recv(SOCKET s, char *buf, int len, int flags);
	int    WSAAPI __real_send(SOCKET s, const char *buf, int len, int flags);
	int    WSAAPI __real_recvfrom(SOCKET s, char *buf, int len, int flags, struct sockaddr *from, int *fromlen);
	int    WSAAPI __real_sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *to, int tolen);

	static inline void wsa_set_errno()
	{
		errno = px4_wsa_error_to_errno(WSAGetLastError());
	}

	SOCKET WSAAPI __wrap_socket(int af, int type, int protocol)
	{
		const SOCKET s = __real_socket(af, type, protocol);

		if (s == INVALID_SOCKET) {
			wsa_set_errno();
		}

		return s;
	}

	int WSAAPI __wrap_bind(SOCKET s, const struct sockaddr *name, int namelen)
	{
		const int rc = __real_bind(s, name, namelen);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_listen(SOCKET s, int backlog)
	{
		const int rc = __real_listen(s, backlog);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	SOCKET WSAAPI __wrap_accept(SOCKET s, struct sockaddr *addr, int *addrlen)
	{
		const SOCKET r = __real_accept(s, addr, addrlen);

		if (r == INVALID_SOCKET) {
			wsa_set_errno();
		}

		return r;
	}

	int WSAAPI __wrap_connect(SOCKET s, const struct sockaddr *name, int namelen)
	{
		const int rc = __real_connect(s, name, namelen);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_setsockopt(SOCKET s, int level, int optname, const char *optval, int optlen)
	{
		const int rc = __real_setsockopt(s, level, optname, optval, optlen);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_shutdown(SOCKET s, int how)
	{
		const int rc = __real_shutdown(s, how);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_recv(SOCKET s, char *buf, int len, int flags)
	{
		const int rc = __real_recv(s, buf, len, flags);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_send(SOCKET s, const char *buf, int len, int flags)
	{
		const int rc = __real_send(s, buf, len, flags);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_recvfrom(SOCKET s, char *buf, int len, int flags, struct sockaddr *from, int *fromlen)
	{
		const int rc = __real_recvfrom(s, buf, len, flags, from, fromlen);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	int WSAAPI __wrap_sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *to, int tolen)
	{
		const int rc = __real_sendto(s, buf, len, flags, to, tolen);

		if (rc == SOCKET_ERROR) {
			wsa_set_errno();
		}

		return rc;
	}

	/* MSVCRT's strerror() table only covers C89 errnos; modern socket
	 * codes mapped from WSAGetLastError() (EADDRINUSE, EADDRNOTAVAIL,
	 * EAFNOSUPPORT, ENOTSOCK, ENETDOWN, ECONNREFUSED, ETIMEDOUT, ...)
	 * all return "Unknown error". Fill those in here; everything else
	 * defers to MSVCRT. */
	char *__real_strerror(int e);

	char *__wrap_strerror(int e)
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

		default:              return __real_strerror(e);
		}
	}

} // extern "C"
