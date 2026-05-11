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
 * @file socket.cpp
 *
 * POSIX socket extras MinGW does not ship: sendmsg / recvmsg
 * (scatter/gather over winsock WSAMsg) and socketpair(2)
 * implemented on AF_INET loopback.
 *
 * Ancillary data is deliberately unsupported. PX4's SITL paths use iovecs for
 * payload scatter/gather, not SCM_RIGHTS or control messages, and there is no
 * direct Winsock equivalent for the Unix control-message model.
 */

#include "px4_windows_internal.h"

#include <vector>

extern "C" ssize_t sendmsg(int fd, const struct msghdr *msg, int flags)
{
	if (!msg || !msg->msg_iov || msg->msg_iovlen < 0) {
		errno = EINVAL;
		return -1;
	}

	if (msg->msg_control && msg->msg_controllen > 0) {
		errno = ENOTSUP;
		return -1;
	}

	std::vector<WSABUF> bufs((size_t)msg->msg_iovlen);

	for (int i = 0; i < msg->msg_iovlen; ++i) {
		/* WSABUF is the Winsock spelling of POSIX iovec. The buffer lifetime is
		 * owned by the caller, just like sendmsg(). */
		bufs[(size_t)i].buf = static_cast<char *>(msg->msg_iov[i].iov_base);
		bufs[(size_t)i].len = (ULONG)msg->msg_iov[i].iov_len;
	}

	DWORD sent = 0;
	const int rc = (msg->msg_name && msg->msg_namelen > 0)
		       ? WSASendTo((SOCKET)fd, bufs.data(), (DWORD)bufs.size(), &sent, (DWORD)flags,
				   (const sockaddr *)msg->msg_name, msg->msg_namelen, nullptr, nullptr)
		       : WSASend((SOCKET)fd, bufs.data(), (DWORD)bufs.size(), &sent, (DWORD)flags, nullptr, nullptr);

	if (rc == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		return -1;
	}

	return (ssize_t)sent;
}

extern "C" ssize_t recvmsg(int fd, struct msghdr *msg, int flags)
{
	if (!msg || !msg->msg_iov || msg->msg_iovlen < 0) {
		errno = EINVAL;
		return -1;
	}

	if (msg->msg_control && msg->msg_controllen > 0) {
		errno = ENOTSUP;
		return -1;
	}

	std::vector<WSABUF> bufs((size_t)msg->msg_iovlen);

	for (int i = 0; i < msg->msg_iovlen; ++i) {
		bufs[(size_t)i].buf = static_cast<char *>(msg->msg_iov[i].iov_base);
		bufs[(size_t)i].len = (ULONG)msg->msg_iov[i].iov_len;
	}

	DWORD received = 0;
	DWORD recv_flags = 0;
	int namelen = (int)msg->msg_namelen;
	const int rc = (msg->msg_name && msg->msg_namelen > 0)
		       ? WSARecvFrom((SOCKET)fd, bufs.data(), (DWORD)bufs.size(), &received, &recv_flags,
				     (sockaddr *)msg->msg_name, &namelen, nullptr, nullptr)
		       : WSARecv((SOCKET)fd, bufs.data(), (DWORD)bufs.size(), &received, &recv_flags, nullptr, nullptr);

	if (rc == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		return -1;
	}

	msg->msg_flags = (int)recv_flags;
	msg->msg_namelen = (socklen_t)namelen;
	msg->msg_controllen = 0;
	return (ssize_t)received;
}

extern "C" int socketpair(int domain, int type, int protocol, int sv[2])
{
	if (!sv) {
		errno = EINVAL;
		return -1;
	}

	if (type != SOCK_STREAM) {
		errno = EOPNOTSUPP;
		return -1;
	}

	if (domain != AF_UNIX && domain != AF_INET) {
		errno = EAFNOSUPPORT;
		return -1;
	}

	sv[0] = -1;
	sv[1] = -1;

	/* Windows AF_UNIX support is version-dependent and does not map cleanly to
	 * CRT file descriptors. A loopback TCP pair gives PX4 a bidirectional
	 * byte-stream pair with the same blocking/read/write behavior it uses. */
	SOCKET listener = INVALID_SOCKET;
	SOCKET client = INVALID_SOCKET;
	SOCKET server = INVALID_SOCKET;

	listener = socket(AF_INET, type, protocol);

	if (listener == INVALID_SOCKET) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		return -1;
	}

	sockaddr_in addr {};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;

	if (bind(listener, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(listener);
		return -1;
	}

	if (listen(listener, 1) == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(listener);
		return -1;
	}

	int addr_len = sizeof(addr);

	if (getsockname(listener, reinterpret_cast<sockaddr *>(&addr), &addr_len) == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(listener);
		return -1;
	}

	client = socket(AF_INET, type, protocol);

	if (client == INVALID_SOCKET) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(listener);
		return -1;
	}

	if (connect(client, reinterpret_cast<const sockaddr *>(&addr), addr_len) == SOCKET_ERROR) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(client);
		closesocket(listener);
		return -1;
	}

	server = accept(listener, nullptr, nullptr);
	closesocket(listener);

	if (server == INVALID_SOCKET) {
		errno = px4_wsa_error_to_errno(WSAGetLastError());
		closesocket(client);
		return -1;
	}

	sv[0] = (int)client;
	sv[1] = (int)server;
	return 0;
}
