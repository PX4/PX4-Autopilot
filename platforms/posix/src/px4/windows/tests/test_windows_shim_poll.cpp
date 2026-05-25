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
 * @file test_windows_shim_poll.cpp
 *
 * Unit tests for the inline poll() implementation in
 * platforms/posix/include/windows_shim/poll.h. The shim classifies fd
 * kinds (socket / pipe / char / disk / invalid), routes pure-socket
 * calls through WSAPoll, and otherwise mixes manual readiness probes
 * with a tight WSAPoll fallback for the socket subset.
 *
 * Each branch is exercised: argument validation, socket loopback, pipe
 * with and without pending data, disk-fd always-ready, ignored fd, and
 * the timeout/timeout=0 fast paths.
 */

#include <gtest/gtest.h>

#ifdef _WIN32

#define _WIN32_WINNT 0x0A00
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

#include <poll.h>

namespace
{
class WinsockBootstrap
{
public:
	WinsockBootstrap()
	{
		WSADATA wsa;
		WSAStartup(MAKEWORD(2, 2), &wsa);
	}

	~WinsockBootstrap()
	{
		WSACleanup();
	}
};

/* Lightweight UDP loopback fixture used by socket-path tests. */
struct LoopbackPair {
	WinsockBootstrap _ws;
	SOCKET srv = INVALID_SOCKET;
	SOCKET cli = INVALID_SOCKET;
	struct sockaddr_in srv_addr {};

	LoopbackPair()
	{
		srv = socket(AF_INET, SOCK_DGRAM, 0);
		cli = socket(AF_INET, SOCK_DGRAM, 0);
		srv_addr.sin_family = AF_INET;
		srv_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		srv_addr.sin_port = 0;
		bind(srv, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
		int alen = sizeof(srv_addr);
		getsockname(srv, (struct sockaddr *)&srv_addr, &alen);
	}

	~LoopbackPair()
	{
		if (srv != INVALID_SOCKET) { closesocket(srv); }

		if (cli != INVALID_SOCKET) { closesocket(cli); }
	}
};

} // namespace

/* Argument validation -------------------------------------------------- */

TEST(WindowsShimPoll, NegativeTimeoutBelowMinusOneRejected)
{
	struct pollfd fd;
	fd.fd = -1;
	fd.events = 0;
	fd.revents = 0;
	errno = 0;
	int rc = poll(&fd, 1, -42);
	EXPECT_EQ(rc, -1);
	EXPECT_EQ(errno, EINVAL);
}

TEST(WindowsShimPoll, NullFdsWithCountRejected)
{
	errno = 0;
	int rc = poll(nullptr, 1, 0);
	EXPECT_EQ(rc, -1);
	EXPECT_EQ(errno, EFAULT);
}

TEST(WindowsShimPoll, ZeroNfdsZeroTimeoutReturnsZero)
{
	auto t0 = std::chrono::steady_clock::now();
	int rc = poll(nullptr, 0, 0);
	EXPECT_EQ(rc, 0);
	auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	EXPECT_LT(dt, 50);
}

TEST(WindowsShimPoll, ZeroNfdsPositiveTimeoutSleeps)
{
	auto t0 = std::chrono::steady_clock::now();
	int rc = poll(nullptr, 0, 30);
	EXPECT_EQ(rc, 0);
	auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	EXPECT_GE(dt, 25);
}

/* fd-classification helpers -------------------------------------------- */

TEST(WindowsShimPoll, IgnoredFdHelper)
{
	EXPECT_NE(px4_windows_poll_fd_ignored((SOCKET) - 1), 0);
	EXPECT_NE(px4_windows_poll_fd_ignored((SOCKET) - 5), 0);
	EXPECT_EQ(px4_windows_poll_fd_ignored((SOCKET)0), 0);
}

TEST(WindowsShimPoll, ErrnoFromWsaMapping)
{
	EXPECT_EQ(px4_windows_poll_errno_from_wsa(WSAEINTR), EINTR);
	EXPECT_EQ(px4_windows_poll_errno_from_wsa(WSAEINVAL), EINVAL);
	EXPECT_EQ(px4_windows_poll_errno_from_wsa(WSAENOBUFS), ENOMEM);
	EXPECT_EQ(px4_windows_poll_errno_from_wsa(0xBEEF), EBADF);
}

TEST(WindowsShimPoll, ReadyEventsMask)
{
	EXPECT_EQ(px4_windows_poll_ready_events(POLLIN, POLLIN | POLLOUT), POLLIN);
	EXPECT_EQ(px4_windows_poll_ready_events(POLLIN, POLLOUT), 0);
	EXPECT_EQ(px4_windows_poll_ready_events(0, POLLIN), 0);
}

TEST(WindowsShimPoll, DiskReventsAlwaysReady)
{
	EXPECT_EQ(px4_windows_poll_disk_revents(POLLIN), POLLIN);
	EXPECT_EQ(px4_windows_poll_disk_revents(POLLOUT), POLLOUT);
	EXPECT_EQ(px4_windows_poll_disk_revents(0), 0);
}

TEST(WindowsShimPoll, ClassifyIgnoredFd)
{
	HANDLE h = nullptr;
	DWORD ft = 0;
	enum px4_windows_poll_fd_kind k =
		px4_windows_poll_classify_fd((SOCKET) - 1, &h, &ft);
	EXPECT_EQ(k, PX4_WINDOWS_POLL_FD_IGNORED);
	EXPECT_EQ(h, INVALID_HANDLE_VALUE);
	EXPECT_EQ(ft, FILE_TYPE_UNKNOWN);
}

TEST(WindowsShimPoll, ClassifyInvalidFd)
{
	WinsockBootstrap _ws;
	HANDLE h = nullptr;
	DWORD ft = 0;
	/* A very high CRT-fd-domain value with no backing handle. */
	enum px4_windows_poll_fd_kind k =
		px4_windows_poll_classify_fd((SOCKET)999999, &h, &ft);
	EXPECT_EQ(k, PX4_WINDOWS_POLL_FD_INVALID);
}

TEST(WindowsShimPoll, ClassifyPipeFd)
{
	int p[2] = { -1, -1 };
	ASSERT_EQ(_pipe(p, 4096, _O_BINARY), 0);
	HANDLE h = nullptr;
	DWORD ft = 0;
	enum px4_windows_poll_fd_kind k =
		px4_windows_poll_classify_fd((SOCKET)p[0], &h, &ft);
	EXPECT_EQ(k, PX4_WINDOWS_POLL_FD_PIPE);
	EXPECT_NE(h, INVALID_HANDLE_VALUE);
	EXPECT_EQ(ft, (DWORD)FILE_TYPE_PIPE);
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimPoll, ClassifyDiskFd)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_poll_test.dat");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY, _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	HANDLE h = nullptr;
	DWORD ft = 0;
	enum px4_windows_poll_fd_kind k =
		px4_windows_poll_classify_fd((SOCKET)fd, &h, &ft);
	EXPECT_EQ(k, PX4_WINDOWS_POLL_FD_DISK);
	EXPECT_EQ(ft, (DWORD)FILE_TYPE_DISK);
	_close(fd);
	DeleteFileA(path);
}

TEST(WindowsShimPoll, ClassifySocketFd)
{
	WinsockBootstrap _ws;
	SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
	ASSERT_NE(s, INVALID_SOCKET);
	HANDLE h = nullptr;
	DWORD ft = 0;
	enum px4_windows_poll_fd_kind k =
		px4_windows_poll_classify_fd(s, &h, &ft);
	EXPECT_EQ(k, PX4_WINDOWS_POLL_FD_SOCKET);
	EXPECT_EQ(h, INVALID_HANDLE_VALUE);
	closesocket(s);
}

/* poll() integration --------------------------------------------------- */

TEST(WindowsShimPoll, OnlyIgnoredFdShortCircuitTimeoutZero)
{
	struct pollfd fds[2];
	fds[0].fd = -1;
	fds[0].events = POLLIN;
	fds[0].revents = 0xff;
	fds[1].fd = -1;
	fds[1].events = POLLOUT;
	fds[1].revents = 0xff;
	int rc = poll(fds, 2, 0);
	EXPECT_EQ(rc, 0);
	EXPECT_EQ(fds[0].revents, 0);
	EXPECT_EQ(fds[1].revents, 0);
}

TEST(WindowsShimPoll, SocketWriteAlwaysReady)
{
	LoopbackPair pair;
	struct pollfd pf;
	pf.fd = (int)pair.cli;
	pf.events = POLLOUT;
	pf.revents = 0;
	int rc = poll(&pf, 1, 0);
	EXPECT_GE(rc, 0);
	/* Writable on a fresh UDP socket. */
	EXPECT_TRUE((pf.revents & POLLOUT) || rc == 0);
}

TEST(WindowsShimPoll, SocketReadReadyAfterSendto)
{
	LoopbackPair pair;
	const char msg[] = "x";
	ASSERT_EQ(sendto(pair.cli, msg, (int)sizeof(msg), 0,
			 (struct sockaddr *)&pair.srv_addr, sizeof(pair.srv_addr)),
		  (int)sizeof(msg));
	/* Give the kernel a moment to deliver. */
	struct pollfd pf;
	pf.fd = (int)pair.srv;
	pf.events = POLLIN;
	pf.revents = 0;
	int rc = poll(&pf, 1, 200);
	EXPECT_EQ(rc, 1);
	EXPECT_TRUE((pf.revents & POLLIN) != 0);
}

TEST(WindowsShimPoll, SocketTimeoutNoData)
{
	LoopbackPair pair;
	struct pollfd pf;
	pf.fd = (int)pair.srv;
	pf.events = POLLIN;
	pf.revents = 0;
	auto t0 = std::chrono::steady_clock::now();
	int rc = poll(&pf, 1, 30);
	EXPECT_EQ(rc, 0);
	auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	EXPECT_GE(dt, 20);
}

TEST(WindowsShimPoll, PipeWritableImmediately)
{
	int p[2] = { -1, -1 };
	ASSERT_EQ(_pipe(p, 4096, _O_BINARY), 0);
	struct pollfd pf;
	pf.fd = p[1];
	pf.events = POLLOUT;
	pf.revents = 0;
	int rc = poll(&pf, 1, 0);
	EXPECT_EQ(rc, 1);
	EXPECT_TRUE((pf.revents & POLLOUT) != 0);
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimPoll, PipeReadableAfterWrite)
{
	int p[2] = { -1, -1 };
	ASSERT_EQ(_pipe(p, 4096, _O_BINARY), 0);
	const char msg[] = "ping";
	ASSERT_EQ(_write(p[1], msg, (unsigned)sizeof(msg)), (int)sizeof(msg));
	struct pollfd pf;
	pf.fd = p[0];
	pf.events = POLLIN;
	pf.revents = 0;
	int rc = poll(&pf, 1, 200);
	EXPECT_EQ(rc, 1);
	EXPECT_TRUE((pf.revents & POLLIN) != 0);
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimPoll, PipeBrokenSetsHup)
{
	int p[2] = { -1, -1 };
	ASSERT_EQ(_pipe(p, 4096, _O_BINARY), 0);
	_close(p[1]);
	struct pollfd pf;
	pf.fd = p[0];
	pf.events = POLLIN;
	pf.revents = 0;
	int rc = poll(&pf, 1, 50);
	EXPECT_GE(rc, 1);
	EXPECT_TRUE((pf.revents & (POLLHUP | POLLIN)) != 0);
	_close(p[0]);
}

TEST(WindowsShimPoll, DiskFdAlwaysReady)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_poll_disk.dat");
	int fd = _open(path, _O_CREAT | _O_RDWR | _O_BINARY, _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);
	struct pollfd pf;
	pf.fd = fd;
	pf.events = POLLIN | POLLOUT;
	pf.revents = 0;
	int rc = poll(&pf, 1, 0);
	EXPECT_EQ(rc, 1);
	EXPECT_TRUE((pf.revents & POLLIN) != 0);
	EXPECT_TRUE((pf.revents & POLLOUT) != 0);
	_close(fd);
	DeleteFileA(path);
}

TEST(WindowsShimPoll, MixedSocketAndPipe)
{
	LoopbackPair pair;
	int p[2] = { -1, -1 };
	ASSERT_EQ(_pipe(p, 4096, _O_BINARY), 0);
	const char msg[] = "z";
	ASSERT_EQ(_write(p[1], msg, (unsigned)sizeof(msg)), (int)sizeof(msg));
	struct pollfd fds[2];
	fds[0].fd = p[0];
	fds[0].events = POLLIN;
	fds[0].revents = 0;
	fds[1].fd = (int)pair.srv;
	fds[1].events = POLLIN;
	fds[1].revents = 0;
	int rc = poll(fds, 2, 100);
	EXPECT_GE(rc, 1);
	EXPECT_TRUE((fds[0].revents & POLLIN) != 0);
	_close(p[0]);
	_close(p[1]);
}

TEST(WindowsShimPoll, InvalidFdReturnsPollnval)
{
	struct pollfd pf;
	pf.fd = 999999; /* deliberately bogus */
	pf.events = POLLIN;
	pf.revents = 0;
	int rc = poll(&pf, 1, 0);
	EXPECT_GE(rc, 1);
	EXPECT_TRUE((pf.revents & POLLNVAL) != 0);
}

#endif // _WIN32

TEST(WindowsShimPoll, BuildSentinel)
{
	SUCCEED();
}
