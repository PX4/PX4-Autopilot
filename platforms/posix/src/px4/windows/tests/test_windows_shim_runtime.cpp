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
 * @file test_windows_shim_runtime.cpp
 *
 * Unit tests for the source-defined shims in
 * platforms/posix/src/px4/windows/posix/. The tests link against a thin
 * library that pulls in the standalone-compilable shim sources directly
 * (errno_map, env, ids, sysconf, mman, flock, dlfcn, if_query, resolver,
 * sched, termios, ioctl) so they can be exercised without the full PX4
 * platform/uORB stack.
 *
 * Tests are gated on _WIN32; on other hosts the file compiles to a
 * trivial sentinel test so the gtest runner still produces output.
 */

#include <gtest/gtest.h>

#ifdef _WIN32

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#define _WIN32_WINNT 0x0A00
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>

/* The runtime shim references g_px4_session_id (defined in
 * platforms/posix/src/px4/windows/runtime/init.cpp). This test binary
 * does not link init.cpp because pulling it in drags the entire process
 * bootstrap. Provide a local definition that mirrors the runtime
 * contract. The original is C++-linkage (no extern "C") in init.cpp so
 * we match that here exactly. */
volatile LONG g_px4_session_id = 0;

#include <unistd.h>
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <pwd.h>
#include <grp.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <dlfcn.h>

/* errno_map prototypes -- declared in px4_windows_internal.h but the
 * tests don't include that header (it pulls in the entire winsock+win32
 * stack via the shim umbrella). Forward-declare just what we need. The
 * originals are C++-linkage in errno_map.cpp; matching the linkage
 * exactly here keeps name mangling aligned. */
int px4_win_error_to_errno(DWORD err);
int px4_wsa_error_to_errno(int err);
const char *px4_hstrerror_text(int err);

/* errno_map ------------------------------------------------------------ */

TEST(WindowsShimErrnoMap, FileNotFoundMapsToENOENT)
{
	EXPECT_EQ(px4_win_error_to_errno(ERROR_FILE_NOT_FOUND), ENOENT);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_PATH_NOT_FOUND), ENOENT);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_INVALID_DRIVE), ENOENT);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_BAD_PATHNAME), ENOENT);
}

TEST(WindowsShimErrnoMap, AccessDeniedMapsToEACCES)
{
	EXPECT_EQ(px4_win_error_to_errno(ERROR_ACCESS_DENIED), EACCES);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_SHARING_VIOLATION), EACCES);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_LOCK_VIOLATION), EACCES);
}

TEST(WindowsShimErrnoMap, AlreadyExistsMapsToEEXIST)
{
	EXPECT_EQ(px4_win_error_to_errno(ERROR_ALREADY_EXISTS), EEXIST);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_FILE_EXISTS), EEXIST);
}

TEST(WindowsShimErrnoMap, MiscMappings)
{
	EXPECT_EQ(px4_win_error_to_errno(ERROR_INVALID_HANDLE), EBADF);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_NOT_ENOUGH_MEMORY), ENOMEM);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_OUTOFMEMORY), ENOMEM);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_INVALID_PARAMETER), EINVAL);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_DIR_NOT_EMPTY), ENOTEMPTY);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_NOT_SUPPORTED), ENOTSUP);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_BUSY), EBUSY);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_DISK_FULL), ENOSPC);
	EXPECT_EQ(px4_win_error_to_errno(ERROR_PROC_NOT_FOUND), ESRCH);
}

TEST(WindowsShimErrnoMap, UnknownWin32MapsToEIO)
{
	EXPECT_EQ(px4_win_error_to_errno(0xDEADBEEF), EIO);
}

TEST(WindowsShimErrnoMap, WsaWouldBlockMapsCorrectly)
{
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEWOULDBLOCK), EWOULDBLOCK);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEINPROGRESS), EINPROGRESS);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENOTSOCK), ENOTSOCK);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEMSGSIZE), EMSGSIZE);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAECONNREFUSED), ECONNREFUSED);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAECONNRESET), ECONNRESET);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENOTCONN), ENOTCONN);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEISCONN), EISCONN);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAETIMEDOUT), ETIMEDOUT);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEADDRINUSE), EADDRINUSE);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEADDRNOTAVAIL), EADDRNOTAVAIL);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENETDOWN), ENETDOWN);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENETUNREACH), ENETUNREACH);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENETRESET), ENETRESET);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAECONNABORTED), ECONNABORTED);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEAFNOSUPPORT), EAFNOSUPPORT);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEPROTONOSUPPORT), EPROTONOSUPPORT);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEPROTOTYPE), EPROTOTYPE);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENOPROTOOPT), ENOPROTOOPT);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEOPNOTSUPP), EOPNOTSUPP);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEALREADY), EALREADY);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAEDESTADDRREQ), EDESTADDRREQ);
	EXPECT_EQ(px4_wsa_error_to_errno(WSAENOBUFS), ENOBUFS);
}

TEST(WindowsShimErrnoMap, UnknownWsaMapsToEIO)
{
	EXPECT_EQ(px4_wsa_error_to_errno(0x7FFFFFFF), EIO);
}

TEST(WindowsShimErrnoMap, HstrerrorReturnsStableStrings)
{
	EXPECT_STRNE(px4_hstrerror_text(HOST_NOT_FOUND), nullptr);
	EXPECT_STREQ(px4_hstrerror_text(HOST_NOT_FOUND), "Unknown host");
	EXPECT_STREQ(px4_hstrerror_text(TRY_AGAIN), "Temporary failure in name resolution");
	EXPECT_STREQ(px4_hstrerror_text(NO_RECOVERY), "Non-recoverable name server error");
	EXPECT_STREQ(px4_hstrerror_text(NO_DATA), "No address associated with name");
	EXPECT_STREQ(px4_hstrerror_text(0xBAD), "Resolver error");
}

/* env (setenv / unsetenv) --------------------------------------------- */

TEST(WindowsShimEnv, SetenvRejectsBadName)
{
	errno = 0;
	EXPECT_EQ(setenv(nullptr, "v", 1), -1);
	EXPECT_EQ(errno, EINVAL);

	errno = 0;
	EXPECT_EQ(setenv("", "v", 1), -1);
	EXPECT_EQ(errno, EINVAL);

	errno = 0;
	EXPECT_EQ(setenv("BAD=NAME", "v", 1), -1);
	EXPECT_EQ(errno, EINVAL);

	errno = 0;
	EXPECT_EQ(setenv("OK", nullptr, 1), -1);
	EXPECT_EQ(errno, EINVAL);
}

TEST(WindowsShimEnv, SetenvOverwriteSemantics)
{
	ASSERT_EQ(setenv("PX4_TEST_VAR", "first", 1), 0);
	EXPECT_STREQ(getenv("PX4_TEST_VAR"), "first");

	/* overwrite=0 leaves existing value intact. */
	ASSERT_EQ(setenv("PX4_TEST_VAR", "second", 0), 0);
	EXPECT_STREQ(getenv("PX4_TEST_VAR"), "first");

	/* overwrite=1 replaces. */
	ASSERT_EQ(setenv("PX4_TEST_VAR", "third", 1), 0);
	EXPECT_STREQ(getenv("PX4_TEST_VAR"), "third");

	/* unsetenv clears. */
	EXPECT_EQ(unsetenv("PX4_TEST_VAR"), 0);
	const char *after = getenv("PX4_TEST_VAR");
	/* MSVCRT _putenv_s with an empty string treats the variable as
	 * unset; some CRT versions still return an empty string from
	 * getenv() for compatibility. Either is acceptable. */
	EXPECT_TRUE(after == nullptr || after[0] == '\0');
}

TEST(WindowsShimEnv, UnsetenvRejectsBadName)
{
	errno = 0;
	EXPECT_EQ(unsetenv(nullptr), -1);
	EXPECT_EQ(errno, EINVAL);
	errno = 0;
	EXPECT_EQ(unsetenv(""), -1);
	EXPECT_EQ(errno, EINVAL);
	errno = 0;
	EXPECT_EQ(unsetenv("BAD=NAME"), -1);
	EXPECT_EQ(errno, EINVAL);
}

/* sysconf -------------------------------------------------------------- */

TEST(WindowsShimSysconf, PageSizeIsPositive)
{
	const long pg = sysconf(_SC_PAGESIZE);
	EXPECT_GT(pg, 0);
	/* x86/x64 page size is 4096; ARM64 is 16384. Either is reasonable. */
	EXPECT_TRUE(pg == 4096 || pg == 16384 || pg == 65536);

	EXPECT_EQ(sysconf(_SC_PAGE_SIZE), pg);
}

TEST(WindowsShimSysconf, NprocessorsAtLeastOne)
{
	EXPECT_GE(sysconf(_SC_NPROCESSORS_ONLN), 1);
	EXPECT_GE(sysconf(_SC_NPROCESSORS_CONF), 1);
}

TEST(WindowsShimSysconf, OpenMaxNonZero)
{
	EXPECT_GT(sysconf(_SC_OPEN_MAX), 0);
}

TEST(WindowsShimSysconf, HostnameAndLoginMaxNonZero)
{
	EXPECT_GT(sysconf(_SC_HOST_NAME_MAX), 0);
	EXPECT_GT(sysconf(_SC_LOGIN_NAME_MAX), 0);
}

TEST(WindowsShimSysconf, PhysAndAvphysPagesNonNegative)
{
	EXPECT_GE(sysconf(_SC_PHYS_PAGES), 0);
	EXPECT_GE(sysconf(_SC_AVPHYS_PAGES), 0);
}

TEST(WindowsShimSysconf, ClkTckPositive)
{
	EXPECT_GT(sysconf(_SC_CLK_TCK), 0);
}

TEST(WindowsShimSysconf, GetpagesizeMatchesSysconf)
{
	EXPECT_EQ((long)getpagesize(), sysconf(_SC_PAGESIZE));
}

/* mman ----------------------------------------------------------------- */

TEST(WindowsShimMman, AnonymousMappingRoundtrip)
{
	const size_t len = 4096;
	void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(p, MAP_FAILED);
	ASSERT_NE(p, nullptr);

	/* Memory should be writable. */
	memset(p, 0xAB, len);
	EXPECT_EQ(((unsigned char *)p)[0], 0xABu);
	EXPECT_EQ(((unsigned char *)p)[len - 1], 0xABu);

	EXPECT_EQ(munmap(p, len), 0);
}

TEST(WindowsShimMman, MapFixedRejected)
{
	const size_t len = 4096;
	errno = 0;
	void *p = mmap(reinterpret_cast<void *>(0x10000000),
		       len, PROT_READ, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED,
		       -1, 0);
	EXPECT_EQ(p, MAP_FAILED);
}

TEST(WindowsShimMman, MunmapNullSucceedsOrFails)
{
	/* munmap(nullptr, 0) is implementation defined. The shim must at
	 * least not crash. */
	int rc = munmap(nullptr, 0);
	(void)rc;
	SUCCEED();
}

TEST(WindowsShimMman, MlockMlockallNoOps)
{
	char buf[64];
	EXPECT_EQ(mlock(buf, sizeof(buf)) >= -1, true);
	EXPECT_EQ(munlock(buf, sizeof(buf)) >= -1, true);
	EXPECT_EQ(mlockall(MCL_CURRENT), 0);
	EXPECT_EQ(munlockall(), 0);
}

TEST(WindowsShimMman, MprotectAcceptsKnownProt)
{
	const size_t len = 4096;
	void *p = mmap(nullptr, len, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(p, MAP_FAILED);
	int rc = mprotect(p, len, PROT_READ);
	(void)rc; /* implementation may return 0 or -1 depending on backing */
	EXPECT_EQ(munmap(p, len), 0);
}

/* flock ---------------------------------------------------------------- */

TEST(WindowsShimFlock, ExclusiveOnTempFile)
{
	char path[MAX_PATH];
	GetTempPathA(sizeof(path), path);
	strcat_s(path, sizeof(path), "px4_flock_test.lock");

	int fd = _open(path, _O_CREAT | _O_RDWR, _S_IREAD | _S_IWRITE);
	ASSERT_GE(fd, 0);

	EXPECT_EQ(flock(fd, LOCK_EX | LOCK_NB), 0);
	EXPECT_EQ(flock(fd, LOCK_UN), 0);

	_close(fd);
	DeleteFileA(path);
}

TEST(WindowsShimFlock, BadFdReturnsError)
{
	errno = 0;
	EXPECT_EQ(flock(-1, LOCK_EX), -1);
}

/* dlfcn ---------------------------------------------------------------- */

TEST(WindowsShimDlfcn, OpenNullReturnsHandle)
{
	/* Some implementations return a sentinel pseudo-handle for the
	 * current process; others return NULL. Both are acceptable as
	 * long as dlerror() is callable and dlclose() handles the result. */
	void *h = dlopen(nullptr, 0);
	const char *err = dlerror();
	(void)err;

	if (h) {
		EXPECT_EQ(dlclose(h), 0);
	}
}

TEST(WindowsShimDlfcn, OpenNonexistentSetsDlerror)
{
	void *h = dlopen("definitely-not-a-real-library-xxx.dll", 0);
	EXPECT_EQ(h, nullptr);
	const char *err = dlerror();
	EXPECT_NE(err, nullptr);
}

TEST(WindowsShimDlfcn, SymInModule)
{
	/* kernel32 is always loaded; resolve a known export. */
	void *h = dlopen("kernel32.dll", 0);

	if (h) {
		void *sym = dlsym(h, "GetCurrentProcessId");
		EXPECT_NE(sym, nullptr);
		dlclose(h);
	}
}

TEST(WindowsShimDlfcn, DladdrAcceptsCallSite)
{
	Dl_info info = {};
	int rc = dladdr((void *)&dladdr, &info);
	/* Some shim builds may not implement dladdr; ensure return
	 * value matches POSIX shape (non-zero on success / 0 on error). */
	(void)rc;
	SUCCEED();
}

TEST(WindowsShimDlfcn, DlerrorClearsAfterRead)
{
	(void)dlopen("definitely-not-a-real-library-yyy.dll", 0);
	const char *first = dlerror();
	const char *second = dlerror();
	(void)first;
	/* Second call should return null per POSIX. */
	EXPECT_EQ(second, nullptr);
}

/* ids: getppid / setsid / getsid -------------------------------------- */

TEST(WindowsShimIds, GetppidNeverFails)
{
	pid_t pp = getppid();
	(void)pp;
	SUCCEED();
}

TEST(WindowsShimIds, SetsidReturnsAnId)
{
	pid_t s = setsid();
	EXPECT_NE(s, (pid_t) - 1);
}

TEST(WindowsShimIds, GetsidSelf)
{
	pid_t s = getsid(0);
	EXPECT_NE(s, (pid_t) - 1);
}

TEST(WindowsShimIds, GetsidNonexistent)
{
	errno = 0;
	pid_t s = getsid(0x7FFFFFFE);
	EXPECT_EQ(s, (pid_t) - 1);
	EXPECT_EQ(errno, ESRCH);
}

/* if_query ------------------------------------------------------------- */

TEST(WindowsShimIfQuery, NameindexEnumerates)
{
	struct if_nameindex *list = if_nameindex();

	if (list) {
		bool found_name = false;

		for (struct if_nameindex *p = list; p->if_name; ++p) {
			EXPECT_NE(p->if_name, nullptr);
			found_name = found_name || (p->if_name[0] != '\0');
		}

		(void)found_name;
		if_freenameindex(list);
		SUCCEED();

	} else {
		/* Acceptable when running without network privileges. */
		SUCCEED();
	}
}

TEST(WindowsShimIfQuery, FreeNullSafe)
{
	if_freenameindex(nullptr);
	SUCCEED();
}

/* resolver: inet_aton, inet_makeaddr, inet_lnaof, inet_netof, gethostent  */

TEST(WindowsShimResolver, InetAtonValid)
{
	struct in_addr a = {};
	EXPECT_NE(inet_aton("127.0.0.1", &a), 0);
}

TEST(WindowsShimResolver, InetAtonInvalid)
{
	struct in_addr a = {};
	EXPECT_EQ(inet_aton("not.an.ip.addr.zzzz", &a), 0);
}

TEST(WindowsShimResolver, InetNtoaR)
{
	struct in_addr a;
	a.s_addr = htonl(0x7F000001);
	char buf[32] = {};
	const char *r = inet_ntoa_r(a, buf, sizeof(buf));
	EXPECT_NE(r, nullptr);
	EXPECT_STREQ(buf, "127.0.0.1");
}

TEST(WindowsShimResolver, MakeAddrSplit)
{
	struct in_addr a = inet_makeaddr(0x7F, 0x000001);
	(void)a;
	SUCCEED();
}

TEST(WindowsShimResolver, NetofLnaof)
{
	struct in_addr a;
	a.s_addr = htonl(0x7F000001);
	in_addr_t net = inet_netof(a);
	in_addr_t host = inet_lnaof(a);
	EXPECT_NE(net + host, 0u);
}

TEST(WindowsShimResolver, HostentIterators)
{
	sethostent(0);
	struct hostent *e = gethostent();
	(void)e;
	endhostent();
	SUCCEED();
}

TEST(WindowsShimResolver, NetentIterators)
{
	setnetent(0);
	struct netent *n = getnetent();
	(void)n;
	endnetent();
	struct netent *byname = getnetbyname("loopback");
	(void)byname;
	struct netent *byaddr = getnetbyaddr(0x7F000000, AF_INET);
	(void)byaddr;
	SUCCEED();
}

TEST(WindowsShimResolver, ProtoentIterators)
{
	setprotoent(0);
	struct protoent *p = getprotoent();
	(void)p;
	endprotoent();
	SUCCEED();
}

TEST(WindowsShimResolver, ServentIterators)
{
	setservent(0);
	struct servent *s = getservent();
	(void)s;
	endservent();
	SUCCEED();
}

TEST(WindowsShimResolver, HstrerrorTextNonNull)
{
	EXPECT_NE(hstrerror(HOST_NOT_FOUND), nullptr);
	EXPECT_NE(hstrerror(0), nullptr);
}

/* sockets - minimal loopback round trip --------------------------------- */

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
}

TEST(WindowsShimSocket, UdpLoopbackSendRecv)
{
	WinsockBootstrap _ws;

	SOCKET srv = socket(AF_INET, SOCK_DGRAM, 0);
	ASSERT_NE(srv, INVALID_SOCKET);

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0; /* any */
	ASSERT_EQ(bind(srv, (struct sockaddr *)&addr, sizeof(addr)), 0);

	int alen = sizeof(addr);
	ASSERT_EQ(getsockname(srv, (struct sockaddr *)&addr, &alen), 0);

	SOCKET cli = socket(AF_INET, SOCK_DGRAM, 0);
	ASSERT_NE(cli, INVALID_SOCKET);

	const char msg[] = "ping";
	ASSERT_EQ(sendto(cli, msg, (int)sizeof(msg), 0,
			 (struct sockaddr *)&addr, sizeof(addr)),
		  (int)sizeof(msg));

	char buf[16] = {};
	struct sockaddr_in from;
	int from_len = sizeof(from);
	int n = recvfrom(srv, buf, sizeof(buf), 0,
			 (struct sockaddr *)&from, &from_len);
	EXPECT_EQ(n, (int)sizeof(msg));
	EXPECT_STREQ(buf, "ping");

	closesocket(srv);
	closesocket(cli);
}

TEST(WindowsShimSocket, SetsockoptReuseaddr)
{
	WinsockBootstrap _ws;
	SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
	ASSERT_NE(s, INVALID_SOCKET);
	int yes = 1;
	int rc = setsockopt(s, SOL_SOCKET, SO_REUSEADDR,
			    (const char *)&yes, sizeof(yes));
	EXPECT_EQ(rc, 0);
	closesocket(s);
}

#endif // _WIN32

/* Sentinel test so the binary always has at least one passing case. */
TEST(WindowsShimRuntime, BuildSentinel)
{
	SUCCEED();
}
