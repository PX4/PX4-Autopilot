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
 * @file test_windows_shim_headers.cpp
 *
 * Unit tests for the inline functions defined in the Windows POSIX shim
 * headers under platforms/posix/include/windows_shim/.
 *
 * Each header that declares logic-bearing inline functions is exercised
 * for happy-path, NULL/empty, boundary, and error cases. Tests gated on
 * _WIN32 because the shim headers are Windows-only by design.
 */

#include <gtest/gtest.h>

#ifdef _WIN32

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>

/* The shim header for usleep references three tunables that are normally
 * defined in platforms/posix/src/px4/windows/runtime/init.cpp. This unit
 * test binary intentionally does not link init.cpp (it would drag in the
 * full runtime), so we provide local definitions matching the documented
 * defaults. Keep them in sync with init.cpp. */
extern "C" {
	long g_usleep_pure_spin_us = 5000;
	long g_usleep_spin_tail_us = 1000;
	long g_usleep_adaptive_min_tail_us = 700;
	int g_usleep_use_wine_deadline_clock = 0;
	uint64_t px4_windows_wine_monotonic_time_us()
	{
		return 0;
	}
	void px4_windows_wine_spin_until_us(uint64_t deadline_us)
	{
		(void)deadline_us;
	}
}

/* Pull headers via the shim include path so the inline functions under
 * test are the same definitions PX4 sources see. */
#include <libgen.h>
#include <dirent.h>
#include <sched.h>
#include <syslog.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

/* basename / dirname --------------------------------------------------- */

TEST(WindowsShimLibgen, BasenameNull)
{
	EXPECT_STREQ(basename(nullptr), ".");
}

TEST(WindowsShimLibgen, BasenameEmpty)
{
	char path[1] = { 0 };
	EXPECT_STREQ(basename(path), ".");
}

TEST(WindowsShimLibgen, BasenameLeafOnly)
{
	char path[] = "file.txt";
	EXPECT_STREQ(basename(path), "file.txt");
}

TEST(WindowsShimLibgen, BasenamePosixSeparators)
{
	char path[] = "a/b/c/foo";
	EXPECT_STREQ(basename(path), "foo");
}

TEST(WindowsShimLibgen, BasenameWindowsSeparators)
{
	char path[] = "a\\b\\c\\foo.bin";
	EXPECT_STREQ(basename(path), "foo.bin");
}

TEST(WindowsShimLibgen, BasenameMixedSeparators)
{
	char path[] = "C:/Users\\Nuno/file";
	EXPECT_STREQ(basename(path), "file");
}

TEST(WindowsShimLibgen, BasenameTrailingSlashReturnsDot)
{
	char path[] = "dir/";
	EXPECT_STREQ(basename(path), ".");
}

TEST(WindowsShimLibgen, DirnameNullOrEmpty)
{
	EXPECT_STREQ(dirname(nullptr), ".");
	char empty[1] = { 0 };
	EXPECT_STREQ(dirname(empty), ".");
}

TEST(WindowsShimLibgen, DirnameNoSeparators)
{
	char path[] = "filename";
	EXPECT_STREQ(dirname(path), ".");
}

TEST(WindowsShimLibgen, DirnamePosixPath)
{
	char path[] = "a/b/c/foo";
	EXPECT_STREQ(dirname(path), "a/b/c");
}

TEST(WindowsShimLibgen, DirnameWindowsPath)
{
	char path[] = "C:\\Users\\foo\\bar";
	EXPECT_STREQ(dirname(path), "C:\\Users\\foo");
}

TEST(WindowsShimLibgen, DirnameRootSeparatorPreserved)
{
	char path[] = "/foo";
	/* When the only separator is at index 0, dirname should leave it
	 * in place to preserve the root anchor. */
	EXPECT_STREQ(dirname(path), "/");
}

/* sched ---------------------------------------------------------------- */

#if defined(_MSC_VER) && !defined(__clang__)
TEST(WindowsShimSched, PriorityRange)
{
	EXPECT_EQ(sched_get_priority_max(SCHED_OTHER), 15);
	EXPECT_EQ(sched_get_priority_max(SCHED_FIFO), 15);
	EXPECT_EQ(sched_get_priority_min(SCHED_OTHER), -15);
	EXPECT_EQ(sched_get_priority_min(SCHED_RR), -15);
}

TEST(WindowsShimSched, YieldReturnsZero)
{
	EXPECT_EQ(sched_yield(), 0);
}
#endif

/* syslog --------------------------------------------------------------- */

TEST(WindowsShimSyslog, OpenAndCloseNoOps)
{
	openlog("ident", LOG_PID, LOG_USER);
	closelog();
	SUCCEED();
}

TEST(WindowsShimSyslog, SetLogMaskRoundtrip)
{
	const int previous = setlogmask(LOG_UPTO(LOG_INFO));
	const int updated = setlogmask(LOG_UPTO(LOG_DEBUG));
	EXPECT_EQ(updated, LOG_UPTO(LOG_INFO));

	/* setlogmask(0) returns the active mask without modifying it. */
	const int current = setlogmask(0);
	EXPECT_EQ(current, LOG_UPTO(LOG_DEBUG));

	/* Restore original. */
	setlogmask(previous);
}

TEST(WindowsShimSyslog, MaskSuppressesLogging)
{
	/* Mute everything; vsyslog should bail out without writing. */
	setlogmask(LOG_MASK(LOG_EMERG));
	syslog(LOG_DEBUG, "should-not-emit-%d", 42);
	setlogmask(LOG_UPTO(LOG_DEBUG));
	SUCCEED();
}

TEST(WindowsShimSyslog, FacilityPriorityHelpers)
{
	const int packed = LOG_MAKEPRI(LOG_USER, LOG_INFO);
	EXPECT_EQ(LOG_PRI(packed), LOG_INFO);
	EXPECT_EQ(LOG_FAC(packed), LOG_FAC(LOG_USER));
}

/* mman ----------------------------------------------------------------- */

TEST(WindowsShimMmanHeader, MadviseAlwaysSucceeds)
{
	char buf[64];
	EXPECT_EQ(madvise(buf, sizeof(buf), MADV_NORMAL), 0);
	EXPECT_EQ(madvise(buf, sizeof(buf), MADV_RANDOM), 0);
	EXPECT_EQ(madvise(nullptr, 0, MADV_DONTNEED), 0);
}

TEST(WindowsShimMmanHeader, ProtFlagBits)
{
	EXPECT_EQ(PROT_NONE, 0);
	EXPECT_NE(PROT_READ & PROT_WRITE, PROT_READ);
	EXPECT_EQ(PROT_READ | PROT_WRITE | PROT_EXEC, 0x7);
}

TEST(WindowsShimMmanHeader, MapAnonAlias)
{
	EXPECT_EQ(MAP_ANON, MAP_ANONYMOUS);
}

TEST(WindowsShimMmanHeader, PosixMadviseAliases)
{
	EXPECT_EQ(POSIX_MADV_NORMAL, MADV_NORMAL);
	EXPECT_EQ(POSIX_MADV_DONTNEED, MADV_DONTNEED);
}

/* time helpers --------------------------------------------------------- */

TEST(WindowsShimTime, GmtimeRRejectsNull)
{
	struct tm out = {};
	time_t t = 0;
	EXPECT_EQ(gmtime_r(nullptr, &out), nullptr);
	EXPECT_EQ(gmtime_r(&t, nullptr), nullptr);
}

TEST(WindowsShimTime, GmtimeRRoundtripEpoch)
{
	struct tm out = {};
	time_t t = 0; /* 1970-01-01 00:00:00 UTC */
	struct tm *r = gmtime_r(&t, &out);
	ASSERT_NE(r, nullptr);
	EXPECT_EQ(out.tm_year, 70);
	EXPECT_EQ(out.tm_mon, 0);
	EXPECT_EQ(out.tm_mday, 1);
}

TEST(WindowsShimTime, LocaltimeRRejectsNull)
{
	struct tm out = {};
	time_t t = 1234567890;
	EXPECT_EQ(localtime_r(nullptr, &out), nullptr);
	EXPECT_EQ(localtime_r(&t, nullptr), nullptr);
}

TEST(WindowsShimTime, LocaltimeRPopulatesYear)
{
	struct tm out = {};
	time_t t = 1234567890; /* 2009-02-13 */
	struct tm *r = localtime_r(&t, &out);
	ASSERT_NE(r, nullptr);
	EXPECT_GT(out.tm_year, 100);
}

TEST(WindowsShimTime, AsctimeRRejectsNull)
{
	struct tm tm_in = {};
	char buf[26] = {};
	EXPECT_EQ(asctime_r(nullptr, buf), nullptr);
	EXPECT_EQ(asctime_r(&tm_in, nullptr), nullptr);
}

TEST(WindowsShimTime, AsctimeRPopulatesBuffer)
{
	struct tm tm_in = {};
	tm_in.tm_year = 70;
	tm_in.tm_mon = 0;
	tm_in.tm_mday = 1;
	tm_in.tm_hour = 0;
	tm_in.tm_min = 0;
	tm_in.tm_sec = 0;
	tm_in.tm_wday = 4;
	char buf[26] = {};
	char *r = asctime_r(&tm_in, buf);
	ASSERT_NE(r, nullptr);
	EXPECT_GT(strlen(buf), 10u);
}

TEST(WindowsShimTime, CtimeRRejectsNull)
{
	time_t t = 0;
	char buf[26] = {};
	EXPECT_EQ(ctime_r(nullptr, buf), nullptr);
	EXPECT_EQ(ctime_r(&t, nullptr), nullptr);
}

TEST(WindowsShimTime, CtimeRRoundtrip)
{
	time_t t = 1700000000;
	char buf[26] = {};
	char *r = ctime_r(&t, buf);
	ASSERT_NE(r, nullptr);
	EXPECT_GT(strlen(buf), 10u);
}

TEST(WindowsShimTime, TimerMacros)
{
	struct timeval a = { 5, 200000 };
	struct timeval b = { 5, 200000 };
	EXPECT_TRUE(timerisset(&a));
	EXPECT_FALSE(timercmp(&a, &b, !=));
	struct timeval r = {};
	timeradd(&a, &b, &r);
	EXPECT_EQ(r.tv_sec, 10);
	EXPECT_EQ(r.tv_usec, 400000);
	struct timeval s = { 6, 0 };
	struct timeval d = {};
	timersub(&s, &a, &d);
	EXPECT_EQ(d.tv_sec, 0);
	EXPECT_EQ(d.tv_usec, 800000);
	timerclear(&a);
	EXPECT_FALSE(timerisset(&a));
}

/* unistd usleep / sleep ----------------------------------------------- */

TEST(WindowsShimUnistd, UsleepZeroReturnsImmediately)
{
	auto t0 = std::chrono::steady_clock::now();
	EXPECT_EQ(usleep(0), 0);
	auto dt = std::chrono::steady_clock::now() - t0;
	/* Should be sub-millisecond on any reasonable host. */
	EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(dt).count(), 50);
}

TEST(WindowsShimUnistd, UsleepShortRespectsLowerBound)
{
	const useconds_t target_us = 2000; /* 2 ms */
	auto t0 = std::chrono::steady_clock::now();
	ASSERT_EQ(usleep(target_us), 0);
	auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	/* Allow generous early-wake margin; the shim explicitly closes the
	 * residual against the QPC target so we should not be early. */
	EXPECT_GE(dt, (long long)target_us - 200);
}

TEST(WindowsShimUnistd, UsleepLongerDuration)
{
	const useconds_t target_us = 20000; /* 20 ms */
	auto t0 = std::chrono::steady_clock::now();
	ASSERT_EQ(usleep(target_us), 0);
	auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	EXPECT_GE(dt, (long long)target_us - 500);
	/* Be generous on the upper bound to tolerate Windows scheduler jitter. */
	EXPECT_LT(dt, (long long)target_us + 100000);
}

TEST(WindowsShimUnistd, UsleepThreadLocalTimerReuse)
{
	/* Two consecutive sleeps in the same thread reuse the cached
	 * waitable timer; both must still hit the deadline. */
	for (int i = 0; i < 3; ++i) {
		auto t0 = std::chrono::steady_clock::now();
		ASSERT_EQ(usleep(5000), 0);
		auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
				  std::chrono::steady_clock::now() - t0).count();
		EXPECT_GE(dt, 4500);
	}
}

#if defined(_MSC_VER) && !defined(__clang__)
TEST(WindowsShimUnistd, SleepOneSecond)
{
	auto t0 = std::chrono::steady_clock::now();
	EXPECT_EQ(sleep(1u), 0u);
	auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
			  std::chrono::steady_clock::now() - t0).count();
	EXPECT_GE(dt, 950);
}
#endif

/* dirent --------------------------------------------------------------- */

TEST(WindowsShimDirent, OpendirNullPath)
{
	errno = 0;
	EXPECT_EQ(opendir(nullptr), nullptr);
	EXPECT_EQ(errno, EINVAL);
}

TEST(WindowsShimDirent, OpendirNonexistentReturnsNull)
{
	errno = 0;
	EXPECT_EQ(opendir("Z:\\definitely\\does\\not\\exist\\px4test"), nullptr);
	EXPECT_EQ(errno, ENOENT);
}

TEST(WindowsShimDirent, OpendirCwd)
{
	DIR *d = opendir(".");
	ASSERT_NE(d, nullptr);
	EXPECT_EQ(closedir(d), 0);
}

TEST(WindowsShimDirent, ReaddirEnumeratesDotEntries)
{
	DIR *d = opendir(".");
	ASSERT_NE(d, nullptr);
	bool saw_any = false;

	while (struct dirent *e = readdir(d)) {
		EXPECT_GT(strlen(e->d_name), 0u);
		EXPECT_LE(strlen(e->d_name), (size_t)NAME_MAX);
		saw_any = true;
	}

	EXPECT_TRUE(saw_any);
	EXPECT_EQ(closedir(d), 0);
}

TEST(WindowsShimDirent, ReaddirNullStream)
{
	errno = 0;
	EXPECT_EQ(readdir(nullptr), nullptr);
	EXPECT_EQ(errno, EBADF);
}

TEST(WindowsShimDirent, ClosedirNullReturnsError)
{
	errno = 0;
	EXPECT_EQ(closedir(nullptr), -1);
	EXPECT_EQ(errno, EBADF);
}

TEST(WindowsShimDirent, RewinddirRestartsIteration)
{
	DIR *d = opendir(".");
	ASSERT_NE(d, nullptr);
	int first_pass = 0;

	while (readdir(d)) {
		++first_pass;
	}

	rewinddir(d);
	int second_pass = 0;

	while (readdir(d)) {
		++second_pass;
	}

	EXPECT_EQ(first_pass, second_pass);
	EXPECT_GT(first_pass, 0);
	EXPECT_EQ(closedir(d), 0);

	/* rewinddir on null is a no-op. */
	rewinddir(nullptr);
}

TEST(WindowsShimDirent, TelldirNull)
{
	errno = 0;
	EXPECT_EQ(telldir(nullptr), -1);
	EXPECT_EQ(errno, EBADF);
}

TEST(WindowsShimDirent, TelldirAdvances)
{
	DIR *d = opendir(".");
	ASSERT_NE(d, nullptr);
	long start = telldir(d);
	EXPECT_EQ(start, 0);

	if (readdir(d) != nullptr) {
		EXPECT_GT(telldir(d), start);
	}

	EXPECT_EQ(closedir(d), 0);
}

TEST(WindowsShimDirent, SeekdirRestartsAndAdvances)
{
	DIR *d = opendir(".");
	ASSERT_NE(d, nullptr);

	/* Read a couple, capture position, rewind, seekdir there. */
	if (readdir(d) && readdir(d)) {
		long pos = telldir(d);
		seekdir(d, 0);
		EXPECT_EQ(telldir(d), 0);
		seekdir(d, pos);
		/* Negative offsets are ignored. */
		seekdir(d, -1);
	}

	/* Null and bad inputs are silent no-ops. */
	seekdir(nullptr, 0);
	EXPECT_EQ(closedir(d), 0);
}

TEST(WindowsShimDirent, AlphasortAndVersionsort)
{
	struct dirent a = {};
	struct dirent b = {};
	strcpy(a.d_name, "alpha");
	strcpy(b.d_name, "beta");
	const struct dirent *pa = &a;
	const struct dirent *pb = &b;
	EXPECT_LT(alphasort(&pa, &pb), 0);
	EXPECT_GT(alphasort(&pb, &pa), 0);
	EXPECT_EQ(versionsort(&pa, &pb), alphasort(&pa, &pb));
}

TEST(WindowsShimDirent, ScandirNullList)
{
	errno = 0;
	EXPECT_EQ(scandir(".", nullptr, nullptr, nullptr), -1);
	EXPECT_EQ(errno, EINVAL);
}

TEST(WindowsShimDirent, ScandirCwdNonNegative)
{
	struct dirent **list = nullptr;
	int n = scandir(".", &list, nullptr, alphasort);
	ASSERT_GE(n, 0);

	for (int i = 0; i < n; ++i) {
		free(list[i]);
	}

	free(list);
}

static int filter_drop_dot(const struct dirent *e)
{
	return e && e->d_name[0] != '.';
}

TEST(WindowsShimDirent, ScandirSelectorFilters)
{
	struct dirent **list = nullptr;
	int n = scandir(".", &list, &filter_drop_dot, nullptr);
	ASSERT_GE(n, 0);

	for (int i = 0; i < n; ++i) {
		EXPECT_NE(list[i]->d_name[0], '.');
		free(list[i]);
	}

	free(list);
}

TEST(WindowsShimDirent, ScandirNonexistent)
{
	struct dirent **list = nullptr;
	int n = scandir("Z:\\definitely\\does\\not\\exist\\px4test_scan",
			&list, nullptr, nullptr);
	EXPECT_EQ(n, -1);
}

#if defined(_MSC_VER) && !defined(__clang__)
/* getopt is MSVC-only; MinGW pulls system getopt. */
#include <getopt.h>

TEST(WindowsShimGetopt, SimpleShortOptions)
{
	/* getopt holds global state; reset it before each scenario. */
	optind = 1;
	px4_getopt_nextchar = 1;

	char *argv[] = {
		(char *)"prog",
		(char *)"-a",
		(char *)"-b",
		(char *)"hello",
		(char *)"rest",
		nullptr
	};
	int argc = 5;

	int c = getopt(argc, argv, "ab:");
	EXPECT_EQ(c, 'a');
	c = getopt(argc, argv, "ab:");
	EXPECT_EQ(c, 'b');
	EXPECT_STREQ(optarg, "hello");
	c = getopt(argc, argv, "ab:");
	EXPECT_EQ(c, -1);
	EXPECT_LT(optind, argc);
}

TEST(WindowsShimGetopt, UnknownOptionReturnsQuestion)
{
	optind = 1;
	px4_getopt_nextchar = 1;
	char *argv[] = { (char *)"prog", (char *)"-x", nullptr };
	int c = getopt(2, argv, "ab:");
	EXPECT_EQ(c, '?');
	EXPECT_EQ(optopt, 'x');
}

TEST(WindowsShimGetopt, MissingArgumentForRequired)
{
	optind = 1;
	px4_getopt_nextchar = 1;
	char *argv[] = { (char *)"prog", (char *)"-a", nullptr };
	int c = getopt(2, argv, "a:");
	/* No leading colon in optstring -> '?'. */
	EXPECT_EQ(c, '?');
}

TEST(WindowsShimGetopt, DashDashTerminator)
{
	optind = 1;
	px4_getopt_nextchar = 1;
	char *argv[] = { (char *)"prog", (char *)"--", (char *)"file", nullptr };
	int c = getopt(3, argv, "ab:");
	EXPECT_EQ(c, -1);
	EXPECT_EQ(optind, 2);
}

TEST(WindowsShimGetopt, GetoptLongMatch)
{
	optind = 1;
	px4_getopt_nextchar = 1;
	option longopts[] = {
		{ "verbose", no_argument,       nullptr, 'v' },
		{ "file",    required_argument, nullptr, 'f' },
		{ nullptr,   0,                 nullptr, 0 }
	};
	char *argv[] = {
		(char *)"prog",
		(char *)"--verbose",
		(char *)"--file=foo.bin",
		nullptr
	};
	int idx = -1;
	int c = getopt_long(3, argv, "vf:", longopts, &idx);
	EXPECT_EQ(c, 'v');
	EXPECT_EQ(idx, 0);
	c = getopt_long(3, argv, "vf:", longopts, &idx);
	EXPECT_EQ(c, 'f');
	EXPECT_STREQ(optarg, "foo.bin");
}

TEST(WindowsShimGetopt, GetoptLongRequiredFromNextArg)
{
	optind = 1;
	px4_getopt_nextchar = 1;
	option longopts[] = {
		{ "name", required_argument, nullptr, 'n' },
		{ nullptr, 0, nullptr, 0 }
	};
	char *argv[] = {
		(char *)"prog",
		(char *)"--name",
		(char *)"alice",
		nullptr
	};
	int idx = -1;
	int c = getopt_long(3, argv, "n:", longopts, &idx);
	EXPECT_EQ(c, 'n');
	EXPECT_STREQ(optarg, "alice");
}
#endif // MSVC

/* Pure-header fcntl tests -- the file logic is in fcntl.h's static inline
 * fcntl(); F_GETFL/F_GETFD/F_SETFD/F_SETFL paths can be exercised in
 * isolation without opening Win32 handles. */
#include <fcntl.h>

TEST(WindowsShimFcntl, GetflReturnsZero)
{
	EXPECT_EQ(fcntl(0, F_GETFL), 0);
	EXPECT_EQ(fcntl(0, F_GETFD), 0);
}

TEST(WindowsShimFcntl, SetfdAcceptsCloexec)
{
	EXPECT_EQ(fcntl(0, F_SETFD, FD_CLOEXEC), 0);
}

TEST(WindowsShimFcntl, SetflWithoutNonblockReturnsZero)
{
	EXPECT_EQ(fcntl(0, F_SETFL, 0), 0);
}

TEST(WindowsShimFcntl, UnknownCommandRejected)
{
	errno = 0;
	EXPECT_EQ(fcntl(0, 0xBAD), -1);
	EXPECT_EQ(errno, EINVAL);
}

#endif // _WIN32

/* On non-Windows builds the file compiles to a single trivial test so the
 * gtest target still produces output. */
TEST(WindowsShimHeaders, BuildSentinel)
{
	SUCCEED();
}
