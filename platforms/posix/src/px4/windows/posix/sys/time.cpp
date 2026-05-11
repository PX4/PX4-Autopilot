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
 * @file time.cpp
 *
 * POSIX time functions missing from MSVC's CRT.
 *
 * MinGW already supplies these through its POSIX-ish runtime. This file is
 * MSVC-only and maps POSIX clock ids onto Windows high-resolution timers:
 * CLOCK_MONOTONIC uses QueryPerformanceCounter, while CLOCK_REALTIME uses
 * FILETIME converted from the Windows epoch to Unix time.
 */

#include "px4_windows_internal.h"

#if defined(_MSC_VER)

namespace
{
static constexpr uint64_t filetime_unix_epoch_100ns = 116444736000000000ULL;
}

extern "C" {

	int clock_gettime(clockid_t clk_id, struct timespec *tp)
	{
		if (!tp) {
			errno = EINVAL;
			return -1;
		}

		if (clk_id == CLOCK_MONOTONIC) {
			/* QPC is monotonic and high resolution, but relative to an arbitrary
			 * boot-time counter. That is exactly what CLOCK_MONOTONIC promises.
			 *
			 * Per Microsoft's QueryPerformanceCounter guidance, the QPC frequency
			 * is fixed at system boot and consistent across processors, so we
			 * only need to query it once. clock_gettime is on PX4's hot path
			 * (drv_hrt's hrt_absolute_time, lockstep_scheduler, every uORB
			 * publish/subscribe), and a syscall here adds up quickly.
			 */
			static const int64_t frequency = []() {
				LARGE_INTEGER f {};
				QueryPerformanceFrequency(&f);
				return f.QuadPart;
			}();

			LARGE_INTEGER counter {};
			QueryPerformanceCounter(&counter);

			const uint64_t seconds = static_cast<uint64_t>(counter.QuadPart / frequency);
			const uint64_t remainder = static_cast<uint64_t>(counter.QuadPart % frequency);
			tp->tv_sec = static_cast<time_t>(seconds);
			tp->tv_nsec = static_cast<long>((remainder * 1000000000ULL) / static_cast<uint64_t>(frequency));
			return 0;
		}

		if (clk_id == CLOCK_REALTIME) {
			/* FILETIME counts 100 ns ticks since 1601-01-01 UTC. Convert to the
			 * Unix epoch expected by timespec. */
			FILETIME ft {};
			GetSystemTimePreciseAsFileTime(&ft);
			const uint64_t ticks = (static_cast<uint64_t>(ft.dwHighDateTime) << 32) | ft.dwLowDateTime;
			const uint64_t unix_100ns = ticks - filetime_unix_epoch_100ns;
			tp->tv_sec = static_cast<time_t>(unix_100ns / 10000000ULL);
			tp->tv_nsec = static_cast<long>((unix_100ns % 10000000ULL) * 100ULL);
			return 0;
		}

		errno = EINVAL;
		return -1;
	}

	int clock_settime(clockid_t clk_id, const struct timespec *tp)
	{
		(void)clk_id;
		(void)tp;
		errno = ENOTSUP;
		return -1;
	}

	int gettimeofday(struct timeval *tv, void *tz)
	{
		(void)tz;

		if (!tv) {
			errno = EINVAL;
			return -1;
		}

		timespec ts {};

		if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
			return -1;
		}

		tv->tv_sec = static_cast<long>(ts.tv_sec);
		tv->tv_usec = ts.tv_nsec / 1000;
		return 0;
	}

	int nanosleep(const struct timespec *req, struct timespec *rem)
	{
		(void)rem;

		if (!req || req->tv_nsec < 0 || req->tv_nsec >= 1000000000L) {
			errno = EINVAL;
			return -1;
		}

		const uint64_t ms = (static_cast<uint64_t>(req->tv_sec) * 1000ULL) +
				    ((static_cast<uint64_t>(req->tv_nsec) + 999999ULL) / 1000000ULL);
		/* Sleep() is millisecond-granular and cannot report remaining time if it is
		 * interrupted, so rem is intentionally unsupported for now. */
		Sleep(ms > MAXDWORD ? MAXDWORD : static_cast<DWORD>(ms));
		return 0;
	}

} // extern "C"

#endif // defined(_MSC_VER)
