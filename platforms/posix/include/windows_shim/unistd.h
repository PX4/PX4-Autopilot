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
 * @file unistd.h
 *
 * unistd.h extension shim for PX4 SITL on Windows.
 *
 * MinGW ships a minimal <unistd.h> but omits POSIX helpers that PX4 and
 * third-party code use directly (pipe, fsync, symlink, dprintf, sysconf,
 * process/session helpers, environment setters, hard links).
 * Forward to the real header via #include_next and layer the missing
 * pieces on top using Win32 CRT equivalents (_pipe, _commit, _write)
 * or Win32 APIs (CreateSymbolicLinkA, GetSystemInfo).
 */
#pragma once

#if defined(_MSC_VER)
#include <sys/types.h>
#elif defined(_WIN32)
/*
 * MinGW declares its own usleep() in <unistd.h>. Pull the rest of that
 * header through normally, but hide only that declaration so PX4 can provide
 * the same high-resolution Windows implementation for system_usleep.
 */
#define usleep _px4_mingw_runtime_usleep
#include_next <unistd.h>
#undef usleep
#else
#include_next <unistd.h>
#endif
#include <io.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <windows.h>

/* POSIX standard fd numbers - MinGW defines them via io.h but belt-and-suspenders. */
#ifndef STDIN_FILENO
#define STDIN_FILENO  0
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif
#ifndef F_OK
#define F_OK 0
#endif
#ifndef X_OK
#define X_OK 1
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef R_OK
#define R_OK 4
#endif

/* POSIX sysconf selectors that MinGW doesn't ship. Numerical values
 * don't need to match Linux - only our own sysconf() implementation
 * inspects them. */
#ifndef _SC_PAGESIZE
#define _SC_PAGESIZE          30
#endif
#ifndef _SC_PAGE_SIZE
#define _SC_PAGE_SIZE         _SC_PAGESIZE
#endif
#ifndef _SC_CLK_TCK
#define _SC_CLK_TCK           2
#endif
#ifndef _SC_NPROCESSORS_ONLN
#define _SC_NPROCESSORS_ONLN  84
#endif
#ifndef _SC_NPROCESSORS_CONF
#define _SC_NPROCESSORS_CONF  83
#endif
#ifndef _SC_OPEN_MAX
#define _SC_OPEN_MAX          4
#endif
#ifndef _SC_HOST_NAME_MAX
#define _SC_HOST_NAME_MAX     180
#endif
#ifndef _SC_LOGIN_NAME_MAX
#define _SC_LOGIN_NAME_MAX    71
#endif
#ifndef _SC_PHYS_PAGES
#define _SC_PHYS_PAGES        85
#endif
#ifndef _SC_AVPHYS_PAGES
#define _SC_AVPHYS_PAGES      86
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
/* CREATE_WAITABLE_TIMER_HIGH_RESOLUTION (Windows 10 1803+; build 17134)
 * may not be defined in older SDK headers - fall back to the literal
 * value documented by Microsoft. Same for the manual-reset flag. */
#ifndef CREATE_WAITABLE_TIMER_MANUAL_RESET
#define CREATE_WAITABLE_TIMER_MANUAL_RESET 0x00000001
#endif
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

#if defined(_MSC_VER)
#define PX4_WINDOWS_SLEEP_TLS __declspec(thread)
#else
#define PX4_WINDOWS_SLEEP_TLS __thread
#endif

/**
 * Runtime-tuned thresholds that drive the spin-residual hybrid below.
 *
 * Defined and (optionally) auto-calibrated by
 * px4_windows_calibrate_usleep_threshold() in
 * platforms/posix/src/px4/windows/runtime/init.cpp. The calibration runs
 * immediately after timeBeginPeriod(1), before any module thread starts
 * calling usleep(), so the first usleep() the process performs already
 * sees the tuned value.
 *
 * Override at process startup with the PX4_USLEEP_SPIN_US environment
 * variable (clamped to [0, 50000] microseconds). Values <= 50000 are
 * accepted; 0 effectively forces every wait > 0 us through the timer +
 * spin-tail path.
 *
 * @c g_usleep_spin_tail_us is the *upper bound* of the QPC spin closing
 * the residual after the high-resolution waitable timer wakes. The
 * adaptive controller in usleep() shrinks the tail per-thread toward
 * the observed timer overshoot via an EWMA, so a quiet host pays only
 * ~p95 jitter of CPU spin per call instead of the worst-case bound.
 */
extern long g_usleep_pure_spin_us;
extern long g_usleep_spin_tail_us;
extern int g_usleep_use_wine_deadline_clock;

/**
 * @brief Wine-only monotonic deadline clock supplied by the Windows runtime.
 *
 * Native Windows keeps the QPC-based path below. Wine enables this hook during
 * runtime init so short sleeps do not call back through Wine's QPC emulation on
 * every spin iteration.
 */
uint64_t px4_windows_wine_monotonic_time_us(void);
void px4_windows_wine_spin_until_us(uint64_t deadline_us);

/* Floor for the per-thread adaptive spin tail. Initialised by the
 * calibration routine to the host-measured P95 waitable-timer jitter so
 * the controller never trims the spin below the value we already know
 * is needed to cover this host's observed long-tail wakes. Defaults to
 * a conservative 700 us when calibration cannot run. */
extern long g_usleep_adaptive_min_tail_us;

/**
 * @brief Sleep for at least @p usec microseconds with microsecond accuracy.
 *
 * Windows Sleep() is quantized to the system timer tick (~15.6 ms by
 * default; 1 ms after timeBeginPeriod(1) in init.cpp). A 4 ms sleep
 * therefore rounds up to a full HPET tick, throttling SITL sim time.
 *
 * The naive Sleep() path loses ~10 % of wall time. A pure HPET-backed
 * waitable timer wakes within 0.3 - 0.7 ms of the target on a quiet
 * system but quantizes to 1 ms under load, so a tight SITL producer
 * (250 Hz - 1 kHz lockstep loop) accumulates 5 - 10 % drift.
 *
 * The current implementation is a spin-residual hybrid:
 *
 *   - Requests <= @c g_usleep_pure_spin_us are held entirely on the QPC
 *     deadline. This covers SIH's normal lockstep wall-sleep cadence
 *     (200 Hz - 2 kHz, 500 - 5000 us). Even a single 0.5 - 1 ms
 *     scheduler-late wake in that band becomes visible as sim/wall
 *     drift, so the short simulation waits pay CPU for determinism.
 *
 *   - For requests > @c g_usleep_pure_spin_us the bulk of the wait runs
 *     on a high-resolution waitable timer (CreateWaitableTimerExW +
 *     CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, Windows 10 1803+). The
 *     timer is armed to wake @c g_usleep_spin_tail_us microseconds
 *     early and the residual is closed by a QueryPerformanceCounter
 *     busy-loop. This trades ~g_usleep_spin_tail_us of CPU per call for
 *     microsecond-scale accuracy against the absolute QPC target.
 *
 * The HANDLE is cached per-thread in compiler-native TLS so we pay one
 * CreateWaitableTimerExW per thread for the lifetime of the process.
 */
static inline int usleep(useconds_t usec)
{
	if (usec == 0) {
		return 0;
	}

	// Snapshot the tuned thresholds once per call. Reads of an unaligned
	// long are atomic on x86_64; the calibration in init.cpp runs before
	// any other thread starts, so no further synchronization is needed.
	const long pure_spin_us = g_usleep_pure_spin_us;
	const long spin_tail_us = g_usleep_spin_tail_us;
	const long adaptive_floor_us = g_usleep_adaptive_min_tail_us;

	if (g_usleep_use_wine_deadline_clock != 0) {
		const uint64_t qpc_target = px4_windows_wine_monotonic_time_us() + (uint64_t)usec;

		if ((long)usec > pure_spin_us) {
			static PX4_WINDOWS_SLEEP_TLS HANDLE timer = NULL;

			if (timer == NULL) {
				timer = CreateWaitableTimerExW(NULL, NULL,
							       CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
							       | CREATE_WAITABLE_TIMER_MANUAL_RESET,
							       TIMER_ALL_ACCESS);

				if (timer == NULL) {
					timer = CreateWaitableTimerW(NULL, TRUE, NULL);
				}
			}

			if (timer != NULL) {
				long tail_us = spin_tail_us;

				if (tail_us < 0) { tail_us = 0; }

				if ((useconds_t)tail_us > usec) { tail_us = (long)usec; }

				LARGE_INTEGER due;
				const LONGLONG bulk_us = (LONGLONG)usec - (LONGLONG)tail_us;
				due.QuadPart = -(bulk_us * 10);

				if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
					const DWORD wait_ms_bulk = (DWORD)((bulk_us + 999LL) / 1000LL);
					const DWORD wait_ms = wait_ms_bulk + 5U;
					WaitForSingleObject(timer, wait_ms);
				}

			} else {
				Sleep((DWORD)(((LONGLONG)usec + 999LL) / 1000LL));
			}
		}

		px4_windows_wine_spin_until_us(qpc_target);

		return 0;
	}

	LARGE_INTEGER qpc_freq;
	LARGE_INTEGER qpc_start;
	QueryPerformanceFrequency(&qpc_freq);
	QueryPerformanceCounter(&qpc_start);

	// Absolute QPC target = start + usec. The conversion uses 64-bit
	// integer math throughout: at 10 MHz QPC and a 1-second sleep the
	// product is 1e7, well within LONGLONG range.
	const LONGLONG qpc_target = qpc_start.QuadPart
				    + ((LONGLONG)usec * qpc_freq.QuadPart) / 1000000LL;

	if ((long)usec > pure_spin_us) {
		// Use compiler-native TLS instead of C++ thread_local because this
		// header is also included from .c translation units.
		static PX4_WINDOWS_SLEEP_TLS HANDLE timer = NULL;
		// Per-thread adaptive spin-tail state. We track the timer wake
		// overshoot (how late WaitForSingleObject returned past the
		// requested bulk deadline) as an EWMA in microseconds, then size
		// the spin tail at (overshoot_ewma + small_margin), bounded by
		// [PX4_USLEEP_ADAPTIVE_MIN_TAIL_US, spin_tail_us].
		// The EWMA is initialized with sentinel -1 so the first call
		// uses the configured upper-bound tail; subsequent calls
		// converge toward the host's actual jitter and trim the spin.
		static PX4_WINDOWS_SLEEP_TLS long adaptive_tail_us = -1;
		static PX4_WINDOWS_SLEEP_TLS long overshoot_ewma_us = -1;

		if (timer == NULL) {
			timer = CreateWaitableTimerExW(NULL, NULL,
						       CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
						       | CREATE_WAITABLE_TIMER_MANUAL_RESET,
						       TIMER_ALL_ACCESS);

			if (timer == NULL) {
				// Older Windows: legacy manual-reset timer
				// still honors timeBeginPeriod(1).
				timer = CreateWaitableTimerW(NULL, TRUE, NULL);
			}
		}

		// Decide the spin tail for this call. First call (sentinel) -
		// fall back to the configured upper bound so we definitely
		// cover the deadline while we collect data. Subsequent calls
		// use the EWMA-derived value.
		long tail_us = (adaptive_tail_us < 0) ? spin_tail_us : adaptive_tail_us;

		// Floor at the host-measured P95 jitter (set by the calibration
		// routine in init.cpp). Trimming below this would force the QPC
		// spin to absorb wakes past the deadline, which directly bleeds
		// into sim/wall ratio.
		if (tail_us < adaptive_floor_us) { tail_us = adaptive_floor_us; }

		if (tail_us > spin_tail_us) { tail_us = spin_tail_us; }

		if (timer != NULL) {
			LARGE_INTEGER due;
			// Wake tail_us early and close the gap by spin.
			// Negative due time = relative interval, 100 ns units.
			// Clamp the bulk wait to >= 0 in case the caller asked
			// for a value just above pure_spin_us with a larger
			// spin_tail_us; the QPC spin still enforces the deadline.
			const LONGLONG bulk_us = (LONGLONG)usec - (LONGLONG)tail_us;
			const LONGLONG bulk_us_clamped = bulk_us > 0 ? bulk_us : 0;
			due.QuadPart = -(bulk_us_clamped * 10);

			if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
				// Use a millisecond timeout slightly longer than
				// the requested sleep rather than INFINITE: a
				// rare WaitForSingleObject misbehavior on Windows
				// (observed under heavy SITL lockstep load) can
				// otherwise hang the producer thread permanently.
				// The QPC spin below still enforces the absolute
				// deadline, so a premature wake is harmless.
				const DWORD wait_ms_bulk = (DWORD)((bulk_us_clamped + 999LL) / 1000LL);
				const DWORD wait_ms = wait_ms_bulk + 5U;   // +5 ms safety margin
				WaitForSingleObject(timer, wait_ms);

				// Adaptive update: measure how late we woke vs the
				// requested bulk deadline (qpc_target - tail_us).
				// Negative = woke early (good); positive = woke late
				// and the spin tail had to absorb it. We track an
				// upper-envelope EWMA: a late wake snaps the value up
				// immediately, a stretch of clean wakes decays it down
				// at 1/64 per call (~30 ms settle at 250 Hz). The plain
				// mean would undersize the tail because the timer jitter
				// distribution has a long upper tail and 5 % of waits
				// can wake far past the mean - each such miss bleeds
				// 100 - 1000 us into wall time and accumulates as
				// sim/wall ratio drift.
				LARGE_INTEGER wake_now;
				QueryPerformanceCounter(&wake_now);
				const LONGLONG bulk_target_qpc = qpc_target
								 - ((LONGLONG)tail_us * qpc_freq.QuadPart) / 1000000LL;
				LONGLONG overshoot_qpc = wake_now.QuadPart - bulk_target_qpc;

				if (overshoot_qpc < 0) { overshoot_qpc = 0; }

				const long overshoot_us =
					(long)((overshoot_qpc * 1000000LL) / qpc_freq.QuadPart);

				if (overshoot_ewma_us < 0) {
					// First sample: seed at the configured upper
					// bound so we don't undershoot before any
					// data has been gathered.
					overshoot_ewma_us = spin_tail_us;
				}

				// Fast attack, slow decay.
				if (overshoot_us > overshoot_ewma_us) {
					overshoot_ewma_us = overshoot_us;

				} else {
					overshoot_ewma_us =
						(overshoot_ewma_us * 63 + overshoot_us + 32) / 64;
				}

				// Size the next call's tail at envelope + 200 us
				// margin. The margin covers the residual gap between
				// the slow-decay envelope and the instantaneous
				// worst-case wake jitter; the floor and upper-bound
				// clamps keep the controller from collapsing or
				// running away.
				adaptive_tail_us = overshoot_ewma_us + 200;

				if (adaptive_tail_us < adaptive_floor_us) {
					adaptive_tail_us = adaptive_floor_us;
				}

				if (adaptive_tail_us > spin_tail_us) {
					adaptive_tail_us = spin_tail_us;
				}

			} else {
				// Arming failed (very rare). Fall through to
				// the QPC spin below; it will still hit the
				// deadline, just with a brief CPU burn.
			}
		} else {
			// No timer available at all: kernel Sleep() rounded
			// up to the nearest millisecond. The spin tail below
			// still corrects the residual.
			const LONGLONG bulk_us = (LONGLONG)usec - (LONGLONG)tail_us;
			const LONGLONG bulk_us_clamped = bulk_us > 0 ? bulk_us : 0;
			Sleep((DWORD)((bulk_us_clamped + 999LL) / 1000LL));
		}
	}

	// Close the residual against the absolute QPC target. For longer waits
	// this is at most ~spin_tail_us of spin (often less because the waitable
	// timer wakes slightly late). For SIH-sized waits it is the full request.
	LARGE_INTEGER now;

	do {
		YieldProcessor();
		QueryPerformanceCounter(&now);
	} while (now.QuadPart < qpc_target);

	return 0;
}

#undef PX4_WINDOWS_SLEEP_TLS

#if defined(_MSC_VER)
/** @brief Sleep for at least @p seconds seconds using Windows Sleep(). */
static inline unsigned int sleep(unsigned int seconds)
{
	Sleep(seconds * 1000U);
	return 0;
}
#endif
#endif

/* POSIX pipe(fd[2]) - default to 64 KiB buffer and binary mode. */
#ifndef _PX4_PIPE_SHIM_DEFINED
#define _PX4_PIPE_SHIM_DEFINED
/** @brief Create a binary CRT pipe with a 64 KiB buffer. */
static inline int pipe(int fds[2]) { return _pipe(fds, 65536, 0x8000 /* _O_BINARY */); }

/**
 * @brief pipe2() compatibility wrapper.
 *
 * Windows CRT pipes do not expose POSIX pipe2 flags; the shim accepts the
 * argument and creates the same binary pipe as pipe().
 */
static inline int pipe2(int fds[2], int flags)
{
	(void)flags;
	return pipe(fds);
}
#endif

/* POSIX fsync(fd) - forwards to _commit (flushes the CRT fd's buffers
 * down to the underlying HANDLE and flushes the HANDLE to disk). */
#ifndef _PX4_FSYNC_SHIM_DEFINED
#define _PX4_FSYNC_SHIM_DEFINED
static inline int fsync(int fd) { return _commit(fd); }
#endif

/* fdatasync: Windows has no separate metadata/data flush. Treat it as
 * equivalent to fsync (POSIX allows this; it's just stricter). */
#ifndef _PX4_FDATASYNC_SHIM_DEFINED
#define _PX4_FDATASYNC_SHIM_DEFINED
static inline int fdatasync(int fd) { return _commit(fd); }
#endif

/* POSIX sysconf - implemented in posix_shim.cpp so we can use
 * <windows.h>'s GetSystemInfo / GlobalMemoryStatusEx without forcing
 * every translation unit to drag in <windows.h>. */
#ifndef _PX4_SYSCONF_SHIM_DEFINED
#define _PX4_SYSCONF_SHIM_DEFINED
/**
 * @brief Query host limits for the POSIX selectors PX4 uses.
 *
 * Implemented in the Windows backend so page size, processor count, and memory
 * values come from GetSystemInfo/GlobalMemoryStatusEx.
 */
long sysconf(int name);
#endif

/* POSIX symlink(target, linkpath) - forwards to CreateSymbolicLinkA
 * (requires the SE_CREATE_SYMBOLIC_LINK_NAME privilege or Windows 10
 * developer-mode). Returns -1 with errno = EPERM if the user lacks
 * the privilege. */
#ifndef _PX4_SYMLINK_SHIM_DEFINED
#define _PX4_SYMLINK_SHIM_DEFINED
/**
 * @brief Create a filesystem symbolic link.
 *
 * @return 0 on success, -1 with errno set. EPERM indicates missing Windows
 *         symlink privilege or disabled developer mode.
 */
int symlink(const char *target, const char *linkpath);
#endif

/* POSIX readlink - Windows has no O(1) path-to-reparse-target readout;
 * fall through to DeviceIoControl on the reparse point. The base SITL
 * build doesn't follow symlinks, so the shim lives in posix_shim.cpp. */
#ifndef _PX4_READLINK_SHIM_DEFINED
#define _PX4_READLINK_SHIM_DEFINED
/**
 * @brief Read the target of a filesystem reparse-point symlink.
 *
 * @return Number of bytes copied into @p buf, or -1 with errno set.
 */
ssize_t readlink(const char *path, char *buf, size_t bufsiz);
#endif

/* POSIX truncate(path, length) - forwards to CreateFileA + SetEndOfFile
 * in posix_shim.cpp. ftruncate on a CRT fd uses _chsize_s. */
#ifndef _PX4_TRUNCATE_SHIM_DEFINED
#define _PX4_TRUNCATE_SHIM_DEFINED
/** @brief Truncate a file by path using CreateFileA/SetEndOfFile. */
int truncate(const char *path, off_t length);
#endif

/* POSIX link(oldpath, newpath) - maps onto CreateHardLinkA via
 * posix_shim.cpp. */
#ifndef _PX4_LINK_SHIM_DEFINED
#define _PX4_LINK_SHIM_DEFINED
/** @brief Create a hard link using CreateHardLinkA. */
int link(const char *existing_path, const char *new_path);
#endif

/* POSIX realpath fallback - CRT _fullpath returns the canonical path. */
#ifndef _PX4_REALPATH_SHIM_DEFINED
#define _PX4_REALPATH_SHIM_DEFINED
#include <stdlib.h>
/** @brief Resolve a path to an absolute CRT path using _fullpath(). */
static inline char *realpath(const char *path, char *resolved_path)
{
	return _fullpath(resolved_path, path, resolved_path ? 260 : 0);
}
#endif

/* POSIX dprintf / vdprintf - forward to _write on the CRT fd. */
#ifndef _PX4_DPRINTF_SHIM_DEFINED
#define _PX4_DPRINTF_SHIM_DEFINED
/** @brief Formatted write to a CRT file descriptor. */
static inline int vdprintf(int fd, const char *fmt, va_list ap)
{
	char buf[1024];
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);

	if (n <= 0) { return n; }

	if ((size_t)n >= sizeof(buf)) { n = (int)sizeof(buf) - 1; }

	return (int)_write(fd, buf, (unsigned)n);
}

/** @brief Variadic formatted write to a CRT file descriptor. */
static inline int dprintf(int fd, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	int n = vdprintf(fd, fmt, ap);
	va_end(ap);
	return n;
}
#endif

#ifndef TEMP_FAILURE_RETRY
#define TEMP_FAILURE_RETRY(expression) (expression)
#endif

/* POSIX getpagesize - Windows has GetSystemInfo.dwPageSize (typically
 * 4096 on x86/x64, 16384 on ARM64). */
#ifndef _PX4_GETPAGESIZE_SHIM_DEFINED
#define _PX4_GETPAGESIZE_SHIM_DEFINED
/** @brief Return the host memory page size in bytes. */
int getpagesize(void);
#endif

/* POSIX/BSD process and environment helpers that MinGW does not
 * declare. These are implemented in posix_shim.cpp with Windows-backed
 * compatibility semantics suitable for PX4 and embedded host apps. */
#ifndef _PX4_GETPPID_SHIM_DEFINED
#define _PX4_GETPPID_SHIM_DEFINED
/** @brief Return the parent process id when Windows can discover it. */
pid_t getppid(void);
#endif

#ifndef _PX4_SETSID_SHIM_DEFINED
#define _PX4_SETSID_SHIM_DEFINED
/** @brief Create a best-effort process session; returns the current pid. */
pid_t setsid(void);
#endif

#ifndef _PX4_GETSID_SHIM_DEFINED
#define _PX4_GETSID_SHIM_DEFINED
/** @brief Return the best-effort session id for @p pid. */
pid_t getsid(pid_t pid);
#endif

#ifndef _PX4_DAEMON_SHIM_DEFINED
#define _PX4_DAEMON_SHIM_DEFINED
/**
 * @brief daemon() compatibility shim.
 *
 * Windows cannot fork and detach in the POSIX sense; the implementation applies
 * the requested cwd/stdio behavior where possible and otherwise reports
 * success for callers that only need a background-compatible code path.
 */
int daemon(int nochdir, int noclose);
#endif

#ifndef _PX4_SETENV_SHIM_DEFINED
#define _PX4_SETENV_SHIM_DEFINED
/** @brief Set an environment variable using the Windows CRT environment. */
int setenv(const char *name, const char *value, int overwrite);
#endif

#ifndef _PX4_UNSETENV_SHIM_DEFINED
#define _PX4_UNSETENV_SHIM_DEFINED
/** @brief Remove an environment variable using the Windows CRT environment. */
int unsetenv(const char *name);
#endif

#ifdef __cplusplus
}
#endif
