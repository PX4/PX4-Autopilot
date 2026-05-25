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
 * @file init.cpp
 *
 * One-time process init for the Windows POSIX-shim subsystem.
 *
 * Owns WSAStartup/WSACleanup, binary-mode stdio, Console Output
 * Code Page selection, native Windows ANSI virtual-terminal processing,
 * and the Wine /dev/tty termios restore path invoked by Ctrl+C handling
 * in main.cpp.
 */

#include "px4_windows_internal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#if defined(_MSC_VER)
#include <intrin.h>
#elif defined(__i386__) || defined(__x86_64__)
#include <x86intrin.h>
#endif

// timeBeginPeriod / timeEndPeriod live in winmm. Without raising the
// system timer resolution, the Windows scheduler quantizes Sleep() to
// the default ~15.6 ms HPET tick, which throttles SITL sim time to
// ~40 % of wall time. Requesting 1 ms resolution drops the floor to
// the documented minimum.
// Older MinGW-w64 headers (<= 8.0) ship the prototypes via <mmsystem.h>
// instead of the split <timeapi.h> introduced in newer SDKs.
#if defined(__MINGW32__) && (__MINGW64_VERSION_MAJOR < 9)
#include <mmsystem.h>
#else
#include <timeapi.h>
#endif

// CREATE_WAITABLE_TIMER_HIGH_RESOLUTION (Windows 10 1803+, build 17134)
// may not be defined in older SDK headers. Mirror the literal values
// documented by Microsoft - same fallback as windows_shim/unistd.h.
#ifndef CREATE_WAITABLE_TIMER_MANUAL_RESET
#define CREATE_WAITABLE_TIMER_MANUAL_RESET 0x00000001
#endif
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

#if defined(_MSC_VER)
// MSVC CRT debug heap: dumps unfreed allocations to stderr.
// We invoke it explicitly from px4_windows_exit() because the daemon
// shuts down via ExitProcess(), which bypasses _CRTDBG_LEAK_CHECK_DF.
#include <crtdbg.h>
#endif

/* --------------------------------------------------------------------------
 * One-time process-wide initialisation.
 *
 * WinSock requires WSAStartup before any socket call, and we want a
 * UTF-8 console so PX4_INFO banner output isn't mangled. Install a
 * global constructor that runs before main().
 * -------------------------------------------------------------------------- */
/* Session id claimed by the first setsid() caller - shared process-wide
 * via InterlockedCompareExchange from proc/ids.cpp. External linkage so
 * the extern declaration in proc/ids.cpp resolves. */
volatile LONG g_px4_session_id = 0;

/* Runtime-tuned thresholds consumed by the inline usleep() shim in
 * platforms/posix/include/windows_shim/unistd.h. The defaults are sized
 * to give a high-resolution-timer-equipped host (Windows 10 1803+) a
 * safe starting point: 5 ms pure-spin ceiling and 1 ms spin-tail. The
 * tail is the *upper bound* the thread-local adaptive controller in
 * usleep() may expand to; it shrinks toward the observed timer overshoot
 * via an EWMA so a quiet host pays only ~p95-jitter of CPU spin per
 * call. They live at file scope because every translation unit that
 * includes <unistd.h> on Windows references them through the inline body
 * of usleep(). */
/* GCC's -Werror trips on `extern <type> name = init;` because an
 * initializer turns the line into a definition, making `extern`
 * redundant. The matching `extern` declarations live in
 * platforms/posix/include/windows_shim/unistd.h; here we only need
 * definitions with external linkage (the default for non-static globals
 * at namespace scope). Wrapping in `extern "C"` keeps the C linkage that
 * the inline shim relies on. */
extern "C" {
	long g_usleep_pure_spin_us = 5000;
	long g_usleep_spin_tail_us = 1000;
	int g_usleep_use_wine_deadline_clock = 0;
}
/* Floor for the per-thread adaptive spin tail. Initialised by
 * px4_windows_calibrate_usleep_threshold() to the host-measured P95
 * waitable-timer jitter so the controller never collapses below the
 * value we already know is needed to cover the observed long-tail wakes.
 * Defaults to a conservative 700 us when calibration cannot run (e.g.
 * pre-1803 Windows with no high-resolution timer). */
extern "C" {
	long g_usleep_adaptive_min_tail_us = 700;
}

extern "C" int px4_windows_running_under_wine(void)
{
	static const int running_under_wine = []() -> int {
		if (HMODULE ntdll = GetModuleHandleW(L"ntdll.dll"))
		{
			return GetProcAddress(ntdll, "wine_get_version") != nullptr ? 1 : 0;
		}

		return 0;
	}();
	return running_under_wine;
}

namespace
{

uint64_t px4_windows_read_cpu_cycles()
{
#if defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
	return __rdtsc();
#elif defined(__i386__) || defined(__x86_64__)
	return __rdtsc();
#else
	return 0;
#endif
}

uint64_t px4_windows_qpc_time_us()
{
	static const LARGE_INTEGER freq = []() {
		LARGE_INTEGER f {};
		QueryPerformanceFrequency(&f);
		return f;
	}();

	LARGE_INTEGER now {};
	QueryPerformanceCounter(&now);
	return static_cast<uint64_t>((static_cast<long double>(now.QuadPart) * 1000000.0L)
				     / static_cast<long double>(freq.QuadPart));
}

struct WineDeadlineClock {
	bool cycle_clock_valid{false};
	uint64_t qpc_base_us{0};
	uint64_t cycle_base{0};
	long double cycles_per_us{0.};
	mutable std::atomic<uint64_t> last_us{0};

	WineDeadlineClock()
	{
		LARGE_INTEGER freq {};
		LARGE_INTEGER qpc0 {};
		LARGE_INTEGER qpc1 {};
		QueryPerformanceFrequency(&freq);
		QueryPerformanceCounter(&qpc0);
		const uint64_t cycles0 = px4_windows_read_cpu_cycles();

		if (cycles0 == 0) {
			qpc_base_us = px4_windows_qpc_time_us();
			last_us.store(qpc_base_us, std::memory_order_relaxed);
			return;
		}

		// One-time calibration only. Tight Wine spin loops use this
		// cycle counter afterwards so they do not call back through
		// Wine's QPC/gettimeofday path on every iteration.
		Sleep(20);
		const uint64_t cycles1 = px4_windows_read_cpu_cycles();
		QueryPerformanceCounter(&qpc1);

		const LONGLONG qpc_delta = qpc1.QuadPart - qpc0.QuadPart;
		const uint64_t cycles_delta = cycles1 - cycles0;

		if (qpc_delta > 0 && cycles_delta > 0) {
			const long double elapsed_us =
				(static_cast<long double>(qpc_delta) * 1000000.0L) / static_cast<long double>(freq.QuadPart);
			cycles_per_us = static_cast<long double>(cycles_delta) / elapsed_us;
			cycle_base = cycles1;
			qpc_base_us = static_cast<uint64_t>((static_cast<long double>(qpc1.QuadPart) * 1000000.0L)
							    / static_cast<long double>(freq.QuadPart));
			cycle_clock_valid = cycles_per_us > 0.;
			last_us.store(qpc_base_us, std::memory_order_relaxed);

		} else {
			qpc_base_us = px4_windows_qpc_time_us();
			last_us.store(qpc_base_us, std::memory_order_relaxed);
		}
	}

	uint64_t now_us() const
	{
		uint64_t raw_us = 0;

		if (!cycle_clock_valid) {
			raw_us = px4_windows_qpc_time_us();

		} else {
			const uint64_t cycles = px4_windows_read_cpu_cycles();
			const long double delta_us = static_cast<long double>(cycles - cycle_base) / cycles_per_us;
			raw_us = qpc_base_us + static_cast<uint64_t>(delta_us);
		}

		uint64_t observed_us = last_us.load(std::memory_order_relaxed);

		while (raw_us > observed_us &&
		       !last_us.compare_exchange_weak(observed_us, raw_us, std::memory_order_relaxed)) {
		}

		return raw_us > observed_us ? raw_us : observed_us;
	}

	void spin_until_us(uint64_t deadline_us) const
	{
		if (!cycle_clock_valid || deadline_us <= qpc_base_us) {
			while (now_us() < deadline_us) {
				YieldProcessor();
			}

			return;
		}

		const long double delta_us = static_cast<long double>(deadline_us - qpc_base_us);
		const uint64_t deadline_cycles = cycle_base + static_cast<uint64_t>(delta_us * cycles_per_us);

		while (px4_windows_read_cpu_cycles() < deadline_cycles) {
			YieldProcessor();
		}

		(void)now_us();
	}
};

const WineDeadlineClock &wine_deadline_clock()
{
	static const WineDeadlineClock clock;
	return clock;
}

} // namespace

extern "C" uint64_t px4_windows_wine_monotonic_time_us(void)
{
	return wine_deadline_clock().now_us();
}

extern "C" void px4_windows_wine_spin_until_us(uint64_t deadline_us)
{
	wine_deadline_clock().spin_until_us(deadline_us);
}

/**
 * @brief Auto-tune g_usleep_pure_spin_us against the host's measured
 *        high-resolution waitable-timer jitter and apply an optional
 *        environment override.
 *
 * Must be invoked exactly once and BEFORE any thread starts calling
 * usleep(). The constructor of PX4WindowsGlobalInit calls it directly
 * after timeBeginPeriod(1) - the earliest hookable point in the PX4
 * Windows startup sequence.
 *
 * Honors PX4_USLEEP_SPIN_US (microseconds, clamped to [0, 50000]). When
 * unset, probes the *exact* primitive the inline usleep() shim uses for
 * the bulk wait: a CREATE_WAITABLE_TIMER_HIGH_RESOLUTION waitable timer
 * armed for 1 ms via SetWaitableTimer + WaitForSingleObject. The chosen
 * threshold is g_usleep_spin_tail_us + p95_jitter + 500 us margin, so
 * any wait above the threshold can be served by (timer + spin tail) and
 * still hit the absolute QPC deadline. Floored to 500 us and capped at
 * 5000 us.
 *
 * The previous heuristic measured Sleep(1) jitter, but Sleep is not on
 * the hot path - usleep() uses the high-resolution waitable timer, which
 * is far more accurate than Sleep on Win10 1803+. Probing the wrong
 * primitive made the auto-tune saturate at 5000 us on quiet hosts, which
 * forced every SIH 4 ms tick into pure-spin and pegged one full core.
 */
static void px4_windows_calibrate_usleep_threshold()
{
	if (px4_windows_running_under_wine()) {
		g_usleep_pure_spin_us = 1000;
		g_usleep_spin_tail_us = 1000;
		g_usleep_adaptive_min_tail_us = 0;
		g_usleep_use_wine_deadline_clock = 1;
		std::printf("INFO  [px4_windows] usleep deadline clock (wine): "
			    "cycle-counter spin, pure spin <= %ld us, timer tail %ld us\n",
			    g_usleep_pure_spin_us, g_usleep_spin_tail_us);
		std::fflush(stdout);
		return;
	}

	// 1. Honor an explicit env override first; most users / CI runs set
	// this from the launcher script, so skip the probe entirely when present.
	if (const char *env = std::getenv("PX4_USLEEP_SPIN_US")) {
		char *end = nullptr;
		long v = std::strtol(env, &end, 10);

		if (end != env && v >= 0 && v <= 50000) {
			g_usleep_pure_spin_us = v;
			// Env override skips probing the host so we
			// have no measured P95 - keep the conservative
			// upper-bound default for the adaptive floor.
			g_usleep_adaptive_min_tail_us = g_usleep_spin_tail_us;
			std::printf("INFO  [px4_windows] usleep spin threshold (env): %ld us "
				    "(adaptive tail floor: %ld us)\n",
				    v, g_usleep_adaptive_min_tail_us);
			std::fflush(stdout);
			return;
		}

		std::printf("WARN  [px4_windows] PX4_USLEEP_SPIN_US=\"%s\" out of range [0, 50000], ignored\n",
			    env);
		std::fflush(stdout);
	}

	// 2. Probe the actual primitive usleep() uses for the bulk wait: a
	// CREATE_WAITABLE_TIMER_HIGH_RESOLUTION waitable timer armed via
	// SetWaitableTimer + WaitForSingleObject. On Win10 1803+ this gives
	// sub-millisecond accuracy; the residual is closed by the QPC spin
	// tail (g_usleep_spin_tail_us). The threshold we want is the smallest
	// value such that (waitable_timer + spin_tail) reliably hits the
	// deadline.
	HANDLE timer = CreateWaitableTimerExW(NULL, NULL,
					      CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
					      | CREATE_WAITABLE_TIMER_MANUAL_RESET,
					      TIMER_ALL_ACCESS);

	if (timer == NULL) {
		// Older Windows (pre-1803) lacks the high-res flag. Keep the
		// historical 5000 us default - on those hosts the legacy timer
		// quantizes to ~1 ms tick and the wide spin band is the safest
		// behavior available.
		g_usleep_pure_spin_us = 5000;
		// Without a high-res timer the legacy 1 ms tick dominates;
		// lock the adaptive floor to spin_tail_us so the controller
		// can't shrink the spin below the safe bound on this host.
		g_usleep_adaptive_min_tail_us = g_usleep_spin_tail_us;
		std::printf("INFO  [px4_windows] usleep spin threshold (auto): 5000 us "
			    "(high-res waitable timer unavailable, using legacy default)\n");
		std::fflush(stdout);
		return;
	}

	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);

	// N=500 keeps ~25 samples in the p95 tail (vs 5 at N=100), which removes
	// the intermittent low-p95 outlier that under-provisioned the spin tail
	// and tripped the sim/wall ratio below 0.99 once every few cold boots.
	// Probe cost is ~500 ms of one-time startup time (each iteration waits 1
	// ms on the high-res timer); negligible vs the robustness gain.
	constexpr int N = 500;
	long jitter_us[N];

	for (int i = 0; i < N; ++i) {
		LARGE_INTEGER t0;
		LARGE_INTEGER t1;
		LARGE_INTEGER due;
		// Ask for 1 ms (10 000 x 100 ns units, negative = relative).
		due.QuadPart = -10000;
		QueryPerformanceCounter(&t0);

		if (SetWaitableTimer(timer, &due, 0, NULL, NULL, FALSE)) {
			WaitForSingleObject(timer, INFINITE);
		}

		QueryPerformanceCounter(&t1);
		const long actual_us = (long)(((t1.QuadPart - t0.QuadPart) * 1000000LL) / freq.QuadPart);
		long delta = actual_us - 1000;

		if (delta < 0) { delta = 0; }

		jitter_us[i] = delta;
	}

	CloseHandle(timer);
	std::sort(jitter_us, jitter_us + N);
	const long p95 = jitter_us[(int)(0.95 * N)];

	// 3. Choose threshold = spin_tail + p95_jitter + 500 us margin. Any
	// wait above this is served by (waitable timer wakes ~p95 us late
	// at most + spin tail closes the residual). Floor at 500 us so the
	// short-sleep band never collapses (avoids degenerate 0-cost loops);
	// cap at 5000 us so a freakishly noisy host still falls back to the
	// legacy behavior.
	long chosen = g_usleep_spin_tail_us + p95 + 500;

	if (chosen < 500) { chosen = 500; }

	if (chosen > 5000) { chosen = 5000; }

	g_usleep_pure_spin_us = chosen;
	// Right-size the spin-tail upper bound to (P95 + 500 us). This is
	// the largest value the per-thread adaptive controller in usleep()
	// is allowed to grow to, so we trade a couple hundred microseconds
	// of CPU spin per call for a robust deadline guarantee. The +500
	// (was +300) absorbs the residual P95 underestimate even when the
	// N=500 probe still under-samples a freakishly quiet host. Floored
	// at 700 us (so quiet hosts still cover the typical Win10 1803+
	// jitter floor) and capped at 2000 us (the historical safe value).
	long sized_tail = p95 + 500;

	if (sized_tail < 700) { sized_tail = 700; }

	if (sized_tail > 2000) { sized_tail = 2000; }

	g_usleep_spin_tail_us = sized_tail;
	// Set the adaptive tail floor to the host-measured P95 jitter
	// (clamped to [200, sized_tail]) so the per-thread controller in
	// usleep() can never trim the spin below the value we already know
	// is needed to cover this host's observed long-tail wakes.
	long adaptive_floor = p95;

	if (adaptive_floor < 200) { adaptive_floor = 200; }

	if (adaptive_floor > sized_tail) { adaptive_floor = sized_tail; }

	g_usleep_adaptive_min_tail_us = adaptive_floor;
	std::printf("INFO  [px4_windows] usleep spin threshold (auto): %ld us "
		    "(p95 high-res timer jitter: %ld us [N=%d], spin tail: %ld us, "
		    "adaptive tail floor: %ld us)\n",
		    chosen, p95, N, sized_tail, adaptive_floor);
	std::fflush(stdout);
}

namespace
{

struct PX4WindowsGlobalInit {
	struct SavedConsoleMode {
		HANDLE handle = nullptr;
		DWORD  mode   = 0;
		bool   valid  = false;
	};

	struct LinuxTermios {
		unsigned int c_iflag;
		unsigned int c_oflag;
		unsigned int c_cflag;
		unsigned int c_lflag;
		unsigned char c_line;
		unsigned char c_cc[19];
	};

	SavedConsoleMode saved_stdin;
	SavedConsoleMode saved_stdout;
	SavedConsoleMode saved_stderr;
	LinuxTermios saved_host_tty {};
	bool saved_host_tty_valid = false;
	bool saved_host_tty_was_cooked = false;
	const bool running_under_wine = px4_windows_running_under_wine() != 0;

	static constexpr long long LX_SYS_open   = 2;
	static constexpr long long LX_SYS_close  = 3;
	static constexpr long long LX_SYS_ioctl  = 16;
	static constexpr long long LX_O_RDWR     = 2;
	static constexpr long long LX_O_NOCTTY   = 0x100;
	static constexpr long long LX_TCGETS     = 0x5401;
	static constexpr long long LX_TCSETS     = 0x5402;
	// Input flags
	static constexpr unsigned int LX_BRKINT = 0x0002, LX_IGNPAR = 0x0004,
				      LX_ICRNL  = 0x0100, LX_IXON   = 0x0400;
	// Output flags
	static constexpr unsigned int LX_OPOST  = 0x0001, LX_ONLCR  = 0x0004;
	// Control flags
	static constexpr unsigned int LX_CSIZE  = 0x0030, LX_CS8    = 0x0030,
				      LX_CREAD  = 0x0080;
	// Local flags
	static constexpr unsigned int LX_ISIG    = 0x0001, LX_ICANON  = 0x0002,
				      LX_ECHO    = 0x0008, LX_ECHOE   = 0x0010,
				      LX_ECHOK   = 0x0020, LX_ECHOCTL = 0x0200,
				      LX_ECHOKE  = 0x0800, LX_IEXTEN  = 0x8000;

	DWORD cooked_stdin_mode() const
	{
		DWORD mode = ENABLE_PROCESSED_INPUT | ENABLE_LINE_INPUT | ENABLE_ECHO_INPUT;

#ifdef ENABLE_EXTENDED_FLAGS
		mode |= ENABLE_EXTENDED_FLAGS;
#endif
#ifdef ENABLE_INSERT_MODE
		mode |= ENABLE_INSERT_MODE;
#endif
#ifdef ENABLE_QUICK_EDIT_MODE
		mode |= ENABLE_QUICK_EDIT_MODE;
#endif

		return mode;
	}

	// Inline Linux syscall helpers (x86_64 ABI).
	static long long linux_syscall1(long long num, long long a)
	{
#if defined(__GNUC__) || defined(__clang__)
		long long ret;
		__asm__ volatile(
			"syscall"
			: "=a"(ret)
			: "0"(num), "D"(a)
			: "rcx", "r11", "memory"
		);
		return ret;
#else
		(void)num;
		(void)a;
		return -1;
#endif
	}

	static long long linux_syscall3(long long num, long long a, long long b, long long c)
	{
#if defined(__GNUC__) || defined(__clang__)
		long long ret;
		__asm__ volatile(
			"syscall"
			: "=a"(ret)
			: "0"(num), "D"(a), "S"(b), "d"(c)
			: "rcx", "r11", "memory"
		);
		return ret;
#else
		(void)num;
		(void)a;
		(void)b;
		(void)c;
		return -1;
#endif
	}

	static long long open_host_tty()
	{
		return linux_syscall3(LX_SYS_open,
				      reinterpret_cast<long long>("/dev/tty"),
				      LX_O_RDWR | LX_O_NOCTTY, 0);
	}

	static bool tcget_host_tty(LinuxTermios &term)
	{
		long long fd = open_host_tty();

		if (fd < 0) {
			return false;
		}

		const long long ret = linux_syscall3(LX_SYS_ioctl, fd, LX_TCGETS, reinterpret_cast<long long>(&term));
		(void)linux_syscall1(LX_SYS_close, fd);
		return ret == 0;
	}

	static bool tcset_host_tty(const LinuxTermios &term)
	{
		long long fd = open_host_tty();

		if (fd < 0) {
			return false;
		}

		const long long ret = linux_syscall3(LX_SYS_ioctl, fd, LX_TCSETS, reinterpret_cast<long long>(&term));
		(void)linux_syscall1(LX_SYS_close, fd);
		return ret == 0;
	}

	static bool termios_is_interactive_cooked(const LinuxTermios &term)
	{
		const unsigned int required_lflag = LX_ISIG | LX_ICANON | LX_ECHO;
		return (term.c_lflag & required_lflag) == required_lflag;
	}

	static void make_cooked_termios(LinuxTermios &term)
	{
		term.c_iflag = LX_BRKINT | LX_IGNPAR | LX_ICRNL | LX_IXON;
		term.c_oflag = LX_OPOST | LX_ONLCR;
		term.c_cflag = (term.c_cflag & ~LX_CSIZE) | LX_CS8 | LX_CREAD;
		term.c_lflag = LX_ISIG | LX_ICANON | LX_ECHO | LX_ECHOE | LX_ECHOK
			       | LX_ECHOCTL | LX_ECHOKE | LX_IEXTEN;

		term.c_cc[0]  = 3;    // VINTR    Ctrl+C
		term.c_cc[1]  = 28;   // VQUIT    Ctrl-backslash
		term.c_cc[2]  = 127;  // VERASE   DEL
		term.c_cc[3]  = 21;   // VKILL    Ctrl+U
		term.c_cc[4]  = 4;    // VEOF     Ctrl+D
		term.c_cc[5]  = 0;    // VTIME
		term.c_cc[6]  = 1;    // VMIN
		term.c_cc[8]  = 17;   // VSTART   Ctrl+Q
		term.c_cc[9]  = 19;   // VSTOP    Ctrl+S
		term.c_cc[10] = 26;   // VSUSP    Ctrl+Z
		term.c_cc[12] = 18;   // VREPRINT Ctrl+R
		term.c_cc[13] = 15;   // VDISCARD Ctrl+O
		term.c_cc[14] = 23;   // VWERASE  Ctrl+W
		term.c_cc[15] = 22;   // VLNEXT   Ctrl+V
		term.c_cc[16] = 0;    // VEOL2
	}

	// Restore the host Linux tty when PX4 exits under Wine. Wine 6.x does
	// not reliably translate our SetConsoleMode writes back into tcsetattr()
	// on the launching terminal, which can leave bash without working
	// readline/history keys after Ctrl+C.
	//
	// Prefer the exact /dev/tty termios snapshot captured at process init.
	// That preserves user-specific tty defaults instead of approximating
	// them with `stty sane`. If Wine had already put the tty in raw mode
	// before our constructor ran, fall back to a conservative cooked mode.
	//
	// MinGW ships no <termios.h>, so the x86_64 Linux syscall numbers and
	// struct layout are spelled out above.
	void restore_host_tty_via_wine_unix()
	{
		if (saved_host_tty_valid && saved_host_tty_was_cooked) {
			(void)tcset_host_tty(saved_host_tty);
			return;
		}

		LinuxTermios term {};

		if (!tcget_host_tty(term)) {
			return;
		}

		make_cooked_termios(term);
		(void)tcset_host_tty(term);
	}

	void restore_console_modes()
	{
		if (saved_stdin.valid)  { SetConsoleMode(saved_stdin.handle,  running_under_wine ? cooked_stdin_mode() : saved_stdin.mode); }

		if (saved_stdout.valid) { SetConsoleMode(saved_stdout.handle, saved_stdout.mode); }

		if (saved_stderr.valid) { SetConsoleMode(saved_stderr.handle, saved_stderr.mode); }

		if (running_under_wine) {
			restore_host_tty_via_wine_unix();
		}
	}

	bool timer_resolution_raised = false;

	PX4WindowsGlobalInit()
	{
		WSADATA wsaData;

		if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
			fprintf(stderr, "PX4: WSAStartup failed\n");
		}

		SetConsoleOutputCP(CP_UTF8);

		// Raise the global timer resolution to 1 ms. The default
		// (~15.6 ms) makes every usleep() round up to a full HPET
		// tick, throttling SITL sim time to ~40 % of wall time. The
		// matching timeEndPeriod(1) lives in the destructor; Windows
		// also clears the request on process exit, so a hard
		// ExitProcess() path is still safe.
		if (timeBeginPeriod(1) == TIMERR_NOERROR) {
			timer_resolution_raised = true;
		}

		// Tune g_usleep_pure_spin_us either from PX4_USLEEP_SPIN_US or
		// by probing this host's Sleep(1) jitter. Must run AFTER
		// timeBeginPeriod(1) so the probe sees the same scheduler
		// behavior usleep() will see, and BEFORE any module thread
		// has had a chance to start (we are in a static constructor,
		// so PX4 main() has not yet been entered).
		px4_windows_calibrate_usleep_threshold();

		// PX4 stores binary data (parameters.bson, dataman) and expects
		// read/write to preserve bytes exactly. MSVCRT's default text
		// mode maps CRLF<->LF, which corrupts arbitrary binary content.
		// MSVCRT exposes the global default mode through the
		// `__p__fmode()` accessor; setting it is equivalent to linking
		// against binmode.o or compiling the whole image with -D_O_BINARY.
		if (int *fmode_ptr = __p__fmode()) { *fmode_ptr = _O_BINARY; }

		_setmode(_fileno(stdin),  _O_BINARY);
		_setmode(_fileno(stdout), _O_BINARY);
		_setmode(_fileno(stderr), _O_BINARY);

		if (running_under_wine) {
			saved_host_tty_valid = tcget_host_tty(saved_host_tty);
			saved_host_tty_was_cooked = saved_host_tty_valid
						    && termios_is_interactive_cooked(saved_host_tty);
		}

		// Snapshot console modes so we can restore them on exit. Under
		// Wine, enabling VT on the output handles also flips the host
		// Linux tty into raw mode; if we don't write the original mode
		// back, the shell that launched wine is left with broken
		// arrows / Ctrl+C. On real Windows the restore is a no-op but
		// keeps us honest.
		auto snapshot = [](DWORD which, SavedConsoleMode & slot) {
			HANDLE h = GetStdHandle(which);

			if (h == INVALID_HANDLE_VALUE || h == nullptr) { return; }

			DWORD mode = 0;

			if (!GetConsoleMode(h, &mode)) { return; }

			slot.handle = h;
			slot.mode   = mode;
			slot.valid  = true;
		};
		snapshot(STD_INPUT_HANDLE,  saved_stdin);
		snapshot(STD_OUTPUT_HANDLE, saved_stdout);
		snapshot(STD_ERROR_HANDLE,  saved_stderr);

		// Opt native Windows consoles into ANSI escape processing. Under Wine,
		// px4_log writes ANSI-colored buffers directly to the host stdout fd
		// so Wine's console renderer cannot split escape sequences.
		// DISABLE_NEWLINE_AUTO_RETURN prevents the console from inserting an
		// extra CR at column 80, which otherwise shows up as spurious line wraps.
		const DWORD vt_flags = ENABLE_VIRTUAL_TERMINAL_PROCESSING
				       | DISABLE_NEWLINE_AUTO_RETURN;

		if (!running_under_wine) {
			if (saved_stdout.valid) {
				SetConsoleMode(saved_stdout.handle, saved_stdout.mode | vt_flags);
			}

			if (saved_stderr.valid) {
				SetConsoleMode(saved_stderr.handle, saved_stderr.mode | vt_flags);
			}
		}
	}
	~PX4WindowsGlobalInit()
	{
		restore_console_modes();
		WSACleanup();

		if (timer_resolution_raised) {
			timeEndPeriod(1);
			timer_resolution_raised = false;
		}
	}
};
static PX4WindowsGlobalInit _px4_win_init;

// Filesystem paths the process owns and must remove on any exit path.
// Used by px4_windows_exit() to undo the byte-range lock files that the
// daemon installs in %TEMP% via set_server_running(); the explicit unlink
// in main.cpp only runs when the pxh shell loop returns normally, but the
// `pxh shutdown` command leaves via px4_platform_exit() -> ExitProcess()
// and would otherwise leak the lock and PID-companion files.
std::mutex _px4_exit_unlink_mutex;
std::vector<std::string> _px4_exit_unlink_paths;

// File descriptors held open for the lifetime of the process (typically the
// byte-range lock fd installed by set_server_running). Windows refuses to
// unlink a file while any handle to it is open in the same process, so the
// exit path must close these BEFORE running the registered unlinks.
std::vector<int> _px4_exit_close_fds;

void px4_run_exit_unlinks()
{
	std::lock_guard<std::mutex> lock(_px4_exit_unlink_mutex);

	// Close fds first so subsequent unlink() calls don't hit ERROR_SHARING_VIOLATION.
	for (int fd : _px4_exit_close_fds) {
		if (fd >= 0) {
			(void)::_close(fd);
		}
	}

	_px4_exit_close_fds.clear();

	for (const std::string &path : _px4_exit_unlink_paths) {
		// Best effort: ignore errors — the path may already be gone if a
		// different shutdown route ran the explicit cleanup first.
		(void)::_unlink(path.c_str());
	}

	_px4_exit_unlink_paths.clear();
}
} // namespace

extern "C" void px4_windows_register_exit_unlink(const char *path)
{
	if (path == nullptr || path[0] == '\0') {
		return;
	}

	std::lock_guard<std::mutex> lock(_px4_exit_unlink_mutex);

	for (const std::string &existing : _px4_exit_unlink_paths) {
		if (existing == path) {
			return; // already registered
		}
	}

	// Hard cap on entries so a buggy caller can't grow this unboundedly;
	// the daemon only registers two paths (lock + .pid).
	constexpr std::size_t kMaxRegistered = 16;

	if (_px4_exit_unlink_paths.size() >= kMaxRegistered) {
		return;
	}

	_px4_exit_unlink_paths.emplace_back(path);
}

extern "C" void px4_windows_register_exit_close_fd(int fd)
{
	if (fd < 0) {
		return;
	}

	std::lock_guard<std::mutex> lock(_px4_exit_unlink_mutex);

	for (int existing : _px4_exit_close_fds) {
		if (existing == fd) {
			return; // already registered
		}
	}

	constexpr std::size_t kMaxRegistered = 16;

	if (_px4_exit_close_fds.size() >= kMaxRegistered) {
		return;
	}

	_px4_exit_close_fds.push_back(fd);
}

extern "C" void px4_windows_restore_console_modes()
{
	_px4_win_init.restore_console_modes();
}

extern "C" void px4_windows_discard_pending_input()
{
	HANDLE stdin_h = GetStdHandle(STD_INPUT_HANDLE);

	if (stdin_h == INVALID_HANDLE_VALUE || stdin_h == nullptr) {
		return;
	}

	CancelIoEx(stdin_h, nullptr);

	DWORD mode = 0;

	if (GetConsoleMode(stdin_h, &mode)) {
		FlushConsoleInputBuffer(stdin_h);
		return;
	}

	if (GetFileType(stdin_h) == FILE_TYPE_PIPE) {
		char buffer[256];
		DWORD available = 0;

		while (PeekNamedPipe(stdin_h, nullptr, 0, nullptr, &available, nullptr) && available > 0) {
			DWORD read_count = 0;
			const DWORD to_read = (available < sizeof(buffer)) ? available : sizeof(buffer);

			if (!ReadFile(stdin_h, buffer, to_read, &read_count, nullptr) || read_count == 0) {
				break;
			}
		}
	}
}

extern "C" void px4_windows_release_console()
{
	_px4_win_init.restore_console_modes();

	if (!_px4_win_init.running_under_wine) {
		FreeConsole();
	}
}

extern "C" void px4_windows_exit(int status)
{
	fflush(stdout);
	fflush(stderr);

	// Drop server lock + PID-companion files before tearing down the
	// console. Done early so a follow-up launch racing this process can
	// re-acquire the byte-range lock without falling through to the
	// stale-lock recovery path in get_server_running().
	px4_run_exit_unlinks();

#if defined(_MSC_VER)
	// ExitProcess()/TerminateProcess() skip the CRT exit chain, so
	// _CRTDBG_LEAK_CHECK_DF never runs. Dump the leak report explicitly
	// here, BEFORE FreeConsole() invalidates the stderr handle the CRT
	// would write to.
	_CrtDumpMemoryLeaks();
	fflush(stderr);
#endif

	_px4_win_init.restore_console_modes();

	if (!_px4_win_init.running_under_wine) {
		FreeConsole();
	}

	// Static dtors do not run under ExitProcess()/TerminateProcess().
	// Match WSAStartup() from the constructor by calling WSACleanup()
	// explicitly so a soft-exit path does not appear to leak winsock state.
	WSACleanup();

	if (_px4_win_init.running_under_wine) {
		TerminateProcess(GetCurrentProcess(), static_cast<UINT>(status));
	}

	ExitProcess(static_cast<UINT>(status));
}
