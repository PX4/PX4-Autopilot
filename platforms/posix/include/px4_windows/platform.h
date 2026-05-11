/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file platform.h
 *
 * Public Windows-host hooks for the POSIX platform implementation.
 */

#pragma once

#include <stdint.h>

#include <visibility.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Return whether this Windows process is currently hosted by Wine.
 *
 * The check is runtime-only and cached in the Windows backend. Native MSVC
 * and MinGW-on-Windows runs return 0 and keep the normal Windows timing and
 * console paths; Wine returns 1 so selected call sites can avoid Wine-specific
 * syscall hot spots without changing native Windows behavior.
 *
 * @return 1 when running under Wine, 0 otherwise.
 */
int px4_windows_running_under_wine(void);

/**
 * @brief Low-overhead monotonic clock for Wine-only spin loops.
 *
 * Wine makes frequent Win32 wall-clock queries expensive enough to dominate
 * sub-millisecond lockstep waits. This helper is only intended for call sites
 * that already gated on px4_windows_running_under_wine(); it uses a cached
 * calibration to avoid repeated Wine server round trips in tight loops.
 *
 * @return Monotonic host time in microseconds.
 */
uint64_t px4_windows_wine_monotonic_time_us(void);

/**
 * @brief Spin until the Wine monotonic clock reaches @p deadline_us.
 *
 * Uses the same cached calibration as px4_windows_wine_monotonic_time_us(),
 * but compares raw CPU-cycle deadlines in the hot loop when possible. This is
 * for Wine-only sub-millisecond waits where even repeated timestamp conversion
 * becomes visible at high lockstep speed factors.
 *
 * @param deadline_us Absolute deadline in Wine monotonic microseconds.
 */
void px4_windows_wine_spin_until_us(uint64_t deadline_us);

/**
 * @brief Restore the console modes captured during Windows platform startup.
 *
 * PX4 enables virtual-terminal output and may switch the input console into a
 * raw-ish mode for the pxh shell. This must run before process teardown so a
 * native console, a Wine-hosted terminal, or a parent shell does not inherit
 * stale input flags.
 */
void px4_windows_restore_console_modes(void);

/**
 * @brief Drop console input that was queued while PX4 was shutting down.
 *
 * Wine can leave bytes for control sequences or buffered pxh input pending in
 * the Linux terminal after Ctrl+C. The shutdown path calls this before
 * returning control to the parent shell.
 */
void px4_windows_discard_pending_input(void);

/**
 * @brief Release Windows console resources owned by the PX4 process.
 *
 * Native Windows builds use this as the last console cleanup step; under Wine
 * it complements the terminal-mode restore and input discard hooks above.
 */
void px4_windows_release_console(void);

/**
 * @brief Exit PX4 after running Windows-specific console cleanup.
 *
 * @param status Process exit code passed to the C runtime after cleanup.
 */
void px4_windows_exit(int status) noreturn_function;

/**
 * @brief Queue a filesystem path to unlink during px4_windows_exit().
 *
 * The graceful `pxh shutdown` path on Windows ends in ExitProcess(), which
 * skips the explicit lock-file cleanup at the bottom of main(). Anything that
 * must survive only as long as the process lives (server lock file, PID
 * companion file, ...) should be registered here so the exit hook unlinks it
 * regardless of the shutdown route taken.
 *
 * Best-effort: silently drops paths if the internal slot table is full, and
 * silently ignores unlink failures. Idempotent for repeated registrations of
 * the same path.
 */
void px4_windows_register_exit_unlink(const char *path);

/**
 * @brief Queue a file descriptor to close BEFORE the registered unlinks run.
 *
 * Windows refuses to unlink a file while any handle to it is open in the same
 * process (ERROR_SHARING_VIOLATION). The byte-range lock fd installed by
 * set_server_running() is intentionally leaked for the lifetime of the
 * daemon, so the exit hook must close it first or the corresponding lock
 * file would leak in %TEMP%. Idempotent for repeated registrations of the
 * same fd. Negative fds are ignored.
 */
void px4_windows_register_exit_close_fd(int fd);

#ifdef __cplusplus
}
#endif
