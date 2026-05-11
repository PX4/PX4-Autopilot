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
 * @file px4_windows_internal.h
 *
 * Shared preamble for the Windows backend of the POSIX platform.
 * Every Windows backend .cpp includes this header before anything else
 * so the Win32 / winsock / POSIX-shim include order is identical
 * everywhere, and so errno-mapping helpers are declared once.
 *
 * The layout under platforms/posix/src/px4/windows is:
 *   runtime/    - process init, console, VT, /dev/tty restore
 *   posix/fs/   - filesystem operations (mman, flock, ...)
 *   posix/sys/  - syscall-adjacent shims (termios, ioctl, sysconf, ...)
 *   posix/net/  - networking: resolvers, sockets, interface queries
 *   posix/proc/ - process / user identity / environment
 *   posix/lib/  - userland helpers (dlfcn, ...)
 *   shell/      - PX4 startup-script backend for Windows
 */
#pragma once

#ifdef _WIN32

#define _WIN32_WINNT 0x0A00
#include <string.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <tlhelp32.h>
#include <io.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>

#include <arpa/inet.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/statfs.h>
#include <sys/wait.h>
#include <dlfcn.h>
#include <pwd.h>
#include <grp.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iphlpapi.h>

#include <px4_windows/platform.h>

/**
 * @brief Translate a Win32 error code into its nearest POSIX errno value.
 *
 * @param err Value returned by GetLastError().
 * @return POSIX errno value. Unknown Windows errors map conservatively to
 *         EINVAL so callers still fail through the normal POSIX path.
 */
int px4_win_error_to_errno(DWORD err);

/**
 * @brief Translate a Winsock error code into a POSIX errno value.
 *
 * @param err Value returned by WSAGetLastError().
 * @return POSIX errno value for socket/resolver callers.
 */
int px4_wsa_error_to_errno(int err);

/**
 * @brief Return a stable string for a legacy resolver h_errno code.
 *
 * @param err HOST_NOT_FOUND, TRY_AGAIN, NO_RECOVERY, NO_DATA, or a Winsock
 *            resolver error.
 * @return Static, human-readable error string.
 */
const char *px4_hstrerror_text(int err);

#endif /* _WIN32 */
