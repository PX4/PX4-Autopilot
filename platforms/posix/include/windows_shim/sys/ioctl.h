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
 * @file sys/ioctl.h
 *
 * SITL does not actually drive ioctl'd character devices on Windows -
 * there is no /dev/tty or /dev/mem. Callers include <sys/ioctl.h>
 * for FIONREAD/FIONBIO/TIOCM* constants. We expose those symbols,
 * route FIONREAD/FIONBIO through winsock's ioctlsocket (which uses the
 * same FIONREAD/FIONBIO values), and return -ENOSYS for anything else.
 */
#pragma once

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FIONREAD
#  define FIONREAD 0x4004667F
#endif
#ifndef FIONBIO
#  define FIONBIO  0x8004667E
#endif
#ifndef TIOCINQ
#  define TIOCINQ FIONREAD
#endif
#ifndef TIOCOUTQ
#  define TIOCOUTQ 0x5411
#endif
#ifndef FIOCLEX
#  define FIOCLEX 0x5451
#endif
#ifndef FIONCLEX
#  define FIONCLEX 0x5450
#endif
#ifndef TIOCEXCL
#  define TIOCEXCL 0x540C
#endif
#ifndef TIOCNXCL
#  define TIOCNXCL 0x540D
#endif
#ifndef TIOCSCTTY
#  define TIOCSCTTY 0x540E
#endif
#ifndef TIOCMGET
#  define TIOCMGET 0x5415
#endif
#ifndef TIOCMSET
#  define TIOCMSET 0x5418
#endif
#ifndef TIOCM_DTR
#  define TIOCM_DTR 0x002
#endif
#ifndef TIOCM_RTS
#  define TIOCM_RTS 0x004
#endif
#ifndef TIOCSBRK
#  define TIOCSBRK 0x5427
#endif
#ifndef TIOCCBRK
#  define TIOCCBRK 0x5428
#endif
#ifndef TCGETS
#  define TCGETS 0x5401
#endif
#ifndef TCSETS
#  define TCSETS 0x5402
#endif
#ifndef TCSETSW
#  define TCSETSW 0x5403
#endif
#ifndef TCSETSF
#  define TCSETSF 0x5404
#endif

/**
 * @brief POSIX ioctl() compatibility subset for sockets and terminal constants.
 *
 * FIONREAD/FIONBIO are forwarded to ioctlsocket(). Unsupported requests fail
 * with ENOSYS so callers can follow their existing POSIX fallback path.
 */
int ioctl(int fd, unsigned long request, ...);

#ifdef __cplusplus
}
#endif
