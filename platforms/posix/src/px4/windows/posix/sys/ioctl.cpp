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
 * @file ioctl.cpp
 *
 * POSIX ioctl(2) dispatcher. Only FIONREAD/FIONBIO on winsock
 * sockets are handled in-process; everything else returns
 * -ENOSYS so the caller surfaces a clean error.
 *
 * Do not add driver-specific ioctls here unless there is a real Windows device
 * implementation behind them. Returning success for unsupported device control
 * would hide broken SITL plumbing.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * ioctl: cover FIONREAD/FIONBIO on sockets. Anything else is rejected
 * with -ENOSYS. SITL drivers that ioctl a character device on Windows
 * are not supported by this port.
 * -------------------------------------------------------------------------- */
extern "C" int ioctl(int fd, unsigned long request, ...)
{
	va_list ap;
	va_start(ap, request);
	void *arg = va_arg(ap, void *);
	va_end(ap);

	if (request == FIONREAD || request == FIONBIO) {
		unsigned long v = arg ? *(unsigned long *)arg : 0;
		/* Winsock uses ioctlsocket for the socket readiness/non-blocking calls
		 * that POSIX code normally sends through ioctl(). */
		int rc = ioctlsocket((SOCKET)fd, (long)request, &v);

		if (rc == 0 && arg) { *(unsigned long *)arg = v; }

		return rc == 0 ? 0 : -1;
	}

	errno = ENOSYS;
	return -1;
}
