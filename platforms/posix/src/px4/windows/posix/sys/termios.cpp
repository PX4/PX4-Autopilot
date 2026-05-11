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
 * @file termios.cpp
 *
 * POSIX termios(3) surface: stubbed. SITL drivers that reference a
 * serial UART compile against this, but no Windows serial I/O is
 * wired up in the base build; calls that should move bytes over a
 * COM port succeed with safe defaults rather than crashing.
 *
 * This is intentionally not a fake serial driver. If a Windows SITL target
 * needs real COM-port traffic, add a separate implementation that owns a Win32
 * HANDLE and uses SetCommState/ReadFile/WriteFile; do not grow silent behavior
 * into these no-op configuration helpers.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * termios: present a sane default attribute set but reject real I/O. No
 * driver in the default SITL image is expected to move bytes over a
 * COM port in this port; real serial would be layered on top of
 * CreateFileA("\\\\.\\COMx", ...) + SetCommState.
 * -------------------------------------------------------------------------- */
extern "C" int tcgetattr(int, struct termios *t)
{
	if (!t) { errno = EINVAL; return -1; }

	memset(t, 0, sizeof(*t));
	t->c_cflag = CS8 | CREAD | CLOCAL;
	t->c_ispeed = t->c_ospeed = B115200;
	return 0;
}
extern "C" int tcsetattr(int, int, const struct termios *)          { return 0; }
extern "C" int tcflush(int, int)                                    { return 0; }
extern "C" int tcdrain(int)                                         { return 0; }
extern "C" int tcflow(int, int)                                     { return 0; }
extern "C" int tcsendbreak(int, int)                                { return 0; }
extern "C" pid_t tcgetsid(int)                                      { return (pid_t)GetCurrentProcessId(); }
extern "C" int cfsetispeed(struct termios *t, speed_t s)            { if (t) { t->c_ispeed = s; } return 0; }
extern "C" int cfsetospeed(struct termios *t, speed_t s)            { if (t) { t->c_ospeed = s; } return 0; }
extern "C" int cfsetspeed(struct termios *t, speed_t s)             { if (t) { t->c_ispeed = t->c_ospeed = s; } return 0; }
extern "C" speed_t cfgetispeed(const struct termios *t)             { return t ? t->c_ispeed : 0; }
extern "C" speed_t cfgetospeed(const struct termios *t)             { return t ? t->c_ospeed : 0; }
extern "C" void cfmakeraw(struct termios *t)
{
	if (!t) { return; }

	/* Match the traditional POSIX cfmakeraw bit clearing so code inspecting the
	 * termios struct sees the same shape it would on Linux. */
	t->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	t->c_oflag &= ~OPOST;
	t->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	t->c_cflag &= ~(CSIZE | PARENB);
	t->c_cflag |= CS8;
}
