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

#include "UavcanNodeShell.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifdef __PX4_NUTTX
#include <nshlib/nshlib.h>
#endif /* __PX4_NUTTX */

using namespace uavcannode;

UavcanNodeShell::~UavcanNodeShell()
{
	// closing the pipes gives EOF, letting the shell task exit on its own
	if (_to_shell_fd >= 0) {
		close(_to_shell_fd);
	}

	if (_from_shell_fd >= 0) {
		close(_from_shell_fd);
	}
}

int UavcanNodeShell::start()
{
#if !defined(__PX4_NUTTX)
	return -1;
#else
	int p1[2], p2[2];

	if (pipe(p1) != 0) {
		return -errno;
	}

	if (pipe(p2) != 0) {
		close(p1[0]);
		close(p1[1]);
		return -errno;
	}

	int ret = 0;

	_from_shell_fd = p1[0];
	_to_shell_fd = p2[1];
	_shell_fds[0] = p2[0];
	_shell_fds[1] = p1[1];

	fcntl(_from_shell_fd, F_SETFL, fcntl(_from_shell_fd, F_GETFL, 0) | O_NONBLOCK);
	fcntl(_to_shell_fd, F_SETFL, fcntl(_to_shell_fd, F_GETFL, 0) | O_NONBLOCK);

	// lock while temporarily redirecting fd 0/1 for the child task
	sched_lock();

	int fd_backups[2];

	for (int i = 0; i < 2; ++i) {
		fd_backups[i] = dup(i);

		if (fd_backups[i] == -1) {
			ret = -errno;
		}
	}

	dup2(_shell_fds[0], 0);
	dup2(_shell_fds[1], 1);

	if (ret == 0) {
		// the new task inherits fd 0/1 at spawn time, so the redirect only needs to hold until here
		_task = px4_task_spawn_cmd("uavcan_shell",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   2048,
					   &UavcanNodeShell::shell_start_thread,
					   nullptr);

		if (_task < 0) {
			ret = -1;
		}
	}

	// restore fd's of the caller task
	for (int i = 0; i < 2; ++i) {
		if (dup2(fd_backups[i], i) == -1) {
			ret = -errno;
		}

		close(fd_backups[i]);
	}

	// the task now owns these through its own fd 0/1
	close(_shell_fds[0]);
	close(_shell_fds[1]);

	sched_unlock();

	return ret;
#endif /* __PX4_NUTTX */
}

int UavcanNodeShell::shell_start_thread(int argc, char *argv[])
{
#ifdef __PX4_NUTTX
	dup2(1, 2); // redirect stderr to stdout

	const int ret = nsh_consolemain(0, NULL);

	if (ret) {
		PX4_ERR("uavcan shell failed: %d%s", ret, (ret == -ENOMEM) ? " (out of memory)" : "");
		return ret;
	}

#endif /* __PX4_NUTTX */

	return 0;
}

size_t UavcanNodeShell::write(const uint8_t *buffer, size_t len)
{
	const int ret = ::write(_to_shell_fd, buffer, len);

	if (ret < 0) {
		return 0;
	}

	return ret;
}

size_t UavcanNodeShell::read(uint8_t *buffer, size_t len)
{
	const int ret = ::read(_from_shell_fd, buffer, len);

	// EAGAIN means nothing available yet
	if (ret < 0) {
		return 0;
	}

	return ret;
}

size_t UavcanNodeShell::available()
{
	int ret = 0;

	if (ioctl(_from_shell_fd, FIONREAD, (unsigned long)&ret) == OK) {
		return ret;
	}

	return 0;
}
