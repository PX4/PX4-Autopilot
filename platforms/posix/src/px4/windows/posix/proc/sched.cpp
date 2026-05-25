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
 * @file sched.cpp
 *
 * POSIX process-scheduler edges: daemon(3), kill(2), waitpid(2).
 * Windows has no unified process-group concept; we approximate
 * what PX4 needs using CreateProcess / GenerateConsoleCtrlEvent /
 * WaitForSingleObject.
 *
 * This is process control, not task scheduling. PX4 tasks are handled by
 * platforms/common and pthreads; this file exists for POSIX APIs that scripts
 * or daemon code may call while setting up or shutting down SITL.
 */

#include "px4_windows_internal.h"

#include <sys/wait.h>
#include <array>

extern "C" int daemon(int nochdir, int noclose)
{
	if (!nochdir) {
		char cwd[MAX_PATH] = {};
		char root[4] = "C:\\";

		if (GetCurrentDirectoryA(sizeof(cwd), cwd) > 1 && cwd[1] == ':') {
			root[0] = cwd[0];
		}

		if (!SetCurrentDirectoryA(root)) {
			errno = px4_win_error_to_errno(GetLastError());
			return -1;
		}
	}

	if (!noclose) {
		/* POSIX daemon() detaches stdio. Redirect to NUL rather than closing the
		 * CRT descriptors outright; many libraries assume fd 0/1/2 stay valid. */
		const int null_fd = _open("NUL", _O_RDWR | _O_BINARY);

		if (null_fd < 0) {
			errno = EIO;
			return -1;
		}

		_dup2(null_fd, STDIN_FILENO);
		_dup2(null_fd, STDOUT_FILENO);
		_dup2(null_fd, STDERR_FILENO);

		if (null_fd > STDERR_FILENO) {
			_close(null_fd);
		}
	}

	(void)setsid();
	return 0;
}

extern "C" int kill(pid_t pid, int sig)
{
	if (pid <= 0 || sig < 0) {
		errno = EINVAL;
		return -1;
	}

	if ((DWORD)pid == GetCurrentProcessId()) {
		if (sig == 0) {
			return 0;
		}

		return raise(sig) == 0 ? 0 : (errno = EINVAL, -1);
	}

	const DWORD access = PROCESS_QUERY_LIMITED_INFORMATION | SYNCHRONIZE | ((sig != 0) ? PROCESS_TERMINATE : 0);
	HANDLE process = OpenProcess(access, FALSE, (DWORD)pid);

	if (!process) {
		errno = ESRCH;
		return -1;
	}

	if (sig == 0) {
		CloseHandle(process);
		return 0;
	}

	const BOOL terminated = TerminateProcess(process, (UINT)(128 + sig));
	CloseHandle(process);

	if (!terminated) {
		errno = EPERM;
		return -1;
	}

	return 0;
}

extern "C" pid_t waitpid(pid_t pid, int *status, int options)
{
	if (pid <= 0) {
		errno = ENOTSUP;
		return -1;
	}

	const int supported_options = WNOHANG | WUNTRACED | WCONTINUED;

	if ((options & ~supported_options) != 0) {
		errno = EINVAL;
		return -1;
	}

	HANDLE process = OpenProcess(SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION, FALSE, (DWORD)pid);

	if (!process) {
		errno = ESRCH;
		return -1;
	}

	const DWORD timeout_ms = (options & WNOHANG) ? 0 : INFINITE;
	const DWORD wait_rc = WaitForSingleObject(process, timeout_ms);

	if (wait_rc == WAIT_TIMEOUT) {
		CloseHandle(process);
		return 0;
	}

	if (wait_rc != WAIT_OBJECT_0) {
		CloseHandle(process);
		errno = EIO;
		return -1;
	}

	DWORD exit_code = STILL_ACTIVE;

	if (!GetExitCodeProcess(process, &exit_code)) {
		CloseHandle(process);
		errno = EIO;
		return -1;
	}

	CloseHandle(process);

	if (exit_code == STILL_ACTIVE) {
		return 0;
	}

	if (status) {
		/* Store a wait status that WEXITSTATUS-style macros can decode. We do
		 * not model signal stops/continues on Windows. */
		*status = ((int)(exit_code & 0xff)) << 8;
	}

	return pid;
}
