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
 * @file ids.cpp
 *
 * POSIX process-identity helpers: getppid(2), setsid(2),
 * getsid(2). Windows has no process-group identity; we return
 * the session PID or the current process ID as a best-effort.
 *
 * PX4 uses this mostly for daemon/session bookkeeping. It does not rely on
 * Unix job-control semantics, so a process-wide synthetic session id is enough
 * and avoids pretending Windows has a real controlling terminal model.
 */

#include "px4_windows_internal.h"

/* Defined in runtime/init.cpp: the session id claimed by the first
 * setsid() caller, shared process-wide via InterlockedCompareExchange. */
extern volatile LONG g_px4_session_id;

extern "C" pid_t getppid(void)
{
	const DWORD current_pid = GetCurrentProcessId();
	DWORD parent_pid = 1;
	HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);

	if (snapshot == INVALID_HANDLE_VALUE) {
		/* POSIX getppid() never fails. Fall back to PID 1 if Windows refuses
		 * the process snapshot. */
		return (pid_t)parent_pid;
	}

	PROCESSENTRY32 entry {};
	entry.dwSize = sizeof(entry);

	if (Process32First(snapshot, &entry)) {
		do {
			if (entry.th32ProcessID == current_pid) {
				parent_pid = entry.th32ParentProcessID ? entry.th32ParentProcessID : 1;
				break;
			}

		} while (Process32Next(snapshot, &entry));
	}

	CloseHandle(snapshot);
	return (pid_t)parent_pid;
}

extern "C" pid_t setsid(void)
{
	const LONG current_pid = (LONG)GetCurrentProcessId();
	const LONG session_id = InterlockedCompareExchange(&g_px4_session_id, current_pid, 0);
	return (pid_t)((session_id == 0) ? current_pid : session_id);
}

extern "C" pid_t getsid(pid_t pid)
{
	const DWORD requested_pid = (pid == 0) ? GetCurrentProcessId() : (DWORD)pid;
	HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, requested_pid);

	if (!process) {
		errno = ESRCH;
		return -1;
	}

	CloseHandle(process);

	LONG session_id = InterlockedCompareExchange(&g_px4_session_id, 0, 0);

	if (session_id == 0) {
		session_id = (LONG)GetCurrentProcessId();
	}

	return (pid_t)session_id;
}
