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
 * @file sysconf.cpp
 *
 * POSIX sysconf(3): backed by Win32 GetSystemInfo /
 * GlobalMemoryStatusEx / GetCurrentProcess queries so reported
 * numbers match what winpthreads / CreateFileMapping actually do.
 *
 * This file also holds a few filesystem helpers declared from unistd/statfs
 * shims. They live here because they are syscall-style glue rather than
 * module-specific PX4 behavior.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * sysconf: just enough of POSIX sysconf for PX4. All values come from
 * Win32 directly so there is no drift between what the shim reports and
 * what winpthreads/CreateFileMapping actually do.
 * -------------------------------------------------------------------------- */
extern "C" int statfs(const char *path, struct statfs *buf)
{
	if (!path || !buf) { errno = EINVAL; return -1; }

	memset(buf, 0, sizeof(*buf));

	ULARGE_INTEGER free_caller, total_bytes, free_total;

	if (!GetDiskFreeSpaceExA(path, &free_caller, &total_bytes, &free_total)) {
		errno = ENOENT;
		return -1;
	}

	/* Win32 doesn't give us a block size in that call; report 4 KiB
	 * which is the NTFS default and Windows' virtual cluster size. */
	buf->f_bsize   = 4096;
	buf->f_blocks  = (long)(total_bytes.QuadPart / buf->f_bsize);
	buf->f_bfree   = (long)(free_total.QuadPart  / buf->f_bsize);
	buf->f_bavail  = (long)(free_caller.QuadPart / buf->f_bsize);
	buf->f_namelen = 255;
	return 0;
}

/* Recursive directory copy. Used as the symlink fallback on systems
 * that can't create symlinks without elevation (stock Windows without
 * Developer Mode) or that don't honour CreateSymbolicLinkA semantics
 * correctly (wine 6 silently returns success but produces no entry).
 * Overwrites existing targets; the caller checks for existence before
 * invoking symlink(). */
static BOOL copy_dir_recursive(const char *src, const char *dst)
{
	if (!CreateDirectoryA(dst, nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
		return FALSE;
	}

	char pattern[MAX_PATH];
	snprintf(pattern, sizeof(pattern), "%s\\*", src);

	WIN32_FIND_DATAA fd;
	HANDLE h = FindFirstFileA(pattern, &fd);

	if (h == INVALID_HANDLE_VALUE) { return TRUE; /* empty dir is fine */ }

	BOOL ok = TRUE;

	do {
		if (strcmp(fd.cFileName, ".") == 0 || strcmp(fd.cFileName, "..") == 0) { continue; }

		char s[MAX_PATH], d[MAX_PATH];
		snprintf(s, sizeof(s), "%s\\%s", src, fd.cFileName);
		snprintf(d, sizeof(d), "%s\\%s", dst, fd.cFileName);

		if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			if (!copy_dir_recursive(s, d)) { ok = FALSE; break; }

		} else {
			if (!CopyFileA(s, d, FALSE)) { ok = FALSE; break; }
		}
	} while (FindNextFileA(h, &fd));

	FindClose(h);
	return ok;
}

extern "C" int symlink(const char *target, const char *linkpath)
{
	if (!target || !linkpath) { errno = EINVAL; return -1; }

	/* Detect whether the target is a directory so we hand the right
	 * flag to CreateSymbolicLinkA. Windows distinguishes file and
	 * directory symlinks at creation time, unlike POSIX. */
	DWORD attrs = GetFileAttributesA(target);
	const bool is_dir = (attrs != INVALID_FILE_ATTRIBUTES) && (attrs & FILE_ATTRIBUTE_DIRECTORY);

	DWORD flags = 0;

	if (is_dir) { flags |= 0x1; /* SYMBOLIC_LINK_FLAG_DIRECTORY */ }

	/* Allow creating symlinks without admin rights when Developer Mode
	 * is enabled (Win10 1703+). Harmless on older Windows. */
	flags |= 0x2;     /* SYMBOLIC_LINK_FLAG_ALLOW_UNPRIVILEGED_CREATE */

	BOOL sym_ok = CreateSymbolicLinkA(linkpath, target, flags);

	/* Verify the link is actually accessible. Wine's CreateSymbolicLinkA
	 * can report success and leave nothing behind. */
	if (sym_ok && GetFileAttributesA(linkpath) != INVALID_FILE_ATTRIBUTES) {
		return 0;
	}

	/* Fall back to a hard link for files and a recursive copy for
	 * directories. This is slower and uses real disk, but it works
	 * without elevation and on hosts with broken symlink support. */
	if (is_dir) {
		return copy_dir_recursive(target, linkpath) ? 0 : (errno = EPERM, -1);
	}

	if (CreateHardLinkA(linkpath, target, nullptr)) { return 0; }

	if (CopyFileA(target, linkpath, FALSE))         { return 0; }

	errno = EPERM;
	return -1;
}

extern "C" int link(const char *existing_path, const char *new_path)
{
	if (!existing_path || !new_path) {
		errno = EINVAL;
		return -1;
	}

	if (CreateHardLinkA(new_path, existing_path, nullptr)) {
		return 0;
	}

	errno = px4_win_error_to_errno(GetLastError());
	return -1;
}

extern "C" ssize_t readlink(const char *path, char *buf, size_t bufsiz)
{
	if (!path || !buf || bufsiz == 0) {
		errno = EINVAL;
		return -1;
	}

	HANDLE h = CreateFileA(path, 0,
			       FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
			       nullptr, OPEN_EXISTING,
			       FILE_FLAG_OPEN_REPARSE_POINT | FILE_FLAG_BACKUP_SEMANTICS,
			       nullptr);

	if (h == INVALID_HANDLE_VALUE) {
		errno = ENOENT;
		return -1;
	}

	char target[MAX_PATH * 4];
	DWORD len = GetFinalPathNameByHandleA(h, target, sizeof(target), FILE_NAME_NORMALIZED);
	CloseHandle(h);

	if (len == 0 || len >= sizeof(target)) {
		errno = EIO;
		return -1;
	}

	const char *normalized = target;

	/* GetFinalPathNameByHandleA returns Win32 namespace prefixes. Strip them
	 * before handing the path back through the POSIX readlink API. */
	if (strncmp(normalized, "\\\\?\\UNC\\", 8) == 0) {
		normalized += 6; // keep leading backslash for UNC

	} else if (strncmp(normalized, "\\\\?\\", 4) == 0) {
		normalized += 4;
	}

	const size_t out_len = strlen(normalized);
	const size_t copy_len = (out_len < bufsiz) ? out_len : bufsiz;
	memcpy(buf, normalized, copy_len);
	return (ssize_t)copy_len;
}

extern "C" int truncate(const char *path, off_t length)
{
	if (!path || length < 0) {
		errno = EINVAL;
		return -1;
	}

	HANDLE h = CreateFileA(path, GENERIC_WRITE,
			       FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
			       nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);

	if (h == INVALID_HANDLE_VALUE) {
		errno = ENOENT;
		return -1;
	}

	LARGE_INTEGER li {};
	li.QuadPart = length;
	const BOOL ok = SetFilePointerEx(h, li, nullptr, FILE_BEGIN) && SetEndOfFile(h);
	CloseHandle(h);

	if (!ok) {
		errno = EIO;
		return -1;
	}

	return 0;
}

extern "C" int getpagesize(void)
{
	SYSTEM_INFO si {};
	GetSystemInfo(&si);
	return (int)si.dwPageSize;
}

extern "C" long sysconf(int name)
{
	/* Numeric constants match the values declared in windows_shim/unistd.h.
	 * Keep this switch synchronized with that header. */
	switch (name) {
	case 30: { /* _SC_PAGESIZE */
			SYSTEM_INFO si;
			GetSystemInfo(&si);
			return (long)si.dwPageSize;
		}

	case 2:  /* _SC_CLK_TCK: Win32 QueryPerformanceFrequency is Hz */
		/* Use 100Hz, matching what Windows' multimedia timers resolve
		 * to by default. Drivers that need higher resolution use
		 * hrt_absolute_time() which goes through QPC directly. */
		return 100;

	case 84: { /* _SC_NPROCESSORS_ONLN */
			SYSTEM_INFO si;
			GetSystemInfo(&si);
			return (long)si.dwNumberOfProcessors;
		}

	case 83: { /* _SC_NPROCESSORS_CONF */
			SYSTEM_INFO si;
			GetSystemInfo(&si);
			return (long)si.dwNumberOfProcessors;
		}

	case 4: /* _SC_OPEN_MAX */
		return (long)_getmaxstdio();

	case 180: /* _SC_HOST_NAME_MAX */
		return (long)(MAX_COMPUTERNAME_LENGTH + 1);

	case 71: /* _SC_LOGIN_NAME_MAX */
		return 256;

	case 85: /* _SC_PHYS_PAGES */
	case 86: { /* _SC_AVPHYS_PAGES */
			MEMORYSTATUSEX mem {};
			mem.dwLength = sizeof(mem);

			if (!GlobalMemoryStatusEx(&mem)) {
				errno = EIO;
				return -1;
			}

			const ULONGLONG page_size = (ULONGLONG)getpagesize();
			const ULONGLONG bytes = (name == 85) ? mem.ullTotalPhys : mem.ullAvailPhys;
			return (long)(bytes / page_size);
		}
	}

	errno = EINVAL;
	return -1;
}
