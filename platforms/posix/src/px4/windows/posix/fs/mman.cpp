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
 * @file mman.cpp
 *
 * POSIX sys/mman.h: mmap / munmap backed by CreateFileMapping +
 * MapViewOfFile. Used by the uORB shared ring buffers.
 *
 * There are two paths: anonymous mappings use VirtualAlloc, and file-backed
 * mappings use CreateFileMapping/MapViewOfFile. munmap() tries both release
 * mechanisms because the POSIX API does not carry enough state to tell us
 * which Windows allocator created the address.
 */

#include "px4_windows_internal.h"

/* --------------------------------------------------------------------------
 * sys/mman: MapViewOfFile-backed mmap.
 * -------------------------------------------------------------------------- */
static DWORD mman_page_protect(int prot)
{
	if ((prot & (PROT_READ | PROT_WRITE)) == (PROT_READ | PROT_WRITE)) { return PAGE_READWRITE; }

	if (prot & PROT_WRITE) { return PAGE_READWRITE; }

	if (prot & PROT_READ)  { return PAGE_READONLY;  }

	return PAGE_NOACCESS;
}

extern "C" void *mmap(void *, size_t length, int prot, int flags, int fd, off_t offset)
{
	/* MAP_FIXED is rejected because the Windows backend (VirtualAlloc /
	 * MapViewOfFile) cannot safely replace existing mappings at a caller
	 * supplied address. The shim header documents this contract. */
	if (flags & MAP_FIXED) {
		errno = ENOTSUP;
		return MAP_FAILED;
	}

	const bool anon = (flags & (MAP_ANON | MAP_ANONYMOUS)) != 0;

	if (anon || fd < 0) {
		void *p = VirtualAlloc(nullptr, length, MEM_COMMIT | MEM_RESERVE,
				       mman_page_protect(prot));
		return p ? p : MAP_FAILED;
	}

	HANDLE h = (HANDLE)_get_osfhandle(fd);

	if (h == INVALID_HANDLE_VALUE) { errno = EBADF; return MAP_FAILED; }

	DWORD map_access = (prot & PROT_WRITE) ? FILE_MAP_WRITE : FILE_MAP_READ;
	DWORD create_protect = (prot & PROT_WRITE) ? PAGE_READWRITE : PAGE_READONLY;

	HANDLE mapping = CreateFileMappingA(h, nullptr, create_protect, 0, 0, nullptr);

	if (!mapping) { errno = EACCES; return MAP_FAILED; }

	/* The mapping handle can be closed after MapViewOfFile succeeds; the view
	 * itself holds the kernel reference until UnmapViewOfFile. */
	LARGE_INTEGER off; off.QuadPart = offset;
	void *ptr = MapViewOfFile(mapping, map_access, off.HighPart, off.LowPart, length);
	CloseHandle(mapping); /* view keeps a ref */

	return ptr ? ptr : MAP_FAILED;
}

extern "C" int munmap(void *addr, size_t length)
{
	if (!addr) { errno = EINVAL; return -1; }

	/* Try UnmapViewOfFile first (file-backed); if that fails, try
	 * VirtualFree for anonymous mappings. Both are idempotent enough
	 * that trying the wrong one is harmless. */
	if (UnmapViewOfFile(addr)) { return 0; }

	if (VirtualFree(addr, 0, MEM_RELEASE)) { return 0; }

	(void)length;
	errno = EINVAL;
	return -1;
}

extern "C" int msync(void *addr, size_t length, int)     { return FlushViewOfFile(addr, length) ? 0 : -1; }
extern "C" int mprotect(void *addr, size_t length, int prot)
{
	DWORD old;
	return VirtualProtect(addr, length, mman_page_protect(prot), &old) ? 0 : -1;
}
extern "C" int mlock(const void *addr, size_t length)    { return VirtualLock((LPVOID)addr, length) ? 0 : -1; }
extern "C" int munlock(const void *addr, size_t length)  { return VirtualUnlock((LPVOID)addr, length) ? 0 : -1; }
extern "C" int mlockall(int)                             { return 0; }
extern "C" int munlockall(void)                          { return 0; }
