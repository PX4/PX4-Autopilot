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
 * @file sys/mman.h
 *
 * Covers the mmap surface SITL uses: anonymous memory for parameter
 * backing stores / shared memory, and file-backed memory for logs and
 * dataman. Backed by VirtualAlloc (anonymous) and CreateFileMapping +
 * MapViewOfFile (file-backed). mlock/mlockall are no-ops - Windows
 * has VirtualLock but requires the SeLockMemoryPrivilege which SITL
 * does not need.
 */
#pragma once

#include <sys/types.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROT_NONE   0x0
#define PROT_READ   0x1
#define PROT_WRITE  0x2
#define PROT_EXEC   0x4

#define MAP_SHARED    0x01
#define MAP_PRIVATE   0x02
#define MAP_FIXED     0x10
#define MAP_ANON      0x20
#define MAP_ANONYMOUS MAP_ANON
#define MAP_NORESERVE 0x4000
#define MAP_POPULATE  0x8000
#define MAP_LOCKED    0x2000

#define MAP_FAILED    ((void *) -1)

#define MS_ASYNC      1
#define MS_SYNC       4
#define MS_INVALIDATE 2

#define MCL_CURRENT   1
#define MCL_FUTURE    2

#define MADV_NORMAL      0
#define MADV_RANDOM      1
#define MADV_SEQUENTIAL  2
#define MADV_WILLNEED    3
#define MADV_DONTNEED    4

#define POSIX_MADV_NORMAL      MADV_NORMAL
#define POSIX_MADV_RANDOM      MADV_RANDOM
#define POSIX_MADV_SEQUENTIAL  MADV_SEQUENTIAL
#define POSIX_MADV_WILLNEED    MADV_WILLNEED
#define POSIX_MADV_DONTNEED    MADV_DONTNEED

/**
 * @brief Map anonymous or file-backed memory into the process.
 *
 * @param addr Requested address. MAP_FIXED is rejected because the Windows
 *             backend does not support safely replacing existing mappings.
 * @param length Mapping length in bytes.
 * @param prot POSIX PROT_* protection bits.
 * @param flags POSIX MAP_* flags. MAP_ANONYMOUS uses VirtualAlloc; file-backed
 *              mappings use CreateFileMapping/MapViewOfFile.
 * @param fd CRT file descriptor for file-backed mappings, or -1 for anonymous.
 * @param offset File offset for file-backed mappings.
 * @return Mapped address, or MAP_FAILED with errno set.
 */
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);

/** @brief Unmap a region returned by mmap(). */
int   munmap(void *addr, size_t length);

/** @brief Flush a mapped file range when the mapping is file-backed. */
int   msync(void *addr, size_t length, int flags);

/** @brief Change page protection for an existing mapping. */
int   mprotect(void *addr, size_t length, int prot);

/** @brief Accept POSIX memory-lock calls; currently a no-op on Windows. */
int   mlock(const void *addr, size_t length);

/** @brief Accept POSIX memory-unlock calls; currently a no-op on Windows. */
int   munlock(const void *addr, size_t length);

/** @brief Accept process-wide memory-lock calls; currently a no-op. */
int   mlockall(int flags);

/** @brief Accept process-wide memory-unlock calls; currently a no-op. */
int   munlockall(void);

/**
 * @brief Accept POSIX memory-advice calls.
 *
 * Windows has PrefetchVirtualMemory on newer releases, but PX4 does not depend
 * on advisory paging behavior. Returning success matches the permissive POSIX
 * interpretation for unsupported advice.
 */
static inline int madvise(void *addr, size_t length, int advice)
{
	(void)addr;
	(void)length;
	(void)advice;
	return 0;
}

#ifdef __cplusplus
}
#endif
