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

/**
 * @file statfs.h
 *
 * Portable px4_statfs(). Linux has statfs() in <sys/statfs.h>, macOS in
 * <sys/mount.h>. QNX only has POSIX statvfs(), so there px4_statfs() fills the
 * fields PX4 uses (f_bsize, f_blocks, f_bfree, f_bavail) from statvfs().
 * statvfs counts blocks in f_frsize units, so f_bsize is set to f_frsize to
 * keep f_blocks * f_bsize equal to the byte size.
 */

#pragma once

#if defined(__PX4_QNX)
#include <sys/statvfs.h>

typedef struct {
	unsigned long f_bsize;
	fsblkcnt_t f_blocks;
	fsblkcnt_t f_bfree;
	fsblkcnt_t f_bavail;
} px4_statfs_t;

static inline int px4_statfs(const char *path, px4_statfs_t *buf)
{
	struct statvfs vfs;

	if (statvfs(path, &vfs) != 0) {
		return -1;
	}

	buf->f_bsize = vfs.f_frsize ? vfs.f_frsize : vfs.f_bsize;
	buf->f_blocks = vfs.f_blocks;
	buf->f_bfree = vfs.f_bfree;
	buf->f_bavail = vfs.f_bavail;
	return 0;
}

#else
#if defined(__PX4_DARWIN)
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

typedef struct statfs px4_statfs_t;

static inline int px4_statfs(const char *path, px4_statfs_t *buf)
{
	return statfs(path, buf);
}
#endif
