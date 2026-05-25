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
 * @file sys/statfs.h
 *
 * PX4 uses statfs() only to check whether SITL's synthetic storage
 * directory looks like a working filesystem (f_blocks > 0). We route
 * the query through GetDiskFreeSpaceExA on Windows - enough to make
 * the arming check pass against %LOCALAPPDATA%\PX4.
 */
#pragma once

#include <sys/types.h>
#include <windows.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef long fsblkcnt_t;
typedef long fsfilcnt_t;

struct statfs {
	unsigned long f_type;
	unsigned long f_bsize;
	unsigned long f_frsize;
	fsblkcnt_t    f_blocks;
	fsblkcnt_t    f_bfree;
	fsblkcnt_t    f_bavail;
	fsfilcnt_t    f_files;
	fsfilcnt_t    f_ffree;
	unsigned long f_fsid;
	unsigned long f_flags;
	unsigned long f_namelen;
	unsigned long f_spare[4];
};

/**
 * @brief Return filesystem capacity information for @p path.
 *
 * Backed by GetDiskFreeSpaceExA and shaped like POSIX statfs().
 */
int statfs(const char *path, struct statfs *buf);

#ifdef __cplusplus
}
#endif
