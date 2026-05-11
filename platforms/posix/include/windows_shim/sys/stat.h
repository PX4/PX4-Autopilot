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
 * @file sys/stat.h
 *
 * MinGW's sys/stat.h provides POSIX's 1-argument `mkdir(path)` (forwarded
 * to _mkdir). PX4 - along with almost every POSIX codebase - uses the
 * 2-argument form `mkdir(path, mode)`. Wrap the real call in an inline
 * that ignores the mode bits (Windows ACLs don't map to POSIX permission
 * bits anyway).
 */
#pragma once

#if defined(_MSC_VER)
#if defined(__has_include)
#  if __has_include(<../ucrt/sys/stat.h>)
#    include <../ucrt/sys/stat.h>
#  endif
#endif
#include <sys/types.h>
#else
#include_next <sys/stat.h>
#endif
#include <direct.h>
#include <sys/types.h>

#ifndef _PX4_MKDIR_SHIM_DEFINED
#define _PX4_MKDIR_SHIM_DEFINED

/* Replace any existing mkdir macro/prototype with a 2-arg form that
 * ignores the mode argument and falls back to _mkdir internally. */
#ifdef mkdir
#undef mkdir
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Two-argument POSIX mkdir() wrapper around Windows _mkdir(). */
static inline int px4_mkdir_shim(const char *path, mode_t mode)
{
	(void)mode;
	return _mkdir(path);
}

#ifdef __cplusplus
}
#endif

#define mkdir(path, mode) px4_mkdir_shim((path), (mode))

#endif /* _PX4_MKDIR_SHIM_DEFINED */

/* POSIX permission bits that Windows's sys/stat.h partially ships. Fill
 * in the rest with matching numeric values so code that OR's them
 * compiles. */
#ifndef S_IRWXU
#define S_IRWXU 0700
#endif
#ifndef S_IFMT
#define S_IFMT 0170000
#endif
#ifndef S_IFIFO
#define S_IFIFO 0010000
#endif
#ifndef S_IFCHR
#define S_IFCHR 0020000
#endif
#ifndef S_IFDIR
#define S_IFDIR 0040000
#endif
#ifndef S_IFBLK
#define S_IFBLK 0060000
#endif
#ifndef S_IFREG
#define S_IFREG 0100000
#endif
#ifndef S_IFLNK
#define S_IFLNK 0120000
#endif
#ifndef S_IFSOCK
#define S_IFSOCK 0140000
#endif
#ifndef S_IRUSR
#define S_IRUSR 0400
#endif
#ifndef S_IWUSR
#define S_IWUSR 0200
#endif
#ifndef S_IXUSR
#define S_IXUSR 0100
#endif
#ifndef S_IRWXG
#define S_IRWXG 0070
#endif
#ifndef S_IRGRP
#define S_IRGRP 0040
#endif
#ifndef S_IWGRP
#define S_IWGRP 0020
#endif
#ifndef S_IXGRP
#define S_IXGRP 0010
#endif
#ifndef S_IRWXO
#define S_IRWXO 0007
#endif
#ifndef S_IROTH
#define S_IROTH 0004
#endif
#ifndef S_IWOTH
#define S_IWOTH 0002
#endif
#ifndef S_IXOTH
#define S_IXOTH 0001
#endif

/* MinGW's sys/stat.h has S_ISDIR / S_ISREG but not S_ISLNK. Windows
 * tracks symlinks through FILE_ATTRIBUTE_REPARSE_POINT which isn't
 * exposed in st_mode, so S_ISLNK always reports "not a symlink" - an
 * acceptable simplification for PX4's symlink recreation path, which
 * already handles the fall-through branch. */
#ifndef S_ISLNK
#define S_ISLNK(m) (0)
#endif
#ifndef S_ISDIR
#define S_ISDIR(m) (((m) & S_IFMT) == S_IFDIR)
#endif
#ifndef S_ISREG
#define S_ISREG(m) (((m) & S_IFMT) == S_IFREG)
#endif
#ifndef S_ISFIFO
#define S_ISFIFO(m) (((m) & S_IFMT) == S_IFIFO)
#endif
#ifndef S_ISCHR
#define S_ISCHR(m) (((m) & S_IFMT) == S_IFCHR)
#endif
#ifndef S_ISBLK
#define S_ISBLK(m) (((m) & S_IFMT) == S_IFBLK)
#endif
#ifndef S_ISSOCK
#define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)
#endif

#ifndef _PX4_LSTAT_SHIM_DEFINED
#define _PX4_LSTAT_SHIM_DEFINED
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief lstat() compatibility shim.
 *
 * Windows CRT stat() does not expose symlink metadata in st_mode, so lstat()
 * intentionally follows the same path as stat(). S_ISLNK() above therefore
 * remains false and callers take their normal non-symlink branch.
 */
static inline int lstat(const char *path, struct stat *buf)
{
	/* Windows has no per-link stat. Fall through to stat(); that's
	 * accurate enough because S_ISLNK above always returns 0, so
	 * callers fall into the non-symlink branch anyway. */
	return stat(path, buf);
}
#ifdef __cplusplus
}
#endif
#endif
