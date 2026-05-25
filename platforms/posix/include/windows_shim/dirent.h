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
 * @file dirent.h
 *
 * MinGW ships a <dirent.h> wrapper over FindFirstFile, but its `struct
 * dirent` is a subset of POSIX: no `d_type`, no DT_REG/DT_DIR. PX4
 * (mavlink_ftp, logger) relies on `d_type` to tell files apart from
 * directories at iteration time without a second stat().
 *
 * Replace MinGW's header entirely with a POSIX-shaped wrapper that
 * populates d_type from the Win32 attributes already carried by
 * WIN32_FIND_DATA.
 */
#pragma once

#include <windows.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* This header is consumed from both C and C++ translation units. Provide a
 * fall-back macro so the inline shim bodies below can spell `nullptr` while
 * remaining valid in C, where `nullptr` is reserved before C23. */
#if !defined(__cplusplus) && !defined(nullptr)
#  define nullptr NULL
#endif

#ifndef NAME_MAX
#define NAME_MAX 260
#endif

/* POSIX d_type values. We only populate DT_DIR / DT_REG; anything the
 * Win32 attributes don't distinguish falls through to DT_UNKNOWN. */
#define DT_UNKNOWN 0
#define DT_FIFO    1
#define DT_CHR     2
#define DT_DIR     4
#define DT_BLK     6
#define DT_REG     8
#define DT_LNK     10
#define DT_SOCK    12

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Directory entry returned by readdir().
 *
 * The fields match the POSIX/BSD layout expected by PX4 callers. d_type is
 * derived from WIN32_FIND_DATA attributes so code can distinguish regular
 * files, directories, and reparse points without an extra stat() call.
 */
struct dirent {
	unsigned long  d_ino;
	long           d_off;
	unsigned short d_reclen;
	unsigned short d_namlen;
	unsigned char  d_type;
	char           d_name[NAME_MAX + 1];
};

#define d_fileno d_ino

/**
 * @brief Opaque directory stream backed by FindFirstFileA/FindNextFileA.
 */
typedef struct px4_dir_shim {
	HANDLE           handle;
	WIN32_FIND_DATAA find_data;
	struct dirent    entry;
	int              first;
	long             offset;
	char             pattern[MAX_PATH];
} DIR;

/**
 * @brief Open a directory stream.
 *
 * @param name Directory path.
 * @return Directory stream, or NULL with errno set.
 */
static inline DIR *opendir(const char *name)
{
	if (!name) { errno = EINVAL; return nullptr; }

	DIR *d = (DIR *)calloc(1, sizeof(DIR));

	if (!d) { errno = ENOMEM; return nullptr; }

	/* Translate "foo" -> "foo\*" so FindFirstFile enumerates the
	 * directory's contents rather than matching the directory itself. */
	int n = snprintf(d->pattern, sizeof(d->pattern), "%s\\*", name);

	if (n <= 0 || (size_t)n >= sizeof(d->pattern)) {
		free(d);
		errno = ENAMETOOLONG;
		return nullptr;
	}

	d->handle = FindFirstFileA(d->pattern, &d->find_data);

	if (d->handle == INVALID_HANDLE_VALUE) {
		free(d);
		errno = ENOENT;
		return nullptr;
	}

	d->first = 1;
	d->offset = 0;
	return d;
}

/**
 * @brief Return the next directory entry.
 *
 * @return Pointer owned by @p d and overwritten by the next readdir() call, or
 *         NULL at end of directory or on error.
 */
static inline struct dirent *readdir(DIR *d)
{
	if (!d) { errno = EBADF; return nullptr; }

	if (d->first) {
		d->first = 0;

	} else if (!FindNextFileA(d->handle, &d->find_data)) {
		return nullptr; /* end of directory */
	}

	strncpy(d->entry.d_name, d->find_data.cFileName, NAME_MAX);
	d->entry.d_name[NAME_MAX] = '\0';
	d->entry.d_off = d->offset++;
	d->entry.d_reclen = (unsigned short)strlen(d->entry.d_name);
	d->entry.d_namlen = d->entry.d_reclen;

	if (d->find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
		d->entry.d_type = DT_DIR;

	} else if (d->find_data.dwFileAttributes & FILE_ATTRIBUTE_REPARSE_POINT) {
		d->entry.d_type = DT_LNK;

	} else {
		d->entry.d_type = DT_REG;
	}

	return &d->entry;
}

/** @brief Close a directory stream and free its resources. */
static inline int closedir(DIR *d)
{
	if (!d) { errno = EBADF; return -1; }

	if (d->handle != INVALID_HANDLE_VALUE) {
		FindClose(d->handle);
	}

	free(d);
	return 0;
}

/** @brief Rewind a directory stream to the first entry. */
static inline void rewinddir(DIR *d)
{
	if (!d) { return; }

	if (d->handle != INVALID_HANDLE_VALUE) {
		FindClose(d->handle);
	}

	d->handle = FindFirstFileA(d->pattern, &d->find_data);
	d->first = (d->handle != INVALID_HANDLE_VALUE) ? 1 : 0;
	d->offset = 0;
}

/** @brief Return the current logical directory offset. */
static inline long telldir(DIR *d)
{
	if (!d) {
		errno = EBADF;
		return -1;
	}

	return d->offset;
}

/** @brief Seek to a logical offset previously returned by telldir(). */
static inline void seekdir(DIR *d, long loc)
{
	if (!d || loc < 0) {
		return;
	}

	rewinddir(d);

	while (d->offset < loc) {
		if (!readdir(d)) {
			break;
		}
	}
}

/** @brief Sort directory entries lexicographically by d_name. */
static inline int alphasort(const struct dirent **a, const struct dirent **b)
{
	return strcoll((*a)->d_name, (*b)->d_name);
}

/** @brief Version-aware sort placeholder; currently identical to alphasort(). */
static inline int versionsort(const struct dirent **a, const struct dirent **b)
{
	return alphasort(a, b);
}

/**
 * @brief Read a directory into an allocated, optionally filtered/sorted list.
 *
 * The caller owns the returned array and each entry in it, matching POSIX
 * scandir() ownership rules.
 *
 * @return Number of entries on success, or -1 with errno set.
 */
static inline int scandir(const char *dirp, struct dirent ***namelist,
			  int (*select)(const struct dirent *),
			  int (*compar)(const struct dirent **, const struct dirent **))
{
	if (!namelist) {
		errno = EINVAL;
		return -1;
	}

	*namelist = nullptr;

	DIR *dir = opendir(dirp);

	if (!dir) {
		return -1;
	}

	size_t count = 0;
	size_t capacity = 0;
	struct dirent **list = nullptr;
	struct dirent *entry = nullptr;

	while ((entry = readdir(dir)) != nullptr) {
		if (select && !select(entry)) {
			continue;
		}

		if (count == capacity) {
			size_t next_capacity = capacity ? capacity * 2 : 16;
			struct dirent **next = (struct dirent **)realloc(list, next_capacity * sizeof(struct dirent *));

			if (!next) {
				for (size_t i = 0; i < count; ++i) {
					free(list[i]);
				}

				free(list);
				closedir(dir);
				errno = ENOMEM;
				return -1;
			}

			list = next;
			capacity = next_capacity;
		}

		struct dirent *copy = (struct dirent *)malloc(sizeof(*copy));

		if (!copy) {
			for (size_t i = 0; i < count; ++i) {
				free(list[i]);
			}

			free(list);
			closedir(dir);
			errno = ENOMEM;
			return -1;
		}

		*copy = *entry;
		list[count++] = copy;
	}

	closedir(dir);

	if (compar && count > 1) {
		qsort(list, count, sizeof(struct dirent *), (int (*)(const void *, const void *))compar);
	}

	*namelist = list;
	return (int)count;
}

#ifdef __cplusplus
}
#endif
