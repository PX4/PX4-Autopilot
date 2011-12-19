/****************************************************************************
 * include/sys/stat.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __SYS_STAT_H
#define __SYS_STAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <time.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* mode_t bit settings (most of these do not apply to Nuttx).
 * This assumes that the full size of a mode_t is 16-bits.
 * (However, mode_t must be size 'int' because it is promoted
 * to size int when passed in varargs).
 */

#define S_IXOTH     0000001 /* Permissions for others: RWX */
#define S_IWOTH     0000002
#define S_IROTH     0000004
#define S_IRWXO     0000007

#define S_IXGRP     0000010 /* Group permissions: RWX */
#define S_IWGRP     0000020
#define S_IRGRP     0000040
#define S_IRWXG     0000070

#define S_IXUSR     0000100 /* Owner permissions: RWX */
#define S_IWUSR     0000200
#define S_IRUSR     0000400
#define S_IRWXU     0000700

#define S_ISVTX     0001000 /* "sticky" bit */
#define S_ISGID     0002000 /* Set group ID bit */
#define S_ISUID     0004000 /* Set UID bit */

#define S_IFIFO     0010000 /* File type bites */
#define S_IFCHR     0020000
#define S_IFDIR     0040000
#define S_IFBLK     0060000
#define S_IFREG     0100000
#define S_IFLNK     0120000
#define S_IFSOCK    0140000
#define S_IFMT      0170000

/* File type macros that operate on an instance of mode_t */

#define S_ISLNK(m)  (((m) & S_IFMT) == S_IFLNK)
#define S_ISREG(m)  (((m) & S_IFMT) == S_IFREG)
#define S_ISDIR(m)  (((m) & S_IFMT) == S_IFDIR)
#define S_ISCHR(m)  (((m) & S_IFMT) == S_IFCHR)
#define S_ISBLK(m)  (((m) & S_IFMT) == S_IFBLK)
#define S_ISFIFO(m) (((m) & S_IFMT) == S_IFIFO)
#define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* This is the simplified struct stat as returned by fstat().
 * This structure provides information about a specific file
 * or directory in the file system.
 */

struct stat
{
  mode_t    st_mode;    /* File type, atributes, and access mode bits */
  off_t     st_size;    /* Size of file/directory, in bytes */
  blksize_t st_blksize; /* Blocksize used for filesystem I/O */
  blkcnt_t  st_blocks;  /* Number of blocks allocated */
  time_t    st_atime;   /* Time of last access */
  time_t    st_mtime;   /* Time of last modification */
  time_t    st_ctime;   /* Time of last status change */
};

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN int mkdir(FAR const char *pathname, mode_t mode);
EXTERN int mkfifo(FAR const char *pathname, mode_t mode);
EXTERN int stat(const char *path, FAR struct stat *buf);
EXTERN int fstat(int fd, FAR struct stat *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __SYS_STAT_H */
