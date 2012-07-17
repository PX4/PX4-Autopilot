/****************************************************************************
 * include/dirent.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_DIRENT_H
#define __INCLUDE_DIRENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* File type code for the d_type field in dirent struct.
 * Note that because of the simplified filesystem organization
 * of NuttX, an inode can be BOTH a file and a directory
 */

#define DTYPE_FILE      0x01
#define DTYPE_CHR       0x02
#define DTYPE_BLK       0x04
#define DTYPE_DIRECTORY 0x08

#define DIRENT_ISFILE(dtype)      (((dtype) & DTYPE_FILE) != 0 )
#define DIRENT_ISCHR(dtype)       (((dtype) & DTYPE_CHR) != 0 )
#define DIRENT_ISBLK(dtype)       (((dtype) & DTYPE_BLK) != 0 )
#define DIRENT_ISDIRECTORY(dtype) (((dtype) & DTYPE_DIRECTORY) != 0 )

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The POSIX specification requires that the caller of readdir_r provide
 * storage "large enough for a dirent with the d_name member and an array
 * of char containing at least {NAME_MAX} plus one elements.
 */

struct dirent
{
  uint8_t   d_type;             /* type of file */
  char      d_name[NAME_MAX+1]; /* filename */
};

typedef void DIR;

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* POSIX-like File System Interfaces */

EXTERN int        closedir(FAR DIR *dirp);
EXTERN FAR DIR   *opendir(FAR const char *path);
EXTERN FAR struct dirent *readdir(FAR DIR *dirp);
EXTERN int        readdir_r(FAR DIR *dirp, FAR struct dirent *entry,
                            FAR struct dirent **result);
EXTERN void       rewinddir(FAR DIR *dirp);
EXTERN void       seekdir(FAR DIR *dirp, off_t loc);
EXTERN off_t      telldir(FAR DIR *dirp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DIRENT_H */
