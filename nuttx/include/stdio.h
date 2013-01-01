/****************************************************************************
 * include/stdio.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_STDIO_H
#define __INCLUDE_STDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdarg.h>
#include <sched.h>
#include <semaphore.h>
#include <time.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* File System Definitions **************************************************/

/* File system error values *************************************************/

#define EOF        (-1)

/* The first three _iob entries are reserved for standard I/O */

#define stdin  (&sched_getstreams()->sl_streams[0])
#define stdout (&sched_getstreams()->sl_streams[1])
#define stderr (&sched_getstreams()->sl_streams[2])

/* These APIs are not implemented and/or can be synthesized from
 * supported APIs.
 */

#define putc(c,s)  fputc((c),(s))
#define putchar(c) fputc(c, stdout)
#define getc(s)    fgetc(s)
#define getchar()  fgetc(stdin)
#define rewind(s)  ((void)fseek((s),0,SEEK_SET))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Streams */

typedef struct file_struct FILE;

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

/* ANSI-like File System Interfaces */

/* Operations on streams (FILE) */

EXTERN void   clearerr(register FILE *stream);
EXTERN int    fclose(FAR FILE *stream);
EXTERN int    fflush(FAR FILE *stream);
EXTERN int    feof(FAR FILE *stream);
EXTERN int    ferror(FAR FILE *stream);
EXTERN int    fileno(FAR FILE *stream);
EXTERN int    fgetc(FAR FILE *stream);
EXTERN int    fgetpos(FAR FILE *stream, FAR fpos_t *pos);
EXTERN char  *fgets(FAR char *s, int n, FAR FILE *stream);
EXTERN FAR FILE *fopen(FAR const char *path, FAR const char *type);
EXTERN int    fprintf(FAR FILE *stream, FAR const char *format, ...);
EXTERN int    fputc(int c, FAR FILE *stream);
EXTERN int    fputs(FAR const char *s, FAR FILE *stream);
EXTERN size_t fread(FAR void *ptr, size_t size, size_t n_items, FAR FILE *stream);
EXTERN int    fseek(FAR FILE *stream, long int offset, int whence);
EXTERN int    fsetpos(FAR FILE *stream, FAR fpos_t *pos);
EXTERN long   ftell(FAR FILE *stream);
EXTERN size_t fwrite(FAR const void *ptr, size_t size, size_t n_items, FAR FILE *stream);
EXTERN FAR char *gets(FAR char *s);
EXTERN int    ungetc(int c, FAR FILE *stream);

/* Operations on the stdout stream, buffers, paths, and the whole printf-family */

EXTERN int    printf(const char *format, ...);
EXTERN int    puts(FAR const char *s);
EXTERN int    rename(FAR const char *oldpath, FAR const char *newpath);
EXTERN int    sprintf(FAR char *buf, const char *format, ...);
EXTERN int    asprintf (FAR char **ptr, const char *fmt, ...);
EXTERN int    snprintf(FAR char *buf, size_t size, const char *format, ...);
EXTERN int    sscanf(const char *buf, const char *fmt, ...);
EXTERN void   perror(FAR const char *s);

EXTERN int    vprintf(FAR const char *format, va_list ap);
EXTERN int    vfprintf(FAR FILE *stream, const char *format, va_list ap);
EXTERN int    vsprintf(FAR char *buf, const char *format, va_list ap);
EXTERN int    avsprintf(FAR char **ptr, const char *fmt, va_list ap);
EXTERN int    vsnprintf(FAR char *buf, size_t size, const char *format, va_list ap);
EXTERN int    vsscanf(char *buf, const char *s, va_list ap);

/* POSIX-like File System Interfaces */

EXTERN FAR FILE *fdopen(int fd, FAR const char *type);
EXTERN int    statfs(FAR const char *path, FAR struct statfs *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_STDIO_H */
