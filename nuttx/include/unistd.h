/****************************************************************************
 * include/unistd.h
 *
 *   Copyright (C) 2007-2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_UNISTD_H
#define __INCLUDE_UNISTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of functions that may be registerd to be called
 * at program exit.
 */

#define ATEXIT_MAX 1

/* Values for seeking */

#define SEEK_SET    0  /* From the start of the file */
#define SEEK_CUR    1  /* From the current file offset */
#define SEEK_END    2  /* From the end of the file */

/* Bit values for the second argument to access */

#define F_OK        0  /* Test existence */
#define R_OK        1  /* Test read permission */
#define W_OK        2  /* Test write permission */
#define X_OK        4  /* Test execute permission */

/* POSIX feature set macros */

#define  POSIX_VERSION
#undef  _POSIX_SAVED_IDS
#undef  _POSIX_JOB_CONTROL
#define _POSIX_REALTIME_SIGNALS 1
#define _POSIX_MESSAGE_PASSING 1
#undef  _POSIX_MAPPED_FILES
#undef  _POSIX_SHARED_MEMORY_OBJECTS
#define _POSIX_PRIORITY_SCHEDULING 1
#define _POSIX_TIMERS
#undef  _POSIX_MEMLOCK
#undef  _POSIX_MEMLOCK_RANGE
#undef  _POSIX_FSYNC
#define _POSIX_SYNCHRONIZED_IO
#undef  _POSIX_ASYNCHRONOUS_IO
#undef  _POSIX_PRIORITIZED_IO

/* Execution time constants (not supported) */

#undef  _POSIX_CHOWN_RESTRICTED
#undef  _POSIX_NO_TRUNC
#undef  _POSIX_VDISABLE

#define _POSIX_SYNC_IO
#undef  _POSIX_ASYNC_IO
#undef  _POSIX_PRIO_IO

#define fdatasync(f) fsync(f)

/****************************************************************************
 * Global Variables
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Used by getopt (obviously NOT thread safe!).  These variables cannot be
 * accessed directly by an external NXFLAT module.  In that case, accessor
 * functions must be used.
 */

#ifndef __NXFLAT__
EXTERN FAR char *optarg; /* Optional argument following option */
EXTERN int       optind; /* Index into argv */
EXTERN int       optopt; /* unrecognized option character */
#else
#  define optarg  (*(getoptargp()))
#  define optind  (*(getopindgp()))
#  define optopt  (*(getoptoptp()))
#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/* Task Control Interfaces */

EXTERN pid_t   vfork(void);
EXTERN pid_t   getpid(void);
EXTERN void    _exit(int status) noreturn_function;
EXTERN unsigned int sleep(unsigned int seconds);
EXTERN int     usleep(useconds_t usec);
EXTERN int     pause(void);

/* File descriptor operations */

EXTERN int     close(int fd);
EXTERN int     dup(int fd);
EXTERN int     dup2(int fd1, int fd2);
EXTERN int     fsync(int fd);
EXTERN off_t   lseek(int fd, off_t offset, int whence);
EXTERN ssize_t read(int fd, FAR void *buf, size_t nbytes);
EXTERN ssize_t write(int fd, FAR const void *buf, size_t nbytes);

/* Special devices */

EXTERN int     pipe(int filedes[2]);

/* Working directory operations */

EXTERN int     chdir(FAR const char *path);
EXTERN FAR char *getcwd(FAR char *buf, size_t size);

/* File path operations */

EXTERN int     unlink(FAR const char *pathname);
EXTERN int     rmdir(FAR const char *pathname);

/* Execution of programs from files */

#ifdef CONFIG_LIBC_EXECFUNCS
EXTERN int     execl(FAR const char *path, ...);
EXTERN int     execv(FAR const char *path, FAR char *const argv[]);

/* Non-standard functions to manage symbol tables */

struct symtab_s; /* See include/nuttx/binfmt/symtab.h */
EXTERN void exec_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols);
EXTERN void exec_setsymtab(FAR const struct symtab_s *symtab, int nsymbols);
#endif

/* Other */

EXTERN int     getopt(int argc, FAR char *const argv[], FAR const char *optstring);

/* Accessor functions intended for use only by external NXFLAT
 * modules.  The global variables optarg, optind, and optopt cannot
 * be referenced directly from external modules.
 */

EXTERN FAR char **getoptargp(void); /* Optional argument following option */
EXTERN int       *getopindgp(void); /* Index into argv */
EXTERN int       *getoptoptp(void); /* unrecognized option character */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_UNISTD_H */
