/****************************************************************************
 * include/sys/wait.h
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_WAIT_H
#define __INCLUDE_SYS_WAIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <signal.h>

#ifdef CONFIG_SCHED_WAITPID

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* The following are provided for analysis of returned status values.
 * Encoded is as follows as 2 bytes of info(MS) then two bytes of code (LS).
 * Code:
 *   0 - Child has exited, info is the exit code.
 *   Other values - Not implemented
 */

#define WEXITSTATUS(s)  (((s) >> 8) & 0xff)/* Return exit status */

#define WIFEXITED(s)    (((s) & 0xff) == 0) /* True: Child exited normally */
#define WIFCONTINUED(s) (false)  /* True: Child has been continued */
#define WIFSIGNALED(s)  (false)  /* True: Child exited due to uncaught signal */
#define WIFSTOPPED(s)   (false)  /* True: Child is currently stopped */
#define WSTOPSIG(s)     (false)  /* Return signal number that caused process to stop */
#define WTERMSIG(s)     (false)  /* Return signal number that caused process to terminate */

/* The following symbolic constants are possible values for the options
 * argument to waitpid() (1) and/or waitid() (2),
 */

#define WCONTINUED      (1 << 0) /* Status for child that has been continued (1)(2) */
#define WNOHANG         (1 << 1) /* Do not wait if status not available (1) */
#define WUNTRACED       (1 << 2) /* Report status of stopped child process (1) */

#define WEXITED         (1 << 3) /* Wait for processes that have exited (2) */
#define WSTOPPED        (1 << 4) /* Status for child stopped on signal (2) */
#define WNOHANG         (1 << 5) /* Return immediately if there are no children (2) */
#define WNOWAIT         (1 << 6) /* Keep the process in a waitable state (2) */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

enum idtype_e
{
  P_PID = 1,
  P_GID = 2,
  P_ALL = 3
};
typedef enum idtype_e idtype_t;

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

EXTERN pid_t wait(FAR int *stat_loc);
EXTERN int   waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options);
EXTERN pid_t waitpid(pid_t pid, FAR int *stat_loc, int options);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SCHED_WAITPID */
#endif /* __INCLUDE_SYS_WAIT_H */
