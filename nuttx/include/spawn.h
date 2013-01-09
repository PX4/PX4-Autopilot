/****************************************************************************
 * include/spawn.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_LIBGEN_H
#define __INCLUDE_LIBGEN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* "The spawn.h header shall define the flags that may be set in a
 * posix_spawnattr_t object using the posix_spawnattr_setflags() function:"
 */

#define POSIX_SPAWN_RESETIDS      (1 << 0)
#define POSIX_SPAWN_SETPGROUP     (1 << 1)
#define POSIX_SPAWN_SETSCHEDPARAM (1 << 2)
#define POSIX_SPAWN_SETSCHEDULER  (1 << 3)
#define POSIX_SPAWN_SETSIGDEF     (1 << 4)
#define POSIX_SPAWN_SETSIGMASK    (1 << 5)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/
/* "The spawn.h header shall define the posix_spawnattr_t and
 * posix_spawn_file_actions_t types used in performing spawn operations.
 */

struct posix_spawnattr_s
{
/* To be provided */
};
typedef struct posix_spawnattr_s posix_spawnattr_t;

struct posix_spawn_file_actions_s
{
/* To be provided */
};
typedef struct posix_spawn_file_actions_s posix_spawn_file_actions_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* "The following shall be declared as functions and may also be defined as
 * macros. Function prototypes shall be provided."
 */

#ifdef __cplusplus
extern "C"
{
#endif

int posix_spawn(FAR pid_t *, FAR const char *,
      FAR const posix_spawn_file_actions_t *, FAR const posix_spawnattr_t *,
      FAR char *const [], FAR char *const []);
int posix_spawnp(FAR pid_t *, FAR const char *,
      FAR const posix_spawn_file_actions_t *, FAR const posix_spawnattr_t *,
      FAR char *const [], FAR char *const []);

int posix_spawn_file_actions_init(FAR posix_spawn_file_actions_t *);
int posix_spawn_file_actions_destroy(FAR posix_spawn_file_actions_t *);

int posix_spawn_file_actions_addclose(FAR posix_spawn_file_actions_t *,
      int);
int posix_spawn_file_actions_adddup2(FAR posix_spawn_file_actions_t *,
      int, int);
int posix_spawn_file_actions_addopen(FAR posix_spawn_file_actions_t *,
      int, FAR const char *, int, mode_t);

int posix_spawnattr_init(FAR posix_spawnattr_t *);
int posix_spawnattr_destroy(FAR posix_spawnattr_t *);

int posix_spawnattr_getflags(FAR const posix_spawnattr_t *, FAR short *);
int posix_spawnattr_getpgroup(FAR const posix_spawnattr_t *, FAR pid_t *);
int posix_spawnattr_getschedparam(FAR const posix_spawnattr_t *,
      FAR struct sched_param *);
int posix_spawnattr_getschedpolicy(FAR const posix_spawnattr_t *,
      FAR int *);
int posix_spawnattr_getsigdefault(FAR const posix_spawnattr_t *,
      FAR sigset_t *);
int posix_spawnattr_getsigmask(FAR const posix_spawnattr_t *,
      FAR sigset_t *);

int posix_spawnattr_setflags(FAR posix_spawnattr_t *, short);
int posix_spawnattr_setpgroup(FAR posix_spawnattr_t *, pid_t);
int posix_spawnattr_setschedparam(FAR posix_spawnattr_t *,
      FAR const struct sched_param *);
int posix_spawnattr_setschedpolicy(FAR posix_spawnattr_t *, int);
int posix_spawnattr_setsigdefault(FAR posix_spawnattr_t *,
      FAR const sigset_t *);
int posix_spawnattr_setsigmask(FAR posix_spawnattr_t *,
      FAR const sigset_t *);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_LIBGEN_H */
