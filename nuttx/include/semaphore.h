/****************************************************************************
 * include/semaphore.h
 *
 *   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SEMAPHORE_H
#define __INCLUDE_SEMAPHORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <limits.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* This structure contains information about the holder of a semaphore */

#ifdef CONFIG_PRIORITY_INHERITANCE
struct semholder_s
{
#if CONFIG_SEM_PREALLOCHOLDERS > 0
  struct semholder_s *flink;     /* Implements singly linked list */
#endif
  void *htcb;                    /* Holder TCB (actual type is _TCB) */
  int16_t counts;                /* Number of counts owned by this holder */
};

#if CONFIG_SEM_PREALLOCHOLDERS > 0
#  define SEMHOLDER_INITIALIZER {NULL, NULL, 0}
#else
#  define SEMHOLDER_INITIALIZER {NULL, 0}
#endif
#endif /* CONFIG_PRIORITY_INHERITANCE */

/* This is the generic semaphore structure. */

struct sem_s
{
  int16_t semcount;              /* >0 -> Num counts available */
                                 /* <0 -> Num tasks waiting for semaphore */
  /* If priority inheritance is enabled, then we have to keep track of which
   * tasks hold references to the semaphore.
   */

#ifdef CONFIG_PRIORITY_INHERITANCE
# if CONFIG_SEM_PREALLOCHOLDERS > 0
  FAR struct semholder_s *hhead; /* List of holders of semaphore counts */
# else
  struct semholder_s holder;     /* Single holder */
# endif
#endif
};

typedef struct sem_s sem_t;

/* Initializers */

#ifdef CONFIG_PRIORITY_INHERITANCE
# if CONFIG_SEM_PREALLOCHOLDERS > 0
#  define SEM_INITIALIZER(c) {(c), NULL}  /* semcount, hhead */
# else
#  define SEM_INITIALIZER(c) {(c), SEMHOLDER_INITIALIZER} /* semcount, holder */
# endif
#else
#  define SEM_INITIALIZER(c) {(c)} /* semcount */
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Forward references needed by some prototypes */

struct timespec; /* Defined in time.h */

/* Counting Semaphore Interfaces (based on POSIX APIs) */

EXTERN int        sem_init(FAR sem_t *sem, int pshared, unsigned int value);
EXTERN int        sem_destroy(FAR sem_t *sem);
EXTERN FAR sem_t *sem_open(FAR const char *name, int oflag, ...);
EXTERN int        sem_close(FAR sem_t *sem);
EXTERN int        sem_unlink(FAR const char *name);
EXTERN int        sem_wait(FAR sem_t *sem);
EXTERN int        sem_timedwait(FAR sem_t *sem,
                                FAR const struct timespec *abstime);
EXTERN int        sem_trywait(FAR sem_t *sem);
EXTERN int        sem_post(FAR sem_t *sem);
EXTERN int        sem_getvalue(FAR sem_t *sem, FAR int *sval);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_SEMAPHORE_H */
