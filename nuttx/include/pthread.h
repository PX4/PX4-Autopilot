/********************************************************************************
 * include/pthread.h
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

#ifndef __INCLUDE_PTHREAD_H
#define __INCLUDE_PTHREAD_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>   /* Default settings */
#include <nuttx/compiler.h> /* Compiler settings, noreturn_function */

#include <sys/types.h>      /* Needed for general types */
#include <stdint.h>         /* C99 fixed width integer types */
#include <stdbool.h>        /* C99 boolean types */
#include <unistd.h>         /* For getpid */
#include <semaphore.h>      /* Needed for sem_t */
#include <signal.h>         /* Needed for sigset_t */
#include <time.h>           /* Needed for struct timespec */

/********************************************************************************
 * Compilation Switches
 ********************************************************************************/

/* Standard POSIX switches */

#ifndef _POSIX_THREADS
#define _POSIX_THREADS
#endif
#ifndef _POSIX_THREAD_ATTR_STACKSIZE
#define _POSIX_THREAD_ATTR_STACKSIZE
#endif

/********************************************************************************
 * Definitions
 ********************************************************************************/

/* Values for the process shared (pshared) attribute */

#define PTHREAD_PROCESS_PRIVATE       0
#define PTHREAD_PROCESS_SHARED        1

/* Values for the mutext type attribute:
 *
 * PTHREAD_MUTEX_NORMAL: This type of mutex does not detect deadlock. A thread
 *   attempting to relock this mutex without first unlocking it will deadlock.
 *   Attempting to unlock a mutex locked by a different thread results in undefined
 *   behavior. Attempting to unlock an unlocked mutex results in undefined behavior. 
 * PTHREAD_MUTEX_ERRORCHECK
 *   This type of mutex provides error checking. A thread attempting to relock this
 *   mutex without first unlocking it will return with an error. A thread attempting
 *   to unlock a mutex which another thread has locked will return with an error. A
 *   thread attempting to unlock an unlocked mutex will return with an error.
 * PTHREAD_MUTEX_RECURSIVE
 *   A thread attempting to relock this mutex without first unlocking it will succeed
 *   in locking the mutex. The relocking deadlock which can occur with mutexes of type
 *   PTHREAD_MUTEX_NORMAL cannot occur with this type of mutex. Multiple locks of this
 *   mutex require the same number of unlocks to release the mutex before another thread
 *   can acquire the mutex. A thread attempting to unlock a mutex which another thread
 *   has locked will return with an error. A thread attempting to unlock an unlocked
 *   mutex will return with an error. 
 * PTHREAD_MUTEX_DEFAULT
 *  An implementation is allowed to map this mutex to one of the other mutex types.
 */

#ifdef CONFIG_MUTEX_TYPES
#  define PTHREAD_MUTEX_NORMAL        0
#  define PTHREAD_MUTEX_ERRORCHECK    1
#  define PTHREAD_MUTEX_RECURSIVE     2
#  define PTHREAD_MUTEX_DEFAULT       PTHREAD_MUTEX_NORMAL
#endif

/* Valid ranges for the pthread stacksize attribute */

#define PTHREAD_STACK_MIN             CONFIG_PTHREAD_STACK_MIN
#define PTHREAD_STACK_DEFAULT         CONFIG_PTHREAD_STACK_DEFAULT

/* Values for the pthread inheritsched attribute */

#define PTHREAD_INHERIT_SCHED         0
#define PTHREAD_EXPLICIT_SCHED        1

#define PTHREAD_PRIO_NONE             0
#define PTHREAD_PRIO_INHERIT          1
#define PTHREAD_PRIO_PROTECT          2

#define PTHREAD_DEFAULT_PRIORITY      100

/* Cancellation states returned by pthread_cancelstate() */

#define PTHREAD_CANCEL_ENABLE         (0)
#define PTHREAD_CANCEL_DISABLE        (1)

/* Thread return value when a pthread is canceled */

#define PTHREAD_CANCELED              ((FAR void*)ERROR)

/* Used to initialize a pthread_once_t */

#define PTHREAD_ONCE_INIT             (false)

/* This is returned by pthread_wait.  It must not match any errno in errno.h */

#define PTHREAD_BARRIER_SERIAL_THREAD 0x1000

/* Definitions to map some non-standard, BSD thread management interfaces to
 * the non-standard Linux-like prctl() interface.  Since these are simple
 * mappings to prctl, they will return 0 sucess and -1 on failure with the
 * err number in errno.  This is an inconsistency with out pthread interfaces.
 */

#define pthread_setname_np(thread, name) \
  prctl((int)PR_SET_NAME, (char*)name, (int)thread)
  
#define pthread_getname_np(thread, name) \
  prctl((int)PR_GET_NAME, (char*)name, (int)thread)

/********************************************************************************
 * Global Type Declarations
 ********************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* pthread-specific types */

typedef int             pthread_key_t;
typedef FAR void       *pthread_addr_t;
typedef pthread_addr_t  any_t;

typedef pthread_addr_t (*pthread_startroutine_t)(pthread_addr_t);
typedef pthread_startroutine_t  pthread_func_t;

struct pthread_attr_s
{
  size_t  stacksize;    /* Size of the stack allocated for the pthead */
  int16_t priority;     /* Priority of the pthread */
  uint8_t policy;       /* Pthread scheduler policy */
  uint8_t inheritsched; /* Inherit parent prio/policy? */
};
typedef struct pthread_attr_s pthread_attr_t;

typedef pid_t pthread_t;

typedef int pthread_condattr_t;

struct pthread_cond_s
{
  sem_t sem;
};
typedef struct pthread_cond_s pthread_cond_t;
#define PTHREAD_COND_INITIALIZER {{0, 0xffff}}

struct pthread_mutexattr_s
{
  uint8_t pshared;  /* PTHREAD_PROCESS_PRIVATE or PTHREAD_PROCESS_SHARED */
#ifdef CONFIG_MUTEX_TYPES
  uint8_t type;     /* Type of the mutex.  See PTHREAD_MUTEX_* definitions */
#endif
};
typedef struct pthread_mutexattr_s pthread_mutexattr_t;

struct pthread_mutex_s
{
  int   pid;      /* ID of the holder of the mutex */
  sem_t sem;      /* Semaphore underlying the implementation of the mutex */
#ifdef CONFIG_MUTEX_TYPES
  uint8_t type;   /* Type of the mutex.  See PTHREAD_MUTEX_* definitions */
  int   nlocks;   /* The number of recursive locks held */
#endif
};
typedef struct pthread_mutex_s pthread_mutex_t;

#ifdef CONFIG_MUTEX_TYPES
#  define PTHREAD_MUTEX_INITIALIZER {0, SEM_INITIALIZER(1), PTHREAD_MUTEX_DEFAULT, 0}
#else
#  define PTHREAD_MUTEX_INITIALIZER {0, SEM_INITIALIZER(1)}
#endif

struct pthread_barrierattr_s
{
  int pshared;
};
typedef struct pthread_barrierattr_s pthread_barrierattr_t;

struct pthread_barrier_s
{
  sem_t        sem;
  unsigned int count;
};
typedef struct pthread_barrier_s pthread_barrier_t;

typedef bool pthread_once_t;

/* Forware references */

struct sched_param; /* Defined in sched.h */

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Global Function Prototypes
 ********************************************************************************/

/* Initializes a thread attributes object (attr) with default values for all of
 * the individual attributes used by a given implementation.
 */

EXTERN int pthread_attr_init(FAR pthread_attr_t *attr);

/* An attributes object can be deleted when it is no longer needed. */

EXTERN int pthread_attr_destroy(pthread_attr_t *attr);

/* Set or obtain the default scheduling algorithm */

EXTERN int pthread_attr_setschedpolicy(FAR pthread_attr_t *attr, int policy);
EXTERN int pthread_attr_getschedpolicy(FAR pthread_attr_t *attr, int *policy);
EXTERN int pthread_attr_setschedparam(FAR pthread_attr_t *attr,
                                      FAR const struct sched_param *param);
EXTERN int pthread_attr_getschedparam(FAR pthread_attr_t *attr,
                                      FAR struct sched_param *param);
EXTERN int pthread_attr_setinheritsched(FAR pthread_attr_t *attr, int inheritsched);
EXTERN int pthread_attr_getinheritsched(FAR const pthread_attr_t *attr,
                                        FAR int *inheritsched);

/* Set or obtain the default stack size */

EXTERN int pthread_attr_setstacksize(FAR pthread_attr_t *attr, long stacksize);
EXTERN int pthread_attr_getstacksize(FAR pthread_attr_t *attr, long *stackaddr);

/* To create a thread object and runnable thread, a routine must be specified
 * as the new thread's start routine.  An argument may be passed to this
 * routine, as an untyped address; an untyped address may also be returned as
 * the routine's value.  An attributes object may be used to specify details
 * about the kind of thread being created.
 */

EXTERN int pthread_create(FAR pthread_t *thread, FAR pthread_attr_t *attr,
                          pthread_startroutine_t startroutine,
                          pthread_addr_t arg);

/* A thread object may be "detached" to specify that the return value and
 * completion status will not be requested.
 */

EXTERN int pthread_detach(pthread_t thread);

/* A thread may terminate it's own execution or the execution of another
 * thread.
 */

EXTERN void pthread_exit(pthread_addr_t value) noreturn_function;
EXTERN int  pthread_cancel(pthread_t thread);
EXTERN int  pthread_setcancelstate(int state, FAR int *oldstate);
EXTERN void pthread_testcancel(void);

/* A thread can await termination of another thread and retrieve the return
 * value of the thread.
 */

EXTERN int pthread_join(pthread_t thread, FAR pthread_addr_t *value);

/* A thread may tell the scheduler that its processor can be made available. */

EXTERN void pthread_yield(void);

/* A thread may obtain a copy of its own thread handle. */

#define pthread_self() ((pthread_t)getpid())

/* Compare two thread IDs. */

#define pthread_equal(t1,t2) (t1 == t2)

/* Thread scheduling parameters */

EXTERN int pthread_getschedparam(pthread_t thread, FAR int *policy,
                                 FAR struct sched_param *param);
EXTERN int pthread_setschedparam(pthread_t thread, int policy,
                                 FAR const struct sched_param *param);
EXTERN int pthread_setschedprio(pthread_t thread, int prio);

/* Thread-specific Data Interfaces */

EXTERN int pthread_key_create(FAR pthread_key_t *key,
                              CODE void (*destructor)(FAR void*));
EXTERN int pthread_setspecific(pthread_key_t key, FAR void *value);
EXTERN FAR void *pthread_getspecific(pthread_key_t key);
EXTERN int pthread_key_delete(pthread_key_t key);

/* Create, operate on, and destroy mutex attributes. */

EXTERN int pthread_mutexattr_init(FAR pthread_mutexattr_t *attr);
EXTERN int pthread_mutexattr_destroy(FAR pthread_mutexattr_t *attr);
EXTERN int pthread_mutexattr_getpshared(FAR pthread_mutexattr_t *attr, FAR int *pshared);
EXTERN int pthread_mutexattr_setpshared(FAR pthread_mutexattr_t *attr, int pshared);
#ifdef CONFIG_MUTEX_TYPES
EXTERN int pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *type);
EXTERN int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type);
#endif

/* The following routines create, delete, lock and unlock mutexes. */

EXTERN int pthread_mutex_init(FAR pthread_mutex_t *mutex, FAR pthread_mutexattr_t *attr);
EXTERN int pthread_mutex_destroy(FAR pthread_mutex_t *mutex);
EXTERN int pthread_mutex_lock(FAR pthread_mutex_t *mutex);
EXTERN int pthread_mutex_trylock(FAR pthread_mutex_t *mutex);
EXTERN int pthread_mutex_unlock(FAR pthread_mutex_t *mutex);

/* Operations on condition variables */

EXTERN int pthread_condattr_init(FAR pthread_condattr_t *attr);
EXTERN int pthread_condattr_destroy(FAR pthread_condattr_t *attr);

/* A thread can create and delete condition variables. */

EXTERN int pthread_cond_init(FAR pthread_cond_t *cond, FAR pthread_condattr_t *attr);
EXTERN int pthread_cond_destroy(FAR pthread_cond_t *cond);

/* A thread can signal to and broadcast on a condition variable. */

EXTERN int pthread_cond_broadcast(FAR pthread_cond_t *cond);
EXTERN int pthread_cond_signal(FAR pthread_cond_t *cond);

/* A thread can wait for a condition variable to be signalled or broadcast. */

EXTERN int pthread_cond_wait(FAR pthread_cond_t *cond, FAR pthread_mutex_t *mutex);

/* A thread can perform a timed wait on a condition variable. */

EXTERN int pthread_cond_timedwait(FAR pthread_cond_t *cond, FAR pthread_mutex_t *mutex,
                                  FAR const struct timespec *abstime);

/* Barrier attributes */

EXTERN int pthread_barrierattr_destroy(FAR pthread_barrierattr_t *attr);
EXTERN int pthread_barrierattr_init(FAR pthread_barrierattr_t *attr);
EXTERN int pthread_barrierattr_getpshared(FAR const pthread_barrierattr_t *attr,
                                          FAR int *pshared);
EXTERN int pthread_barrierattr_setpshared(FAR pthread_barrierattr_t *attr,
                                          int pshared);

/* Barriers */

EXTERN int pthread_barrier_destroy(FAR pthread_barrier_t *barrier);
EXTERN int pthread_barrier_init(FAR pthread_barrier_t *barrier,
                                FAR const pthread_barrierattr_t *attr,
                                unsigned int count);
EXTERN int pthread_barrier_wait(FAR pthread_barrier_t *barrier);

/* Pthread initialization */

EXTERN int pthread_once(FAR pthread_once_t *once_control,
                        CODE void (*init_routine)(void));

/* Pthread signal management APIs */

EXTERN int pthread_kill(pthread_t thread, int sig);
EXTERN int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_PTHREAD_H */

