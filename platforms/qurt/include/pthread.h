/*
 * Copyright (c) 1993, 1994 by Chris Provenzano, proven@mit.edu
 * Copyright (c) 1995-1998 by John Birrell <jb@cimlogic.com.au>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *  This product includes software developed by Chris Provenzano.
 * 4. The name of Chris Provenzano may not be used to endorse or promote
 *	  products derived from this software without specific prior written
 *	  permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CHRIS PROVENZANO ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CHRIS PROVENZANO BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#pragma once

/*
 * Header files.
 */
#include <sys/cdefs.h>
#include <limits.h>
#include "dspal_types.h"
#include <stdint.h>
#include <sys/_sigset.h>
#include <sched.h>
#include "dspal_time.h"

/*
 * Run-time invariant values:
 */
#define PTHREAD_DESTRUCTOR_ITERATIONS	4
#define PTHREAD_KEYS_MAX		256
#define PTHREAD_STACK_MIN		__MINSIGSTKSZ
#define PTHREAD_THREADS_MAX		__ULONG_MAX
#define PTHREAD_BARRIER_SERIAL_THREAD	-1

/*
 * Flags for threads and thread attributes.
 */
#define PTHREAD_DETACHED            0x1
#define PTHREAD_SCOPE_SYSTEM        0x2
#define PTHREAD_INHERIT_SCHED       0x4
#define PTHREAD_NOFLOAT             0x8

#define PTHREAD_CREATE_DETACHED     PTHREAD_DETACHED
#define PTHREAD_CREATE_JOINABLE     0
#define PTHREAD_SCOPE_PROCESS       0
#define PTHREAD_EXPLICIT_SCHED      0

/*
 * Flags for read/write lock attributes
 */
#define PTHREAD_PROCESS_PRIVATE     0
#define PTHREAD_PROCESS_SHARED      1

/*
 * Flags for cancelling threads
 */
#define PTHREAD_CANCEL_ENABLE		0
#define PTHREAD_CANCEL_DISABLE		1
#define PTHREAD_CANCEL_DEFERRED		0
#define PTHREAD_CANCEL_ASYNCHRONOUS	2
#define PTHREAD_CANCELED		((void *) 1)

/*
 * Flags for once initialization.
 */
#define PTHREAD_NEEDS_INIT  0
#define PTHREAD_DONE_INIT   1

/*
 * Static once initialization values.
 */
#define PTHREAD_ONCE_INIT   { PTHREAD_NEEDS_INIT, NULL }

/*
 * Static initialization values.
 */
#define PTHREAD_MUTEX_INITIALIZER	0xFFFFFFFF
#define PTHREAD_ADAPTIVE_MUTEX_INITIALIZER_NP	((pthread_mutex_t)1)
#define PTHREAD_COND_INITIALIZER	0xFFFFFFFF
#define PTHREAD_RWLOCK_INITIALIZER	0xFFFFFFFF

/*
 * Default attribute arguments (draft 4, deprecated).
 */
#ifndef PTHREAD_KERNEL
#define pthread_condattr_default    NULL
#define pthread_mutexattr_default   NULL
#define pthread_attr_default        NULL
#endif

#define PTHREAD_PRIO_NONE	0
#define PTHREAD_PRIO_INHERIT	1
#define PTHREAD_PRIO_PROTECT	2

/*
 * Mutex types (Single UNIX Specification, Version 2, 1997).
 *
 * Note that a mutex attribute with one of the following types:
 *
 *	PTHREAD_MUTEX_NORMAL
 *	PTHREAD_MUTEX_RECURSIVE
 *
 * will deviate from POSIX specified semantics.
 */

/* mutex type */
enum pthread_mutextype {
	PTHREAD_MUTEX_ERRORCHECK = 0, 	/* Default POSIX mutex */
	PTHREAD_MUTEX_NORMAL = 1, 	/* No error checking */
	PTHREAD_MUTEX_RECURSIVE = 2, 	/* Recursive mutex */
	PTHREAD_MUTEX_ADAPTIVE_NP = 4, 	/* Adaptive mutex, spins briefly before blocking on lock */
	PTHREAD_MUTEX_TYPE_MAX
};

#define PTHREAD_MUTEX_DEFAULT		PTHREAD_MUTEX_RECURSIVE

struct _pthread_cleanup_info {
	uintptr_t pthread_cleanup_pad[8];
};

/*
 * Functions implemented inline to the caller's code.
 */
static inline int pthread_equal(pthread_t t1, pthread_t t2)
{
	return t1 == t2;
}

static inline void pthread_yield(void)
{
	return;
}

__BEGIN_DECLS
/*
 * Qualcomm (not POSIX compliant) additions to pthread get/set attribute functions:
 */
int pthread_attr_setthreadname(pthread_attr_t *attr, const char *name);
int pthread_attr_getthreadname(const pthread_attr_t *attr, char *name, int size);
int pthread_attr_settimetestid(pthread_attr_t *attr, unsigned int tid);
int pthread_attr_gettimetestid(const pthread_attr_t *attr, unsigned int *tid);

int pthread_getattr_np(pthread_t thread, pthread_attr_t *restrict attr);

/*
 * POSIX compliant function prototype definitions:
 */

int pthread_create(pthread_t *, const pthread_attr_t *, void *(*)(void *),
		   void *);
void pthread_exit(void *);
int pthread_join(pthread_t, void **);
pthread_t pthread_self(void);
int pthread_cancel(pthread_t);
int pthread_kill(pthread_t thread, int sig);
int pthread_getschedparam(pthread_t pthread, int *restrict, struct sched_param *restrict);
int pthread_setschedparam(pthread_t, int, const struct sched_param *);
int pthread_attr_init(pthread_attr_t *);
int pthread_attr_destroy(pthread_attr_t *);
int pthread_attr_setschedparam(pthread_attr_t *restrict, const struct sched_param *restrict);
int pthread_attr_getschedparam(const pthread_attr_t *, struct sched_param *);
int pthread_attr_setstacksize(pthread_attr_t *, size_t);
int pthread_attr_getstacksize(const pthread_attr_t *, size_t *);
int pthread_attr_setstackaddr(pthread_attr_t *, void *);
int pthread_attr_getstackaddr(const pthread_attr_t *, void **);
int pthread_mutex_init(pthread_mutex_t *__mutex, const pthread_mutexattr_t *);
int pthread_mutex_lock(pthread_mutex_t *__mutex);
int pthread_mutex_unlock(pthread_mutex_t *__mutex);
int pthread_mutex_trylock(pthread_mutex_t *__mutex);
int pthread_mutex_destroy(pthread_mutex_t *__mutex);
int pthread_mutexattr_init(pthread_mutexattr_t *attr);
int pthread_mutexattr_destroy(pthread_mutexattr_t *);
int pthread_mutexattr_gettype(const pthread_mutexattr_t *restrict, int *restrict);
int pthread_mutexattr_settype(pthread_mutexattr_t *, int);
int pthread_mutexattr_getprotocol(const pthread_mutexattr_t *restrict, int *restrict);
int pthread_mutexattr_setprotocol(pthread_mutexattr_t *, int);
int pthread_mutexattr_getpshared(const pthread_mutexattr_t *restrict, int *restrict);
int pthread_mutexattr_setpshared(pthread_mutexattr_t *, int);
// Not Supported in DSPAL
//int pthread_spin_init(pthread_spinlock_t *__spin, int);
//int pthread_spin_destroy(pthread_spinlock_t *__spin);
//int pthread_spin_lock(pthread_spinlock_t *__spin);
//int pthread_spin_trylock(pthread_spinlock_t *__spin);
//int pthread_spin_unlock(pthread_spinlock_t *__spin);
int pthread_condattr_init(pthread_condattr_t *);
int pthread_condattr_destroy(pthread_condattr_t *);
int pthread_condattr_setpshared(pthread_condattr_t *, int);
int pthread_condattr_getpshared(const pthread_condattr_t *restrict, int *restrict);
int pthread_cond_init(pthread_cond_t *, const pthread_condattr_t *);
int pthread_cond_destroy(pthread_cond_t *);
int pthread_cond_signal(pthread_cond_t *);
int pthread_cond_broadcast(pthread_cond_t *);
int pthread_cond_wait(pthread_cond_t *, pthread_mutex_t *__mutex);
int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *time);
// Not supported in DSPAL
//int pthread_barrier_init(pthread_barrier_t *restrict barrier, const pthread_barrierattr_t *restrict attr, unsigned count);
//int pthread_barrier_destroy(pthread_barrier_t *barrier);
//int pthread_barrier_wait(pthread_barrier_t *barrier);
//int pthread_barrierattr_init(pthread_barrierattr_t *attr);
//int pthread_barrierattr_destroy(pthread_barrierattr_t *attr);
//int pthread_barrierattr_getpshared(const pthread_barrierattr_t *restrict attr, int *restrict pshared);
int pthread_key_create(pthread_key_t *, void (*)(void *));
int pthread_key_delete(pthread_key_t);
int pthread_setspecific(pthread_key_t, const void *);
void *pthread_getspecific(pthread_key_t);
int pthread_attr_setaffinity_np(pthread_attr_t *attr, size_t cpusetsize, const cpu_set_t *cpuset);
__END_DECLS
