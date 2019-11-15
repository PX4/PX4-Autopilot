/****************************************************************************
 * Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#pragma once

/*==========================================================================
 * FILE:         types.c
 *
 * SERVICES:     types usded in POSIX API interface
 *
 * DESCRIPTION:  POSIX API interface based upon POSIX 1003.1-2004
 *==========================================================================*/

#include <sys/_types.h>
#include <stddef.h>
#include <sys/cdefs.h>

__BEGIN_DECLS

#ifdef __GNUC__
#define restrict __restrict__
#else
#define restrict
#endif


#define PTHREAD_MUTEX_OPAQUE

typedef int   		pid_t;
typedef unsigned int	uid_t;

#define MAX_LEN_DEVICE_PATH_IN_BYTES 32

#define PTHREAD_MAX_THREADS          512

#define PTHREAD_NAME_LEN             16
#define PTHREAD_MIN_STACKSIZE        512 //4096
#define PTHREAD_MAX_STACKSIZE        1048576
#define PTHREAD_DEFAULT_STACKSIZE    1024

#define PTHREAD_MIN_PRIORITY         0
#define PTHREAD_MAX_PRIORITY         255
#define PTHREAD_DEFAULT_PRIORITY     20

typedef signed int   ssize_t;
#define _SSIZE_T

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE    0
#endif

typedef unsigned int cpu_set_t;

typedef unsigned int pthread_t;
typedef struct pthread_attr_t {
	void         *stackaddr;
	int          internal_stack; /* this flag==1 means the stack needs to be freed by posix */
	size_t       stacksize;
	int          priority;
	unsigned int timetest_id;
	cpu_set_t    cpumask;
	char         name[PTHREAD_NAME_LEN];
	/* This flag indicates whether pthread lib should create thread contexts for other OSALs */
	/* This is used internally by POSIX and not available for general usage */
	int          ext_context;
} pthread_attr_t;

#define PTHREAD_MUTEX_ATTR_UNINITIALIZED    0
#define PTHREAD_MUTEX_ATTR_INITIALIZED      1

#define PTHREAD_COND_ATTR_UNINITIALIZED     0
#define PTHREAD_COND_ATTR_INITIALIZED       1

#define PTHREAD_DEFAULT_NAME                "Anonymous"

#ifndef PTHREAD_MUTEX_OPAQUE
#define PTHREAD_MUTEX_INITIALIZER    {(qurt_mutex_t*)0, \
		qurt_rmutex_lock, \
		qurt_rmutex_unlock, \
		qurt_rmutex_try_lock, \
		{PTHREAD_MUTEX_ATTR_INITIALIZED, \
		 PTHREAD_MUTEX_RECURSIVE, \
		 PTHREAD_PROCESS_PRIVATE, \
		 PTHREAD_PRIO_INHERIT} \
	}
#endif

/* mutex and cond_var shared */
#define PTHREAD_PROCESS_PRIVATE      0
#define PTHREAD_PROCESS_SHARED       1

/* mutex protocol */
#define PTHREAD_PRIO_NONE            0
#define PTHREAD_PRIO_INHERIT         1
#define PTHREAD_PRIO_PROTECT         2

//mutex attr
typedef struct pthread_mutexattr_t   pthread_mutexattr_t;
struct pthread_mutexattr_t {
	int is_initialized;
	int type;
	int pshared;
	int protocol;
};

typedef void dspal_mutex_t;
typedef void dspal_cond_t;
typedef void dspal_barrier_t;

#ifdef PTHREAD_MUTEX_OPAQUE
typedef unsigned int pthread_mutex_t;
typedef struct _pthread_mutex_t _pthread_mutex_t;

struct _pthread_mutex_t {
	pthread_mutexattr_t attr;
	dspal_mutex_t       *mutex;		/* holding qurt mutex or rmutex pointer */
	void (*lock)(dspal_mutex_t *);		/* the function pointer for lock */
	void (*unlock)(dspal_mutex_t *);	/* the function pointer for unlock */
	int (*trylock)(dspal_mutex_t *);	/* the function pointer for trylock */
};

#else
typedef struct pthread_mutex_t {
	dspal_mutex_t       *kmutex;		/* holding kernel mutex (qurt mutex or rmutex) pointer */
	void (*lock)(dspal_mutex_t *);		/* the function pointer for lock */
	void (*unlock)(dspal_mutex_t *);	/* the function pointer for unlock */
	int (*trylock)(dspal_mutex_t *);	/* the function pointer for trylock */
	pthread_mutexattr_t attr;
} pthread_mutex_t;
#endif

#define PTHREAD_SPINLOCK_UNLOCKED    0
#define PTHREAD_SPINLOCK_LOCKED      1

typedef unsigned int pthread_spinlock_t;

typedef struct pthread_condattr_t {
	int is_initialized;
	int pshared;
} pthread_condattr_t;

typedef unsigned int pthread_cond_t;

typedef unsigned int timer_type;
typedef unsigned int timer_group_type;

typedef struct _pthread_cond_t _pthread_cond_t;
struct _pthread_cond_t {
	pthread_condattr_t attr;
	dspal_cond_t      *qurt_cond;
	timer_type         pthread_wait_timer;
	timer_group_type   pthread_wait_timer_group;
	int                is_timed_out;
};

typedef struct pthread_barrierattr_t {
	int is_initialized;
	int pshared;
} pthread_barrierattr_t;

typedef unsigned int pthread_barrier_t;

typedef struct _pthread_barrier_t _pthread_barrier_t;
struct _pthread_barrier_t {
	pthread_barrierattr_t attr;
	dspal_barrier_t       *qurt_barrier;
};

typedef long off_t;

#ifndef _TIME_T
typedef long int time_t;
#define _TIME_T
#endif

typedef int pthread_key_t;

__END_DECLS
