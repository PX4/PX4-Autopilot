/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
/**
 * @file pthread.h
 *
 * MinGW-w64 links winpthreads which ships a full <pthread.h>, but it
 * omits a handful of POSIX macros that PX4 uses (PTHREAD_STACK_MIN). We
 * forward to the real header via #include_next and add the missing
 * pieces with values consistent with how Windows actually behaves
 * (allocation granularity = 64 KiB, pthread_attr_setstacksize rounds
 * up to it anyway).
 */
#pragma once

#if defined(_MSC_VER)
#include <windows.h>
#include <time.h>
#include <stdint.h>
#include <sched.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Thread handle stored as an integer-sized Windows HANDLE value. */
typedef uintptr_t pthread_t;

/** @brief Thread-local storage key backed by TlsAlloc/TlsFree. */
typedef DWORD pthread_key_t;

/**
 * @brief Thread creation attributes understood by the MSVC pthread shim.
 *
 * Scheduling fields are stored for POSIX API round-tripping; Windows thread
 * creation only consumes detach state and stack size.
 */
typedef struct pthread_attr_t {
	void *stack_addr;
	size_t stack_size;
	int detach_state;
	int sched_policy;
	int inherit_sched;
	int scope;
	struct sched_param sched;
} pthread_attr_t;

/** @brief Mutex attribute object; currently carries only the mutex type. */
typedef struct pthread_mutexattr_t {
	int type;
} pthread_mutexattr_t;

/** @brief Condition-variable attribute object; stores the requested clock. */
typedef struct pthread_condattr_t {
	int clock_id;
} pthread_condattr_t;

/**
 * @brief Mutex object backed by a lazily initialized CRITICAL_SECTION.
 *
 * The INIT_ONCE field lets static PTHREAD_MUTEX_INITIALIZER instances be
 * initialized on first lock without requiring a constructor.
 */
typedef struct pthread_mutex_t {
	INIT_ONCE once;
	CRITICAL_SECTION critical_section;
	int type;
} pthread_mutex_t;

/** @brief Condition variable object backed directly by CONDITION_VARIABLE. */
typedef CONDITION_VARIABLE pthread_cond_t;

/** @brief One-time initialization guard backed by INIT_ONCE. */
typedef INIT_ONCE pthread_once_t;

#define PTHREAD_MUTEX_INITIALIZER { INIT_ONCE_STATIC_INIT, { 0 }, 0 }
#define PTHREAD_COND_INITIALIZER CONDITION_VARIABLE_INIT
#define PTHREAD_ONCE_INIT INIT_ONCE_STATIC_INIT

#define PTHREAD_MUTEX_NORMAL 0
#define PTHREAD_MUTEX_RECURSIVE 1
#define PTHREAD_PROCESS_PRIVATE 0
#define PTHREAD_CREATE_JOINABLE 0
#define PTHREAD_CREATE_DETACHED 1
#define PTHREAD_INHERIT_SCHED 0
#define PTHREAD_EXPLICIT_SCHED 1
#define PTHREAD_SCOPE_SYSTEM 0
#define PTHREAD_SCOPE_PROCESS 1

/** @name Thread attribute functions
 *
 * Store POSIX pthread attributes for later pthread_create() calls. Unsupported
 * scheduling/scope attributes are retained for round-trip queries and ignored
 * by Windows thread creation.
 *
 * @{
 */
int pthread_attr_init(pthread_attr_t *attr);
int pthread_attr_destroy(pthread_attr_t *attr);
int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stack_size);
int pthread_attr_getstacksize(const pthread_attr_t *attr, size_t *stack_size);
int pthread_attr_setstack(pthread_attr_t *attr, void *stack_addr, size_t stack_size);
int pthread_attr_getstack(const pthread_attr_t *attr, void **stack_addr, size_t *stack_size);
int pthread_attr_getschedparam(const pthread_attr_t *attr, struct sched_param *param);
int pthread_attr_setschedparam(pthread_attr_t *attr, const struct sched_param *param);
int pthread_attr_getschedpolicy(const pthread_attr_t *attr, int *policy);
int pthread_attr_setschedpolicy(pthread_attr_t *attr, int policy);
int pthread_attr_getinheritsched(const pthread_attr_t *attr, int *inheritsched);
int pthread_attr_setinheritsched(pthread_attr_t *attr, int inheritsched);
int pthread_attr_getdetachstate(const pthread_attr_t *attr, int *detachstate);
int pthread_attr_setdetachstate(pthread_attr_t *attr, int detachstate);
int pthread_attr_getscope(const pthread_attr_t *attr, int *scope);
int pthread_attr_setscope(pthread_attr_t *attr, int scope);
/** @} */

/** @name Mutex functions
 *
 * Implement POSIX mutex operations on top of CRITICAL_SECTION. Recursive
 * mutexes are accepted through pthread_mutexattr_settype().
 *
 * @{
 */
int pthread_mutexattr_init(pthread_mutexattr_t *attr);
int pthread_mutexattr_destroy(pthread_mutexattr_t *attr);
int pthread_mutexattr_gettype(const pthread_mutexattr_t *attr, int *type);
int pthread_mutexattr_settype(pthread_mutexattr_t *attr, int type);
int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr);
int pthread_mutex_destroy(pthread_mutex_t *mutex);
int pthread_mutex_lock(pthread_mutex_t *mutex);
int pthread_mutex_trylock(pthread_mutex_t *mutex);
int pthread_mutex_unlock(pthread_mutex_t *mutex);
/** @} */

/** @name Condition-variable functions
 *
 * Implement pthread condition variables with SleepConditionVariableCS and
 * WakeConditionVariable/WakeAllConditionVariable.
 *
 * @{
 */
int pthread_condattr_init(pthread_condattr_t *attr);
int pthread_condattr_destroy(pthread_condattr_t *attr);
int pthread_condattr_setclock(pthread_condattr_t *attr, clockid_t clock_id);
int pthread_cond_init(pthread_cond_t *cond, const pthread_condattr_t *attr);
int pthread_cond_destroy(pthread_cond_t *cond);
int pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
int pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, const struct timespec *abstime);
int pthread_cond_signal(pthread_cond_t *cond);
int pthread_cond_broadcast(pthread_cond_t *cond);
/** @} */

#if defined(_MSC_VER)
typedef void (*px4_pthread_cond_notify_callback_t)(pthread_cond_t *cond, int broadcast);
int px4_pthread_cond_set_notify_callback(px4_pthread_cond_notify_callback_t callback);
#endif

/** @name Thread lifecycle functions
 *
 * Wrap CreateThread/WaitForSingleObject/CloseHandle with pthread-compatible
 * ownership and return codes.
 *
 * @{
 */
int pthread_create(pthread_t *thread, const pthread_attr_t *attr, void *(*start_routine)(void *), void *arg);
int pthread_join(pthread_t thread, void **value_ptr);
int pthread_detach(pthread_t thread);
void pthread_exit(void *value_ptr);
pthread_t pthread_self(void);
int pthread_equal(pthread_t t1, pthread_t t2);
int pthread_getschedparam(pthread_t thread, int *policy, struct sched_param *param);
int pthread_setschedparam(pthread_t thread, int policy, const struct sched_param *param);
int pthread_cancel(pthread_t thread);
int pthread_kill(pthread_t thread, int sig);
/** @} */

/** @name Thread-local storage functions
 *
 * Back POSIX pthread keys with the Windows TLS API.
 *
 * @{
 */
int pthread_key_create(pthread_key_t *key, void (*destructor)(void *));
int pthread_key_delete(pthread_key_t key);
int pthread_setspecific(pthread_key_t key, const void *value);
void *pthread_getspecific(pthread_key_t key);
/** @} */

/** @brief Run an initialization routine exactly once. */
int pthread_once(pthread_once_t *once_control, void (*init_routine)(void));

/**
 * @brief Name another thread for debuggers.
 *
 * @param thread Thread returned by pthread_create() or pthread_self().
 * @param name UTF-8 thread name.
 */
int px4_pthread_setname_np(pthread_t thread, const char *name);

/**
 * @brief Name the current thread for debuggers.
 *
 * pthread_setname_np has one-argument and two-argument variants across POSIX
 * platforms, so the macro below dispatches to this helper when only a name is
 * supplied.
 */
int px4_pthread_setname_current_np(const char *name);

/** @brief Read a thread name previously set through the shim when available. */
int pthread_getname_np(pthread_t thread, char *name, size_t len);

#define PX4_PTHREAD_SETNAME_SELECT(_1, _2, NAME, ...) NAME
#define pthread_setname_np(...) PX4_PTHREAD_SETNAME_SELECT(__VA_ARGS__, px4_pthread_setname_np, px4_pthread_setname_current_np)(__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#else
#include_next <pthread.h>
#endif

#ifndef PTHREAD_STACK_MIN
#define PTHREAD_STACK_MIN 16384
#endif

#if (defined(__PX4_WINDOWS) || defined(_WIN32)) && !defined(_MSC_VER) && \
	(defined(ENABLE_LOCKSTEP_SCHEDULER) || defined(PX4_WINDOWS_PTHREAD_LOCKSTEP_BRIDGE))
#ifdef __cplusplus
extern "C" {
#endif
int px4_lockstep_pthread_cond_signal(pthread_cond_t *cond);
int px4_lockstep_pthread_cond_broadcast(pthread_cond_t *cond);
#ifdef __cplusplus
}
#endif
#define pthread_cond_signal(cond_) px4_lockstep_pthread_cond_signal(cond_)
#define pthread_cond_broadcast(cond_) px4_lockstep_pthread_cond_broadcast(cond_)
#endif
