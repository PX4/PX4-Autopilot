/****************************************************************************
 * syscall/syscall_stublookup.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syscall.h>

/* This will need to be extended if there are reserved syscall numbers */

#if CONFIG_CONFIG_SYS_RESERVED == 0

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/****************************************************************************
 * Stub Function Prototypes
 ****************************************************************************/

/* These first system calls are supported regardless of the NuttX
 * configuration
 */

extern uintptr_t STUB__exit(uintptr_t parm1);
extern uintptr_t STUB_exit(uintptr_t parm1);
extern uintptr_t STUB_get_errno(void);
extern uintptr_t STUB_getpid(void);
extern uintptr_t STUB_sched_getparam(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_sched_getscheduler(uintptr_t parm1);
extern uintptr_t STUB_sched_lock(void);
extern uintptr_t STUB_sched_lockcount(void);
extern uintptr_t STUB_sched_rr_get_interval(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_sched_setparam(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_sched_setscheduler(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sched_unlock(void);
extern uintptr_t STUB_sched_yield(void);
extern uintptr_t STUB_sem_close(uintptr_t parm1);
extern uintptr_t STUB_sem_destroy(uintptr_t parm1);
extern uintptr_t STUB_sem_open(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_sem_post(uintptr_t parm1);
extern uintptr_t STUB_sem_trywait(uintptr_t parm1);
extern uintptr_t STUB_sem_unlink(uintptr_t parm1);
extern uintptr_t STUB_sem_wait(uintptr_t parm1);
extern uintptr_t STUB_set_errno(uintptr_t parm1);
extern uintptr_t STUB_task_create(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_task_delete(uintptr_t parm1);
extern uintptr_t STUB_task_restart(uintptr_t parm1);
extern uintptr_t STUB_up_assert(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_up_assert_code(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);

/* The following can be individually enabled */

extern uintptr_t STUB_atexit(uintptr_t parm1);
extern uintptr_t STUB_waitpid(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);

/* The following are only defined is signals are supported in the NuttX
 * configuration.
 */

extern uintptr_t STUB_kill(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_sigaction(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sigpending(uintptr_t parm1);
extern uintptr_t STUB_sigprocmask(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sigqueue(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sigsuspend(uintptr_t parm1);
extern uintptr_t STUB_sigtimedwait(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sigwaitinfo(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_sleep(uintptr_t parm1);
extern uintptr_t STUB_usleep(uintptr_t parm1);

/* The following are only defined if the system clock is enabled in the
 * NuttX configuration.
 */

extern uintptr_t STUB_clock_systimer(void);
extern uintptr_t STUB_clock_getres(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_clock_gettime(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_clock_settime(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_gettimeofday(uintptr_t parm1, uintptr_t parm2);

/* The following are defined only if POSIX timers are supported */

extern uintptr_t STUB_timer_create(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_timer_delete(uintptr_t parm1);
extern uintptr_t STUB_timer_getoverrun(uintptr_t parm1);
extern uintptr_t STUB_timer_gettime(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_timer_settime(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);

/* The following are defined if either file or socket descriptor are
 * enabled.
 */

extern uintptr_t STUB_close(uintptr_t parm1);
extern uintptr_t STUB_ioctl(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_poll(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_read(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_select(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_write(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);

/* The following are defined if file descriptors are enabled */

extern uintptr_t STUB_closedir(uintptr_t parm1);
extern uintptr_t STUB_dup(uintptr_t parm1);
extern uintptr_t STUB_dup2(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_fcntl(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_lseek(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_mkfifo(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_mmap(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_open(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_opendir(uintptr_t parm1);
extern uintptr_t STUB_pipe(uintptr_t parm1);
extern uintptr_t STUB_readdir(uintptr_t parm1);
extern uintptr_t STUB_rewinddir(uintptr_t parm1);
extern uintptr_t STUB_seekdir(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_stat(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_statfs(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_telldir(uintptr_t parm1);

extern uintptr_t STUB_fs_fdopen(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_sched_getstreams(void);

extern uintptr_t STUB_fsync(uintptr_t parm1);
extern uintptr_t STUB_mkdir(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_mount(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_rename(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_rmdir(uintptr_t parm1);
extern uintptr_t STUB_umount(uintptr_t parm1);
extern uintptr_t STUB_unlink(uintptr_t parm1);

/* The following are defined if pthreads are enabled */

extern uintptr_t STUB_pthread_barrier_destroy(uintptr_t parm1);
extern uintptr_t STUB_pthread_barrier_init(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_pthread_barrier_wait(uintptr_t parm1);
extern uintptr_t STUB_pthread_cancel(uintptr_t parm1);
extern uintptr_t STUB_pthread_cond_broadcast(uintptr_t parm1);
extern uintptr_t STUB_pthread_cond_destroy(uintptr_t parm1);
extern uintptr_t STUB_pthread_cond_init(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_cond_signal(uintptr_t parm1);
extern uintptr_t STUB_pthread_cond_wait(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_create(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);
extern uintptr_t STUB_pthread_detach(uintptr_t parm1);
extern uintptr_t STUB_pthread_exit(uintptr_t parm1);
extern uintptr_t STUB_pthread_getschedparam(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_pthread_getspecific(uintptr_t parm1);
extern uintptr_t STUB_pthread_join(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_key_create(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_key_delete(uintptr_t parm1);
extern uintptr_t STUB_pthread_mutex_destroy(uintptr_t parm1);
extern uintptr_t STUB_pthread_mutex_init(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_mutex_lock(uintptr_t parm1);
extern uintptr_t STUB_pthread_mutex_trylock(uintptr_t parm1);
extern uintptr_t STUB_pthread_mutex_unlock(uintptr_t parm1);
extern uintptr_t STUB_pthread_once(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_setcancelstate(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_setschedparam(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_pthread_setschedprio(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_setspecific(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_yield(void);

extern uintptr_t STUB_pthread_cond_timedwait(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_pthread_kill(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_pthread_sigmask(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);

/* The following are defined only if message queues are enabled */

extern uintptr_t STUB_mq_close(uintptr_t parm1);
extern uintptr_t STUB_mq_notify(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_mq_open(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_mq_receive(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);
extern uintptr_t STUB_mq_send(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);
extern uintptr_t STUB_mq_timedreceive(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_mq_timedsend(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_mq_unlink(uintptr_t parm1);

/* The following are defined only if environment variables are supported */

extern uintptr_t STUB_clearenv(void);
extern uintptr_t STUB_getenv(uintptr_t parm1);
extern uintptr_t STUB_putenv(uintptr_t parm1);
extern uintptr_t STUB_setenv(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_unsetenv(uintptr_t parm1);

/* The following are defined only if networking AND sockets are supported */

extern uintptr_t STUB_accept(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_bind(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_connect(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);
extern uintptr_t STUB_getsockopt(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_listen(uintptr_t parm1, uintptr_t parm2);
extern uintptr_t STUB_recv(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);
extern uintptr_t STUB_recvfrom(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_send(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4);
extern uintptr_t STUB_sendto(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5, uintptr_t parm6);
extern uintptr_t STUB_setsockopt(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);
extern uintptr_t STUB_socket(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3);

/* The following is defined only if CONFIG_TASK_NAME_SIZE > 0 */

extern uintptr_t STUB_prctl(uintptr_t parm1, uintptr_t parm2, uintptr_t parm3, uintptr_t parm4, uintptr_t parm5);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Stub lookup tables.  Each table is indexed by the system call numbers
 * defined above.  Given the system call number, the corresponding entry in
 * these tables describes how to call the stub dispatch function.
 */

const union syscall_stubfunc_u g_stublookup[SYS_nsyscalls] =
{
#  undef STUB_LOOKUP1
#  define STUB_LOOKUP1(n,p) (union syscall_stubfunc_u)p
#  undef STUB_LOOKUP
#  define STUB_LOOKUP(n,p)  , (union syscall_stubfunc_u)p
#  include "stub_lookup.h"
};

const uint8_t g_stubnparms[SYS_nsyscalls] =
{
#  undef STUB_LOOKUP1
#  define STUB_LOOKUP1(n,p) n
#  undef STUB_LOOKUP
#  define STUB_LOOKUP(n,p)  , n
#  include "stub_lookup.h"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_CONFIG_SYS_RESERVED */
