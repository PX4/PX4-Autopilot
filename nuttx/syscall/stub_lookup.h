/****************************************************************************
 * syscall/stub_lookup.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * Pre-processor Definitions
 ****************************************************************************/

/* STUB_LOOKUP must be defined before including this file.
 *
 * These first system calls are supported regardless of the NuttX
 * configuration
 */

STUB_LOOKUP1(1, STUB__exit)                     /* SYS__exit */
STUB_LOOKUP(1, STUB_exit)                       /* SYS_exit */
STUB_LOOKUP(0, STUB_get_errno)                  /* SYS_get_errno */
STUB_LOOKUP(0, STUB_getpid)                     /* SYS_getpid */
STUB_LOOKUP(2, STUB_sched_getparam)             /* SYS_sched_getparam */
STUB_LOOKUP(1, STUB_sched_getscheduler)         /* SYS_sched_getscheduler */
STUB_LOOKUP(0, STUB_sched_lock)                 /* SYS_sched_lock */
STUB_LOOKUP(0, STUB_sched_lockcount)            /* SYS_sched_lockcount */
STUB_LOOKUP(2, STUB_sched_rr_get_interval)      /* SYS_sched_rr_get_interval */
STUB_LOOKUP(2, STUB_sched_setparam)             /* SYS_sched_setparam */
STUB_LOOKUP(3, STUB_sched_setscheduler)         /* SYS_sched_setscheduler */
STUB_LOOKUP(0, STUB_sched_unlock)               /* SYS_sched_unlock */
STUB_LOOKUP(0, STUB_sched_yield)                /* SYS_sched_yield */
STUB_LOOKUP(1, STUB_sem_close)                  /* SYS_sem_close */
STUB_LOOKUP(2, STUB_sem_destroy)                /* SYS_sem_destroy */
STUB_LOOKUP(6, STUB_sem_open)                   /* SYS_sem_open */
STUB_LOOKUP(1, STUB_sem_post)                   /* SYS_sem_post */
STUB_LOOKUP(1, STUB_sem_trywait)                /* SYS_sem_trywait */
STUB_LOOKUP(1, STUB_sem_unlink)                 /* SYS_sem_unlink */
STUB_LOOKUP(1, STUB_sem_wait)                   /* SYS_sem_wait */
STUB_LOOKUP(1, STUB_set_errno)                  /* SYS_set_errno */
STUB_LOOKUP(5, STUB_task_create)                /* SYS_task_create */
STUB_LOOKUP(1, STUB_task_delete)                /* SYS_task_delete */
STUB_LOOKUP(1, STUB_task_restart)               /* SYS_task_restart */
STUB_LOOKUP(2, STUB_up_assert)                  /* SYS_up_assert */
STUB_LOOKUP(3, STUB_up_assert_code)             /* SYS_up_assert_code */

/* The following can be individually enabled */

#ifdef CONFIG_SCHED_ATEXIT
  STUB_LOOKUP(1, STUB_atexit)                   /* SYS_atexit */
#endif

#ifdef CONFIG_SCHED_ONEXIT
  STUB_LOOKUP(2, STUB_onexit)                   /* SYS_onexit */
#endif

#ifdef CONFIG_SCHED_WAITPID
  STUB_LOOKUP(3, STUB_waitpid)                  /* SYS_waitpid */
#endif

/* The following are only defined is signals are supported in the NuttX
 * configuration.
 */

#ifndef CONFIG_DISABLE_SIGNALS
  STUB_LOOKUP(2, STUB_kill)                     /* SYS_kill */
  STUB_LOOKUP(3, STUB_sigaction)                /* SYS_sigaction */
  STUB_LOOKUP(1, STUB_sigpending)               /* SYS_sigpending */
  STUB_LOOKUP(3, STUB_sigprocmask)              /* SYS_sigprocmask */
  STUB_LOOKUP(3, STUB_sigqueue)                 /* SYS_sigqueue */
  STUB_LOOKUP(1, STUB_sigsuspend)               /* SYS_sigsuspend */
  STUB_LOOKUP(3, STUB_sigtimedwait)             /* SYS_sigtimedwait */
  STUB_LOOKUP(2, STUB_sigwaitinfo)              /* SYS_sigwaitinfo */
  STUB_LOOKUP(1, STUB_sleep)                    /* SYS_sleep */
  STUB_LOOKUP(1, STUB_usleep)                   /* SYS_usleep */
#endif

/* The following are only defined if the system clock is enabled in the
 * NuttX configuration.
 */

#ifndef CONFIG_DISABLE_CLOCK
  STUB_LOOKUP(0, STUB_clock_systimer)           /* SYS_clock_systimer */
  STUB_LOOKUP(2, STUB_clock_getres)             /* SYS_clock_getres */
  STUB_LOOKUP(2, STUB_clock_gettime)            /* SYS_clock_gettime */
  STUB_LOOKUP(2, STUB_clock_settime)            /* SYS_clock_settime */
  STUB_LOOKUP(2, STUB_gettimeofday)             /* SYS_gettimeofday */
#endif

/* The following are defined only if POSIX timers are supported */

#ifndef CONFIG_DISABLE_POSIX_TIMERS
  STUB_LOOKUP(3, STUB_timer_create)             /* SYS_timer_create */
  STUB_LOOKUP(1, STUB_timer_delete)             /* SYS_timer_delete */
  STUB_LOOKUP(1, STUB_timer_getoverrun)         /* SYS_timer_getoverrun */
  STUB_LOOKUP(2, STUB_timer_gettime)            /* SYS_timer_gettime */
  STUB_LOOKUP(4, STUB_timer_settime)            /* SYS_timer_settime */
#endif

/* The following are defined if either file or socket descriptor are
 * enabled.
 */

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0 
  STUB_LOOKUP(1, STUB_close)                    /* SYS_close */
  STUB_LOOKUP(3, STUB_ioctl)                    /* SYS_ioctl */
  STUB_LOOKUP(3, STUB_read)                     /* SYS_read */
  STUB_LOOKUP(3, STUB_write)                    /* SYS_write */
#  ifndef CONFIG_DISABLE_POLL
  STUB_LOOKUP(3, STUB_poll)                     /* SYS_poll */
  STUB_LOOKUP(5, STUB_select)                   /* SYS_select */
#  endif
#endif

/* The following are defined if file descriptors are enabled */

#if CONFIG_NFILE_DESCRIPTORS > 0
  STUB_LOOKUP(1, STUB_closedir)                 /* SYS_closedir */
  STUB_LOOKUP(1, STUB_dup)                      /* SYS_dup */
  STUB_LOOKUP(2, STUB_dup2)                     /* SYS_dup2 */
  STUB_LOOKUP(6, STUB_fcntl)                    /* SYS_fcntl */
  STUB_LOOKUP(3, STUB_lseek)                    /* SYS_lseek */
  STUB_LOOKUP(2, STUB_mkfifo)                   /* SYS_mkfifo */
  STUB_LOOKUP(6, STUB_mmap)                     /* SYS_mmap */
  STUB_LOOKUP(6, STUB_open)                     /* SYS_open */
  STUB_LOOKUP(1, STUB_opendir)                  /* SYS_opendir */
  STUB_LOOKUP(1, STUB_pipe)                     /* SYS_pipe */
  STUB_LOOKUP(1, STUB_readdir)                  /* SYS_readdir */
  STUB_LOOKUP(1, STUB_rewinddir)                /* SYS_rewinddir */
  STUB_LOOKUP(2, STUB_seekdir)                  /* SYS_seekdir */
  STUB_LOOKUP(2, STUB_stat)                     /* SYS_stat */
  STUB_LOOKUP(2, STUB_statfs)                   /* SYS_statfs */
  STUB_LOOKUP(1, STUB_telldir)                  /* SYS_telldir */

#  if CONFIG_NFILE_STREAMS > 0
    STUB_LOOKUP(3, STUB_fs_fdopen)              /* SYS_fs_fdopen */
    STUB_LOOKUP(0, STUB_sched_getstreams)       /* SYS_sched_getstreams */
#endif

#  if !defined(CONFIG_DISABLE_MOUNTPOINT)
    STUB_LOOKUP(1, STUB_fsync)                  /* SYS_fsync */
    STUB_LOOKUP(2, STUB_mkdir)                  /* SYS_mkdir */
    STUB_LOOKUP(5, STUB_mount)                  /* SYS_mount */
    STUB_LOOKUP(2, STUB_rename)                 /* SYS_rename */
    STUB_LOOKUP(1, STUB_rmdir)                  /* SYS_rmdir */
    STUB_LOOKUP(1, STUB_umount)                 /* SYS_umount */
    STUB_LOOKUP(1, STUB_unlink)                 /* SYS_unlink */
#  endif
#endif

/* The following are defined if pthreads are enabled */

#ifndef CONFIG_DISABLE_PTHREAD
  STUB_LOOKUP(1, STUB_pthread_barrier_destroy)  /* SYS_pthread_barrier_destroy */
  STUB_LOOKUP(3, STUB_pthread_barrier_init)     /* SYS_pthread_barrier_init */
  STUB_LOOKUP(1, STUB_pthread_barrier_wait)     /* SYS_pthread_barrier_wait */
  STUB_LOOKUP(1, STUB_pthread_cancel)           /* SYS_pthread_cancel */
  STUB_LOOKUP(1, STUB_pthread_cond_broadcast)   /* SYS_pthread_cond_broadcast */
  STUB_LOOKUP(1, STUB_pthread_cond_destroy)     /* SYS_pthread_cond_destroy */
  STUB_LOOKUP(2, STUB_pthread_cond_init)        /* SYS_pthread_cond_init */
  STUB_LOOKUP(1, STUB_pthread_cond_signal)      /* SYS_pthread_cond_signal */
  STUB_LOOKUP(2, STUB_pthread_cond_wait)        /* SYS_pthread_cond_wait */
  STUB_LOOKUP(4, STUB_pthread_create)           /* SYS_pthread_create */
  STUB_LOOKUP(1, STUB_pthread_detach)           /* SYS_pthread_detach */
  STUB_LOOKUP(1, STUB_pthread_exit)             /* SYS_pthread_exit */
  STUB_LOOKUP(3, STUB_pthread_getschedparam)    /* SYS_pthread_getschedparam */
  STUB_LOOKUP(1, STUB_pthread_getspecific)      /* SYS_pthread_getspecific */
  STUB_LOOKUP(2, STUB_pthread_join)             /* SYS_pthread_join */
  STUB_LOOKUP(2, STUB_pthread_key_create)       /* SYS_pthread_key_create */
  STUB_LOOKUP(1, STUB_pthread_key_delete)       /* SYS_pthread_key_delete */
  STUB_LOOKUP(1, STUB_pthread_mutex_destroy)    /* SYS_pthread_mutex_destroy */
  STUB_LOOKUP(2, STUB_pthread_mutex_init)       /* SYS_pthread_mutex_init */
  STUB_LOOKUP(1, STUB_pthread_mutex_lock)       /* SYS_pthread_mutex_lock */
  STUB_LOOKUP(1, STUB_pthread_mutex_trylock)    /* SYS_pthread_mutex_trylock */
  STUB_LOOKUP(1, STUB_pthread_mutex_unlock)     /* SYS_pthread_mutex_unlock */
  STUB_LOOKUP(2, STUB_pthread_once)             /* SYS_pthread_once */
  STUB_LOOKUP(2, STUB_pthread_setcancelstate)   /* SYS_pthread_setcancelstate */
  STUB_LOOKUP(3, STUB_pthread_setschedparam)    /* SYS_pthread_setschedparam */
  STUB_LOOKUP(2, STUB_pthread_setschedprio)     /* SYS_pthread_setschedprio */
  STUB_LOOKUP(2, STUB_pthread_setspecific)      /* SYS_pthread_setspecific */
  STUB_LOOKUP(0, STUB_pthread_yield)            /* SYS_pthread_yield */
#  ifndef CONFIG_DISABLE_SIGNAL
    STUB_LOOKUP(3, STUB_pthread_cond_timedwait) /* SYS_pthread_cond_timedwait */
    STUB_LOOKUP(2, STUB_pthread_kill)           /* SYS_pthread_kill */
    STUB_LOOKUP(3, STUB_pthread_sigmask)        /* SYS_pthread_sigmask */
#  endif
#endif

/* The following are defined only if message queues are enabled */

#ifndef CONFIG_DISABLE_MQUEUE
  STUB_LOOKUP(1, STUB_mq_close)                 /* SYS_mq_close */
  STUB_LOOKUP(2, STUB_mq_notify)                /* SYS_mq_notify */
  STUB_LOOKUP(6, STUB_mq_open)                  /* SYS_mq_open */
  STUB_LOOKUP(4, STUB_mq_receive)               /* SYS_mq_receive */
  STUB_LOOKUP(4, STUB_mq_send)                  /* SYS_mq_send */
  STUB_LOOKUP(5, STUB_mq_timedreceive)          /* SYS_mq_timedreceive */
  STUB_LOOKUP(5, STUB_mq_timedsend)             /* SYS_mq_timedsend */
  STUB_LOOKUP(1, STUB_mq_unlink)                /* SYS_mq_unlink */
#endif

/* The following are defined only if environment variables are supported */

#ifndef CONFIG_DISABLE_ENVIRON
  STUB_LOOKUP(0, STUB_clearenv)                 /* SYS_clearenv */
  STUB_LOOKUP(1, STUB_getenv)                   /* SYS_getenv */
  STUB_LOOKUP(1, STUB_putenv)                   /* SYS_putenv */
  STUB_LOOKUP(3, STUB_setenv)                   /* SYS_setenv */
  STUB_LOOKUP(1, STUB_unsetenv)                 /* SYS_unsetenv */
#endif

/* The following are defined only if networking AND sockets are supported */

#if CONFIG_NSOCKET_DESCRIPTORS > 0 && defined(CONFIG_NET)
  STUB_LOOKUP(3, STUB_accept)                   /* SYS_accept */
  STUB_LOOKUP(3, STUB_bind)                     /* SYS_bind */
  STUB_LOOKUP(3, STUB_connect)                  /* SYS_connect */
  STUB_LOOKUP(5, STUB_getsockopt)               /* SYS_getsockopt */
  STUB_LOOKUP(2, STUB_listen)                   /* SYS_listen */
  STUB_LOOKUP(4, STUB_recv)                     /* SYS_recv */
  STUB_LOOKUP(6, STUB_recvfrom)                 /* SYS_recvfrom */
  STUB_LOOKUP(4, STUB_send)                     /* SYS_send */
  STUB_LOOKUP(6, STUB_sendto)                   /* SYS_sendto */
  STUB_LOOKUP(5, STUB_setsockopt)               /* SYS_setsockopt */
  STUB_LOOKUP(3, STUB_socket)                   /* SYS_socket */
#endif

/* The following is defined only if CONFIG_TASK_NAME_SIZE > 0 */

#if CONFIG_TASK_NAME_SIZE > 0
  STUB_LOOKUP(5, STUB_prctl)                    /* SYS_prctl */
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/


