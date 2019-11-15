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
 * FILE:         signal.h
 *
 * SERVICES:     POSIX Signal API interface
 *
 * DESCRIPTION:  POSIX Signal API interface based upon POSIX 1003.1-2004
 *==========================================================================*/

#include <sys/cdefs.h>
#include "dspal_types.h"

/* POSIX signal bits */
#define DSPAL_SIGALRM  1 /* Alarm clock */
#define POSIX_MSG      7 /* POSIX msg type used in Qube API */
#define POSIX_NOTIF    8 /* POSIX msg type used in Qube API */
#define SIGKILL        9 /* kill (cannot be caught or ignored) */

#define SIGRTMIN       10
#define SIGRTMAX       32

/* Notification Types. */
/* No asynchronous notification is delivered when the event of interest occurs. */
#define SIGEV_NONE      0

/* The signal specified in sigev_signo shall be generated for the process when
   the event of interest occurs. */
#define SIGEV_SIGNAL    1

/* A notification function is called to perform notification. */
#define SIGEV_THREAD    2
#define SA_SIGINFO      1

/*
 * Flags for sigprocmask:
 */
#define SIG_BLOCK       1 /* block specified signal set */
#define SIG_UNBLOCK     2 /* unblock specified signal set */
#define SIG_SETMASK     3 /* set specified signal set */

__BEGIN_DECLS

typedef unsigned long int   sigset_t;

union sigval {
	int  sival_int;   /* Integer signal value. */
	void *sival_ptr;  /* Pointer signal value. */
};

typedef struct sigevent   sigevent;
struct sigevent {
	int            sigev_notify;			/* Notification type.       */
	int            sigev_signo;			/* Signal number.           */
	union sigval   sigev_value;			/* Signal value.            */
	void (*sigev_notify_function)(union sigval);	/* Notification function.   */
	pthread_attr_t *sigev_notify_attributes;
};

typedef struct siginfo_t   siginfo_t;
struct siginfo_t {
	int          si_signo;
	int          si_code;
	union sigval si_value;
};
struct sigaction {
	void (*sa_handler)(int);
	sigset_t sa_mask;
	int      sa_flags;
	void (*sa_sigaction)(int, siginfo_t *, void *);
};

/* Signal functions */

/** \details
 * This provides POSIX Signal API. Please note that this
 * implementation does not fully comply with POSIX standard.
 *
 * In POSIX standard, Signal can be used as 'interrupt', which means
 * an incoming signal will interrupt a running thread. After the
 * registered signal handler is executed, the thread will resume.
 * This behavior cannot be implemented w/o modifying L4 or QURT kernel.
 * On the ohter hand, appliation need to be carefully written to avoid
 * problems caused by 'interrupting' signals.
 *
 * Therefore, in this implementation of POSIX signal, thread will
 * only receive signals when it explicitly waits for signals, i.e., when
 * the thread calls either sigwait() or sigsuspend().
 *
 * Therefore, pthread_sigmask(), which set or get signal mask for a thread,
 * is not supported, since the signal mask will be set by sigwait() and
 * sigsuspend().
 *
 * Since this implementation of POSIX kernel API is a subset of PSE51,
 * only threads can send and receive signals. The functions related to
 * signal operations with processes, such as kill(), sigqueue(),
 * sigprocmask(), are not provided.
 *
 * Queued signal is not supported.
 *
 * Applications will use signals from SIGRTMIN to SIGRTMAX.
 *
 * SIGEV_SIGNAL and SIGEV_THREAD are supported. SIGEV_NONE is not
 * supported.
 *
 */

/** \defgroup signal POSIX Signal API */
/** \ingroup signal */
/** @{ */

/** Wait for signals. This implementation does not support queued signals.
 *
 * Please refer to POSIX standard for details.
 */
int sigwait(const sigset_t *restrict set, int *restrict sig);

/** Examine and Change Signal Action.
 * Please refer to POSIX standard for details.
 *
 * @param act [in] A pointer to the sigaction structure that describes the
 * action to be taken for the signal. Can be NULL.
 * The following flags for sa_flags field in struct sigaction are not
 * supported: SA_NOCLDSTOP, SA_ONSTACK, SA_RESETHAND, SA_RESTART,
 * SA_NOCLDWAIT and SA_NODEFER. Only flag SA_SIGINFO is supported.
 *
 * @note Define sigaction as macro to avoid a warning when included from
 * C++ code - it's causing a "sigaction(...) hides constructor for
 * 'struct sigaction'" warning.
 */
/*lint -esym(123,sigaction) Suppress "macro used with no arguments" */
#define sigaction(sig,act,oact) _sigaction(sig,act,oact)

/** Wait for signals.
 * Please refer to POSIX standard for details.
 */
int sigsuspend(const sigset_t *sigmask);

/** Add Signal to Signal Set.
 * Please refer to POSIX standard for details.
 */
int sigaddset(sigset_t *set, int signo);

/** Delete Signal from Signal Set.
 * Please refer to POSIX standard for details.
 */
int sigdelset(sigset_t *set, int signo);

/** Initialize and Empty Signal Set.
 * Please refer to POSIX standard for details.
 */
int sigemptyset(sigset_t *set);

/** Initialize and Fill Signal Set.
 * Please refer to POSIX standard for details.
 */
int sigfillset(sigset_t *set);

/** Test for Signal in Signal Set.
 * Please refer to POSIX standard for details.
 */
int sigismember(const sigset_t *set, int signo);

/** @} */

/* this is not a public api function */
int _sigaction(int sig, const struct sigaction *act, struct sigaction *oact);

/* have to move #include here to solve circular include problems between time.h and signal.h */
#include "dspal_time.h"

/** Wait for the time interval specified in the timespec structure referenced
 * by timeout. This implementation does not support queued signals.
 * For struct siginfo_t, si_code and si_value are ignored in this implementation.
 *
 * Please refer to POSIX standard for details.
 */
int sigtimedwait(const sigset_t *restrict set, siginfo_t *restrict info,
		 const struct timespec *restrict timeout);

__END_DECLS
