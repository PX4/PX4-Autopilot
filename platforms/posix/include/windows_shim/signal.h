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
 * @file signal.h
 *
 * Re-exports the MinGW signal.h and adds the POSIX signal numbers that PX4
 * code references but MinGW does not define (SIGCONT, SIGUSR1, etc.).
 * These values never reach the Win32 kernel - the PX4 runtime intercepts
 * them in px4_task_kill() and treats them as thread wake-up hints
 * (implemented via pthread_kill / no-op, since winpthreads can't signal
 * a thread with an arbitrary value).
 */
#pragma once

#if defined(_MSC_VER)
#if defined(__has_include)
#  if __has_include(<../ucrt/signal.h>)
#    include <../ucrt/signal.h>
#  endif
#endif
/** @brief Function-pointer type used by the MSVC signal() declaration. */
typedef void (*__px4_signal_handler_t)(int);
#ifndef SIG_ERR
#define SIG_ERR ((__px4_signal_handler_t)-1)
#endif
#ifndef SIG_DFL
#define SIG_DFL ((__px4_signal_handler_t)0)
#endif
#ifndef SIG_IGN
#define SIG_IGN ((__px4_signal_handler_t)1)
#endif
#ifndef SIGINT
#define SIGINT 2
#endif
#ifndef SIGILL
#define SIGILL 4
#endif
#ifndef SIGABRT
#define SIGABRT 22
#endif
#ifndef SIGFPE
#define SIGFPE 8
#endif
#ifndef SIGSEGV
#define SIGSEGV 11
#endif
#ifndef SIGTERM
#define SIGTERM 15
#endif
#ifdef __cplusplus
extern "C" {
#endif

/** @brief Register a C runtime signal handler. */
__px4_signal_handler_t signal(int sig, __px4_signal_handler_t func);

/** @brief Raise a C runtime signal in the current process. */
int raise(int sig);
#ifdef __cplusplus
}
#endif
#else
#include_next <signal.h>
#endif

#ifndef SIGHUP
#define SIGHUP  1
#endif
#ifndef SIGQUIT
#define SIGQUIT 3
#endif
#ifndef SIGKILL
#define SIGKILL 9
#endif
#ifndef SIGUSR1
#define SIGUSR1 10
#endif
#ifndef SIGUSR2
#define SIGUSR2 12
#endif
#ifndef SIGPIPE
#define SIGPIPE 13
#endif
#ifndef SIGBUS
#define SIGBUS 7
#endif
#ifndef SIGTRAP
#define SIGTRAP 5
#endif
#ifndef SIGALRM
#define SIGALRM 14
#endif
#ifndef SIGCHLD
#define SIGCHLD 17
#endif
#ifndef SIGCONT
#define SIGCONT 18
#endif
#ifndef SIGSTOP
#define SIGSTOP 19
#endif
#ifndef SIGTSTP
#define SIGTSTP 20
#endif
#ifndef SIGTTIN
#define SIGTTIN 21
#endif
#ifndef SIGTTOU
#define SIGTTOU 22
#endif
#ifndef SIGWINCH
#define SIGWINCH 28
#endif
#ifndef SIGPOLL
#define SIGPOLL 29
#endif
#ifndef SIG_BLOCK
#define SIG_BLOCK 0
#endif
#ifndef SIG_UNBLOCK
#define SIG_UNBLOCK 1
#endif
#ifndef SIG_SETMASK
#define SIG_SETMASK 2
#endif
#ifndef SA_RESTART
#define SA_RESTART 0x10000000
#endif
#ifndef SA_NOCLDSTOP
#define SA_NOCLDSTOP 0x00000001
#endif
#ifndef SA_NOCLDWAIT
#define SA_NOCLDWAIT 0x00000002
#endif
#ifndef SA_NODEFER
#define SA_NODEFER 0x40000000
#endif
#ifndef SA_RESETHAND
#define SA_RESETHAND 0x80000000
#endif
#ifndef SA_SIGINFO
#define SA_SIGINFO 4
#endif
#ifndef NSIG
#define NSIG 32
#endif
