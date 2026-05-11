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
 * @file sys/wait.h
 *
 * MinGW-w64 does not ship a POSIX waitpid/wait status header. PX4's
 * Windows target does not support fork/exec semantics, but a number of
 * third-party libraries still expect the wait status macros and a
 * waitpid() declaration to exist. The implementation in
 * posix_shim.cpp waits on a Windows process id/handle and synthesizes
 * POSIX-style exit status words.
 */
#pragma once

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WNOHANG
#define WNOHANG    0x00000001
#endif
#ifndef WUNTRACED
#define WUNTRACED  0x00000002
#endif
#ifndef WCONTINUED
#define WCONTINUED 0x00000008
#endif

#ifndef WEXITSTATUS
#define WEXITSTATUS(status) (((status) >> 8) & 0xff)
#endif
#ifndef WTERMSIG
#define WTERMSIG(status) ((status) & 0x7f)
#endif
#ifndef WSTOPSIG
#define WSTOPSIG(status) WEXITSTATUS(status)
#endif
#ifndef WIFEXITED
#define WIFEXITED(status) (WTERMSIG(status) == 0)
#endif
#ifndef WIFSIGNALED
#define WIFSIGNALED(status) (WTERMSIG(status) != 0 && WTERMSIG(status) != 0x7f)
#endif
#ifndef WIFSTOPPED
#define WIFSTOPPED(status) (WTERMSIG(status) == 0x7f)
#endif
#ifndef WIFCONTINUED
#define WIFCONTINUED(status) ((status) == 0xffff)
#endif

/**
 * @brief Wait for a Windows process id and synthesize a POSIX wait status.
 *
 * @param pid Process id to wait for.
 * @param status Optional POSIX wait status output.
 * @param options Supports WNOHANG; other options are accepted for source
 *                compatibility.
 * @return @p pid on completion, 0 for WNOHANG timeout, or -1 with errno set.
 */
pid_t waitpid(pid_t pid, int *status, int options);

#ifdef __cplusplus
}
#endif
