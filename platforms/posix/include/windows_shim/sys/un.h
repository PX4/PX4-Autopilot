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
 * @file sys/un.h
 *
 * AF_UNIX was added to Windows 10 in build 17063 (April 2018) and is
 * exposed through afunix.h. MinGW-w64 headers do not yet forward a
 * POSIX-style <sys/un.h>, so we supply one that re-exposes the struct
 * under the expected name. Windows' AF_UNIX supports SOCK_STREAM over
 * file-system paths but not abstract namespace sockets - SITL only
 * uses file-system paths, so this is sufficient for the daemon IPC.
 */
#pragma once

#ifdef _WIN32

#include <stddef.h>
#include <string.h>
#include <winsock2.h>
#include <afunix.h>

#ifndef UNIX_PATH_MAX
#  define UNIX_PATH_MAX 108
#endif

typedef struct sockaddr_un sockaddr_un;

#ifndef SUN_LEN
#  define SUN_LEN(ptr) ((socklen_t)(offsetof(struct sockaddr_un, sun_path) + strlen((ptr)->sun_path)))
#endif

/* afunix.h already provides struct sockaddr_un. Re-export sun_path/sun_family
 * aliases if any translation unit expects them. afunix.h on Windows already
 * names them that way, so no further work is needed. */

#endif /* _WIN32 */
