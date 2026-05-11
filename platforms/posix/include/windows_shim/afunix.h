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
 * @file afunix.h
 *
 * The Windows SDK's afunix.h is intentionally small: it defines the AF_UNIX
 * socket address payload (`SOCKADDR_UN`, `PSOCKADDR_UN`, `UNIX_PATH_MAX`) and
 * the AF_UNIX-specific WSAIoctl constants. Older MinGW sysroots do not ship
 * that header at all, while newer ones may grow it over time. Mirror the
 * native Windows SDK surface when present, provide a compatible fallback when
 * absent, and layer the common POSIX convenience aliases (`sockaddr_un`,
 * `SUN_LEN`) on top.
 */
#pragma once

#ifdef _WIN32

#include <stddef.h>
#include <string.h>
#include <winsock2.h>

#if defined(__has_include_next)
#  if __has_include_next(<afunix.h>)
#    include_next <afunix.h>
#  endif
#endif

#ifndef _AFUNIX_
#define _AFUNIX_

#ifndef AF_UNIX
#define AF_UNIX 1
#endif

#ifndef PF_UNIX
#define PF_UNIX AF_UNIX
#endif

#ifndef UNIX_PATH_MAX
#define UNIX_PATH_MAX 108
#endif

/**
 * @brief AF_UNIX socket address compatible with the Windows SDK definition.
 *
 * The typedef aliases mirror the SDK names so code can use either POSIX
 * `struct sockaddr_un` or Windows `SOCKADDR_UN`.
 */
typedef struct sockaddr_un {
	ADDRESS_FAMILY sun_family;          /* AF_UNIX */
	char           sun_path[UNIX_PATH_MAX]; /* pathname */
} SOCKADDR_UN, *PSOCKADDR_UN;

#ifndef SIO_AF_UNIX_GETPEERPID
#define SIO_AF_UNIX_GETPEERPID _WSAIOR(IOC_VENDOR, 256)
#endif

#ifndef SIO_AF_UNIX_SETBINDPARENTPATH
#define SIO_AF_UNIX_SETBINDPARENTPATH _WSAIOW(IOC_VENDOR, 257)
#endif

#ifndef SIO_AF_UNIX_SETCONNPARENTPATH
#define SIO_AF_UNIX_SETCONNPARENTPATH _WSAIOW(IOC_VENDOR, 258)
#endif

#endif /* _AFUNIX_ */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _PX4_SOCKADDR_UN_ALIAS_DEFINED
#define _PX4_SOCKADDR_UN_ALIAS_DEFINED
/** @brief POSIX alias for struct sockaddr_un. */
typedef struct sockaddr_un sockaddr_un;
#endif

#ifndef SUN_LEN
#define SUN_LEN(ptr) ((socklen_t)(offsetof(struct sockaddr_un, sun_path) + strlen((ptr)->sun_path)))
#endif

#ifdef __cplusplus
}
#endif

#endif /* _WIN32 */
