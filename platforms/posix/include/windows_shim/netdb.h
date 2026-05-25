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
 * @file netdb.h
 *
 * WinSock2 provides getaddrinfo/freeaddrinfo/gethostbyname and the
 * struct addrinfo / hostent types, but it ships them through
 * <ws2tcpip.h> and <winsock2.h>. Forward to those so the POSIX
 * <netdb.h> includes work unchanged, and fill in the legacy BSD pieces
 * that Windows omits (`netent`, `hstrerror`, `getnet*`, and the old
 * set/end host/service/protocol database walkers).
 */
#pragma once

#ifdef _WIN32

#include <stdint.h>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <netinet/in.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HOST_NOT_FOUND
#define HOST_NOT_FOUND 11001
#endif
#ifndef TRY_AGAIN
#define TRY_AGAIN 11002
#endif
#ifndef NO_RECOVERY
#define NO_RECOVERY 11003
#endif
#ifndef NO_DATA
#define NO_DATA 11004
#endif
#ifndef NO_ADDRESS
#define NO_ADDRESS NO_DATA
#endif

#ifndef NI_MAXHOST
#define NI_MAXHOST 1025
#endif
#ifndef NI_MAXSERV
#define NI_MAXSERV 32
#endif

#ifndef EAI_SYSTEM
#define EAI_SYSTEM 11
#endif
#ifndef EAI_OVERFLOW
#define EAI_OVERFLOW WSAEFAULT
#endif

#ifndef NI_NUMERICSCOPE
#define NI_NUMERICSCOPE 0x20
#endif

#ifndef IPPORT_RESERVED
#define IPPORT_RESERVED 1024
#endif

#ifndef h_errno
#define h_errno WSAGetLastError()
#endif

/**
 * @brief Return human-readable text for legacy resolver errors.
 *
 * @param err HOST_NOT_FOUND, TRY_AGAIN, NO_RECOVERY, NO_DATA, or a Winsock
 *            resolver code.
 */
const char *hstrerror(int err);

/** @name Host database compatibility functions
 *
 * Windows does not expose the old sequential /etc/hosts database API. These
 * functions are provided so POSIX-oriented code links; gethostent() returns
 * NULL when no synthetic entry is available.
 *
 * @{
 */
void sethostent(int stay_open);
void endhostent(void);
struct hostent *gethostent(void);
/** @} */

/** @name Network database compatibility functions
 *
 * The legacy netent API has no Windows equivalent. The implementation returns
 * empty results while preserving source and link compatibility.
 *
 * @{
 */
void setnetent(int stay_open);
void endnetent(void);
struct netent *getnetent(void);
struct netent *getnetbyname(const char *name);
struct netent *getnetbyaddr(uint32_t net, int type);
/** @} */

/** @name Protocol database compatibility functions
 *
 * Protocol lookup by explicit name/number is handled by Winsock where
 * available. Sequential database walking is a no-op compatibility surface.
 *
 * @{
 */
void setprotoent(int stay_open);
void endprotoent(void);
struct protoent *getprotoent(void);
/** @} */

/** @name Service database compatibility functions
 *
 * Sequential service database walking is kept as a stub for POSIX source
 * compatibility.
 *
 * @{
 */
void setservent(int stay_open);
void endservent(void);
struct servent *getservent(void);
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _WIN32 */
