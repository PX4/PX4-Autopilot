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
 * @file netinet/in.h
 *
 * Re-export Winsock's IPv4/IPv6 types and normalize the remaining POSIX
 * networking surface: fixed-width in_addr_t / in_port_t, POSIX-sized
 * INET_ADDRSTRLEN / INET6_ADDRSTRLEN, multicast constants, and a few
 * helper aliases that Unix-oriented code expects.
 */
#pragma once

#ifdef _WIN32

#include <stdint.h>

#include <sys/socket.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef INADDR_NONE
#define INADDR_NONE ((in_addr_t)0xffffffffUL)
#endif
#ifndef IPPROTO_UDP
#  define IPPROTO_UDP 17
#endif
#ifndef IPPROTO_TCP
#  define IPPROTO_TCP 6
#endif

#ifndef _PX4_IN_ADDR_T_DEFINED_SHIM
#define _PX4_IN_ADDR_T_DEFINED_SHIM
/** @brief IPv4 address integer type used by POSIX networking APIs. */
typedef uint32_t in_addr_t;
#endif
#ifndef _PX4_IN_PORT_T_DEFINED_SHIM
#define _PX4_IN_PORT_T_DEFINED_SHIM
/** @brief TCP/UDP port integer type used by POSIX networking APIs. */
typedef uint16_t in_port_t;
#endif

#ifdef INET_ADDRSTRLEN
#undef INET_ADDRSTRLEN
#endif
#define INET_ADDRSTRLEN 16

#ifdef INET6_ADDRSTRLEN
#undef INET6_ADDRSTRLEN
#endif
#define INET6_ADDRSTRLEN 46

#ifndef MCAST_INCLUDE
#define MCAST_INCLUDE 1
#endif
#ifndef MCAST_EXCLUDE
#define MCAST_EXCLUDE 0
#endif

#ifndef IN6_ARE_ADDR_EQUAL
#define IN6_ARE_ADDR_EQUAL(a, b) IN6_ADDR_EQUAL((a), (b))
#endif

#endif /* _WIN32 */
