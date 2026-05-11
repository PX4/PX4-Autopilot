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
 * @file arpa/inet.h
 *
 * Re-export Winsock's address-conversion surface through the POSIX include
 * path and provide the classic BSD helpers that Windows does not ship:
 * `inet_aton`, `inet_ntoa_r`, `inet_network`, `inet_makeaddr`,
 * `inet_lnaof`, and `inet_netof`.
 */
#pragma once

#ifdef _WIN32

#include <stddef.h>

#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Parse an IPv4 address string into struct in_addr. */
int inet_aton(const char *cp, struct in_addr *inp);

/** @brief Thread-safe inet_ntoa() variant writing into caller storage. */
char *inet_ntoa_r(struct in_addr in, char *buf, size_t buflen);

/** @brief Parse a classful IPv4 network number. */
in_addr_t inet_network(const char *cp);

/** @brief Return the local-network portion of a classful IPv4 address. */
in_addr_t inet_lnaof(struct in_addr in);

/** @brief Return the classful network portion of an IPv4 address. */
in_addr_t inet_netof(struct in_addr in);

/** @brief Compose an IPv4 address from classful network and host parts. */
struct in_addr inet_makeaddr(in_addr_t net, in_addr_t host);

#ifdef __cplusplus
}
#endif

#endif /* _WIN32 */
