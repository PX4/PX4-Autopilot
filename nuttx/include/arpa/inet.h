/****************************************************************************
 * include/arpa/inet.h
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __INCLUDE_ARPA_INET_H
#define __INCLUDE_ARPA_INET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <netinet/in.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/
/*  Length of the string form for IP address (excludes NULL termination) */

#define INET_ADDRSTRLEN 16

/*  Length of the string form for IPv6 address (excludes NULL termination) */

#define INET6_ADDRSTRLEN 46

/* This macro to convert a 16/32-bit constant values quantity from host byte
 * order to network byte order.  The 16-bit version of this macro is required
 * for uIP:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 */

#ifdef CONFIG_ENDIAN_BIG
# define HTONS(ns) (ns)
# define HTONL(nl) (nl)
#else
# define HTONS(ns) \
  (unsigned short) \
    (((((unsigned short)(ns)) & 0x00ff) << 8) | \
     ((((unsigned short)(ns)) >> 8) & 0x00ff))
# define HTONL(nl) \
  (unsigned long) \
    (((((unsigned long)(nl)) & 0x000000ffUL) << 24) | \
     ((((unsigned long)(nl)) & 0x0000ff00UL) <<  8) | \
     ((((unsigned long)(nl)) & 0x00ff0000UL) >>  8) | \
     ((((unsigned long)(nl)) & 0xff000000UL) >> 24))
#endif

#define NTOHS(hs) HTONS(hs)
#define NTOHL(hl) HTONL(hl)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Functions to convert between nost and network byte ordering */

EXTERN uint32_t    ntohl(uint32_t nl);
EXTERN uint16_t    ntohs(uint16_t ns);
EXTERN uint32_t    htonl(uint32_t hl);
EXTERN uint16_t    htons(uint16_t hs);

/* Functions to manipulate address representations */

EXTERN int         inet_aton(FAR const char *cp, FAR struct in_addr *inp);
EXTERN in_addr_t   inet_addr(FAR const char *cp);
EXTERN in_addr_t   inet_network(FAR const char *cp);

#ifdef CONFIG_CAN_PASS_STRUCTS
EXTERN FAR char   *inet_ntoa(struct in_addr in);
EXTERN in_addr_t   inet_lnaof(struct in_addr in);
EXTERN in_addr_t   inet_netof(struct in_addr in);
#else
EXTERN FAR char   *_inet_ntoa(in_addr_t in);
# define inet_ntoa(in) _inet_ntoa(in.s_addr);

EXTERN in_addr_t   _inet_lnaof(in_addr_t in);
# define inet_lnaof(in) _inet_lnaof(in.s_addr);

EXTERN in_addr_t   _inet_netof(in_addr_t in);
# define inet_netof(in) _inet_netof(in.s_addr);
#endif
EXTERN struct in_addr inet_makeaddr(in_addr_t net, in_addr_t host);

EXTERN int         inet_pton(int af, FAR const char *src, FAR void *dst);
EXTERN const char *inet_ntop(int af, FAR const void *src, FAR char *dst, socklen_t size);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_ARPA_INET_H */
