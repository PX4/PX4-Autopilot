/****************************************************************************
 * apps/include/netutils/resolv.h
 * DNS resolver code header file.
 * Author Adam Dunkels <adam@dunkels.com>
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Inspired by/based on uIP logic by Adam Dunkels:
 *
 *   Copyright (c) 2002-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_RESOLVE_H
#define __APPS_INCLUDE_NETUTILS_RESOLVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/uip/uipopt.h>

#include <netinet/in.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Functions. */

EXTERN int resolv_init(void);
EXTERN int resolv_create(int *sockfd);
EXTERN int resolv_release(int *sockfd);
EXTERN int resolv_gethostip_socket(int sockfd, const char *hostname, in_addr_t *ipaddr);
EXTERN int resolv_gethostip(const char *hostname, in_addr_t *ipaddr);

#ifdef CONFIG_NET_IPv6
EXTERN void resolv_conf(FAR const struct in6_addr *dnsserver);
EXTERN void resolv_getserver(FAR const struct in_addr *dnsserver);
EXTERN int  resolv_query(FAR const char *name, FAR struct sockaddr_in6 *addr);
EXTERN int  resolv_query_socket(int sockfd, FAR const char *name, FAR struct sockaddr_in6 *addr);
#else
EXTERN void resolv_conf(FAR const struct in_addr *dnsserver);
EXTERN void resolv_getserver(FAR struct in_addr *dnsserver);
EXTERN int  resolv_query(FAR const char *name, FAR struct sockaddr_in *addr);
EXTERN int  resolv_query_socket(int sockfd, FAR const char *name, FAR struct sockaddr_in *addr);
#endif

EXTERN int  dns_gethostip(const char *hostname, in_addr_t *ipaddr);

#define dns_init        resolv_init
#define dns_bind        resolv_create
#define dns_query       resolv_gethostip_socket
#define dns_free        resolv_release

#define dns_setserver   resolv_conf
#define dns_getserver   resolv_getserver
#define dns_whois       resolv_query

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_RESOLVE_H */
