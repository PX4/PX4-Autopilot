/****************************************************************************
 *  apps/include/netutils/uiplib.h
 * Various non-standard APIs to support netutils.  All non-standard and
 * intended only for internal use.
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some of these APIs derive from uIP but all of them use the uip_ prefix
 * to identify them as members of this library.  uIP also has a BSD style
 * license:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2002, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
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

#ifndef __APPS_INCLUDE_NETUTILS_UIPLIB_H
#define __APPS_INCLUDE_NETUTILS_UIPLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include <netinet/in.h>
#include <nuttx/net/uip/uipopt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SOCK_DGRAM is the preferred socket type to use when we just want a
 * socket for performing drive ioctls.  However, we can't use SOCK_DRAM
 * if UDP is disabled.
 */

#ifdef CONFIG_NET_UDP
# define UIPLIB_SOCK_IOCTL SOCK_DGRAM
#else
# define UIPLIB_SOCK_IOCTL SOCK_STREAM
#endif

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

/* Convert a textual representation of an IP address to a numerical representation.
 *
 * This function takes a textual representation of an IP address in
 * the form a.b.c.d and converts it into a 4-byte array that can be
 * used by other uIP functions.
 *
 * addrstr A pointer to a string containing the IP address in
 * textual form.
 *
 * addr A pointer to a 4-byte array that will be filled in with
 * the numerical representation of the address.
 *
 * Return: 0 If the IP address could not be parsed.
 * Return: Non-zero If the IP address was parsed.
 */

EXTERN bool uiplib_ipaddrconv(const char *addrstr, uint8_t *addr);
EXTERN bool uiplib_hwmacconv(const char *hwstr, uint8_t *hw);

/* Get and set IP/MAC addresses (Ethernet L2 only) */

#ifdef CONFIG_NET_ETHERNET
EXTERN int uip_setmacaddr(const char *ifname, const uint8_t *macaddr);
EXTERN int uip_getmacaddr(const char *ifname, uint8_t *macaddr);
#endif

/* IP address support */

#ifdef CONFIG_NET_IPv6
EXTERN int uip_gethostaddr(const char *ifname, struct in6_addr *addr);
EXTERN int uip_sethostaddr(const char *ifname, const struct in6_addr *addr);
EXTERN int uip_setdraddr(const char *ifname, const struct in6_addr *addr);
EXTERN int uip_setnetmask(const char *ifname, const struct in6_addr *addr);
#else
EXTERN int uip_gethostaddr(const char *ifname, struct in_addr *addr);
EXTERN int uip_sethostaddr(const char *ifname, const struct in_addr *addr);
EXTERN int uip_setdraddr(const char *ifname, const struct in_addr *addr);
EXTERN int uip_setnetmask(const char *ifname, const struct in_addr *addr);
#endif

/* HTTP support */

EXTERN int  uip_parsehttpurl(const char *url, uint16_t *port,
                             char *hostname, int hostlen,
                             char *filename, int namelen);

/* Generic server logic */

EXTERN int uip_listenon(uint16_t portno);
EXTERN void uip_server(uint16_t portno, pthread_startroutine_t handler, int stacksize);

EXTERN int uip_getifstatus(const char *ifname, bool *status);
EXTERN int uip_ifup(const char *ifname);
EXTERN int uip_ifdown(const char *ifname);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_UIPLIB_H */
