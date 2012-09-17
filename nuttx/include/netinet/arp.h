/****************************************************************************
 * include/netinet/arp.h
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __INCLUDE_NETINET_ARP_H
#define __INCLUDE_NETINET_ARP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Three ioctls are available on all PF_INET sockets, but only if the NuttX
 * configuration CONFIG_NET_ARPIOCTLS is defined. Each ioctl takes a pointer
 * to a 'struct arpreq' as its parameter.
 */

#define SIOCSARP        _ARPIOC(1) /* Set a ARP mapping */
#define SIOCDARP        _ARPIOC(2) /* Delete an ARP mapping */
#define SIOCGARP        _ARPIOC(3) /* Get an ARP mapping */

/* Definitions for bits in field arp_flags of struct arpreq.  If the
 * ATF_NETMASK flag is set, then arp_netmask should be valid.  This should
 * be set to 0xffffffff, or 0 to remove an existing arp entry.
 */

#define ATF_COM         (1 << 0)   /* Lookup complete */
#define ATF_PERM        (1 << 1)   /* Permanent entry */
#define ATF_PUBL        (1 << 2)   /* Publish entry */
#define ATF_USETRAILERS (1 << 3)   /* Trailers requested (obsolete) */
#define ATF_NETMASK     (1 << 4)   /* Use a netmask */
#define ATF_DONTPUB     (1 << 5)   /* Don't answer */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* All ARP ioctls take a pointer to a struct arpreq as their parameter: */

struct arpreq
{
  struct sockaddr arp_pa;          /* Protocol address */
  struct sockaddr arp_ha;          /* Hardware address */
  struct sockaddr arp_netmask;     /* Netmask of protocol address */
  uint8_t         arp_flags;       /* Flags */
  uint8_t         arp_dev[IFNAMSIZ+1]; /* Device name (zero terminated)*/
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* If CONFIG_NET_ARPIOCTLS is defined then the semi-standard ioctl commands
 * described above are supported.  If not, you can call the uIP ARP interfaces
 * directly in a very non-standard way.  See include/nuttx/net/uip/uip-arp.h for
 * prototypes.
 */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /*   __INCLUDE_NETINET_ARP_H */
