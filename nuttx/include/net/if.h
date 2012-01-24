/****************************************************************************
 * include/net/if.h
 *
 *   Copyright (C) 2007, 2008, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __NET_IF_H
#define __NET_IF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Sizing parameters */

#define IFNAMSIZ        6   /* Older naming standard */
#define IF_NAMESIZE     6   /* Newer naming standard */
#define IFHWADDRLEN     6

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct ifreq
{
  char       ifr_name[IFNAMSIZ];      /* Network device name (e.g. "eth0") */
  union
  {
    struct sockaddr ifru_addr;        /* IP Address */
    struct sockaddr ifru_dstaddr;     /* P-to-P Address */
    struct sockaddr ifru_broadaddr;   /* Broadcast address */
    struct sockaddr ifru_netmask;     /* Netmask */
    struct sockaddr ifru_hwaddr;      /* MAC address */
    int     ifru_count;               /* Number of devices */
    int     ifru_mtu;                 /* MTU size */
  } ifr_ifru;
};

#define ifr_addr        ifr_ifru.ifru_addr      /* IP address */
#define ifr_dstaddr     ifr_ifru.ifru_dstaddr   /* P-to-P Address */
#define ifr_broadaddr   ifr_ifru.ifru_broadaddr /* Broadcast address */
#define ifr_netmask     ifr_ifru.ifru_netmask   /* Interface net mask */
#define ifr_hwaddr      ifr_ifru.ifru_hwaddr    /* MAC address */
#define ifr_mtu         ifr_ifru.ifru_mtu       /* MTU */
#define ifr_count       ifr_ifru.ifru_count     /* Number of devices */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __NET_IF_H */
