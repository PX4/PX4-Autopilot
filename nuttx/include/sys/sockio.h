/****************************************************************************
 * include/sys/sockio.h
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_SYS_SOCKIO_H
#define __INCLUDE_SYS_SOCKIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Get NuttX configuration and NuttX-specific network IOCTL definitions */

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/net/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define IMSFNAMSIZ 8

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

 /* RFC3678: IPv4 Options
  *
  * o  ioctl() SIOCGIPMSFILTER: to retrieve the list of source addresses
  *    that comprise the source filter along with the current filter mode.
  *
  * o  ioctl() SIOCSIPMSFILTER: to set or modify the source filter content
  *    (e.g., unicast source address list) or mode (exclude or include).
  *
  * Ioctl option                  Argument type
  * ----------------------------- ----------------------
  * SIOCGIPMSFILTER               struct ip_msfilter
  * SIOCSIPMSFILTER               struct ip_msfilter
  *
  * The imsf_fmode mode is a 32-bit integer that identifies the filter
  * mode.  The value of this field must be either MCAST_INCLUDE or
  * MCAST_EXCLUDE, which are likewise defined in <netinet/in.h>.
  */

#if 0 /* REVISIT: Current NuttX implementation is non-standard.
       * Lookup is by device name, not IP address.
       */

struct ip_msfilter
{
   struct in_addr imsf_multiaddr;  /* IP multicast address of group */
   struct in_addr imsf_interface;  /* Local IP address of interface */
   uint32_t       imsf_fmode;      /* Filter mode */
#ifdef CONFIG_NET_IGMPv3
   uint32_t       imsf_numsrc;     /* number of sources in src_list */
   struct in_addr imsf_slist[1];   /* start of source list */
#endif
};

#else

struct ip_msfilter  
{
   char imsf_name[IMSFNAMSIZ];     /* Network device name, e.g., "eth0" */
   struct in_addr imsf_multiaddr;  /* IP multicast address of group */
   uint32_t       imsf_fmode;      /* Filter mode */
};

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SOCKIO_H */
