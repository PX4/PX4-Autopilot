/****************************************************************************
 * netutils/uiplib/uip_setmultiaddr.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <netinet/in.h>
#include <sys/sockio.h>

#include <apps/netutils/uiplib.h>
#include <apps/netutils/ipmsfilter.h>

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipmsfilter
 *
 * Description:
 *   Add or remove an IP address from a multicast filter set.
 *
 * Parameters:
 *   ifname     The name of the interface to use, size must less than IMSFNAMSIZ
 *   multiaddr  Multicast group address to add/remove (network byte order)
 *   fmode      MCAST_INCLUDE: Add multicast address
 *              MCAST_EXCLUDE: Remove multicast address
 *
 * Return:
 *   0 on sucess; Negated errno on failure
 *
 ****************************************************************************/

int ipmsfilter(FAR const char *ifname, FAR const struct in_addr *multiaddr,
               uint32_t fmode)
{
  int ret = ERROR;

  nvdbg("ifname: %s muliaddr: %08x fmode: %ld\n", ifname, *multiaddr, fmode);
  if (ifname && multiaddr)
    {
      /* Get a socket (only so that we get access to the INET subsystem) */

      int sockfd = socket(PF_INET, UIPLIB_SOCK_IOCTL, 0);
      if (sockfd >= 0)
        {
          struct ip_msfilter imsf;

          /* Put the driver name into the request */

          strncpy(imsf.imsf_name, ifname, IMSFNAMSIZ);

          /* Put the new address into the request */

          imsf.imsf_multiaddr.s_addr = multiaddr->s_addr;

          /* Perforom the ioctl to set the MAC address */

          imsf.imsf_fmode = fmode;
          ret = ioctl(sockfd, SIOCSIPMSFILTER, (unsigned long)&imsf);
          close(sockfd);
        }
    }
  return ret;
}

#endif /* CONFIG_NET_IGM */
