/****************************************************************************
 * netutils/uiplib/uip_setdraddr.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <netinet/in.h>
#include <net/if.h>

#include <apps/netutils/uiplib.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_setdraddr
 *
 * Description:
 *   Set the default router IP address
 *
 * Parameters:
 *   ifname   The name of the interface to use
 *   ipaddr   The address to set
 *
 * Return:
 *   0 on sucess; -1 on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int uip_setdraddr(const char *ifname, const struct in6_addr *addr)
#else
int uip_setdraddr(const char *ifname, const struct in_addr *addr)
#endif
{
  int ret = ERROR;
  if (ifname && addr)
    {
      int sockfd = socket(PF_INET, UIPLIB_SOCK_IOCTL, 0);
      if (sockfd >= 0)
        {
          struct ifreq req;
#ifdef CONFIG_NET_IPv6
          struct sockaddr_in6 *inaddr;
#else
          struct sockaddr_in  *inaddr;
#endif
          /* Add the device name to the request */

          strncpy(req.ifr_name, ifname, IFNAMSIZ);

          /* Add the INET address to the request */

#ifdef CONFIG_NET_IPv6
          inaddr             = (struct sockaddr_in6 *)&req.ifr_addr;
          inaddr->sin_family = AF_INET6;
          inaddr->sin_port   = 0;
          memcpy(&inaddr->sin6_addr, addr, sizeof(struct in6_addr));
#else
          inaddr             = (struct sockaddr_in *)&req.ifr_addr;
          inaddr->sin_family = AF_INET;
          inaddr->sin_port   = 0;
          memcpy(&inaddr->sin_addr, addr, sizeof(struct in_addr));
#endif
          ret = ioctl(sockfd, SIOCSIFDSTADDR, (unsigned long)&req);
          close(sockfd);
        }
    }
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
