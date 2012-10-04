/****************************************************************************
 * examples/igmp/igmp.c
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

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <debug.h>

#include <net/if.h>
#include <nuttx/net/uip/uip.h>
#include <apps/netutils/uiplib.h>
#include <apps/netutils/ipmsfilter.h>

#include "igmp.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Check if the destination address is a multicast address
 *
 * - IPv4: multicast addresses lie in the class D group -- The address range
 *   224.0.0.0 to 239.255.255.255 (224.0.0.0/4)
 *
 * - IPv6 multicast addresses are have the high-order octet of the
 *   addresses=0xff (ff00::/8.)
 */

#if ((CONFIG_EXAMPLES_IGMP_GRPADDR & 0xffff0000) < 0xe0000000ul) || \
    ((CONFIG_EXAMPLES_IGMP_GRPADDR & 0xffff0000) > 0xeffffffful)
#  error "Bad range for IGMP group address"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * igmp_main
 ****************************************************************************/

int igmp_main(int argc, char *argv[])
{
  struct in_addr addr;
#if defined(CONFIG_EXAMPLES_IGMP_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif

  message("Configuring Ethernet...\n");
  
  /* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLES_IGMP_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_IGMP_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_IGMP_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_IGMP_NETMASK);
  uip_setnetmask("eth0", &addr);

  /* Not much of a test for now */
  /* Join the group */

  message("Join group...\n");
  addr.s_addr = HTONL(CONFIG_EXAMPLES_IGMP_GRPADDR);
  ipmsfilter("eth0", &addr, MCAST_INCLUDE);

  /* Wait a while */

  message("Wait for timeout...\n");
  sleep(5);

  /* Leave the group */

  message("Leave group...\n");
  ipmsfilter("eth0", &addr, MCAST_EXCLUDE);

  /* Wait a while */

  sleep(5);
  message("Exiting...\n");
  return 0;
}
