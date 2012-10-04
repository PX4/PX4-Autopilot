/****************************************************************************
 * examples/dhcpd/target.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <apps/netutils/uiplib.h>
#include <apps/netutils/dhcpd.h>

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/* Configuation Checkes *****************************************************/
/* BEWARE:
 * There are other configuration settings needed in netutils/dhcpd/dhcpdc.c,
 * but there are default values for those so we cannot check them here.
 */

#ifndef CONFIG_EXAMPLES_DHCPD_IPADDR
#  error "You must define CONFIG_EXAMPLES_DHCPD_IPADDR"
#endif

#ifndef CONFIG_EXAMPLES_DHCPD_DRIPADDR
#  error "You must define CONFIG_EXAMPLES_DHCPD_DRIPADDR"
#endif

#ifndef CONFIG_EXAMPLES_DHCPD_NETMASK
#  error "You must define CONFIG_EXAMPLES_DHCPD_NETMASK"
#endif

#ifndef CONFIG_NET
#  error "You must define CONFIG_NET"
#endif

#ifndef CONFIG_NET_UDP
#  error "You must define CONFIG_NET_UDP"
#endif

#ifndef CONFIG_NET_BROADCAST
#  error "You must define CONFIG_NET_BROADCAST"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: dhcpd_main
 ****************************************************************************/

int dhcpd_main(int argc, char *argv[])
{
  struct in_addr addr;
#if defined(CONFIG_EXAMPLES_DHCPD_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLES_DHCPD_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_DHCPD_IPADDR);
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_DHCPD_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLES_DHCPD_NETMASK);
  uip_setnetmask("eth0", &addr);

  /* Then start the server */
  
  dhcpd_run();
  return 0;
}
