/****************************************************************************
 * examples/discover/main.c
 *
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *
 *   Authors: Max Holtzberg <mh@uvc.de>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <debug.h>

#include <net/if.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arp.h>

#include <apps/netutils/uiplib.h>
#include <apps/netutils/discover.h>

#ifdef CONFIG_EXAMPLE_DISCOVER_DHCPC
#  include <arpa/inet.h>
#endif

/* Here we include the header file for the application(s) we use in
 * our project as defined in the config/<board-name>/defconfig file
 */

/* DHCPC may be used in conjunction with any other feature (or not) */

#ifdef CONFIG_EXAMPLE_DISCOVER_DHCPC
#  include <apps/netutils/resolv.h>
#  include <apps/netutils/dhcpc.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * discover_main
 ****************************************************************************/

int discover_main(int argc, char *argv[])
{
#ifndef CONFIG_NSH_BUILTIN_APPS
  struct in_addr addr;
#if defined(CONFIG_EXAMPLE_DISCOVER_DHCPC) || defined(CONFIG_EXAMPLE_DISCOVER_NOMAC)
  uint8_t mac[IFHWADDRLEN];
#endif
#ifdef CONFIG_EXAMPLE_DISCOVER_DHCPC
  void *handle;
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_EXAMPLE_DISCOVER_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

#ifdef CONFIG_EXAMPLE_DISCOVER_DHCPC
  addr.s_addr = 0;
#else
  addr.s_addr = HTONL(CONFIG_EXAMPLE_DISCOVER_IPADDR);
#endif
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_DISCOVER_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_EXAMPLE_DISCOVER_NETMASK);
  uip_setnetmask("eth0", &addr);

#ifdef CONFIG_EXAMPLE_DISCOVER_DHCPC
  /* Set up the resolver */

  resolv_init();

  /* Get the MAC address of the NIC */

  uip_getmacaddr("eth0", mac);

  /* Set up the DHCPC modules */

  handle = dhcpc_open(&mac, IFHWADDRLEN);

  /* Get an IP address.  Note:  there is no logic here for renewing the address in this
   * example.  The address should be renewed in ds.lease_time/2 seconds.
   */

  printf("Getting IP address\n");
  if (handle)
    {
        struct dhcpc_state ds;
        (void)dhcpc_request(handle, &ds);
        uip_sethostaddr("eth1", &ds.ipaddr);

        if (ds.netmask.s_addr != 0)
          {
            uip_setnetmask("eth0", &ds.netmask);
          }

        if (ds.default_router.s_addr != 0)
          {
            uip_setdraddr("eth0", &ds.default_router);
          }

        if (ds.dnsaddr.s_addr != 0)
          {
            resolv_conf(&ds.dnsaddr);
          }

        dhcpc_close(handle);
        printf("IP: %s\n", inet_ntoa(ds.ipaddr));
    }
#endif
#endif /* CONFIG_NSH_BUILTIN_APPS */

  if (discover_start() < 0)
    {
      ndbg("Could not start discover daemon.\n");
      return ERROR;
    }

  return OK;
}

