/****************************************************************************
 * apps/nshlib/nsh_netinit.c
 *
 *   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is influenced by similar logic from uIP:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <net/if.h>

#include <apps/netutils/uiplib.h>
#if defined(CONFIG_NSH_DHCPC) || defined(CONFIG_NSH_DNS)
#  include <apps/netutils/resolv.h>
#  include <apps/netutils/dhcpc.h>
#endif

#include "nsh.h"

#ifdef CONFIG_NET

/****************************************************************************
 * Definitions
 ****************************************************************************/

#if defined(CONFIG_NSH_DRIPADDR) && !defined(CONFIG_NSH_DNSIPADDR)
#  define CONFIG_NSH_DNSIPADDR CONFIG_NSH_DRIPADDR
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_netinit
 *
 * Description:
 *   Initialize the network per the selected NuttX configuration
 *
 ****************************************************************************/

int nsh_netinit(void)
{
 struct in_addr addr;
#if defined(CONFIG_NSH_DHCPC)
 FAR void *handle;
#endif
#if defined(CONFIG_NSH_DHCPC) || defined(CONFIG_NSH_NOMAC)
 uint8_t mac[IFHWADDRLEN];
#endif

/* Many embedded network interfaces must have a software assigned MAC */

#ifdef CONFIG_NSH_NOMAC
  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0xef;
  uip_setmacaddr("eth0", mac);
#endif

  /* Set up our host address */

#if !defined(CONFIG_NSH_DHCPC)
  addr.s_addr = HTONL(CONFIG_NSH_IPADDR);
#else
  addr.s_addr = 0;
#endif
  uip_sethostaddr("eth0", &addr);

  /* Set up the default router address */

  addr.s_addr = HTONL(CONFIG_NSH_DRIPADDR);
  uip_setdraddr("eth0", &addr);

  /* Setup the subnet mask */

  addr.s_addr = HTONL(CONFIG_NSH_NETMASK);
  uip_setnetmask("eth0", &addr);

#if defined(CONFIG_NSH_DHCPC) || defined(CONFIG_NSH_DNS)
  /* Set up the resolver */

  resolv_init();
#if defined(CONFIG_NSH_DNS)
  addr.s_addr = HTONL(CONFIG_NSH_DNSIPADDR);
  resolv_conf(&addr);
#endif
#endif

#if defined(CONFIG_NSH_DHCPC)
  /* Get the MAC address of the NIC */

  uip_getmacaddr("eth0", mac);

  /* Set up the DHCPC modules */

  handle = dhcpc_open(&mac, IFHWADDRLEN);

  /* Get an IP address.  Note that there is no logic for renewing the IP address in this
   * example.  The address should be renewed in ds.lease_time/2 seconds.
   */

  if (handle)
    {
        struct dhcpc_state ds;
        (void)dhcpc_request(handle, &ds);
        uip_sethostaddr("eth0", &ds.ipaddr);
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
    }
#endif

  return OK;
}

#endif /* CONFIG_NET */
