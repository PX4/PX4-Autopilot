/****************************************************************************
 * net/uip/uip_arptab.c
 * Implementation of the ARP Address Resolution Protocol.
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based originally on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET

#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <netinet/in.h>

#include <net/ethernet.h>
#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip-arch.h>
#include <nuttx/net/uip/uip-arp.h>

#ifdef CONFIG_NET_ARP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The table of known address mappings */

static struct arp_entry g_arptable[CONFIG_NET_ARPTAB_SIZE];
static uint8_t g_arptime;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_arp_init
 *
 * Description:
 *   Initialize the ARP module. This function must be called before any of
 *   the other ARP functions.
 *
 ****************************************************************************/

void uip_arp_init(void)
{
  int i;
  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      memset(&g_arptable[i].at_ipaddr, 0, sizeof(in_addr_t));
    }
}

/****************************************************************************
 * Name: uip_arp_timer
 *
 * Description:
 *   This function performs periodic timer processing in the ARP module
 *   and should be called at regular intervals. The recommended interval
 *   is 10 seconds between the calls.  It is responsible for flushing old
 *   entries in the ARP table.
 *
 ****************************************************************************/

void uip_arp_timer(void)
{
  struct arp_entry *tabptr;
  int i;

  ++g_arptime;
  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];
      if (tabptr->at_ipaddr != 0 && g_arptime - tabptr->at_time >= UIP_ARP_MAXAGE)
        {
          tabptr->at_ipaddr = 0;
        }
    }
}

/****************************************************************************
 * Name: uip_arp_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input parameters:
 *   pipaddr - Refers to an IP address uint16_t[2]
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Assumptions
 *   Interrupts are disabled
 *
 ****************************************************************************/

void uip_arp_update(uint16_t *pipaddr, uint8_t *ethaddr)
{
  struct arp_entry *tabptr = NULL;
  in_addr_t         ipaddr = uip_ip4addr_conv(pipaddr);
  int               i;

  /* Walk through the ARP mapping table and try to find an entry to
   * update. If none is found, the IP -> MAC address mapping is
   * inserted in the ARP table.
   */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];

      /* Only check those entries that are actually in use. */

      if (tabptr->at_ipaddr != 0)
        {
          /* Check if the source IP address of the incoming packet matches
           * the IP address in this ARP table entry.
           */

          if (uip_ipaddr_cmp(ipaddr, tabptr->at_ipaddr))
            {
              /* An old entry found, update this and return. */

              memcpy(tabptr->at_ethaddr.ether_addr_octet, ethaddr, ETHER_ADDR_LEN);
              tabptr->at_time = g_arptime;
              return;
            }
        }
    }

  /* If we get here, no existing ARP table entry was found, so we create one. */

  /* First, we try to find an unused entry in the ARP table. */

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];
      if (tabptr->at_ipaddr == 0)
        {
          break;
        }
    }

  /* If no unused entry is found, we try to find the oldest entry and
   * throw it away.
   */

  if (i == CONFIG_NET_ARPTAB_SIZE)
    {
      uint8_t tmpage = 0;
      int   j      = 0;
      for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
        {
          tabptr = &g_arptable[i];
          if (g_arptime - tabptr->at_time > tmpage)
            {
              tmpage = g_arptime - tabptr->at_time;
              j = i;
            }
        }
      i = j;
      tabptr = &g_arptable[i];
    }

  /* Now, i is the ARP table entry which we will fill with the new
   * information.
   */

  tabptr->at_ipaddr = ipaddr;
  memcpy(tabptr->at_ethaddr.ether_addr_octet, ethaddr, ETHER_ADDR_LEN);
  tabptr->at_time = g_arptime;
}

/****************************************************************************
 * Name: uip_arp_find
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address.
 *
 * Input parameters:
 *   ipaddr - Refers to an IP addressin network order
 *
 * Assumptions
 *   Interrupts are disabled; Returned value will become unstable when
 *   interrupts are re-enabled or if any other uIP APIs are called.
 *
 ****************************************************************************/

struct arp_entry *uip_arp_find(in_addr_t ipaddr)
{
  struct arp_entry *tabptr;
  int i;

  for (i = 0; i < CONFIG_NET_ARPTAB_SIZE; ++i)
    {
      tabptr = &g_arptable[i];
      if (uip_ipaddr_cmp(ipaddr, tabptr->at_ipaddr))
        {
          return tabptr;
        }
    }
  return NULL;
}

#endif /* CONFIG_NET_ARP */
#endif /* CONFIG_NET */
