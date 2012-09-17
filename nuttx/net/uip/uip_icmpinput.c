/****************************************************************************
 * net/uip/uip_icmpinput.c
 * Handling incoming ICMP/ICMP6 input
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
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

#include <stdint.h>
#include <debug.h>

#include <net/if.h>
#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-arch.h>

#include "uip_internal.h"

#ifdef CONFIG_NET_ICMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPBUF ((struct uip_icmpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_PING
struct uip_callback_s *g_echocallback = NULL;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_icmpinput
 *
 * Description:
 *   Handle incoming ICMP/ICMP6 input
 *
 * Parameters:
 *   dev - The device driver structure containing the received ICMP/ICMP6
 *         packet
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_icmpinput(struct uip_driver_s *dev)
{
  struct uip_icmpip_hdr *picmp = ICMPBUF;

#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.recv++;
#endif

#ifndef CONFIG_NET_IPv6
  /* ICMPv4 processing code follows. */

  /* ICMP echo (i.e., ping) processing. This is simple, we only change the
   * ICMP type from ECHO to ECHO_REPLY and adjust the ICMP checksum before
   * we return the packet.
   */

  if (picmp->type == ICMP_ECHO_REQUEST)
    {
      /* If we are configured to use ping IP address assignment, we use
       * the destination IP address of this ping packet and assign it to
       * ourself.
       */

#ifdef CONFIG_NET_PINGADDRCONF
      if (dev->d_ipaddr == 0)
        {
          dev->d_ipaddr = picmp->destipaddr;
        }
#endif

      /* Change the ICMP type */

      picmp->type = ICMP_ECHO_REPLY;

      /* Swap IP addresses. */

      uiphdr_ipaddr_copy(picmp->destipaddr, picmp->srcipaddr);
      uiphdr_ipaddr_copy(picmp->srcipaddr, &dev->d_ipaddr);

      /* Recalculate the ICMP checksum */

#if 0
      /* The slow way... sum over the ICMP message */

      picmp->icmpchksum = 0;
      picmp->icmpchksum = ~uip_icmpchksum(dev, (((uint16_t)picmp->len[0] << 8) | (uint16_t)picmp->len[1]) - UIP_IPH_LEN);
      if (picmp->icmpchksum == 0)
        {
          picmp->icmpchksum = 0xffff;
        }
#else
      /* The quick way -- Since only the type has changed, just adjust the
       * checksum for the change of type
       */

      if( picmp->icmpchksum >= HTONS(0xffff - (ICMP_ECHO_REQUEST << 8)))
        {
          picmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8) + 1;
        }
      else
        {
          picmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8);
        }
#endif

      nllvdbg("Outgoing ICMP packet length: %d (%d)\n",
              dev->d_len, (picmp->len[0] << 8) | picmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
      uip_stat.icmp.sent++;
      uip_stat.ip.sent++;
#endif
    }

  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

#ifdef CONFIG_NET_ICMP_PING
  else if (picmp->type == ICMP_ECHO_REPLY && g_echocallback)
    {
      (void)uip_callbackexecute(dev, picmp, UIP_ECHOREPLY, g_echocallback);
    }
#endif

  /* Otherwise the ICMP input was not processed */

  else
    {
      nlldbg("Unknown ICMP cmd: %d\n", picmp->type);
      goto typeerr;
    }

  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.typeerr++;
  uip_stat.icmp.drop++;
#endif
  dev->d_len = 0;

#else /* !CONFIG_NET_IPv6 */

  /* If we get a neighbor solicitation for our address we should send
   * a neighbor advertisement message back.
   */

  if (picmp->type == ICMP6_NEIGHBOR_SOLICITATION)
    {
      if (uip_ipaddr_cmp(picmp->icmp6data, dev->d_ipaddr))
        {
          if (picmp->options[0] == ICMP6_OPTION_SOURCE_LINK_ADDRESS)
            {
              /* Save the sender's address in our neighbor list. */

              uiphdr_neighbor_add(picmp->srcipaddr, &(picmp->options[2]));
            }

          /* We should now send a neighbor advertisement back to where the
           * neighbor solicication came from.
           */

          picmp->type = ICMP6_NEIGHBOR_ADVERTISEMENT;
          picmp->flags = ICMP6_FLAG_S; /* Solicited flag. */

          picmp->reserved1 = picmp->reserved2 = picmp->reserved3 = 0;

          uiphdr_ipaddr_copy(picmp->destipaddr, picmp->srcipaddr);
          uiphdr_ipaddr_copy(picmp->srcipaddr, &dev->d_ipaddr);
          picmp->options[0] = ICMP6_OPTION_TARGET_LINK_ADDRESS;
          picmp->options[1] = 1;  /* Options length, 1 = 8 bytes. */
          memcpy(&(picmp->options[2]), &dev->d_mac, IFHWADDRLEN);
          picmp->icmpchksum = 0;
          picmp->icmpchksum = ~uip_icmp6chksum(dev);
        }
      else
        {
          goto drop;
        }
    }
  else if (picmp->type == ICMP6_ECHO_REQUEST)
    {
      /* ICMP echo (i.e., ping) processing. This is simple, we only
       * change the ICMP type from ECHO to ECHO_REPLY and update the
       * ICMP checksum before we return the packet.
       */

      picmp->type = ICMP6_ECHO_REPLY;

      uiphdr_ipaddr_copy(picmp->destipaddr, picmp->srcipaddr);
      uiphdr_ipaddr_copy(picmp->srcipaddr, &dev->d_ipaddr);
      picmp->icmpchksum = 0;
      picmp->icmpchksum = ~uip_icmp6chksum(dev);
    }

  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

#ifdef CONFIG_NET_ICMP_PING
  else if (picmp->type == ICMP6_ECHO_REPLY && g_echocallback)
    {
      uint16_t flags = UIP_ECHOREPLY;

      if (g_echocallback)
        {
          /* Dispatch the ECHO reply to the waiting thread */

          flags = uip_callbackexecute(dev, picmp, flags, g_echocallback);
        }

      /* If the ECHO reply was not handled, then drop the packet */

      if (flags == UIP_ECHOREPLY)
        {
          /* The ECHO reply was not handled */

          goto drop;
        }
    }
#endif

  else
    {
      nlldbg("Unknown ICMP6 cmd: %d\n", picmp->type);
      goto typeerr;
    }

  nllvdbg("Outgoing ICMP6 packet length: %d (%d)\n",
          dev->d_len, (picmp->len[0] << 8) | picmp->len[1]);

#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.sent++;
  uip_stat.ip.sent++;
#endif
  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.typeerr++;
#endif

drop:
#ifdef CONFIG_NET_STATISTICS
  uip_stat.icmp.drop++;
#endif
  dev->d_len = 0;

#endif /* !CONFIG_NET_IPv6 */
}

#endif /* CONFIG_NET_ICMP */
#endif /* CONFIG_NET */
