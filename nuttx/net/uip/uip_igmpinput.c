/****************************************************************************
 * net/uip/uip_igminput.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY 
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
 * SUCH DAMAGE. 
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <wdog.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/uip/uipopt.h>
#include <nuttx/net/uip/uip.h>
#include <nuttx/net/uip/uip-igmp.h>

#include "uip_internal.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IGMPBUF     ((struct uip_igmphdr_s *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmpinput
 *
 * Description:
 *   An IGMP packet has been received.
 *
 *                              ________________
 *                             |                |
 *                             |                |
 *                             |                |
 *                             |                |
 *                  +--------->|   Non-Member   |<---------+
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |________________|          |
 *                  |                   |                  |
 *                  | leave group       | join group       | leave group
 *                  | (stop timer,      |(send report,     | (send leave
 *                  |  send leave if    | set flag,        |  if flag set)
 *                  |  flag set)        | start timer)     |
 *          ________|________           |          ________|________
 *         |                 |<---------+         |                 |
 *         |                 |                    |                 |
 *         |                 |<-------------------|                 |
 *         |                 |   query received   |                 |
 *         | Delaying Member |    (start timer)   |   Idle Member   |
 *   +---->|                 |------------------->|                 |
 *   |     |                 |   report received  |                 |
 *   |     |                 |    (stop timer,    |                 |
 *   |     |                 |     clear flag)    |                 |
 *   |     |_________________|------------------->|_________________|
 *   | query received    |        timer expired
 *   | (reset timer if   |        (send report,
 *   |  Max Resp Time    |         set flag)
 *   |  < current timer) |
 *   +-------------------+
 *
 * NOTE: This is most likely executing from an interrupt handler.
 *
 ****************************************************************************/

void uip_igmpinput(struct uip_driver_s *dev)
{
  FAR struct igmp_group_s *group;
  uip_ipaddr_t destipaddr;
  uip_ipaddr_t grpaddr;
  unsigned int ticks;

  nllvdbg("IGMP message: %04x%04x\n", IGMPBUF->destipaddr[1], IGMPBUF->destipaddr[0]);

  /* Verify the message length */

  if (dev->d_len < UIP_LLH_LEN+UIP_IPIGMPH_LEN)
    {
      IGMP_STATINCR(uip_stat.igmp.length_errors);
      nlldbg("Length error\n");
      return;
    }

  /* Calculate and check the IGMP checksum */

  if (uip_chksum((uint16_t*)&IGMPBUF->type, UIP_IGMPH_LEN) != 0)
    {
      IGMP_STATINCR(uip_stat.igmp.chksum_errors);
      nlldbg("Checksum error\n");
      return;
    }

  /* Find the group (or create a new one) using the incoming IP address*/

  destipaddr = uip_ip4addr_conv(IGMPBUF->destipaddr);
  group = uip_grpallocfind(dev, &destipaddr);
  if (!group)
    {
      nlldbg("Failed to allocate/find group: %08x\n", destipaddr);
      return;
    }

  /* Now handle the message based on the IGMP message type */

  switch (IGMPBUF->type)
    {
      case IGMP_MEMBERSHIP_QUERY:
        /* RFC 2236, 2.2.  ax Response Time
         *  "The Max Response Time field is meaningful only in Membership Query
         *   messages, and specifies the maximum allowed time before sending a
         *   responding report in units of 1/10 second.  In all other messages, it
         *   is set to zero by the sender and ignored by receivers.
         */

        /* Check if the query was sent to all systems */

        if (uip_ipaddr_cmp(destipaddr, g_allsystems))
          {
            /* Yes... Now check the if this this is a general or a group
             * specific query.
             *
             * RFC 2236, 2.1.  Type
             * There are two sub-types of Membership Query messages:
             * - General Query, used to learn which groups have members on an
             *   attached network.
             * - Group-Specific Query, used to learn if a particular group
             *   has any members on an attached network.
             *
             * RFC 2236, 2.4. Group Address
             *   "In a Membership Query message, the group address field is
             *    set to zero when sending a General Query, and set to the
             *    group address being queried when sending a Group-Specific
             *    Query."
             */

            if (IGMPBUF->grpaddr == 0)
              {
                FAR struct igmp_group_s *member;

                /* This is the general query */

                nllvdbg("General multicast query\n");
                if (IGMPBUF->maxresp == 0)
                  {
                    IGMP_STATINCR(uip_stat.igmp.v1_received);
                    IGMPBUF->maxresp = 10;

                    nlldbg("V1 not implemented\n");
                  }

                IGMP_STATINCR(uip_stat.igmp.query_received);
                for (member = (FAR struct igmp_group_s *)dev->grplist.head;
                     member;
                     member = member->next)
                  {
                    /* Skip over the all systems group entry */

                    if (!uip_ipaddr_cmp(member->grpaddr, g_allsystems))
                      {
                        ticks = uip_decisec2tick((int)IGMPBUF->maxresp);
                        if (IS_IDLEMEMBER(member->flags) ||
                            uip_igmpcmptimer(member, ticks))
                          {
                            uip_igmpstartticks(member, ticks);
                            CLR_IDLEMEMBER(member->flags);
                          }
                      }
                  }
              }
            else /* if (IGMPBUF->grpaddr != 0) */
              {
                nllvdbg("Group-specific multicast queury\n");

                /* We first need to re-lookup the group since we used dest last time.
                 * Use the incoming IPaddress!
                 */

                IGMP_STATINCR(uip_stat.igmp.ucast_query);
                grpaddr = uip_ip4addr_conv(IGMPBUF->grpaddr);
                group   = uip_grpallocfind(dev, &grpaddr);
                ticks   = uip_decisec2tick((int)IGMPBUF->maxresp);
                if (IS_IDLEMEMBER(group->flags) || uip_igmpcmptimer(group, ticks))
                  {
                    uip_igmpstartticks(group, ticks);
                    CLR_IDLEMEMBER(group->flags);
                  }
              }
          }

        /* Not sent to all systems -- Unicast query */

        else if (group->grpaddr != 0)
          {
            nllvdbg("Unitcast queury\n");
            IGMP_STATINCR(uip_stat.igmp.ucast_query);

            nlldbg("Query to a specific group with the group address as destination\n");

            ticks = uip_decisec2tick((int)IGMPBUF->maxresp);
            if (IS_IDLEMEMBER(group->flags) || uip_igmpcmptimer(group, ticks))
              {
                uip_igmpstartticks(group, ticks);
                CLR_IDLEMEMBER(group->flags);
              }
          }
        break;

      case IGMPv2_MEMBERSHIP_REPORT:
        {
          nllvdbg("Membership report\n");

          IGMP_STATINCR(uip_stat.igmp.report_received);
          if (!IS_IDLEMEMBER(group->flags))
            {
              /* This is on a specific group we have already looked up */

              wd_cancel(group->wdog);
              SET_IDLEMEMBER(group->flags);
              CLR_LASTREPORT(group->flags);
            }
        }
      break;

      default:
        {
          nlldbg("Unexpected msg %02x\n", IGMPBUF->type);
        }
      break;
    }
}

#endif /* CONFIG_NET_IGMP */
