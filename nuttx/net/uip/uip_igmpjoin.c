/****************************************************************************
 * net/uip/uip_igmpjoin.c
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_joingroup
 *
 * Description:
 *   Add the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host joins a multicast group, it should immediately transmit
 *   an unsolicited Version 2 Membership Report for that group, in case it
 *   is the first member of that group on the network.  To cover the
 *   possibility of the initial Membership Report being lost or damaged,
 *   it is recommended that it be repeated once or twice after short
 *   delays [Unsolicited Report Interval].  (A simple way to accomplish
 *   this is to send the initial Version 2 Membership Report and then act
 *   as if a Group-Specific Query was received for that group, and set a
 *   timer appropriately)."
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
 * Assumptions:
 *   This function cannot be called from interrupt handling logic!
 *
 ****************************************************************************/

int igmp_joingroup(struct uip_driver_s *dev, FAR const struct in_addr *grpaddr)
{
  struct igmp_group_s *group;

  DEBUGASSERT(dev && grpaddr);

  /* Check if a this address is already in the group */
 
  group = uip_grpfind(dev, &grpaddr->s_addr);
  if (!group)
    {
       /* No... allocate a new entry */
 
       nvdbg("Join to new group: %08x\n", grpaddr->s_addr);
       group = uip_grpalloc(dev, &grpaddr->s_addr);
       IGMP_STATINCR(uip_stat.igmp.joins);

       /* Send the Membership Report */
 
       IGMP_STATINCR(uip_stat.igmp.report_sched);
       uip_igmpwaitmsg(group, IGMPv2_MEMBERSHIP_REPORT);

       /* And start the timer at 10*100 msec */

       uip_igmpstarttimer(group, 10);

       /* Add the group (MAC) address to the ether drivers MAC filter list */

       uip_addmcastmac(dev, (FAR uip_ipaddr_t *)&grpaddr->s_addr);
       return OK;
    }

  /* Return EEXIST if the address is already a member of the group */

  return -EEXIST;
}

#endif /* CONFIG_NET_IGMP */
