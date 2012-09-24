/****************************************************************************
 * net/uip/uip_igmpleave.c
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host leaves a multicast group, if it was the last host to
 *   reply to a Query with a Membership Report for that group, it SHOULD
 *   send a Leave Group message to the all-routers multicast group
 *   (224.0.0.2). If it was not the last host to reply to a Query, it MAY
 *   send nothing as there must be another member on the subnet.  This is
 *   an optimization to reduce traffic; a host without sufficient storage
 *   to remember whether or not it was the last host to reply MAY always
 *   send a Leave Group message when it leaves a group.  Routers SHOULD
 *   accept a Leave Group message addressed to the group being left, in
 *   order to accommodate implementations of an earlier version of this
 *   standard.  Leave Group messages are addressed to the all-routers
 *   group because other group members have no need to know that a host
 *   has left the group, but it does no harm to address the message to the
 *   group."
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
 * Assumptions:
 *   This function cannot be called from interrupt handling logic!
 *
 ****************************************************************************/

int igmp_leavegroup(struct uip_driver_s *dev, FAR const struct in_addr *grpaddr)
{
  struct igmp_group_s *group;
  uip_lock_t flags;

  DEBUGASSERT(dev && grpaddr);

  /* Find the entry corresponding to the address leaving the group */

  group = uip_grpfind(dev, &grpaddr->s_addr);
  ndbg("Leaving group: %p\n", group);
  if (group)
    {
      /* Cancel the timer and discard any queued Membership Reports.  Canceling
       * the timer will prevent any new Membership Reports from being sent;
       * clearing the flags will discard any pending Membership Reports that
       * could interfere with the Leave Group.
       */
 
      flags = uip_lock();
      wd_cancel(group->wdog);
      CLR_SCHEDMSG(group->flags);
      CLR_WAITMSG(group->flags);
      uip_unlock(flags);

      IGMP_STATINCR(uip_stat.igmp.leaves);

      /* Send a leave if the flag is set according to the state diagram */

      if (IS_LASTREPORT(group->flags))
        {
          ndbg("Schedule Leave Group message\n");
          IGMP_STATINCR(uip_stat.igmp.leave_sched);
          uip_igmpwaitmsg(group, IGMP_LEAVE_GROUP);
        }

      /* Free the group structure (state is now Non-Member */

      uip_grpfree(dev, group);

      /* And remove the group address from the ethernet drivers MAC filter set */

      uip_removemcastmac(dev, (FAR uip_ipaddr_t *)&grpaddr->s_addr);
      return OK;
    }

  /* Return ENOENT if the address is not a member of the group */

  nvdbg("Return -ENOENT\n");
  return -ENOENT;
}

#endif /* CONFIG_NET_IGMP */
