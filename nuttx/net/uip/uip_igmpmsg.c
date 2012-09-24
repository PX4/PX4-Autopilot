/****************************************************************************
 * net/uip/uip_igmpmgs.c
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
 * Name: uip_igmpschedmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval.
 *
 * Assumptions:
 *   This function may be called in most any context.
 *
 ****************************************************************************/

void uip_igmpschedmsg(FAR struct igmp_group_s *group, uint8_t msgid)
{
  uip_lock_t flags;

  /* The following should be atomic */

  flags = uip_lock();
  DEBUGASSERT(!IS_SCHEDMSG(group->flags));
  group->msgid = msgid;
  SET_SCHEDMSG(group->flags);
  uip_unlock(flags);
}

/****************************************************************************
 * Name: uip_igmpwaitmsg
 *
 * Description:
 *   Schedule a message to be send at the next driver polling interval and
 *   block, waiting for the message to be sent.
 *
 * Assumptions:
 *   This function cannot be called from an interrupt handler (if you try it,
 *   uip_lockedwait will assert).
 *
 ****************************************************************************/

void uip_igmpwaitmsg(FAR struct igmp_group_s *group, uint8_t msgid)
{
  uip_lock_t flags;

  /* Schedule to send the message */

  flags = uip_lock();
  DEBUGASSERT(!IS_WAITMSG(group->flags));
  SET_WAITMSG(group->flags);
  uip_igmpschedmsg(group, msgid);

  /* Then wait for the message to be sent */

  while (IS_SCHEDMSG(group->flags))
    {
      /* Wait for the semaphore to be posted */

      while (uip_lockedwait(&group->sem) != 0)
        {
          /* The only error that should occur from uip_lockedwait() is if
           * the wait is awakened by a signal.
           */
 
          ASSERT(errno == EINTR);
        }
    }

  /* The message has been sent and we are no longer waiting */

  CLR_WAITMSG(group->flags);
  uip_unlock(flags);
}

#endif /* CONFIG_NET_IGMP */
