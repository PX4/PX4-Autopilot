/****************************************************************************
 * net/uip/uip_igmptimer.c
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
#include <nuttx/compiler.h>

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

/* Debug ********************************************************************/

#undef IGMP_GTMRDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GTMRDEBUG
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef IGMP_GTMRDEBUG
#    define gtmrdbg(format, arg...)    ndbg(format, ##arg)
#    define gtmrlldbg(format, arg...)  nlldbg(format, ##arg)
#    define gtmrvdbg(format, arg...)   nvdbg(format, ##arg)
#    define gtmrllvdbg(format, arg...) nllvdbg(format, ##arg)
#  else
#    define gtmrdbg(x...)
#    define gtmrlldbg(x...)
#    define gtmrvdbg(x...)
#    define gtmrllvdbg(x...)
#  endif
#else
#  ifdef IGMP_GTMRDEBUG
#    define gtmrdbg    ndbg
#    define gtmrlldbg  nlldbg
#    define gtmrvdbg   nvdbg
#    define gtmrllvdbg nllvdbg
#  else
#    define gtmrdbg    (void)
#    define gtmrlldbg  (void)
#    define gtmrvdbg   (void)
#    define gtmrllvdbg (void)
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmptimeout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void uip_igmptimeout(int argc, uint32_t arg, ...)
{
  FAR struct igmp_group_s *group;

  /* If the state is DELAYING_MEMBER then we send a report for this group */

  nllvdbg("Timeout!\n");
  group = (FAR struct igmp_group_s *)arg;
  DEBUGASSERT(argc == 1 && group);

  /* If the group exists and is no an IDLE MEMBER, then it must be a DELAYING
   * member.  Race conditions are avoided because (1) the timer is not started
   * until after the first IGMPv2_MEMBERSHIP_REPORT during the join, and (2)
   * the timer is canceled before sending the IGMP_LEAVE_GROUP during a leave.
   */

  if (!IS_IDLEMEMBER(group->flags))
    {
      /* Schedule (and forget) the Membership Report.  NOTE:
       * Since we are executing from a timer interrupt, we cannot wait
       * for the message to be sent.
       */

      IGMP_STATINCR(uip_stat.igmp.report_sched);
      uip_igmpschedmsg(group, IGMPv2_MEMBERSHIP_REPORT);

      /* Also note:  The Membership Report is sent at most two times becasue
       * the timer is not reset here.  Hmm.. does this mean that the group
       * is stranded if both reports were lost?  This is consistent with the
       * RFC that states: "To cover the possibility of the initial Membership
       * Report being lost or damaged, it is recommended that it be repeated
       * once or twice after shortdelays [Unsolicited Report Interval]..."
       */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  uip_igmpstarttimer
 *
 * Description:
 *   Start the IGMP timer.
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

int uip_decisec2tick(int decisecs)
{
  /* Convert the deci-second comparison value to clock ticks.  The CLK_TCK
   * value is the number of clock ticks per second; decisecs argument is the
   * maximum delay in 100's of milliseconds.  CLK_TCK/10 is then the number
   * of clock ticks in 100 milliseconds.
   */

  return CLK_TCK * decisecs / 10;
}

/****************************************************************************
 * Name:  uip_igmpstartticks and uip_igmpstarttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

void uip_igmpstartticks(FAR struct igmp_group_s *group, int ticks)
{
  int ret;

  /* Start the timer */

  gtmrlldbg("ticks: %d\n", ticks);
  ret = wd_start(group->wdog, ticks, uip_igmptimeout, 1, (uint32_t)group);
  DEBUGASSERT(ret == OK);
}

void uip_igmpstarttimer(FAR struct igmp_group_s *group, uint8_t decisecs)
{
  /* Convert the decisec value to system clock ticks and start the timer.
   * Important!! this should be a random timer from 0 to decisecs
   */
  
  gtmrdbg("decisecs: %d\n", decisecs);
  uip_igmpstartticks(group, uip_decisec2tick(decisecs));
}

/****************************************************************************
 * Name:  uip_igmpcmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 * Assumptions:
 *   This function may be called from most any context.  If true is retuend
 *   then the caller must call uip_igmpstartticks() to restart the timer
 *
 ****************************************************************************/

bool uip_igmpcmptimer(FAR struct igmp_group_s *group, int maxticks)
{
  uip_lock_t flags;
  int remaining;

  /* Disable interrupts so that there is no race condition with the actual
   * timer expiration.
   */

  flags = uip_lock();

  /* Get the timer remaining on the watchdog.  A time of <= zero means that
   * the watchdog was never started.
   */

  remaining = wd_gettime(group->wdog);

  /* A remaining time of zero means that the watchdog was never started
   * or has already expired.  That case should be covered in the following
   * test as well.
   */

  gtmrdbg("maxticks: %d remaining: %d\n", maxticks, remaining);
  if (maxticks > remaining)
    {
      /* Cancel the watchdog timer and return true */

      wd_cancel(group->wdog);
      uip_unlock(flags);
      return true;
    }

  uip_unlock(flags);
  return false;
}

#endif /* CONFIG_NET_IGMP */
