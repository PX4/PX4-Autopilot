/****************************************************************************
 * arch/arm/src/stm32/stm32_wwdg.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/watchdog.h>

#include "up_arch.h"
#include "stm32_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_STM32_WWDG)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct stm32_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_STM32_WWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr);
static void     stm32_putreg(uint32_t val, uint32_t addr);
#else
# define        stm32_getreg(addr)     getreg32(addr)
# define        stm32_putreg(val,addr) putreg32(val,addr)
#endif

/* "Lower half" driver methods **********************************************/

static int      stm32_start(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      stm32_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);
static xcpt_t   stm32_capture(FAR struct watchdog_lowerhalf_s *lower,
                  xcpt_t handler);
static int      stm32_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                  unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdgops =
{
  .start      = stm32_start,
  .stop       = stm32_stop,
  .keepalive  = stm32_keepalive,
  .getstatus  = stm32_getstatus,
  .settimeout = stm32_settimeout,
  .capture    = stm32_capture,
  .ioctl      = stm32_ioctl,
};

/* "Lower half" driver state */

static struct stm32_lowerhalf_s g_wdgdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_WWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_WWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the atchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             driver state structure.
 *   stawtus - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower
 *             driver state structure.
 *   timeout - The new timeout value in millisecnds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm32_capture
 *
 * Description:
 *   Don't reset on watchdog timer timeout; instead, call this user provider
 *   timeout handler.  NOTE:  Providing handler==NULL will restore the reset
 *   behavior.
 *
 * Input Parameters:
 *   lower      - A pointer the publicly visible representation of the "lower
 *                driver state structure.
 *   newhandler - The new watchdog expiration function pointer.  If this
 *                function pointer is NULL, then the the reset-on-expiration
 *                behavior is restored,
 *
 * Returned Values:
 *   The previous watchdog expiration function pointer or NULL is there was
 *   no previous function pointer, i.e., if the previous behavior was
 *   reset-on-expiration (NULL is also returned if an error occurs).
 *
 ****************************************************************************/

static xcpt_t stm32_capture(FAR struct watchdog_lowerhalf_s *lower,
                            xcpt_t handler)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return NULL;
}

/****************************************************************************
 * Name: stm32_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower
 *           driver state structure.
 *   cmd   - The ioctol command value
 *   arg   - The optional argument that accompanies the 'cmd'.  The
 *           interpretation of this argument depends on the particular
 *           command.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_ioctl(FAR struct watchdog_lowerhalf_s *lower, int cmd,
                    unsigned long arg)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  DEBUGASSERT(priv);
#warning "Missing logic"
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_wwdginitialize
 *
 * Description:
 *   Initialize the WWDG watchdog time.  The watchdog timer is intialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void stm32_wwdginitialize(FAR const char *devpath)
{
  FAR struct stm32_lowerhalf_s *priv = &g_wdgdev;

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  This function is
   * only called once so it is never necessary to re-zero the structure.
   */

  priv->ops     = &g_wdgops;

  /* Register the watchdog driver as /dev/watchdog0 */

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_STM32_WWDG */
