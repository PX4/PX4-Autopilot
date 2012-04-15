/****************************************************************************
 * arch/arm/src/stm32/stm32_iwdg.c
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
#include "stm32_rcc.h"
#include "stm32_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_STM32_IWDG)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_STM32_IWDG_DEFTIMOUT
#  define CONFIG_STM32_IWDG_DEFTIMOUT 3000 /* Three seconds */
#endif

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
  uint32_t lsifreq;   /* The calibrated frequency of the LSI oscillator */
  uint32_t timeout;   /* The (actual) selected timeout */
  bool     started;   /* true: The watchdog timer has been started */
  uint8_t  prescaler; /* Clock prescaler value */
  uint16_t reload;    /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_STM32_IWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr);
static void     stm32_putreg(uint32_t val, uint32_t addr);
#else
# define        stm32_getreg(addr)     getreg16(addr)
# define        stm32_putreg(val,addr) putreg16(val,addr)
#endif

/* "Lower half" driver methods **********************************************/

static int      stm32_start(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_stop(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int      stm32_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                  FAR struct watchdog_status_s *status);
static int      stm32_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                  uint32_t timeout);

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
  .capture    = NULL,
  .ioctl      = NULL,
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
 *   Get the contents of an STM32 IWDG register
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_IWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t stm32_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t count = 0;
  static uint16_t preval = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

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

  lldbg("%08x->%04x\n", addr, val);
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

#if defined(CONFIG_STM32_IWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
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

  /* Reload IWDG counter */

  stm32_putreg(IWDG_KR_KEY_RELOAD, STM32_IWDG_KR);

  /* Enable IWDG (the LSI oscillator will be enabled by hardware).  NOTE:
   * If the "Hardware watchdog" feature is enabled through the device option
   * bits, the watchdog is automatically enabled at power-on.
   */

  stm32_putreg(IWDG_KR_KEY_START, STM32_IWDG_KR);
  priv->started = true;
  return OK;
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
  /* There is no way to disable the IDWG timer once it has been started */

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
  /* Reload the IWDG timer */

  stm32_putreg(IWDG_KR_KEY_RELOAD, STM32_IWDG_KR);
  return OK;
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
  uint32_t elapsed;
  uint16_t reload;

  DEBUGASSERT(priv);

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }

  status->timeout = priv->timeout;

  /* I am not sure what will be returned when reading from the reload register.
   * Documentation says that the value from the VDD section should be read
   * so presume it the decremented value.
   */

  reload = stm32_getreg(STM32_IWDG_RLR);
  elapsed = priv->reload - reload;
  status->timeleft = (priv->timeout * elapsed) / priv->reload;
  return OK;
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
  uint32_t frequency;
  uint64_t reload;
  int pr;
  int shift;

  DEBUGASSERT(priv);

  /* Select the smallest prescaler that will result in a reload value that is
   * less than the maximum.
   */

  for (pr = 0; ; pr++)
    {
      /* PR = 0 -> Divider = 4   = 1 << 2
       * PR = 1 -> Divider = 8   = 1 << 3
       * PR = 2 -> Divider = 16  = 1 << 4
       * PR = 3 -> Divider = 32  = 1 << 5
       * PR = 4 -> Divider = 64  = 1 << 6
       * PR = 5 -> Divider = 128 = 1 << 7
       * PR = 6 -> Divider = 256 = 1 << 8
       */

      shift = pr + 2;

      /* Is the IWDG counter frequency in Hz. For a nominal 32Khz LSI clock,
       * this is value in the range of 7500 and 125.
       */

      frequency = priv->lsifreq >> shift;

      /* We want:
       *  1000 * reload / frequency = timeout
       * Or:
       *  reload = frequency * timeout / 1000
       */

      reload = (uint64_t)frequency * (uint64_t)timeout / 1000;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

      if (reload <= IWDG_RLR_MAX || pr == 6)
        {
          /* Note that we explicity break out of the loop rather than using
           * the 'for' loop termination logic because we do not want the
           * value of pr to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > IWDG_RLR_MAX)
    {
      reload = IWDG_RLR_MAX;
    }

  /* Enable write access to IWDG_PR and IWDG_RLR registers */

  stm32_putreg(IWDG_KR_KEY_ENABLE, STM32_IWDG_KR);

  /* Set the prescaler */

  stm32_putreg((uint16_t)pr << IWDG_PR_SHIFT, STM32_IWDG_PR);

  /* Set the reload value */

  stm32_putreg((uint16_t)reload, STM32_IWDG_RLR);

  /* Get the actual timeout value in milliseconds.
   *
   * We have:
   *  reload = frequency * timeout / 1000
   * So we want:
   *  timeout = 1000 * reload / frequency
   */
 
  priv->timeout = (1000 * reload) / frequency;

  /* Save setup values for later use */

  priv->prescaler = pr;     /* Not needed */
  priv->reload    = reload;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_iwdginitialize
 *
 * Description:
 *   Initialize the IWDG watchdog time.  The watchdog timer is intialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath - The full path to the watchdog.  This should be of the form
 *     /dev/watchdog0
 *   lsifreq - The calibrated LSI clock frequency
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void stm32_iwdginitialize(FAR const char *devpath, uint32_t lsifreq)
{
  FAR struct stm32_lowerhalf_s *priv = &g_wdgdev;

  /* Initialize the driver state structure. */

  priv->ops = &g_wdgops;
  priv->lsifreq = lsifreq;
  priv->lsifreq = false;

  /* Make sure that the LSI ocsillator is enabled.  NOTE:  The LSI oscillator
   * is enabled here but is not disabled by this file (because this file does
   * not know the the global usage of the oscillator.  Any clock management
   * logic (say, as part of a power management scheme) needs handle other
   * LSI controls outside of this file.
   */

  stm32_rcc_enablelsi();

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  stm32_settimeout((FAR struct watchdog_lowerhalf_s *)priv, CONFIG_STM32_IWDG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);
}

#endif /* CONFIG_WATCHDOG && CONFIG_STM32_IWDG */
