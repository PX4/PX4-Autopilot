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
#include <debug.h>

#include <nuttx/watchdog.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip/stm32_dbgmcu.h"
#include "stm32_wdg.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_STM32_WWDG)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Clocking *****************************************************************/
/* The minimum frequency of the WWDG clock is:
 *
 *  Fmin = PCLK1 / 4096 / 8
 *
 * So the maximum delay (in milliseconds) is then:
 *
 *   1000 * (WWDG_CR_T_MAX+1) / Fmin
 *
 * For example, if PCLK1 = 42MHz, then the maximum delay is:
 *
 *   Fmin = 1281.74
 *   1000 * 64 / Fmin = 49.93 msec
 */

#define WWDG_FMIN       (STM32_PCLK1_FREQUENCY / 4096 / 8)
#define WWDG_MAXTIMEOUT (1000 * (WWDG_CR_T_MAX+1) / WWDG_FMIN)

/* Configuration ************************************************************/

#ifndef CONFIG_STM32_WWDG_DEFTIMOUT
#  define CONFIG_STM32_WWDG_DEFTIMOUT WWDG_MAXTIMEOUT
#endif

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog
 * driver.  NOTE: that only lldbg types are used so that the output is
 * immediately available.
 */

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wddbg    lldbg
#  define wdvdbg   llvdbg
#else
#  define wddbg(x...)
#  define wdvdbg(x...)
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
  xcpt_t   handler;  /* Current EWI interrupt handler */
  uint32_t timeout;  /* The actual timeout value */
  uint32_t fwwdg;    /* WWDG clock frequency */
  bool     started;  /* The timer has been started */
  uint8_t  reload;   /* The 7-bit reload field reset value */
  uint8_t  window;   /* The 7-bit window (W) field value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Register operations ******************************************************/

#if defined(CONFIG_STM32_WWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t stm32_getreg(uint32_t addr);
static void     stm32_putreg(uint16_t val, uint32_t addr);
#else
# define        stm32_getreg(addr)     getreg32(addr)
# define        stm32_putreg(val,addr) putreg32(val,addr)
#endif
static void     stm32_setwindow(FAR struct stm32_lowerhalf_s *priv,
                  uint8_t window);

/* Interrupt hanlding *******************************************************/

static int      stm32_interrupt(int irq, FAR void *context);

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

#if defined(CONFIG_STM32_WWDG_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_setwindow
 *
 * Description:
 *   Set the CFR window value. The window value is compared to the down-
 *   counter when the counter is updated.  The WWDG counter should be updated
 *   only when the counter is below this window value (and greater than 64)
 *   otherwise a reset will be generated
 *
 ****************************************************************************/

static void stm32_setwindow(FAR struct stm32_lowerhalf_s *priv, uint8_t window)
{
  uint16_t regval;

  /* Set W[6:0] bits according to selected window value */

  regval = stm32_getreg(STM32_WWDG_CFR);
  regval &= ~WWDG_CFR_W_MASK;
  regval |= window << WWDG_CFR_W_SHIFT;
  stm32_putreg(regval, STM32_WWDG_CFR);

  /* Remember the window setting */

  priv->window = window;
}

/****************************************************************************
 * Name: stm32_interrupt
 *
 * Description:
 *   WWDG early warning interrupt
 *
 * Input Parameters:
 *   Usual interrupt handler arguments.
 *
 * Returned Values:
 *   Always returns OK.
 *
 ****************************************************************************/

static int stm32_interrupt(int irq, FAR void *context)
{
  FAR struct stm32_lowerhalf_s *priv = &g_wdgdev;
  uint16_t regval;

  /* Check if the EWI interrupt is really pending */

  regval = stm32_getreg(STM32_WWDG_SR);
  if ((regval & WWDG_SR_EWIF) != 0)
    {
      /* Is there a registered handler? */

      if (priv->handler)
        {
          /* Yes... NOTE:  This interrupt service routine (ISR) must reload
           * the WWDG counter to prevent the reset.  Otherwise, we will reset
           * upon return.
           */

          priv->handler(irq, context);
        }

      /* The EWI interrupt is cleared by writing '0' to the EWIF bit in the
       * WWDG_SR register.
       */

      regval &= ~WWDG_SR_EWIF;
      stm32_putreg(regval, STM32_WWDG_SR);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

  stm32_putreg(WWDG_CR_WDGA | WWDG_CR_T_RESET | priv->reload, STM32_WWDG_CR);
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
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  /* The watchdog is always disabled after a reset. It is enabled by setting
   * the WDGA bit in the WWDG_CR register, then it cannot be disabled again
   * except by a reset.
   */

  wdvdbg("Entry\n");
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
 *   The application program must write in the WWDG_CR register at regular
 *   intervals during normal operation to prevent an MCU reset. This operation
 *   must occur only when the counter value is lower than the window register
 *   value. The value to be stored in the WWDG_CR register must be between
 *   0xff and 0xC0:
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int stm32_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Write to T[6:0] bits to configure the counter value, no need to do
   * a read-modify-write; writing a 0 to WDGA bit does nothing.
   */

  stm32_putreg((WWDG_CR_T_RESET | priv->reload), STM32_WWDG_CR);
  return OK;
}

/****************************************************************************
 * Name: stm32_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
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

  wdvdbg("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
    {
      status->flags |= WDFLAGS_ACTIVE;
    }
 
  if (priv->handler)
    {
      status->flags |= WDFLAGS_CAPTURE;
    }

  /* Return the actual timeout is milliseconds */

  status->timeout = priv->timeout;

  /* Get the time remaining until the watchdog expires (in milliseconds) */

  reload = (stm32_getreg(STM32_WWDG_CR) >> WWDG_CR_T_SHIFT) & 0x7f;
  elapsed = priv->reload - reload;
  status->timeleft = (priv->timeout * elapsed) / (priv->reload + 1);

  wdvdbg("Status     :\n");
  wdvdbg("  flags    : %08x\n", status->flags);
  wdvdbg("  timeout  : %d\n", status->timeout);
  wdvdbg("  timeleft : %d\n", status->flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
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
  uint32_t fwwdg;
  uint32_t reload;
  uint16_t regval;
  int wdgtb;

  DEBUGASSERT(priv);
  wdvdbg("Entry: timeout=%d\n", timeout);

  /* Can this timeout be represented? */

  if (timeout < 1 || timeout > WWDG_MAXTIMEOUT)
    {
      wddbg("Cannot represent timeout=%d > %d\n",
            timeout, WWDG_MAXTIMEOUT);
      return -ERANGE;
    }

  /* Determine prescaler value.
   *
   * Fwwdg = PCLK1/4096/prescaler.
   *
   * Where
   *  Fwwwdg is the frequency of the WWDG clock
   *  wdgtb is one of {1, 2, 4, or 8}
   */

  /* Select the smallest prescaler that will result in a reload field value that is
   * less than the maximum.
   */

  for (wdgtb = 0; ; wdgtb++)
    {
      /* WDGTB = 0 -> Divider = 1  = 1 << 0
       * WDGTB = 1 -> Divider = 2  = 1 << 1
       * WDGTB = 2 -> Divider = 4  = 1 << 2
       * WDGTB = 3 -> Divider = 8  = 1 << 3
       */

      /* Get the WWDG counter frequency in Hz. */

      fwwdg = (STM32_PCLK1_FREQUENCY/4096) >> wdgtb;

      /* The formula to calculate the timeout value is given by:
       *
       * timeout =  1000 * (reload + 1) / Fwwdg, OR
       * reload = timeout * Fwwdg / 1000 - 1
       *
       * Where
       *  timeout is the desired timout in milliseconds
       *  reload is the contents of T{5:0]
       *  Fwwdg is the frequency of the WWDG clock
       */

       reload = timeout * fwwdg / 1000 - 1;

      /* If this reload valid is less than the maximum or we are not ready
       * at the prescaler value, then break out of the loop to use these
       * settings.
       */

#if 0
      wdvdbg("wdgtb=%d fwwdg=%d reload=%d timout=%d\n",
             wdgtb, fwwdg, reload,  1000 * (reload + 1) / fwwdg);
#endif
      if (reload <= WWDG_CR_T_MAX || wdgtb == 3)
        {
          /* Note that we explicity break out of the loop rather than using
           * the 'for' loop termination logic because we do not want the
           * value of wdgtb to be incremented.
           */

          break;
        }
    }

  /* Make sure that the final reload value is within range */

  if (reload > WWDG_CR_T_MAX)
    {
      reload = WWDG_CR_T_MAX;
    }

  /* Calculate and save the actual timeout value in milliseconds:
   *
   * timeout =  1000 * (reload + 1) / Fwwdg
   */

  priv->timeout = 1000 * (reload + 1) / fwwdg;

  /* Remember the selected values */

  priv->fwwdg  = fwwdg;
  priv->reload = reload;

  wdvdbg("wdgtb=%d fwwdg=%d reload=%d timout=%d\n",
         wdgtb, fwwdg, reload, priv->timeout);
  
  /* Set WDGTB[1:0] bits according to calculated value */

  regval = stm32_getreg(STM32_WWDG_CFR);
  regval &= ~WWDG_CFR_WDGTB_MASK;
  regval |= (uint16_t)wdgtb << WWDG_CFR_WDGTB_SHIFT;
  stm32_putreg(regval, STM32_WWDG_CFR);

  /* Reset the 7-bit window value to the maximum value.. essentially disabling
   * the lower limit of the watchdog reset time.
   */

  stm32_setwindow(priv, 0x7f);
  return OK;
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
 *   lower      - A pointer the publicly visible representation of the "lower-half"
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
  irqstate_t flags;
  xcpt_t oldhandler;
  uint16_t regval;

  DEBUGASSERT(priv);
  wdvdbg("Entry: handler=%p\n", handler);

  /* Get the old handler return value */

  flags = irqsave();
  oldhandler = priv->handler;

  /* Save the new handler */

   priv->handler = handler;

  /* Are we attaching or detaching the handler? */

  regval = stm32_getreg(STM32_WWDG_CFR);
  if (handler)
    {
      /* Attaching... Enable the EWI interrupt */

      regval |= WWDG_CFR_EWI;
      stm32_putreg(regval, STM32_WWDG_CFR);
 
      up_enable_irq(STM32_IRQ_WWDG);
    }
  else
    {
      /* Detaching... Disable the EWI interrupt */

      regval &= ~WWDG_CFR_EWI;
      stm32_putreg(regval, STM32_WWDG_CFR);

      up_disable_irq(STM32_IRQ_WWDG);
    }

  irqrestore(flags);
  return oldhandler;
}

/****************************************************************************
 * Name: stm32_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half" driver
 *   are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
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
  int ret = -ENOTTY;

  DEBUGASSERT(priv);
  wdvdbg("Entry: cmd=%d arg=%ld\n", cmd, arg);

  /* WDIOC_MINTIME: Set the minimum ping time.  If two keepalive ioctls
   * are received within this time, a reset event will be generated.
   * Argument: A 32-bit time value in milliseconds.
   */

  if (cmd == WDIOC_MINTIME)
    {
      uint32_t mintime = (uint32_t)arg;
 
      /* The minimum time should be strictly less than the total delay
       * which, in turn, will be less than or equal to WWDG_CR_T_MAX
       */

      ret = -EINVAL;
      if (mintime < priv->timeout)
        {
          uint32_t window = (priv->timeout - mintime) * priv->fwwdg / 1000 - 1;
          DEBUGASSERT(window < priv->reload);
          stm32_setwindow(priv, window | WWDG_CR_T_RESET);
          ret = OK;
        }
    }

  return ret;
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

  wdvdbg("Entry: devpath=%s\n", devpath);

  /* NOTE we assume that clocking to the IWDG has already been provided by
   * the RCC initialization logic.
   */

  /* Initialize the driver state structure.  Here we assume: (1) the state
   * structure lies in .bss and was zeroed at reset time.  (2) This function
   * is only called once so it is never necessary to re-zero the structure.
   */

  priv->ops = &g_wdgops;

  /* Attach our EWI interrupt handler (But don't enable it yet) */

  (void)irq_attach(STM32_IRQ_WWDG, stm32_interrupt);

  /* Select an arbitrary initial timeout value.  But don't start the watchdog
   * yet. NOTE: If the "Hardware watchdog" feature is enabled through the
   * device option bits, the watchdog is automatically enabled at power-on.
   */

  stm32_settimeout((FAR struct watchdog_lowerhalf_s *)priv,
                   CONFIG_STM32_WWDG_DEFTIMOUT);

  /* Register the watchdog driver as /dev/watchdog0 */

  (void)watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

  /* When the microcontroller enters debug mode (Cortex™-M4F core halted),
   * the WWDG counter either continues to work normally or stops, depending
   * on DBG_WWDG_STOP configuration bit in DBG module.
   */

#if defined(CONFIG_STM32_JTAG_FULL_ENABLE) || \
    defined(CONFIG_STM32_JTAG_NOJNTRST_ENABLE) || \
    defined(CONFIG_STM32_JTAG_SW_ENABLE)
    {
      uint32_t cr = getreg32(STM32_DBGMCU_CR);
      cr |= DBGMCU_CR_WWDGSTOP;
      putreg32(cr, STM32_DBGMCU_CR);
    }
#endif
}

#endif /* CONFIG_WATCHDOG && CONFIG_STM32_WWDG */
