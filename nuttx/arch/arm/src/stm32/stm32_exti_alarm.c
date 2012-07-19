/****************************************************************************
 * arch/arm/src/stm32/stm32_exti_alarm.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Diego Sanchez <dsanchez@nx-engineering.com>
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Interrupt handlers attached to the ALARM EXTI */

static xcpt_t stm32_exti_callback;

/****************************************************************************
 * Public Data
 ****************************************************************************/

 /****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_alarm_isr
 *
 * Description:
 *   EXTI ALARM interrupt service routine/dispatcher
 *
 ****************************************************************************/

static int stm32_exti_alarm_isr(int irq, void *context)
{
  int ret = OK;

  /* Clear the pending EXTI interrupt */

  putreg32(EXTI_RTC_ALARM, STM32_EXTI_PR);

  /* And dispatch the interrupt to the handler */

  if (stm32_exti_callback)
    {
      ret = stm32_exti_callback(irq, context);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_alarm
 *
 * Description:
 *   Sets/clears EXTI alarm interrupt.
 *
 * Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edget
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *
 * Returns:
 *   The previous value of the interrupt handler function pointer.  This
 *   value may, for example, be used to restore the previous handler when
 *   multiple handlers are used.
 *
 ****************************************************************************/

xcpt_t stm32_exti_alarm(bool risingedge, bool fallingedge, bool event,
                        xcpt_t func)
{
  xcpt_t oldhandler;

  /* Get the previous GPIO IRQ handler; Save the new IRQ handler. */

  oldhandler          = stm32_exti_callback;
  stm32_exti_callback = func;

  /* Install external interrupt handlers (if not already attached) */

  if (func)
    {
      irq_attach(STM32_IRQ_RTCALRM, stm32_exti_alarm_isr);
      up_enable_irq(STM32_IRQ_RTCALRM);
    }
  else
    {
      up_disable_irq(STM32_IRQ_RTCALRM);
    }

  /* Configure rising/falling edges */

  modifyreg32(STM32_EXTI_RTSR,
              risingedge ? 0 : EXTI_RTC_ALARM,
              risingedge ? EXTI_RTC_ALARM : 0);
  modifyreg32(STM32_EXTI_FTSR,
              fallingedge ? 0 : EXTI_RTC_ALARM,
              fallingedge ? EXTI_RTC_ALARM : 0);

  /* Enable Events and Interrupts */

  modifyreg32(STM32_EXTI_EMR,
              event ? 0 : EXTI_RTC_ALARM,
              event ? EXTI_RTC_ALARM : 0);
  modifyreg32(STM32_EXTI_IMR,
              func ? 0 : EXTI_RTC_ALARM,
              func ? EXTI_RTC_ALARM : 0);

  /* Return the old IRQ handler */

  return oldhandler;
}
