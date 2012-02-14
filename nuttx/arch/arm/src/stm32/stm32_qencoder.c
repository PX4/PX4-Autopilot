/************************************************************************************
 * arch/arm/src/stm32/stm32_qencoder.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_internal.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include "stm32_qencoder.h"

#ifdef CONFIG_QENCODER

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Debug ****************************************************************************/
/* Non-standard debug that may be enabled just for testing QENCODER */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_QENCODER
#endif

#ifdef CONFIG_DEBUG_QENCODER
#  define qedbg                 dbg
#  define qelldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define qevdbg              vdbg
#    define qellvdbg            llvdbg
#    define stm32_dumpgpio(p,m) stm32_dumpgpio(p,m)
#  else
#    define qelldbg(x...)
#    define qellvdbg(x...)
#    define stm32_dumpgpio(p,m)
#  endif
#else
#  define qedbg(x...)
#  define qelldbg(x...)
#  define qevdbg(x...)
#  define qellvdbg(x...)
#  define stm32_dumpgpio(p,m)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct stm32_qeconfig_s
{
  uint8_t  timid;   /* Timer ID {1,2,3,4,5,8} */
  uint8_t  irq;     /* Timer update IRQ */
  uint32_t base;    /* Register base address */
  xcpt_t   handler; /* Interrupt handler for this IRQ */
};

/* Overall, RAM-based state structure */

struct stm32_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  FAR const struct qe_ops_s *ops;  /* Lower half callback structure */

  /* STM32 driver-specific fields: */

  FAR const struct stm32_qeconfig_s *config; /* static onfiguration */

  bool             inuse;    /* True: The lower-half driver is in-use */
  volatile int32_t position; /* The current position offset */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/
/* Helper functions */

static uint16_t stm32_getreg16(struct stm32_lowerhalf_s *priv, int offset);
static void stm32_putreg16(struct stm32_lowerhalf_s *priv, int offset, uint16_t value);
static uint32_t stm32_getreg32(FAR struct stm32_lowerhalf_s *priv, int offset);
static void stm32_putreg32(FAR struct stm32_lowerhalf_s *priv, int offset, uint32_t value);

#if defined(CONFIG_DEBUG_QENCODER) && defined(CONFIG_DEBUG_VERBOSE)
static void stm32_dumpregs(struct stm32_lowerhalf_s *priv, FAR const char *msg);
#else
#  define stm32_dumpregs(priv,msg)
#endif

static FAR struct stm32_lowerhalf_s *stm32_tim2lower(int tim);

/* Interrupt handling */

static int stm32_interrupt(FAR struct stm32_lowerhalf_s *priv);
#ifdef CONFIG_STM32_TIM1
static int stm32_tim1interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM2
static int stm32_tim2interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM3
static int stm32_tim3interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM4
static int stm32_tim4interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM5
static int stm32_tim5interrupt(int irq, FAR void *context);
#endif
#ifdef CONFIG_STM32_TIM8
static int stm32_tim8interrupt(int irq, FAR void *context);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int stm32_setup(FAR struct qe_lowerhalf_s *lower);
static int stm32_shutdown(FAR struct qe_lowerhalf_s *lower);
static int stm32_position(FAR struct qe_lowerhalf_s *lower, int32_t *pos);
static int stm32_reset(FAR struct qe_lowerhalf_s *lower);
static int stm32_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup    = stm32_setup,
  .shutdown = stm32_shutdown,
  .position = stm32_position,
  .reset    = stm32_reset,
  .ioctl    = stm32_ioctl,
};

/* Per-timer state structures */

#ifdef CONFIG_STM32_TIM1
static const struct stm32_qeconfig_s g_tim1config =
{
  .timid    = 1,
  .irq      = STM32_IRQ_TIM1UP,
  .base     = STM32_TIM1_BASE,
  .handler  = stm32_tim1interrupt,
};

static struct stm32_lowerhalf_s g_tim1lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim1config,
  .inuse    = false,
};

#endif
#ifdef CONFIG_STM32_TIM2
static const struct stm32_qeconfig_s g_tim2config =
{
  .timid    = 2,
  .irq      = STM32_IRQ_TIM2,
  .base     = STM32_TIM2_BASE,
  .handler  = stm32_tim2interrupt,
};

static struct stm32_lowerhalf_s g_tim2lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim2config,
  .inuse    = false,
};

#endif
#ifdef CONFIG_STM32_TIM3
static const struct stm32_qeconfig_s g_tim3config =
{
  .timid    = 3,
  .irq      = STM32_IRQ_TIM3,
  .base     = STM32_TIM3_BASE,
  .handler  = stm32_tim3interrupt,
};

static struct stm32_lowerhalf_s g_tim3lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim3config,
  .inuse    = false,
};

#endif
#ifdef CONFIG_STM32_TIM4
static const struct stm32_qeconfig_s g_tim4config =
{
  .timid    = 4,
  .irq      = STM32_IRQ_TIM4,
  .base     = STM32_TIM4_BASE,
  .handler  = stm32_tim4interrupt,
};

static struct stm32_lowerhalf_s g_tim4lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim4config,
  .inuse    = false,
};

#endif
#ifdef CONFIG_STM32_TIM5
static const struct stm32_qeconfig_s g_tim5config =
{
  .timid    = 5,
  .irq      = STM32_IRQ_TIM5,
  .base     = STM32_TIM5_BASE,
  .handler  = stm32_tim5interrupt,
};

static struct stm32_lowerhalf_s g_tim5lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim5config,
  .inuse    = false,
};

#endif
#ifdef CONFIG_STM32_TIM8
static const struct stm32_qeconfig_s g_tim8config =
{
  .timid    = 8,
  .irq      = STM32_IRQ_TIM8UP,
  .base     = STM32_TIM8_BASE,
  .handler  = stm32_tim8interrupt,
};

static struct stm32_lowerhalf_s g_tim8lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim8config
  .inuse    = false,
};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_getreg16
 *
 * Description:
 *   Read the value of a 16-bit timer register.
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ************************************************************************************/

static uint16_t stm32_getreg16(struct stm32_lowerhalf_s *priv, int offset)
{
  return getreg16(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_putreg16
 *
 * Description:
 *   Write a value to a 16-bit timer register.
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void stm32_putreg16(struct stm32_lowerhalf_s *priv, int offset, uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_getreg32
 *
 * Description:
 *   Read the value of a 32-bit timer register.  This applies only for the STM32 F4
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5 (but works OK
 *   with the 16-bit TIM1,8 and F1 registers).
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ************************************************************************************/

static uint32_t stm32_getreg32(FAR struct stm32_lowerhalf_s *priv, int offset)
{
  return getreg32(priv->config->base + offset);
}

/************************************************************************************
 * Name: stm32_putreg16
 *
 * Description:
 *   Write a value to a 32-bit timer register.  This applies only for the STM32 F4
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5 (but works OK
 *   with the 16-bit TIM1,8 and F1 registers).
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void stm32_putreg32(FAR struct stm32_lowerhalf_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input parameters:
 *   priv - A reference to the QENCODER block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_QENCODER) && defined(CONFIG_DEBUG_VERBOSE)
static void stm32_dumpregs(struct stm32_lowerhalf_s *priv, FAR const char *msg)
{
  qevdbg("%s:\n", msg);
  qevdbg("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CR2_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_SMCR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_DIER_OFFSET));
  qevdbg("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
         stm32_getreg16(priv, STM32_GTIM_SR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_EGR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCMR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCMR2_OFFSET));
  qevdbg(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CNT_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_PSC_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_ARR_OFFSET));
  qevdbg(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CCR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR2_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR3_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32_TIM1_QENCODER) || defined(CONFIG_STM32_TIM8_QENCODER)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      qevdbg("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
             stm32_getreg16(priv, STM32_ATIM_RCR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_BDTR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_DCR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
#endif
    {
      qevdbg("  DCR: %04x DMAR: %04x\n",
             stm32_getreg16(priv, STM32_GTIM_DCR_OFFSET),
             stm32_getreg16(priv, STM32_GTIM_DMAR_OFFSET));
    }
}
#endif

/************************************************************************************
 * Name: stm32_tim2lower
 *
 * Description:
 *   Map a timer number to a device structure  
 *
 ************************************************************************************/

static FAR struct stm32_lowerhalf_s *stm32_tim2lower(int tim)
{
  switch (tim)
    {
#ifdef CONFIG_STM32_TIM1
    case 1:
      return &g_tim1lower;
#endif
#ifdef CONFIG_STM32_TIM2
    case 2:
      return &g_tim2lower;
#endif
#ifdef CONFIG_STM32_TIM3
    case 3:
#endif
#ifdef CONFIG_STM32_TIM4
    case 4:
      return &g_tim4lower;
#endif
#ifdef CONFIG_STM32_TIM5
    case 5:
      return &g_tim5lower;
#endif
#ifdef CONFIG_STM32_TIM8
    case 8:
      return &g_tim8lower;
#endif
    default:
      return NULL;
    }
}

/************************************************************************************
 * Name: stm32_setup
 *
 * Description:
 *   Common timer interrupt handling
 *
 ************************************************************************************/

static int stm32_interrupt(FAR struct stm32_lowerhalf_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = stm32_getreg16(priv, STM32_GTIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  stm32_putreg16(priv, STM32_GTIM_SR_OFFSET, regval & ~GTIM_SR_UIF);

  /* Check the direction bit in the CR1 register and add or subtract the
   * maximum value, as appropriate.
   */

  regval = stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET);
  if ((regval & ATIM_CR1_DIR) != 0)
    {
      priv->position -= (int32_t)0x0000fff0;
    }
   else
    {
      priv->position += (int32_t)0x0000fff0;
    }

  return OK;
}

/************************************************************************************
 * Name: stm32_intNinterrupt
 *
 * Description:
 *   TIMN interrupt handler   
 *
 ************************************************************************************/

 #ifdef CONFIG_STM32_TIM1
static int stm32_tim1interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim1lower);
}
#endif

#ifdef CONFIG_STM32_TIM2
static int stm32_tim2interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim2lower);
}
#endif

#ifdef CONFIG_STM32_TIM3
static int stm32_tim3interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim3lower);
}
#endif

#ifdef CONFIG_STM32_TIM4
static int stm32_tim4interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim4lower);
}
#endif

#ifdef CONFIG_STM32_TIM5
static int stm32_tim5interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim5lower);
}
#endif

#ifdef CONFIG_STM32_TIM8
static int stm32_tim8interrupt(int irq, FAR void *context)
{
  return stm32_interrupt(&g_tim8lower);
}
#endif

/************************************************************************************
 * Name: stm32_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *   
 *
 ************************************************************************************/

static int stm32_setup(FAR struct qe_lowerhalf_s *lower)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   should stop data collection, free any resources, disable timer hardware, and
 *   put the system into the lowest possible power usage state *   
 *
 ************************************************************************************/

static int stm32_shutdown(FAR struct qe_lowerhalf_s *lower)
{
#warning "Missing logic"
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ************************************************************************************/

static int stm32_position(FAR struct qe_lowerhalf_s *lower, int32_t *pos)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;
  int32_t position;
  int32_t verify;
  uint32_t count;

  DEBUGASSERT(lower && priv->inuse);

  /* Loop until we are certain that no interrupt occurred between samples */

  do
    {
      /* Don't let another task pre-empt us until we get the measurement.  The timer
       * interrupt may still be processed
       */

      sched_lock();
      position = priv->position;
      count    = stm32_getreg32(priv, STM32_GTIM_CNT_OFFSET);
      verify   = priv->position;
      sched_unlock();
    }
  while (position != verify);

  /* Return the position measurement */

  *pos = position + (int32_t)count;
  return OK;
}

/************************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ************************************************************************************/

static int stm32_reset(FAR struct qe_lowerhalf_s *lower)
{
  FAR struct stm32_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;
  irqstate_t flags;

  DEBUGASSERT(lower && priv->inuse);

  /* Reset the timer and the counter.  Interrupts are disabled to make this atomic
   * (if possible)
   */

  flags = irqsave();
  stm32_putreg32(priv, STM32_GTIM_CNT_OFFSET, 0);
  priv->position = 0;
  irqrestore(flags);
  return OK;
}

/************************************************************************************
 * Name: stm32_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ************************************************************************************/

static int stm32_ioctl(FAR struct qe_lowerhalf_s *lower, int cmd, unsigned long arg)
{
  /* No ioctl commands supported */

  return -ENOTTY;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be called from
 *   board-specific logic after input pins have been configured.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tim     - The timer number to used.  time must be an element of {1,2,3,4,5,8}
 *
 * Returned Values:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ************************************************************************************/

int stm32_qeinitialize(FAR const char *devpath, int tim)
{
  FAR struct stm32_lowerhalf_s *priv;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this timer */

  priv = stm32_tim2lower(tim);
  if (priv)
    {
      return -ENXIO;
    }

  /* Make sure that it is available */

  if (priv->inuse)
    {
      return -EBUSY;
    }

  /* Register the priv-half driver */

  ret = qe_register(devpath, (FAR struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      return ret;
    }

  /* The driver is now in-use */

  priv->inuse = true;
  return OK;
}

#endif /* CONFIG_QENCODER */
