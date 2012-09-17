/************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_dac.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from from the LPC17 version:
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-05 initial version
 * 
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/analog/dac.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

#include "lpc43_syscon.h"
#include "lpc43_pinconn.h"
#include "lpc43_dac.h"

#ifdef CONFIG_LPC43_DAC

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);
static int  dac_interrupt(int irq, void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset =dac_reset,
  .ao_setup = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint = dac_txint,
  .ao_send = dac_send,
  .ao_ioctl = dac_ioctl,
};

static struct dac_dev_s g_dacdev =
{
  .ad_ops = &g_dacops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Reset the DAC device.  Called early to initialize the hardware. This
 * is called, before ao_setup() and on error conditions.
 */

static void dac_reset(FAR struct dac_dev_s *dev)
{
  irqstate_t flags;
  uint32_t regval;
    
  flags = irqsave();

  regval  = getreg32(LPC43_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_DAC_MASK;
  regval |= (SYSCON_PCLKSEL_CCLK8 << SYSCON_PCLKSEL0_DAC_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL0);

  //putreg32(DAC_CTRL_DBLBUFEN,LPC43_DAC_CTRL); ?

  lpc43_configgpio(GPIO_AOUT);

  irqrestore(flags);
}

/* Configure the DAC. This method is called the first time that the DAC
 * device is opened.  This will occur when the port is first opened.
 * This setup includes configuring and attaching DAC interrupts.  Interrupts
 * are all disabled upon return.
 */

static int  dac_setup(FAR struct dac_dev_s *dev)
{
  return OK;
}

/* Disable the DAC.  This method is called when the DAC device is closed.
 * This method reverses the operation the setup method.
 */

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
}

/* Call to enable or disable TX interrupts */

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
}

static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  putreg32((msg->am_data>>16)&0xfffff,LPC43_DAC_CR);
  dac_txdone(&g_dacdev);
  return 0;
}

/* All ioctl calls will be routed through this method */

static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  dbg("Fix me:Not Implemented\n");
  return 0;
}

static int  dac_interrupt(int irq, void *context)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Returned Value:
 *   Valid dac device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct dac_dev_s *lpc43_dacinitialize(void)
{
  return &g_dacdev;
}

#endif /* CONFIG_LPC43_DAC */

