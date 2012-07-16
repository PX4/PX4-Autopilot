/****************************************************************************
 * arch/arm/src/lpc43/lpc43_rgu.c
 * arch/arm/src/chip/lpc43_rgu.c
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

#include <arch/irq.h>
#include <nuttx/arch.h>

#include "nvic.h"
#include "up_arch.h"

#include "chip.h"
#include "lpc43_rgu.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_softreset
 *
 * Description:
 *   Reset as many of the LPC43 peripherals as possible. This is necessary
 *   because the LPC43 does not provide any way of performing a full system
 *   reset under debugger control.  So, if CONFIG_DEBUG is set (indicating
 *   that a debugger is being used?), the the boot logic will call this 
 *   function on all restarts.
 *
 * Assumptions:
 *   Since this function is called early in the boot sequence, it cannot
 *   depend on anything such as initialization of .bss or .data.  It can
 *   only assume that it has a stack.
 *
 ****************************************************************************/

void lpc43_softreset(void)
{
  irqstate_t flags;

  /* Disable interrupts */

  flags = irqsave();
 
  /* Reset all of the peripherals that we can (safely) */

  putreg32((RGU_CTRL0_LCD_RST     | RGU_CTRL0_USB0_RST     |
            RGU_CTRL0_USB1_RST    | RGU_CTRL0_DMA_RST      |
            RGU_CTRL0_SDIO_RST    | RGU_CTRL0_ETHERNET_RST |
            RGU_CTRL0_GPIO_RST), LPC43_RGU_CTRL0);
  putreg32((RGU_CTRL1_TIMER0_RST  | RGU_CTRL1_TIMER1_RST   |
            RGU_CTRL1_TIMER2_RST  | RGU_CTRL1_TIMER3_RST   |
            RGU_CTRL1_RITIMER_RST | RGU_CTRL1_SCT_RST      |
            RGU_CTRL1_MCPWM_RST   | RGU_CTRL1_QEI_RST      |
            RGU_CTRL1_ADC0_RST    | RGU_CTRL1_ADC1_RST     |
            RGU_CTRL1_USART0_RST  | RGU_CTRL1_UART1_RST    |
            RGU_CTRL1_USART2_RST  | RGU_CTRL1_USART3_RST   |
            RGU_CTRL1_I2C0_RST    | RGU_CTRL1_I2C1_RST     |
            RGU_CTRL1_SSP0_RST    | RGU_CTRL1_SSP1_RST     |
            RGU_CTRL1_I2S_RST     | RGU_CTRL1_CAN1_RST     |
            RGU_CTRL1_CAN0_RST    | RGU_CTRL1_M0APP_RST),
            LPC43_RGU_CTRL1);

  /* A delay seems to be necessary somewhere around here */

  up_mdelay(20);

  /* Clear all pending interupts */

  putreg32(0xffffffff, NVIC_IRQ0_31_CLRPEND);
  putreg32(0xffffffff, NVIC_IRQ32_63_CLRPEND);
  irqrestore(flags);
}
