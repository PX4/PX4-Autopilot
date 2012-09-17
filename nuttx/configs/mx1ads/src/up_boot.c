/************************************************************************************
 * configs/mx1ads/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "imx_gpio.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: imx_boardinitialize
 *
 * Description:
 *   All i.MX architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 ************************************************************************************/

void imx_boardinitialize(void)
{
  uint32_t regval;

  putreg32(0x000003ab, IMX_SC_GPCR); /* I/O pad driving strength */
  putreg32(IMX_MPCTL0_VALUE, IMX_PLL_MPCTL0);
  putreg32(IMX_SPCTL0_VALUE, IMX_PLL_SPCTL0);

  regval = (CSCR_CLKOSEL_FCLK |                             /* Output FCLK on CLK0 */
            (IMX_CSCR_USBDIV << PLL_CSCR_USBDIV_SHIFT) |    /* USB divider */
            CSCR_SDCNT_4thEDGE |                            /* Shutdown on 4th edge */
            (IMX_CSCR_BCLKDIV << PLL_CSCR_BCLKDIV_SHIFT) |  /* Bclock divider */
	    PLL_CSCR_SPEN | PLL_CSCR_MPEN);                /* Enable MUC and System PLL */
  putreg32(regval, IMX_PLL_CSCR);

  /* Use these new frequencies now */

  putreg32(IMX_PLL_CSCR, regval | (PLL_CSCR_MPLLRESTART|PLL_CSCR_SPLLRESTART));

  /* Setup peripheral clocking */

  putreg32(IMX_PCDR_VALUE, IMX_PLL_PCDR);

  /* Configure CS4 for cs8900 Ethernet */

#ifdef CONFIG_NET
  putreg32(0x00000f00, IMX_EIM_CS4H);
  putreg32(0x00001501, IMX_EIM_CS4L);

  imxgpio_configprimary(GPIOA, 21);
  imxgpio_configprimary(GPIOA, 22);

  regval = getreg32(IMX_CS4_VSECTION + 0x0c);
  regval = getreg32(IMX_CS4_VSECTION + 0x0c);
#endif
}
