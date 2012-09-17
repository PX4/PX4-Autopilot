/************************************************************************************
 * configs/stm3210e-eval/src/up_selectlcd.c
 * arch/arm/src/board/up_selectlcd.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm3240g-internal.h"

#ifdef CONFIG_STM32_FSMC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/* SRAM pin definitions */

#define LCD_NADDRLINES   1
#define LCD_NDATALINES   16

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/*   Pin Usage (per schematic)
 *                         SRAM       LCD
 *   D[0..15]              [0..15]    [0..15]
 *   A[0..25]              [0..22]    [0] RS
 *   FSMC_NBL0  PE0   OUT  ---        ---
 *   FSMC_NBL1  PE1   OUT  ---        ---
 *   FSMC_NE2   PG9   OUT  ---        ---
 *   FSMC_NE3   PG10  OUT  ---        ~CS
 *   FSMC_NE4   PG12  OUT  ---        ---
 *   FSMC_NWE   PD5   OUT  ---        ~WR/SCL
 *   FSMC_NOE   PD4   OUT  ---        ~RD
 *   FSMC_NWAIT PD6   IN   ---        ---
 *   FSMC_INT2  PG6*  IN   ---        ---
 *   FSMC_INT3
 *   FSMC_INTR
 *   FSMC_CD
 *   FSMC_CLK
 *   FSMC_NCE2
 *   FSMC_NCE3   
 *   FSMC_NCE4_1
 *   FSMC_NCE4_2
 *   FSMC_NIORD
 *   FSMC_NIOWR
 *   FSMC_NL
 *   FSMC_NREG
 */

/* GPIO configurations unique to the LCD  */

static const uint32_t g_lcdconfig[] =
{
  /* NOE, NWE, and NE3  */

  GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NE3
};
#define NLCD_CONFIG (sizeof(g_lcdconfig)/sizeof(uint32_t))

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ************************************************************************************/

void stm32_selectlcd(void)
{
  /* Configure new GPIO pins */

  stm32_extmemaddr(LCD_NADDRLINES);             /* Common address lines: A0 */
  stm32_extmemdata(LCD_NDATALINES);             /* Common data lines: D0-D15 */
  stm32_extmemgpios(g_lcdconfig, NLCD_CONFIG);  /* LCD-specific control lines */

  /* Enable AHB clocking to the FSMC */

  stm32_enablefsmc();

  /* Color LCD configuration (LCD configured as follow):
   * 
   *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
   *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
   *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
   *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
   *   - Write Operation   = Enable    "FSMC_BCR_WREN"
   *   - Asynchronous Wait = Disable
   */

  /* Bank3 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR3);

  /* Bank3 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(0) | FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTRUN(0) |
           FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) | FSMC_BTR_ACCMODA, STM32_FSMC_BTR3);

  putreg32(0xffffffff, STM32_FSMC_BWTR3);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR3);
}

#endif /* CONFIG_STM32_FSMC */


