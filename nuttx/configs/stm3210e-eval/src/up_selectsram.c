/************************************************************************************
 * configs/stm3210e-eval/src/up_selectsram.c
 * arch/arm/src/board/up_selectsram.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm3210e-internal.h"

#ifdef CONFIG_STM32_FSMC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if STM32_NGPIO_PORTS < 6
#  error "Required GPIO ports not enabled"
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* 512Kx16 SRAM is connected to bank2 of the FSMC interface and both 8- and 16-bit
 * accesses are allowed by BLN0 and BLN1 connected to BLE and BHE of SRAM,
 * respectively.
 *
 * Pin Usage (per schematic)
 *                         FLASH   SRAM    NAND    LCD
 *   D[0..15]              [0..15] [0..15] [0..7]  [0..15]
 *   A[0..23]              [0..22] [0..18] [16,17] [0]
 *   FSMC_NBL0  PE0   OUT  ~BLE    ---     ---     ---
 *   FSMC_NBL1  PE1   OUT  ~BHE    ---     ---     ---
 *   FSMC_NE2   PG9   OUT  ---     ~E      ---     ---
 *   FSMC_NE3   PG10  OUT  ~CE     ---     ---     ---
 *   FSMC_NE4   PG12  OUT  ---     ---     ---     ~CS
 *   FSMC_NWE   PD5   OUT  ~WE     ~W      ~W      ~WR/SCL
 *   FSMC_NOE   PD4   OUT  ~OE     ~G      ~R      ~RD
 *   FSMC_NWAIT PD6   IN   ---     R~B     ---     ---
 *   FSMC_INT2  PG6*  IN   ---     ---     R~B     ---
 *
 *   *JP7 will switch to PD6
 */

/* GPIO configurations unique to SRAM  */

static const uint16_t g_sramconfig[] =
{
  /* NE3, NBL0, NBL1,  */

  GPIO_NPS_NE3, GPIO_NPS_NBL0, GPIO_NPS_NBL1
};
#define NSRAM_CONFIG (sizeof(g_sramconfig)/sizeof(uint16_t))

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_selectsram
 *
 * Description:
 *   Initialize to access external SRAM
 *
 ************************************************************************************/

void stm32_selectsram(void)
{
  /* Configure new GPIO state */

  stm32_extmemgpios(g_commonconfig, NCOMMON_CONFIG);
  stm32_extmemgpios(g_sramconfig, NSRAM_CONFIG);

  /* Enable AHB clocking to the FSMC */

  stm32_enablefsmc();

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR3);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(1)|FSMC_BTR_ADDHLD(1)|FSMC_BTR_DATAST(3)|FSMC_BTR_BUSTRUN(1)|
           FSMC_BTR_CLKDIV(1)|FSMC_BTR_DATLAT(2)|FSMC_BTR_ACCMODA, STM32_FSMC_BTR3);

  putreg32(0xffffffff, STM32_FSMC_BWTR3);

  /* Enable the bank */

  putreg32(FSMC_BCR_MBKEN|FSMC_BCR_MWID16|FSMC_BCR_WREN, STM32_FSMC_BCR3);
}

#endif /* CONFIG_STM32_FSMC */


