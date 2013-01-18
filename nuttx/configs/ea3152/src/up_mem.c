/****************************************************************************
 * configs/ea3152/src/up_mem.c
 * arch/arm/src/board/up_mem.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   - NXP UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - NXP lpc313x.cdl.drivers.zip example driver code
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "lpc31_syscreg.h"
#include "lpc31_cgudrvr.h"
#include "lpc31_mpmc.h"
#include "ea3152_internal.h"

#ifdef CONFIG_ARCH_EXTDRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The MPMC delay based on trace lengths between SDRAM and the chip and on
 * the delay strategy used for SDRAM.
 */

#define EA3152_MPMC_DELAY           0x824

/* Delay constants in nanosecondss for MT48LC32M16LF SDRAM on board */

#define EA3152_SDRAM_TRP            (20)
#define EA3152_SDRAM_TRFC           (66)
#define EA3152_SDRAM_TRAS           (44)
#define EA3152_SDRAM_TREX           (75)
#define EA3152_SDRAM_TARP           4
#define EA3152_SDRAM_TWR            (75)
#define EA3152_SDRAM_TRC            (66)
#define EA3152_SDRAM_TRRD           (15)
#define EA3152_SDRAM_TMRD           (20)
#define EA3152_SDRAM_TXSR           (75)
#define EA3152_SDRAM_TDAL           (50)
#define EA3152_SDRAM_REFRESH        (100)
#define EA3152_SDRAM_OPERREFRESH    (7812)

/* Macro used to convert the above values (in nanoseconds) into units of
 * the HCLK.
 */

#define NS2HCLKS(ns,hclk2,mask) \
  ((uint32_t)(((uint64_t)ns *(uint64_t)hclk2) / 1000000000ull) & mask)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_sdraminitialize
 *
 * Description:
 *   Configure SDRAM on the EA3152 board
 *
 * Micron Initialization Sequence from their data sheet for the Micron
 * MT48LC32M16A2 32M x 16 SDRAM chip:
 *   
 *   "SDRAMs must be powered up and initialized in a predefined manner.
 *    Operational procedures other than those specified may result in
 *    undefined operation. Once power is applied to VDD and VDDQ
 *    (simultaneously) and the clock is stable (stable clock is defined as
 *    a signal cycling within timing constraints specified for the clock
 *    pin), the SDRAM requires a 100µs delay prior to issuing any command
 *    other than a COMMAND INHIBIT or NOP.
 *   
 *   "Starting at some point during this 100µs period and continuing at least
 *    through the end of this period, COMMAND INHIBIT or NOP commands should
 *    be applied.  Once the 100µs delay has been satisfied with at least one
 *    COMMAND INHIBIT or NOP command having been applied, a PRECHARGE command
 *    should be applied. All banks must then be precharged, thereby placing
 *    the device in the all banks idle state.
 *   
 *   "Once in the idle state, two AUTO REFRESH cycles must be performed. After
 *    the AUTO REFRESH cycles are complete, the SDRAM is ready for mode
 *    register programming.
 *   
 *   "Because the mode register will power up in an unknown state, it should
 *    be loaded prior to applying any operational command."
 *
 *  The JEDEC recommendation for initializing SDRAM is:
 *    
 *    APPLY POWER (Vdd/Vddq equally, and CLK is stable)
 *    Wait 200uS
 *    PRECHARGE all
 *    8 AUTO REFRESH COMMANDS
 *    LOAD MODE REGISTER
 *    SDRAM is ready for operation
 *    
 *  The Micron SDRAM parts will work fine with the JEDEC sequence, but also
 *  allow for a quicker init sequence of:
 *    
 *    APPLY POWER (Vdd/Vddq equally, and CLK is stable)
 *    Wait at least 100uS (during which time start applying and
 *       continue applying NOP or COMMAND INHIBIT)
 *    PRECHARGE all
 *    2 AUTO REFRESH COMMANDS (min requirement, more than 2 is also ok)
 *    LOAD MODE REGISTER
 *    SDRAM is ready for operation
 *
 ****************************************************************************/

static void lpc31_sdraminitialize(void)
{
  uint32_t tmp;
  uint32_t regval;

  /* These run-time calculations can be reduced dramatically if hclk is
   * replaced with an apriori value.
   */

#ifdef CONFIG_LPC31_SDRAMHCLK
# define HCLK CONFIG_LPC31_SDRAMHCLK
#else
  uint32_t hclk = lpc31_clkfreq(CLKID_MPMCCFGCLK2, DOMAINID_SYS);
# define HCLK hclk
#endif

  /* Check RTL for divide by 2 possible. If so change then enable the followng logic */
#if 0
  uint32_t hclk2 = hclk;

  if (((getreg32(LPC31_MPMC_CONFIG) & MPMC_CONFIG_CLK)) != 0)
    {
      hclk2 >>= 1;
    }
# define HCLK2 hclk2
#else
# define HCLK2 hclk
#endif
  up_udelay(100);

  /* Set command delay startergy */

  putreg32(MPMC_DYNREADCONFIG_CMDDEL, LPC31_MPMC_DYNREADCONFIG);

  /* Configure device config register nSDCE0 for proper width SDRAM */

  putreg32((MPMC_DYNCONFIG0_MDSDRAM|MPMC_DYNCONFIG_HP16_32MX16),
           LPC31_MPMC_DYNCONFIG0);
  putreg32((MPMC_DYNRASCAS0_RAS2CLK|MPMC_DYNRASCAS0_CAS2CLK),
           LPC31_MPMC_DYNRASCAS0);

  /* Min 20ns program 1 so that at least 2 HCLKs are used */

  putreg32(NS2HCLKS(EA3152_SDRAM_TRP,  HCLK2, MPMC_DYNTRP_MASK),
           LPC31_MPMC_DYNTRP);
  putreg32(NS2HCLKS(EA3152_SDRAM_TRAS, HCLK2, MPMC_DYNTRAS_MASK),
           LPC31_MPMC_DYNTRAS);
  putreg32(NS2HCLKS(EA3152_SDRAM_TREX, HCLK2, MPMC_DYNTSREX_MASK),
           LPC31_MPMC_DYNTSREX);
  putreg32(EA3152_SDRAM_TARP,
           LPC31_MPMC_DYNTAPR);
  putreg32(NS2HCLKS(EA3152_SDRAM_TDAL, HCLK2, MPMC_DYNTDAL_MASK),
           LPC31_MPMC_DYNTDAL);
  putreg32(NS2HCLKS(EA3152_SDRAM_TWR,  HCLK2, MPMC_DYNTWR_MASK),
           LPC31_MPMC_DYNTWR);
  putreg32(NS2HCLKS(EA3152_SDRAM_TRC,  HCLK2, MPMC_DYNTRC_MASK),
           LPC31_MPMC_DYNTRC);
  putreg32(NS2HCLKS(EA3152_SDRAM_TRFC, HCLK2, MPMC_DYNTRFC_MASK),
           LPC31_MPMC_DYNTRFC);
  putreg32(NS2HCLKS(EA3152_SDRAM_TXSR, HCLK2, MPMC_DYNTXSR_MASK),
           LPC31_MPMC_DYNTXSR);
  putreg32(NS2HCLKS(EA3152_SDRAM_TRRD, HCLK2, MPMC_DYNTRRD_MASK),
           LPC31_MPMC_DYNTRRD);
  putreg32(NS2HCLKS(EA3152_SDRAM_TMRD, HCLK2, MPMC_DYNTMRD_MASK),
           LPC31_MPMC_DYNTMRD);
  up_udelay(100);
  
  /* Issue continuous NOP commands  */

  putreg32((MPMC_DYNCONTROL_CE|MPMC_DYNCONTROL_CS|MPMC_DYNCONTROL_INOP),
           LPC31_MPMC_DYNCONTROL);

  /* Load ~200us delay value to timer1 */

  up_udelay(200);
  
  /* Issue a "pre-charge all" command */

  putreg32((MPMC_DYNCONTROL_CE|MPMC_DYNCONTROL_CS|MPMC_DYNCONTROL_IPALL),
           LPC31_MPMC_DYNCONTROL);

  /* Minimum refresh pulse interval (tRFC) for MT48LC32M16A2=80nsec,
   * 100nsec provides more than adequate interval.
   */

  putreg32(NS2HCLKS(EA3152_SDRAM_REFRESH, HCLK, MPMC_DYNREFRESH_TIMER_MASK),
           LPC31_MPMC_DYNREFRESH);

  /* Load ~250us delay value to timer1 */

  up_udelay(250);
  
  /* Recommended refresh interval for normal operation of the Micron
   * MT48LC16LFFG = 7.8125usec (128KHz rate). ((HCLK / 128000) - 1) =
    * refresh counter interval rate, (subtract one for safety margin).
   */

  putreg32(NS2HCLKS(EA3152_SDRAM_OPERREFRESH, HCLK, MPMC_DYNREFRESH_TIMER_MASK),
           LPC31_MPMC_DYNREFRESH);

  /* Select mode register update mode */

  putreg32((MPMC_DYNCONTROL_CE|MPMC_DYNCONTROL_CS|MPMC_DYNCONTROL_IMODE),
           LPC31_MPMC_DYNCONTROL);

  /* Program the SDRAM internal mode registers on bank nSDCE0 and reconfigure
   * the SDRAM chips.  Bus speeds up to 90MHz requires use of a CAS latency = 2.
   * To get correct value on address bus CAS cycle, requires a shift by 13 for
   * 16bit mode
   */

  tmp = getreg32(LPC31_EXTSDRAM0_VSECTION | (0x23 << 13));
  
  putreg32((MPMC_DYNCONFIG0_MDSDRAM|MPMC_DYNCONFIG_HP16_32MX16),
           LPC31_MPMC_DYNCONFIG0);
  putreg32((MPMC_DYNRASCAS0_RAS2CLK|MPMC_DYNRASCAS0_CAS2CLK),
           LPC31_MPMC_DYNRASCAS0);

  /* Select normal operating mode */

  putreg32((MPMC_DYNCONTROL_CE|MPMC_DYNCONTROL_CS|MPMC_DYNCONTROL_INORMAL),
           LPC31_MPMC_DYNCONTROL);

  /* Enable buffers */

  regval  = getreg32(LPC31_MPMC_DYNCONFIG0);
  regval |= MPMC_DYNCONFIG0_B;
  putreg32(regval, LPC31_MPMC_DYNCONFIG0);

  putreg32((MPMC_DYNCONTROL_INORMAL|MPMC_DYNCONTROL_CS),
           LPC31_MPMC_DYNCONTROL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_meminitialize
 *
 * Description:
 *   Initialize external memory resources (sram, sdram, nand, nor, etc.)
 *
 ****************************************************************************/

void lpc31_meminitialize(void)
{
  /* Configure the LCD pins in external bus interface (EBI/MPMC) memory mode.
   * 
   * LCD_CSB   -> MPMC_NSTCS_0
   * LCD_DB_1  -> MPMC_NSTCS_1
   * LCD_DB_0  -> MPMC_CLKOUT
   * LCD_E_RD  -> MPMC_CKE
   * LCD_RS    -> MPMC_NDYCS
   * LCD_RW_WR -> MPMC_DQM_1
   * LCD_DB_2  -> EBI_A_2
   * LCD_DB_3  -> EBI_A_3 l
   * LCD_DB_4  -> EBI_A_4 l
   * LCD_DB_5  -> EBI_A_5 l
   * LCD_DB_6  -> EBI_A_6
   * LCD_DB_7  -> EBI_A_7
   * LCD_DB_8  -> EBI_A_8
   * LCD_DB_9  -> EBI_A_9
   * LCD_DB_10 -> EBI_A_10
   * LCD_DB_11 -> EBI_A_11
   * LCD_DB_12 -> EBI_A_12
   * LCD_DB_13 -> EBI_A_13
   * LCD_DB_14 -> EBI_A_14
   * LCD_DB_15 -> EBI_A_15
   */

  putreg32(SYSCREG_MUX_LCDEBISEL_EBIMPMC, LPC31_SYSCREG_MUX_LCDEBISEL);

  /* Enable EBI clock */

  lpc31_enableclock(CLKID_EBICLK);
  
  /* Enable MPMC controller clocks */

  lpc31_enableclock(CLKID_MPMCCFGCLK);
  lpc31_enableclock(CLKID_MPMCCFGCLK2);
  lpc31_enableclock(CLKID_MPMCCFGCLK3);

  /* Enable the external memory controller */

  putreg32(MPMC_CONTROL_E, LPC31_MPMC_CONTROL);

  /* Force HCLK to MPMC_CLK to 1:1 ratio, little-endian mode */

  putreg32(0, LPC31_MPMC_CONFIG);

  /* Set MPMC delay based on trace lengths between SDRAM and the chip
   * and on the delay strategy used for SDRAM.
   */

  putreg32(EA3152_MPMC_DELAY, LPC31_SYSCREG_MPMC_DELAYMODES);
  
  /* Configure Micron MT48LC32M16A2 SDRAM on the EA3152 board */

  lpc31_sdraminitialize();
}
#endif /* CONFIG_ARCH_EXTDRAM */
