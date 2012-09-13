/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_irq.c
 *
 *   Copyright (C) 2010 Rommel Marcelo. All rights reserved.
 *   Author: Rommel Marcelo
 *
 * This file is part of the NuttX RTOS:
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

/* This file holds the NuttX start logic that runs when the LPC2378
 * is reset.  This logic must be located at address 0x0000:0000 in
 * flash but may be linked to run at different locations based on
 * the selected mode:
 *
 * default: Executes from 0x0000:0000.  In non-default modes, the
 *   MEMAP register is set override the settings of the CPU configuration
 *   pins.
 *
 *  CONFIG_EXTMEM_MODE: Code executes from external memory starting at
 *    address 0x8000:0000.
 *
 *  CONFIG_RAM_MODE: Code executes from on-chip RAM at address
 *     0x4000:0000.
 *
 * Starupt Code must be linked to run at the correct address
 * corresponding to the selected mode.
 */

/***********************************************************************
 * Included Files
 **********************************************************************/

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <sys/types.h>

#include "arm.h"
#include "up_arch.h"
#include "internal.h"
#include "lpc23xx_pinsel.h"
#include "lpc23xx_scb.h"

extern void IO_Init(void);

/***********************************************************************
 * Definitions
 **********************************************************************/

#if ((FOSC < 32000) || (FOSC > 50000000))
#  error Fosc out of range (32KHz-50MHz)
#  error correct and recompile
#endif

#if ((CCLK < 10000000) || (CCLK > 72000000))
#  error cclk out of range (10MHz-72MHz)
#  error correct PLL MULTIPLIER and recompile
#endif

#if ((FCCO < 275000000) || (FCCO > 550000000))
#  error Fcco out of range (275MHz-550MHz)
#  error internal algorithm error
#endif

/* Phase Locked Loop (PLL) initialization values
 *
 * Bit 0:14 MSEL: PLL Multiplier "M" Value
 *               CCLK = 57 600 000 Hz
 * Bit 16:23 NSEL: PLL Divider "N" Value
 *               Fcco = (2 * M * F_in) / N
 *                               275MHz <= Fcco <= 550MHz
 *
 * PLL clock sources:
 * Internal RC          0 default on reset
 * Main Oscillator      1
 * RTC                  2
 */

#ifdef CONFIG_PLL_CLKSRC
#       if ( (CONFIG_PLL_CLKSRC < 0) || (CONFIG_PLL_CLKSRC > 2) )
#               error "PLL clock source not valid, check configuration "
#       endif
#else
#       error "PLL clock source not defined, check configuration file"
#endif

/* PLL provides CCLK and must always be configured */

#define PLL     ( PLL_M | (PLL_N << 16) )

/* Memory Accelerator Module (MAM) initialization values
 *
 * MAM Control Register
 *   Bit 0:1 Mode
 *           0 = Disabled
 *           1 = Partially Enabled
 *           2 = Fully Enabled
 * MAM Timing Register
 *   Bit 0:2 Fetch Cycles
 *           0 = Reserved
 *           1 = 1 CCLK
 *           2 = 2 CCLK
 *           3 = 3 CCLK
 *           4 = 4 CCLK
 *           5 = 5 CCLK
 *           6 = 6 CCLK
 *           7 = 7 CCLK
 */

/* LPC2378 Rev. '-' errata MAM may not work if fully enabled */

#ifdef CONFIG_MAM_SETUP
#  ifndef CONFIG_MAMCR_VALUE    /* Can be selected from config file */
#    define CONFIG_MAMCR_VALUE  (MAMCR_PART)
#  endif

#  ifndef CONFIG_MAMTIM_VALUE   /* Can be selected from config file */
#    define CONFIG_MAMTIM_VALUE (0x00000003)
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_scbpllfeed
 ****************************************************************************/

static inline void up_scbpllfeed(void)
{
  SCB_PLLFEED = 0xAA;
  SCB_PLLFEED = 0x55;
}

/****************************************************************************
 * Name: ConfigurePLL
 ****************************************************************************/

void ConfigurePLL(void)
{
  uint32_t MSel, NSel;

  /* LPC2378 Rev.'-' errata Enable the Ethernet block to enable 16k EnetRAM */
  SCB_PCONP |= PCENET;

  /* Vectors are remapped to Flash */
  SCB_MEMMAP = MEMMAP2FLASH;

  /* Enable PLL, disconnected */
  if (SCB_PLLSTAT & (1 << 25))
    {
      SCB_PLLCON = 0x01;
      up_scbpllfeed();
    }

  /* Disable PLL, disconnected */
  SCB_PLLCON = 0;
  up_scbpllfeed();

  /* Enable main OSC */
  SCB_SCS |= 0x20;

  /* Wait until main OSC is usable */
  while (!(SCB_SCS & 0x40));

  /* select main OSC, 12MHz, as the PLL clock source */
  SCB_CLKSRCSEL = CONFIG_PLL_CLKSRC;

  /* Reconfigure PLL */
  SCB_PLLCFG = PLL;
  up_scbpllfeed();

  /* Enable PLL */
  SCB_PLLCON = 0x01;
  up_scbpllfeed();

  /* Set clock divider */
  SCB_CCLKCFG = CCLK_DIV;

#ifdef CONFIG_USBDEV
  /* usbclk = 288 MHz/6 = 48 MHz */
  SCB_USBCLKCFG = USBCLK_DIV;
  /* Turn On USB PCLK */
  SCB_PCONP |= PCUSB;
#endif

  /* Wait for PLL to lock */
  while ((SCB_PLLSTAT & (1 << 26)) == 0);

  MSel = SCB_PLLSTAT & 0x00007FFF;
  NSel = (SCB_PLLSTAT & 0x00FF0000) >> 16;
  while ((MSel != PLL_M) && (NSel != PLL_N));

  /* Enable and connect */
  SCB_PLLCON = 0x03;
  up_scbpllfeed();

  /* Check connect bit status */
  while ((SCB_PLLSTAT & (1 << 25)) == 0);

  /* Set memory accelerater module */
  SCB_MAMCR = 0;
  SCB_MAMTIM = CONFIG_MAMTIM_VALUE;
  SCB_MAMCR = CONFIG_MAMCR_VALUE;

  /* Enable FastIO on P0:P1 */
  SCB_SCS |= 0x01;

  IO_Init();

  return;
}
