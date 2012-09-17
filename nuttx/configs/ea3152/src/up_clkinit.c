/****************************************************************************
 * configs/ea3152/src/up_clkinit.c
 * arch/arm/src/board/up_clkinit.c
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

#include "lpc31_cgu.h"
#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Sub-domain Clock Bitsets *************************************************/
/* The following bitsets group clocks into bitsets associated with each
 * domain and fractional divider subdomain.
 *
 * Domain 0 (DOMAINID_SYS), Clocks 0 - 29, Fraction dividers 0-6.  Clocks not
 * defined in the clock sets will be sourced with SYS_BASE_CLK.
 */

/* Domain 0, Fractional divider 0: */

#define CGU_CLKSET_DOMAIN0_DIV0 \
  (_D0B(CLKID_APB0CLK)|_D0B(CLKID_APB1CLK)|_D0B(CLKID_APB2CLK)|\
   _D0B(CLKID_APB3CLK)|_D0B(CLKID_APB4CLK)|_D0B(CLKID_AHB2INTCCLK)|\
   _D0B(CLKID_AHB0CLK)|_D0B(CLKID_DMAPCLK)|_D0B(CLKID_DMACLKGATED)|\
   _D0B(CLKID_NANDFLASHS0CLK)|_D0B(CLKID_NANDFLASHPCLK)|\
   _D0B(CLKID_ARM926BUSIFCLK)|_D0B(CLKID_SDMMCHCLK)|_D0B(CLKID_USBOTGAHBCLK)|\
   _D0B(CLKID_ISRAM0CLK)|_D0B(CLKID_ISRAM1CLK)|_D0B(CLKID_ISROMCLK)|\
   _D0B(CLKID_MPMCCFGCLK)|_D0B(CLKID_MPMCCFGCLK2)|_D0B(CLKID_INTCCLK))

/* Domain 0, Fractional divider 1: */

#define CGU_CLKSET_DOMAIN0_DIV1 \
  (_D0B(CLKID_ARM926CORECLK))

/* Domain 0, Fractional divider 2: */

#define CGU_CLKSET_DOMAIN0_DIV2 \
  (_D0B(CLKID_NANDFLASHAESCLK)|_D0B(CLKID_NANDFLASHNANDCLK))

/* Domain 0, Fractional divider 3: */

#define CGU_CLKSET_DOMAIN0_DIV3 \
  (_D0B(CLKID_NANDFLASHECCCLK))

/* Domain 0, Fractional divider 4: */

#define CGU_CLKSET_DOMAIN0_DIV4 \
  (_D0B(CLKID_SDMMCCCLKIN))

/* Domain 0, Fractional divider 5: */

#define CGU_CLKSET_DOMAIN0_DIV5 \
  (_D0B(CLKID_CLOCKOUT))

/* Domain 0, Fractional divider 6: */

#define CGU_CLKSET_DOMAIN0_DIV6 \
  (_D0B(CLKID_EBICLK))

/* Domain 1 (DOMAINID_AHB0APB0), Clocks 30-39, Fraction dividers 7-8.  Clocks
 * not defined in the clock sets will be sourced with AHB_APB0_BASE_CLK.
 */

/* Domain 1, Fractional divider 7: */

#define CGU_CLKSET_DOMAIN1_DIV7 \
  (_D1B(CLKID_ADCCLK))

/* Domain 1, Fractional divider 8: */

#define CGU_CLKSET_DOMAIN1_DIV8 \
  (_D1B(CLKID_AHB2APB0PCLK)|_D1B(CLKID_EVENTROUTERPCLK)|\
   _D1B(CLKID_ADCPCLK)|_D1B(CLKID_WDOGPCLK)|_D1B(CLKID_IOCONFPCLK)|\
   _D1B(CLKID_CGUPCLK)|_D1B(CLKID_SYSCREGPCLK)|_D1B(CLKID_OTPPCLK)|\
   _D1B(CLKID_RNGPCLK))

/* Domain 2 (DOMAINID_AHB0APB1), Clocks 40-49, Fraction dividers 9-10.  Clocks
 * not defined in the clock sets will be sourced with AHB_APB1_BASE_CLK.
 */

/* Domain 2, Fractional divider 9: */

#define CGU_CLKSET_DOMAIN2_DIV9 \
  (_D2B(CLKID_AHB2APB1PCLK)|_D2B(CLKID_TIMER0PCLK)|_D2B(CLKID_TIMER1PCLK)|\
   _D2B(CLKID_TIMER2PCLK)|_D2B(CLKID_TIMER3PCLK)|_D2B(CLKID_PWMPCLK)|\
   _D2B(CLKID_PWMPCLKREGS)|_D2B(CLKID_I2C0PCLK)|_D2B(CLKID_I2C1PCLK))

/* Domain 2, Fractional divider 10: */

#define CGU_CLKSET_DOMAIN2_DIV10 \
  (_D2B(CLKID_PWMCLK))

/* Domain 3 (DOMAINID_AHB0APB2), Clocks 50-57, Fraction dividers 11-13.  Clocks
 * not defined in the clock sets will be sourced with AHB_APB2_BASE_CLK.
 */

/* Domain 3, Fractional divider 11: */

#define CGU_CLKSET_DOMAIN3_DIV11 \
  ( _D3B(CLKID_AHB2APB2PCLK)|_D3B(CLKID_PCMPCLK)|_D3B(CLKID_PCMAPBPCLK)|\
    _D3B(CLKID_UARTAPBCLK)|_D3B(CLKID_LCDPCLK)|_D3B(CLKID_SPIPCLK)|\
    _D3B(CLKID_SPIPCLKGATED))

/* Domain 3, Fractional divider 12: */

#define CGU_CLKSET_DOMAIN3_DIV12 \
  (_D3B(CLKID_LCDCLK))

/* Domain 3, Fractional divider 13: */

#define CGU_CLKSET_DOMAIN3_DIV13 \
  (0)

/* Domain 4 (DOMAINID_AHB0APB3), Clocks 58-70, Fraction divider 14.  Clocks
 * not defined in the clock sets will be sourced with AHB_APB3_BASE_CLK.
 */

#define CGU_CLKSET_DOMAIN4_DIV14 \
  (_D4B(CLKID_AHB2APB3PCLK)|_D4B(CLKID_I2SCFGPCLK)|_D4B(CLKID_EDGEDETPCLK)|\
   _D4B(CLKID_I2STXFIFO0PCLK)|_D4B(CLKID_I2STXIF0PCLK)|_D4B(CLKID_I2STXFIFO1PCLK)|\
   _D4B(CLKID_I2STXIF1PCLK)|_D4B(CLKID_I2SRXFIFO0PCLK)|_D4B(CLKID_I2SRXIF0PCLK)|\
   _D4B(CLKID_I2SRXFIFO1PCLK)|_D4B(CLKID_I2SRXIF1PCLK))

/* Domain 5 (DOMAINID_PCM), Clock 71, Fraction divider 15.  Clocks not
 * defined in the clock sets will be sourced with AHB_APB3_BASE_CLK.
 */

#define CGU_CLKSET_DOMAIN5_DIV15 \
  (_D5B(CLKID_PCMCLKIP))

/* Domain 6 (DOMAINID_UART), Clock 72, Fraction divider 16.  Clocks mpt
 * defined in the clock sets will be sourced with UART_BASE_CLK.
 */

#define CGU_CLKSET_DOMAIN6_DIV16 \
  (0)

/* Domain 7 (DOMAINID_CLK1024FS), Clocks 73-86, Fraction dividers 17-22.  Clocks
 * not defined in the clock sets will be sourced with CLK1024FS_BASE_CLK.
 */

/* Domain 7, Fractional divider 17: */

#define CGU_CLKSET_DOMAIN7_DIV17 \
  ( _D7B(CLKID_I2SEDGEDETECTCLK)|_D7B(CLKID_I2STXWS0)|_D7B(CLKID_I2STXWS1)|\
    _D7B(CLKID_I2SRXWS0)|_D7B(CLKID_I2SRXWS1))

/* Domain 7, Fractional divider 18: */

#define CGU_CLKSET_DOMAIN7_DIV18 \
  ( _D7B(CLKID_I2STXBCK0N)|_D7B(CLKID_I2STXBCK1N))

/* Domain 7, Fractional divider 19: */

#define CGU_CLKSET_DOMAIN7_DIV19 \
  ( _D7B(CLKID_I2STXCLK0)|_D7B(CLKID_CLK256FS))

/* Domain 7, Fractional divider 20: */

#define CGU_CLKSET_DOMAIN7_DIV20 \
  ( _D7B(CLKID_I2SRXBCK0N)|_D7B(CLKID_I2SRXBCK1N))

/* Domain 7, Fractional divider 21: */

#define CGU_CLKSET_DOMAIN7_DIV21 \
  (0)

/* Domain 7, Fractional divider 22: */

#define CGU_CLKSET_DOMAIN7_DIV22 \
  (0)

/* Domain 8 (DOMAINID_BCK0, clock 87, and domain 9 (DOMAINID_BCK1), clock 88,
 * are directly connected
 */

/* Domain 10 (DOMAINID_SPI), Clocks 89-90, Fraction divider 23.  Clocks
 * not defined in the clock sets will be sourced with SPI_CLK_BASE_CLK.
 */

#define CGU_CLKSET_DOMAIN10_DIV23 \
  ( _D10B(CLKID_SPICLK)|_D10B(CLKID_SPICLKGATED))

/* Domain 11 (DOMAINID_SYSCLKO, clock 91, is directly connected */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Default clock configuration for the EA3152 board.  Every board must
 * provide an implementation of g_boardclks.  This rather complex structure
 * is used by the boot-up logic to configure initial lpc313x clocking.
 *
 *   FFAST:           12MHz
 *   MASTER PLL Freq: 180MHz;
 *   AUDIOPLL Freq:   1024Fs, Fs = 44.1kHz
 *
 * Domain                   Input             Subdomain         Divider Ratio
 * ------------------------ ----------------- ----------------- -------------
 * 0 - DOMAIN_SYS           MASTER PLL(HPLL1) DOMAIN0_DIV0      1/2
 *                                            DOMAIN0_DIV1      1
 *                                            DOMAIN0_DIV2      1/2
 *                                            DOMAIN0_DIV3      1/4
 *                                            DOMAIN0_DIV4      1/4
 *                                            DOMAIN0_DIV5      1/2
 *                                            DOMAIN0_DIV6      1/2
 *
 * 1 - DOMAIN_AHB0APB0      FFAST             DOMAIN1_DIV7      1/38
 *                                            DOMAIN1_DIV8      1/2
 *
 * 2 - DOMAIN_AHB0APB1      FFAST             DOMAIN2_DIV9      1/2
 *                                            DOMAIN2_DIV10     1/2
 *
 * 3 - DOMAIN_AHB0APB2      MASTER PLL(HPLL1) DOMAIN3_DIV11     1/2
 *                                            DOMAIN3_DIV12     1/40
 *                                            DOMAIN3_DIV13     1 (not used)
 *
 * 4 - DOMAIN_AHB0APB3      FFAST             DOMAIN4_DIV14     1/2
 *
 * 5 - DOMAIN_PCM           MASTER PLL(HPLL1) DOMAIN5_DIV15     1/2
 *
 * 6 - DOMAIN_UART          FFAST             DOMAIN6_DIV16     1
 *
 * 7 - DOMAIN_CLCK1024FS    AUDIO PLL(HPLL0)  DOMAIN7_DIV17     1/256
 *                                            DOMAIN7_DIV18     1/4
 *                                            DOMAIN7_DIV19     1
 *                                            DOMAIN7_DIV20     1/4
 *                                            DOMAIN7_DIV21     1/32
 *                                            DOMAIN7_DIV22     1/2
 *
 * 8 - DOMAIN_I2SRXBCK0     I2SRX_BCK0        -                 -
 *
 * 9 - DOMAIN_I2SRXBCK1     I2SRX_BCK1        -                 -
 *
 * 10 - DOMAIN_SPI          MASTER PLL(HPLL1) DOMAIN10_DIV23    1/2
 *
 * 11 - DOMAIN_SYSCLKO      FFAST             -                 -
 */

const struct lpc31_clkinit_s g_boardclks =
{
  /* Domain 0 (DOMAINID_SYS), Clocks 0 - 29, Fraction dividers 0-6 */

  {
    CGU_FREQIN_HPPLL1,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN0_DIV0},
      {{0, 0, 0}, CGU_CLKSET_DOMAIN0_DIV1},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN0_DIV2},
      {{1, 1, 4}, CGU_CLKSET_DOMAIN0_DIV3},
      {{1, 1, 4}, CGU_CLKSET_DOMAIN0_DIV4},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN0_DIV5},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN0_DIV6}
    }
  },

  /* Domain 1 (DOMAINID_AHB0APB0), Clocks 30-39, Fraction dividers 7-8 */

  {
    CGU_FREQIN_FFAST,
    {
      {{1, 1, 38}, CGU_CLKSET_DOMAIN1_DIV7},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN1_DIV8}
    }
  },

  /* Domain 2 (DOMAINID_AHB0APB1), Clocks 40-49, Fraction dividers 9-10 */

  {
    CGU_FREQIN_FFAST,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN2_DIV9},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN2_DIV10}
    }
  },

  /* Domain 3 (DOMAINID_AHB0APB2), Clocks 50-57, Fraction dividers 11-13 */

  {
    CGU_FREQIN_HPPLL1,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN3_DIV11},
      {{1, 1, 40}, CGU_CLKSET_DOMAIN3_DIV12},
      {{0, 0, 0}, CGU_CLKSET_DOMAIN3_DIV13}
    }
  },

  /* Domain 4 (DOMAINID_AHB0APB3), Clocks 58-70, Fraction divider 14 */

  {
    CGU_FREQIN_FFAST,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN4_DIV14}
    }
  },

  /* Domain 5 (DOMAINID_PCM), Clock 71, Fraction divider 15 */

  {
    CGU_FREQIN_HPPLL1,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN5_DIV15}
    }
  },

  /* Domain 6 (DOMAINID_UART), Clock 72, Fraction divider 16 */

  {
    CGU_FREQIN_FFAST,
    {
      {{0, 0, 0}, CGU_CLKSET_DOMAIN6_DIV16}
    }
  },

  /* Domain 7 (DOMAINID_CLK1024FS), Clocks 73-86, Fraction dividers 17-22 */

  {
    CGU_FREQIN_HPPLL0,
    {
      {{1, 1, 256}, CGU_CLKSET_DOMAIN7_DIV17},
      {{1, 1, 4}, CGU_CLKSET_DOMAIN7_DIV18},
      {{0, 0, 0}, CGU_CLKSET_DOMAIN7_DIV19},
      {{1, 1, 4}, CGU_CLKSET_DOMAIN7_DIV20},
      {{1, 1, 32}, CGU_CLKSET_DOMAIN7_DIV21},
      {{1, 1, 2}, CGU_CLKSET_DOMAIN7_DIV22}
    }
  },

  /* Domain 8 (DOMAINID_BCK0, clock 87 */

  {
    CGU_FREQIN_I2SRXBCK0
  },

  /* Domain 9 (DOMAINID_BCK1, clock 88 */

  {
    CGU_FREQIN_I2SRXBCK1
  },

  /* Domain 10 (DOMAINID_SPI), Clocks 89-90, Fraction divider 23 */

  {
    CGU_FREQIN_HPPLL1,
    {
      {{1, 1, 2}, CGU_CLKSET_DOMAIN10_DIV23}
    }
  },

  /* Domain 11 (DOMAINID_SYSCLKO, clock 91 */

  {
    CGU_FREQIN_FFAST
  },

  /* Dynamic fractional divider configuration (7) */

#if 0 /* Dynamic fractional divider initialization not implemented */
  {
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 64}
    },
    {
      CGU_DYNSEL_ALLBITS, {0, 0, 0}
    },
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 3}
    },
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 6}
    },
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 6}
    },
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 6}
    },
    {
      CGU_DYNSEL_ALLBITS, {1, 1, 3}
    }
  }
#endif
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ledinit
 ****************************************************************************/

