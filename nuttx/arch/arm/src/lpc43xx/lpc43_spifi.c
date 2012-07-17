/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_spifi.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip.h"
#include "lpc43_cgu.h"
#include "lpc43_spifi.h"
#include "lpc43_pinconfig.h"

#ifdef CONFIG_LPC43_SPIFI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select the divider to use as SPIFI input based on definitions in the
 * board.h header file.
 */

#if defined(BOARD_SPIFI_PLL1)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_PLL1
#elif defined(BOARD_SPIFI_DIVA)
#  define LPC43_IDIV_CTRL        LPC43_IDIVA_CTRL
#  define IDIV_CTRL_PD           IDIVA_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVA_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVA_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     4
#  define IDIV_CTRL_CLKSEL_MASK  IDIVA_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVA_CLKSEL_PLL1 | IDIVA_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVA
#elif defined(BOARD_SPIFI_DIVB)
#  define LPC43_IDIV_CTRL        LPC43_IDIVB_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVB
#elif defined(BOARD_SPIFI_DIVC)
#  define LPC43_IDIV_CTRL        LPC43_IDIVC_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVC
#elif defined(BOARD_SPIFI_DIVD)
#  define LPC43_IDIV_CTRL        LPC43_IDIVD_CTRL
#  define IDIV_CTRL_PD           IDIVBCD_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVBCD_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVBCD_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     16
#  define IDIV_CTRL_CLKSEL_MASK  IDIVBCD_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVBCD_CLKSEL_PLL1 | IDIVBCD_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVD
#elif defined(BOARD_SPIFI_DIVE)
#  define LPC43_IDIV_CTRL        LPC43_IDIVE_CTRL
#  define IDIV_CTRL_PD           IDIVE_CTRL_PD
#  define IDIV_CTRL_IDIV_MASK    IDIVE_CTRL_IDIV_MASK
#  define IDIV_CTRL_IDIV         IDIVE_CTRL_IDIV(BOARD_SPIFI_DIVIDER)
#  define IDIV_CTRL_IDIV_MAX     256
#  define IDIV_CTRL_CLKSEL_MASK  IDIVE_CTRL_CLKSEL_MASK
#  define IDIV_CTRL_CLKSEL_PLL1  (IDIVE_CLKSEL_PLL1 | IDIVE_CTRL_AUTOBLOCK)
#  define BASE_SPIFI_CLKSEL      BASE_SPIFI_CLKSEL_IDIVE
#endif

#if BOARD_SPIFI_DIVIDER < 1 || BOARD_SPIFI_DIVIDER > IDIV_CTRL_IDIV_MAX
#  error "Invalid value for  BOARD_SPIFI_DIVIDER"
#endif

/* The final parameter of the spifi_init() ROM driver call should be the
 * serial clock rate divided by 1000000, rounded to an integer.  The SPIFI
 * supports transfer rates of up to SPIFI_CLK/2 bytes per second.  The SPIF_CLK
 * is the output of the LPC43_BASE_SPIFI_CLK configured above; The frequency should
 * be given by BOARD_SPIFI_FREQUENCY as provided by the board.h header file.
 */

#define SCLK_MHZ (BOARD_SPIFI_FREQUENCY + (1000000 / 2)) / 1000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spifi_idiv_input
 *
 * Description:
 *   Configure PLL1 as the input to the selected divider and enable the
 *   divider.
 *
 ****************************************************************************/

#ifndef BOARD_SPIFI_PLL1
static inline void spifi_idiv_input(void)
{
  uint32_t regval;

  /* Configure PLL1 as the input to the selected divider */

  regval  = getreg32(LPC43_IDIV_CTRL);
  regval &= ~IDIV_CTRL_CLKSEL_MASK;
  regval |= IDIV_CTRL_CLKSEL_PLL1;
  putreg32(regval, LPC43_IDIV_CTRL);

  /* Enable the divider (by making sure that the power down bit is clear) */

  regval &= ~IDIV_CTRL_PD;
  putreg32(regval, LPC43_IDIV_CTRL);

  /* Set the divider value */

  regval &= ~IDIVA_CTRL_IDIV_MASK;
  regval |= IDIV_CTRL_IDIV;
  putreg32(regval, LPC43_IDIV_CTRL);
}
#else
#  define spifi_idiv_input()
#endif

/****************************************************************************
 * Name: spifi_spifi_input
 *
 * Description:
 *   Configure the selected divider (or PLL1) as the input to the SPIFI
 *   and enable the SPIFI clock.
 *
 ****************************************************************************/

static inline void spifi_spifi_input(void)
{
  uint32_t regval;

  /* Configure the selected divider (or PLL1) as the input to the SPIFI */

  regval  = getreg32(LPC43_BASE_SPIFI_CLK);
  regval &= ~BASE_SPIFI_CLK_CLKSEL_MASK;
  regval |= BASE_SPIFI_CLKSEL;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);

  /* Enable the SPIFI clocking (by making sure that the power down bit is
   * clear)
   */

  regval &= ~IDIVA_CTRL_PD;
  putreg32(regval, LPC43_BASE_SPIFI_CLK);
}

/****************************************************************************
 * Name: spifi_pinconfig
 *
 * Description:
 *   Configure SPIFI pins
 *
 ****************************************************************************/

static inline void spifi_pinconfig(void)
{
  /* Configure SPIFI pins */

  lpc43_pin_config(PINCONF_SPIFI_CS);   /* Input buffering not needed */
  lpc43_pin_config(PINCONF_SPIFI_MISO); /* High drive for SCLK */
  lpc43_pin_config(PINCONF_SPIFI_MOSI);
  lpc43_pin_config(PINCONF_SPIFI_SCK);
  lpc43_pin_config(PINCONF_SPIFI_SIO2);
  lpc43_pin_config(PINCONF_SPIFI_SIO3);
}

/****************************************************************************
 * Name: spifi_rominit
 *
 * Description:
 *   Initialize the SPIFI ROM driver
 *
 ****************************************************************************/

static inline int spifi_rominit(void)
{
  struct spifi_driver_s *pspifi = *((struct spifi_driver_s **)SPIFI_ROM_PTR);
  struct spifi_dev_s dev;
  int32_t result;

 /* The final parameter of the spifi_init() ROM driver call should be the
  * serial clock rate divided by 1000000, rounded to an integer.  The SPIFI
  * supports transfer rates of up to SPIFI_CLK/2 bytes per second.  The SPIF_CLK
  * is the output of the LPC43_BASE_SPIFI_CLK configured above; The frequency should
  * be given by BOARD_SPIFI_FREQUENCY as provided by the board.h header file.
  *
  * A return value of zero frp spifi_init() indicates success.  Non-zero error
  * codes include:
  *
  *   0x2000A  No operative serial flash (JEDEC ID all zeroes or all ones)
  *   0x20009  Unknown manufacturer code
  *   0x20008  Unknown device type code
  *   0x20007  Unknown device ID code
  *   0x20006  Unknown extended device ID value (only for Spansion 25FL12x
  *            in the initial API)
  *   0x20005  Device status error
  *   0x20004  Operand error: S_MODE3+S_FULLCLK+S_RCVCLK in options
  */

  result = pspifi->spifi_init(&dev, 9, SPIFI_RCVCLK | SPIFI_FULLCLK, SCLK_MHZ);
  if (result != 0)
    {
      fdbg("ERROR: spifi_init failed: %05x\n", result);

      /* Try again */

      result = pspifi->spifi_init(&dev, 9, SPIFI_RCVCLK | SPIFI_FULLCLK, SCLK_MHZ);
      if (result != 0)
        {
          fdbg("ERROR: spifi_init failed: %05x\n", result);
          return -ENODEV;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  lpc43_spifi_initialize
 *
 * Description:
 *   Initialize the SPIFI interface per settings in the board.h file
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success; on failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int lpc43_spifi_initialize(void)
{
  irqstate_t flags;
  int ret = OK;

  flags = irqsave();

  /* The SPIFI will receive clocking from a divider per the settings
   * provided in the board.h file.  Configure PLL1 as the input clock
   * for the selected divider
   */

  spifi_idiv_input();

  /* Configure SPIFI to received clocking from the selected divider */

  spifi_spifi_input();

  /* Configure SPIFI pins */

  spifi_pinconfig();


  /* Initialize the SPIFI ROM driver */

  ret = spifi_rominit();
  irqrestore(flags);
  return ret;
}

#endif /* CONFIG_LPC43_SPIFI */
