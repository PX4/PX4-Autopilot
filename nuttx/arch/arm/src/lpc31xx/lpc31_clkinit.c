/************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_clkinit.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#include "lpc31_cgu.h"
#include "lpc31_cgudrvr.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This structure describes the configuration of one domain */

struct lpc31_domainconfig_s
{
  enum lpc31_domainid_e dmnid; /* Domain ID */
  uint32_t finsel;               /* Frequency input selection */
  uint32_t clk1;                 /* ID of first clock in the domain */
  uint32_t nclks;                /* Number of clocks in the domain */
  uint32_t fdiv1;                /* First frequency divider in the domain */
  uint32_t nfdiv;                /* Number of frequency dividers in the domain */
  const struct lpc31_subdomainconfig_s* sub; /* Sub=domain array */
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc31_domaininit
 *
 * Description:
 *   Initialize one clock domain based on board-specific clock  configuration data
 *
 ************************************************************************************/

static void lpc31_domaininit(struct lpc31_domainconfig_s* dmn)
{
  const struct lpc31_subdomainconfig_s * sub = dmn->sub;
  uint32_t fdivcfg;
  uint32_t regaddr;
  uint32_t regval;
  int fdndx;
  int clkndx;
  int bcrndx = lpc31_bcrndx(dmn->dmnid);
  int esrndx;

  if (bcrndx != BCRNDX_INVALID)
    {
      /* Disable BCR for domain */

      regaddr = LPC31_CGU_BCR(bcrndx);
      putreg32(0, regaddr);
    }

  /* Configure the fractional dividers in this domain */
 
  for (fdndx = 0; fdndx < dmn->nfdiv; fdndx++, sub++)
    {
      /* Set fractional divider confiruation but don't enable it yet */

      fdivcfg = lpc31_fdivinit(fdndx + dmn->fdiv1, &sub->fdiv, false);

      /* Enable frac divider only if it has valid settings */

      if (fdivcfg != 0)
        {
          /* Select the fractional dividir for each clock in this
           * sub domain.
           */

          for (clkndx = 0; clkndx <= dmn->nclks; clkndx++)
            {
              /* Does this clock have an ESR register? */
 
              esrndx = lpc31_esrndx((enum lpc31_clockid_e)(clkndx + dmn->clk1));
              if (esrndx != ESRNDX_INVALID)
                {
                  /* Yes.. Check if this clock belongs to this sub-domain */

                  if (sub->clkset & (1 << clkndx))
                    {
                      /* Yes.. configure the clock to use this fractional divider */

                      regaddr = LPC31_CGU_ESR(esrndx);
                      putreg32((fdndx << CGU_ESR_ESRSEL_SHIFT) | CGU_ESR_ESREN, regaddr);
                    }
                 }
             }

          /* Enable the fractional divider */

          regaddr = LPC31_CGU_FDC(fdndx + dmn->fdiv1);
          regval  = getreg32(regaddr);
          regval |= CGU_FDC_RUN;
          putreg32(regval, regaddr);
        }
    }

  if (bcrndx != BCRNDX_INVALID)
    {
      /* Enable the BCR for domain */

      regaddr = LPC31_CGU_BCR(bcrndx);
      putreg32(CGU_BCR_FDRUN, regaddr);
    }

   /* Select input base clock for domain*/

  lpc31_selectfreqin(dmn->dmnid, dmn->finsel);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc31_clkinit
 *
 * Description:
 *   Initialize all clock domains based on board-specific clock configuration data
 *
 ************************************************************************************/

void lpc31_clkinit(const struct lpc31_clkinit_s* cfg)
{
  struct lpc31_domainconfig_s domain;

  /* Reset all clocks and connect them to FFAST */

  lpc31_resetclks();

  /* Initialize Domain0 = SYS_BASE clocks */

  domain.dmnid  = DOMAINID_SYS;
  domain.finsel = cfg->domain0.finsel;
  domain.clk1   = CLKID_SYSBASE_FIRST;
  domain.nclks  = (CLKID_SYSBASE_LAST - CLKID_SYSBASE_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE0_LOW;
  domain.nfdiv  = FRACDIV_BASE0_CNT;
  domain.sub    = cfg->domain0.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain1 = AHB0APB0_BASE clocks */

  domain.dmnid  = DOMAINID_AHB0APB0;
  domain.finsel = cfg->domain1.finsel;
  domain.clk1   = CLKID_AHB0APB0_FIRST;
  domain.nclks  = (CLKID_AHB0APB0_LAST - CLKID_AHB0APB0_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE1_LOW;
  domain.nfdiv  = FRACDIV_BASE1_CNT;
  domain.sub    = cfg->domain1.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain2 = AHB0APB1_BASE clocks */

  domain.dmnid  = DOMAINID_AHB0APB1;
  domain.finsel = cfg->domain2.finsel;
  domain.clk1   = CLKID_AHB0APB1_FIRST;
  domain.nclks  = (CLKID_AHB0APB1_LAST - CLKID_AHB0APB1_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE2_LOW;
  domain.nfdiv  = FRACDIV_BASE2_CNT;
  domain.sub    = cfg->domain2.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain3 = AHB0APB2_BASE clocks */

  domain.dmnid  = DOMAINID_AHB0APB2;
  domain.finsel = cfg->domain3.finsel;
  domain.clk1   = CLKID_AHB0APB2_FIRST;
  domain.nclks  = (CLKID_AHB0APB2_LAST - CLKID_AHB0APB2_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE3_LOW;
  domain.nfdiv  = FRACDIV_BASE3_CNT;
  domain.sub    = cfg->domain3.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain4 = AHB0APB3_BASE clocks */

  domain.dmnid  = DOMAINID_AHB0APB3;
  domain.finsel = cfg->domain4.finsel;
  domain.clk1   = CLKID_AHB0APB3_FIRST;
  domain.nclks  = (CLKID_AHB0APB3_LAST - CLKID_AHB0APB3_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE4_LOW;
  domain.nfdiv  = FRACDIV_BASE4_CNT;
  domain.sub    = cfg->domain4.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain5 = PCM_BASE clocks */

  domain.dmnid  = DOMAINID_PCM;
  domain.finsel = cfg->domain5.finsel;
  domain.clk1   = CLKID_PCM_FIRST;
  domain.nclks  = 1;
  domain.fdiv1  = FRACDIV_BASE5_LOW;
  domain.nfdiv  = FRACDIV_BASE5_CNT;
  domain.sub    = cfg->domain5.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain6 = UART_BASE clocks */

  domain.dmnid  = DOMAINID_UART;
  domain.finsel = cfg->domain6.finsel;
  domain.clk1   = CLKID_UART_FIRST;
  domain.nclks  = 1;
  domain.fdiv1  = FRACDIV_BASE6_LOW;
  domain.nfdiv  = FRACDIV_BASE6_CNT;
  domain.sub    = cfg->domain6.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain7 = CLK1024FS_BASE clocks */

  domain.dmnid  = DOMAINID_CLK1024FS;
  domain.finsel = cfg->domain7.finsel;
  domain.clk1   = CLKID_CLK1024FS_FIRST;
  domain.nclks  = (CLKID_CLK1024FS_LAST - CLKID_CLK1024FS_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE7_LOW;
  domain.nfdiv  = FRACDIV_BASE7_CNT;
  domain.sub    = cfg->domain7.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain8 = I2SRX_BCK0_BASE clocks */

  lpc31_selectfreqin(DOMAINID_BCK0, cfg->domain8.finsel);

  /* Initialize Domain9 = I2SRX_BCK1_BASE clocks */

  lpc31_selectfreqin(DOMAINID_BCK1, cfg->domain9.finsel);

  /* Initialize Domain10 = SPI_BASE clocks */

  domain.dmnid  = DOMAINID_SPI;
  domain.finsel = cfg->domain10.finsel;
  domain.clk1   = CLKID_SPI_FIRST;
  domain.nclks  = (CLKID_SPI_LAST - CLKID_SPI_FIRST) + 1;
  domain.fdiv1  = FRACDIV_BASE10_LOW;
  domain.nfdiv  = FRACDIV_BASE10_CNT;
  domain.sub    = cfg->domain10.sub;
  lpc31_domaininit(&domain);

  /* Initialize Domain11 = SYSCLK_O_BASE clocks */

  lpc31_selectfreqin(DOMAINID_SYSCLKO, cfg->domain11.finsel);

  /* Initialize Dynamic fractional dividers -- to be provided */
}
