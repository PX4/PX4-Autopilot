/****************************************************************************
 * configs/same70-xplained/src/sam_sdram.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Most of this file derives from Atmel sample code for the SAME70-XPLD
 * board.  That sample code has licensing that is compatible with the NuttX
 * modified BSD license:
 *
 *   Copyright (c) 2012, Atmel Corporation
 *   All rights reserved.
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
 * 3. Neither the name NuttX nor Atmel nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
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

#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "sam_periphclks.h"
#include "chip/sam_memorymap.h"
#include "chip/sam_pinmap.h"
#include "chip/sam_pmc.h"
#include "chip/sam_matrix.h"
#include "chip/sam_sdramc.h"

#include "board_config.h"

#ifdef CONFIG_SAMV7_SDRAMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SDRAM_BA0 (1 << 20)
#define SDRAM_BA1 (1 << 21)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures the on-board SDRAM.  SAME70 Xplained features one external
 *   IS42S16100E-7BLI, 512Kx16x2, 10ns, SDRAM. SDRAM0 is connected to chip
 *   select NCS1.
 *
 *  Input Parameters:
 *     None
 *
 *  Assumptions:
 *    This test runs early in initialization before I- and D-caches are
 *    enabled.
 *
 *    NOTE: Since the delay loop is calibrate with caches in enabled, the
 *    calls to up_udelay() are wrong ty orders of magnitude.
 *
 ****************************************************************************/

void sam_sdram_config(void)
{
	volatile uint8_t *psdram = (uint8_t *)SAM_SDRAMCS_BASE;
	uint32_t regval;
	int i;

	/* Configure SDRAM pins */

	sam_configgpio(GPIO_SMC_D0);
	sam_configgpio(GPIO_SMC_D1);
	sam_configgpio(GPIO_SMC_D2);
	sam_configgpio(GPIO_SMC_D3);
	sam_configgpio(GPIO_SMC_D4);
	sam_configgpio(GPIO_SMC_D5);
	sam_configgpio(GPIO_SMC_D6);
	sam_configgpio(GPIO_SMC_D7);
	sam_configgpio(GPIO_SMC_D8);
	sam_configgpio(GPIO_SMC_D9);
	sam_configgpio(GPIO_SMC_D10);
	sam_configgpio(GPIO_SMC_D11);
	sam_configgpio(GPIO_SMC_D12);
	sam_configgpio(GPIO_SMC_D13);
	sam_configgpio(GPIO_SMC_D14);
	sam_configgpio(GPIO_SMC_D15);

	/*   SAME70          SDRAM
	 *   --------------- -----------
	 *   PC20 A2         A0
	 *   PC21 A3         A1
	 *   PC22 A4         A2
	 *   PC23 A5         A3
	 *   PC24 A6         A4
	 *   PC25 A7         A5
	 *   PC26 A8         A6
	 *   PC27 A9         A7
	 *   PC28 A10        A8
	 *   PC29 A11        A9
	 *   PD13 SDA10      A10
	 *   PA20 BA0        A11
	 *   PD17 CAS        nCAS
	 *   PD14 SDCKE      CKE
	 *   PD23 SDCK       CLK
	 *   PC15 SDCS       nCS
	 *   PC18 A0/NBS0    LDQM
	 *   PD16 RAS        nRAS
	 *   PD15 NWR1/NBS1  UDQM
	 *   PD29 SDWE       nWE
	 */

	sam_configgpio(GPIO_SMC_A2);        /* PC20 A2        -> A0 */
	sam_configgpio(GPIO_SMC_A3);        /* PC21 A3        -> A1 */
	sam_configgpio(GPIO_SMC_A4);        /* PC22 A4        -> A2 */
	sam_configgpio(GPIO_SMC_A5);        /* PC23 A5        -> A3 */
	sam_configgpio(GPIO_SMC_A6);        /* PC24 A6        -> A4 */
	sam_configgpio(GPIO_SMC_A7);        /* PC25 A7        -> A5 */
	sam_configgpio(GPIO_SMC_A8);        /* PC26 A8        -> A6 */
	sam_configgpio(GPIO_SMC_A9);        /* PC27 A9        -> A7 */
	sam_configgpio(GPIO_SMC_A10);       /* PC28 A10       -> A8 */
	sam_configgpio(GPIO_SMC_A11);       /* PC29 A11       -> A9 */
	sam_configgpio(GPIO_SDRAMC_A10_2);  /* PD13 SDA10     -> A10 */
	sam_configgpio(GPIO_SDRAMC_BA0);    /* PA20 BA0       -> A11 */

	sam_configgpio(GPIO_SDRAMC_CKE);    /* PD14 SDCKE     -> CKE */
	sam_configgpio(GPIO_SDRAMC_CK);     /* PD23 SDCK      -> CLK */
	sam_configgpio(GPIO_SDRAMC_CS_1);   /* PC15 SDCS      -> nCS */
	sam_configgpio(GPIO_SDRAMC_RAS);    /* PD16 RAS       -> nRAS */
	sam_configgpio(GPIO_SDRAMC_CAS);    /* PD17 CAS       -> nCAS */
	sam_configgpio(GPIO_SDRAMC_WE);     /* PD29 SDWE      -> nWE */
	sam_configgpio(GPIO_SMC_NBS0);      /* PC18 A0/NBS0   -> LDQM */
	sam_configgpio(GPIO_SMC_NBS1);      /* PD15 NWR1/NBS1 -> UDQM */

	/* Enable the SDRAMC peripheral */

	sam_sdramc_enableclk();

	regval  = getreg32(SAM_MATRIX_CCFG_SMCNFCS);
	regval |= MATRIX_CCFG_SMCNFCS_SDRAMEN;
	putreg32(regval, SAM_MATRIX_CCFG_SMCNFCS);

	/* 1. SDRAM features must be set in the configuration register:
	 *    asynchronous timings (TRC, TRAS, etc.), number of columns, rows, CAS
	 *    latency, and the data bus width.
	 *
	 *    SDRAMC_CR_NC_COL8          8 column bits
	 *    SDRAMC_CR_NR_ROW11         1 row bits
	 *    SDRAMC_CR_NB_BANK2         2 banks
	 *    SDRAMC_CR_CAS_LATENCY3     3 cycle CAS latency
	 *    SDRAMC_CR_DBW             16 bit
	 *    SDRAMC_CR_TWR(4)           4 cycle write recovery delay
	 *    SDRAMC_CR_TRCTRFC(11)     63 ns min
	 *    SDRAMC_CR_TRP(5)          21 ns min Command period (PRE to ACT)
	 *    SDRAMC_CR_TRCD(5)         21 ns min Active Command to read/Write Command delay time
	 *    SDRAMC_CR_TRAS(8)         42 ns min Command period (ACT to PRE)
	 *    SDRAMC_CR_TXSR(13)        70 ns min Exit self-refresh to active time
	 */

	regval = SDRAMC_CR_NC_COL8       |  /* 8 column bits */
		 SDRAMC_CR_NR_ROW11      |  /* 11 row bits */
		 SDRAMC_CR_NB_BANK2      |  /* 2 banks */
		 SDRAMC_CR_CAS_LATENCY3  |  /* 3 cycle CAS latency */
		 SDRAMC_CR_DBW           |  /* 16 bit */
		 SDRAMC_CR_TWR(4)        |  /* 4 cycle write recovery delay */
		 SDRAMC_CR_TRCTRFC(11)   |  /* 63 ns min */
		 SDRAMC_CR_TRP(5)        |  /* 21 ns min Command period (PRE to ACT) */
		 SDRAMC_CR_TRCD(5)       |  /* 21 ns min Active Command to read/Write Command delay time */
		 SDRAMC_CR_TRAS(8)       |  /* 42 ns min Command period (ACT to PRE) */
		 SDRAMC_CR_TXSR(13);        /* 70 ns min Exit self-refresh to active time */

	putreg32(regval, SAM_SDRAMC_CR);

	/* 2. For mobile SDRAM, temperature-compensated self refresh (TCSR), drive
	 *    strength (DS) and partial array self refresh (PASR) must be set in
	 *    the Low Power Register.
	 */

	putreg32(0, SAM_SDRAMC_LPR);

	/* 3. The SDRAM memory type must be set in the Memory Device Register.*/

	putreg32(SDRAMC_MDR_SDRAM, SAM_SDRAMC_MDR);

	/* 4. A minimum pause of 200 usec is provided to precede any signal toggle.*/

	up_udelay(200);

	/* 5. A NOP command is issued to the SDRAM devices. The application must
	 *    set Mode to 1 in the Mode Register and perform a write access to any
	 *    SDRAM address.
	 */

	putreg32(SDRAMC_MR_MODE_NOP, SAM_SDRAMC_MR);
	*psdram = 0;
	up_udelay(200);

	/* 6. An All Banks Precharge command is issued to the SDRAM devices. The
	 *    application must set Mode to 2 in the Mode Register and perform a
	 *    write access to any SDRAM address.
	 */

	putreg32(SDRAMC_MR_MODE_PRECHARGE, SAM_SDRAMC_MR);
	*psdram = 0;
	up_udelay(200);

	/* 7. Eight auto-refresh (CBR) cycles are provided. The application must
	 *    set the Mode to 4 in the Mode Register and perform a write access to
	 *    any SDRAM location eight times.
	 */

	for (i = 0 ; i < 8; i++) {
		putreg32(SDRAMC_MR_MODE_AUTOREFRESH, SAM_SDRAMC_MR);
		*psdram = 0;
	}

	up_udelay(200);

	/* 8. A Mode Register set (MRS) cycle is issued to program the parameters
	 *    of the SDRAM devices, in particular CAS latency and burst length.
	 *    The application must set Mode to 3 in the Mode Register and perform
	 *    a write access to the SDRAM. The write address must be chosen so
	 *    that BA[1:0] are set to 0. For example, with a 16-bit 128 MB SDRAM
	 *    (12 rows, 9 columns, 4 banks) bank address, the SDRAM write access
	 *    should be done at the address 0x70000000.
	  */

	putreg32(SDRAMC_MR_MODE_LOADMODE, SAM_SDRAMC_MR);
	*psdram = 0;
	up_udelay(200);

	/* 9. For mobile SDRAM initialization, an Extended Mode Register set
	 *    (EMRS) cycle is issued to program the SDRAM parameters (TCSR, PASR,
	 *     DS). The application must set Mode to 5 in the Mode Register and
	 *     perform a write access to the SDRAM. The write address must be
	 *     chosen so that BA[1] or BA[0] are set to 1.
	 *
	 *     For example, with a 16-bit 128 MB SDRAM, (12 rows, 9 columns, 4
	 *     banks) bank address the SDRAM write access should be done at the
	 *     address 0x70800000 or 0x70400000.
	 */

	//putreg32(SDRAMC_MR_MODE_EXTLOADMODE, SDRAMC_MR_MODE_EXT_LOAD_MODEREG);
	// *((uint8_t *)(psdram + SDRAM_BA0)) = 0;

	/* 10. The application must go into Normal Mode, setting Mode to 0 in the
	 *     Mode Register and performing a write access at any location in the
	 *     SDRAM.
	 */

	putreg32(SDRAMC_MR_MODE_NORMAL, SAM_SDRAMC_MR);
	*psdram = 0;
	up_udelay(200);

	/* 11. Write the refresh rate into the count field in the SDRAMC Refresh
	 *     Timer register. (Refresh rate = delay between refresh cycles). The
	 *     SDRAM device requires a refresh every 15.625 usec or 7.81 usec. With
	 *     a 100 MHz frequency, the Refresh Timer Counter Register must be set
	 *     with the value 1562(15.625 usec x 100 MHz) or 781(7.81 usec x 100
	 *     MHz).
	 *
	 * For IS42S16100E, 2048 refresh cycle every 32ms, every 15.625 usec
	 */

	regval = (32 * (BOARD_MCK_FREQUENCY / 1000)) / 2048 ;
	putreg32(regval, SAM_SDRAMC_TR);

	regval  = getreg32(SAM_SDRAMC_CFR1);
	regval |= SDRAMC_CFR1_UNAL;
	putreg32(regval, SAM_SDRAMC_CFR1);

	/* After initialization, the SDRAM devices are fully functional. */
}

#endif /* CONFIG_SAMV7_SDRAMC */
