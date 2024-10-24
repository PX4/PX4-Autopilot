/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file adc.cpp
 *
 * Driver for the imxrt ADC.
 *
 * This is a low-rate driver, designed for sampling things like voltages
 * and so forth. It avoids the gross complexity of the NuttX ADC driver.
 */

#include <board_config.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>

#include <hardware/imxrt_adc.h>
#include <imxrt_periphclks.h>

typedef uint32_t 	adc_chan_t;
#define ADC_TOTAL_CHANNELS 		16

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* ADC register accessors */

#define REG(base_address, _reg)	_REG((base_address) + (_reg))

#define rVERID(base_address)   REG(base_address, IMXRT_LPADC_VERID_OFFSET)   /* Version ID Register */
#define rPARAM(base_address)   REG(base_address, IMXRT_LPADC_PARAM_OFFSET)   /* Parameter Register */
#define rCTRL(base_address)    REG(base_address, IMXRT_LPADC_CTRL_OFFSET)    /* LPADC Control Register */
#define rSTAT(base_address)    REG(base_address, IMXRT_LPADC_STAT_OFFSET)    /* LPADC Status Register */
#define rIE(base_address)      REG(base_address, IMXRT_LPADC_IE_OFFSET)      /* Interrupt Enable Register */
#define rDE(base_address)      REG(base_address, IMXRT_LPADC_DE_OFFSET)      /* DMA Enable Register */
#define rCFG(base_address)     REG(base_address, IMXRT_LPADC_CFG_OFFSET)     /* LPADC Configuration Register */
#define rPAUSE(base_address)   REG(base_address, IMXRT_LPADC_PAUSE_OFFSET)   /* LPADC Pause Register */
#define rFCTRL(base_address)   REG(base_address, IMXRT_LPADC_FCTRL_OFFSET)   /* LPADC FIFO Control Register */
#define rSWTRIG(base_address)  REG(base_address, IMXRT_LPADC_SWTRIG_OFFSET)  /* Software Trigger Register */
#define rCMDL1(base_address)   REG(base_address, IMXRT_LPADC_CMDL1_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH1(base_address)   REG(base_address, IMXRT_LPADC_CMDH1_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL2(base_address)   REG(base_address, IMXRT_LPADC_CMDL2_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH2(base_address)   REG(base_address, IMXRT_LPADC_CMDH2_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL3(base_address)   REG(base_address, IMXRT_LPADC_CMDL3_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH3(base_address)   REG(base_address, IMXRT_LPADC_CMDH3_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL4(base_address)   REG(base_address, IMXRT_LPADC_CMDL4_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH4(base_address)   REG(base_address, IMXRT_LPADC_CMDH4_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL5(base_address)   REG(base_address, IMXRT_LPADC_CMDL5_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH5(base_address)   REG(base_address, IMXRT_LPADC_CMDH5_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL6(base_address)   REG(base_address, IMXRT_LPADC_CMDL6_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH6(base_address)   REG(base_address, IMXRT_LPADC_CMDH6_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL7(base_address)   REG(base_address, IMXRT_LPADC_CMDL7_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH7(base_address)   REG(base_address, IMXRT_LPADC_CMDH7_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL8(base_address)   REG(base_address, IMXRT_LPADC_CMDL8_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH8(base_address)   REG(base_address, IMXRT_LPADC_CMDH8_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL9(base_address)   REG(base_address, IMXRT_LPADC_CMDL9_OFFSET)   /* LPADC Command Low Buffer Register */
#define rCMDH9(base_address)   REG(base_address, IMXRT_LPADC_CMDH9_OFFSET)   /* LPADC Command High Buffer Register */
#define rCMDL10(base_address)  REG(base_address, IMXRT_LPADC_CMDL10_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH10(base_address)  REG(base_address, IMXRT_LPADC_CMDH10_OFFSET)  /* LPADC Command High Buffer Register */
#define rCMDL11(base_address)  REG(base_address, IMXRT_LPADC_CMDL11_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH11(base_address)  REG(base_address, IMXRT_LPADC_CMDH11_OFFSET)  /* LPADC Command High Buffer Register */
#define rCMDL12(base_address)  REG(base_address, IMXRT_LPADC_CMDL12_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH12(base_address)  REG(base_address, IMXRT_LPADC_CMDH12_OFFSET)  /* LPADC Command High Buffer Register */
#define rCMDL13(base_address)  REG(base_address, IMXRT_LPADC_CMDL13_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH13(base_address)  REG(base_address, IMXRT_LPADC_CMDH13_OFFSET)  /* LPADC Command High Buffer Register */
#define rCMDL14(base_address)  REG(base_address, IMXRT_LPADC_CMDL14_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH14(base_address)  REG(base_address, IMXRT_LPADC_CMDH14_OFFSET)  /* LPADC Command High Buffer Register */
#define rCMDL15(base_address)  REG(base_address, IMXRT_LPADC_CMDL15_OFFSET)  /* LPADC Command Low Buffer Register */
#define rCMDH15(base_address)  REG(base_address, IMXRT_LPADC_CMDH15_OFFSET)  /* LPADC Command High Buffer Register */
#define rRESFIFO(base_address) REG(base_address, IMXRT_LPADC_RESFIFO_OFFSET) /* LPADC Data Result FIFO Register */
#define rTCTRL0(base_address)  REG(base_address, IMXRT_LPADC_TCTRL0_OFFSET)  /* Trigger Control Register */
#define rTCTRL1(base_address)  REG(base_address, IMXRT_LPADC_TCTRL1_OFFSET)  /* Trigger Control Register */
#define rTCTRL2(base_address)  REG(base_address, IMXRT_LPADC_TCTRL2_OFFSET)  /* Trigger Control Register */
#define rTCTRL3(base_address)  REG(base_address, IMXRT_LPADC_TCTRL3_OFFSET)  /* Trigger Control Register */
#define rTCTRL4(base_address)  REG(base_address, IMXRT_LPADC_TCTRL4_OFFSET)  /* Trigger Control Register */
#define rTCTRL5(base_address)  REG(base_address, IMXRT_LPADC_TCTRL5_OFFSET)  /* Trigger Control Register */
#define rTCTRL6(base_address)  REG(base_address, IMXRT_LPADC_TCTRL6_OFFSET)  /* Trigger Control Register */
#define rTCTRL7(base_address)  REG(base_address, IMXRT_LPADC_TCTRL7_OFFSET)  /* Trigger Control Register */
#define rCV1(base_address)     REG(base_address, IMXRT_LPADC_CV1_OFFSET)     /* Compare Value Register */
#define rCV2(base_address)     REG(base_address, IMXRT_LPADC_CV2_OFFSET)     /* Compare Value Register */
#define rCV3(base_address)     REG(base_address, IMXRT_LPADC_CV3_OFFSET)     /* Compare Value Register */
#define rCV4(base_address)     REG(base_address, IMXRT_LPADC_CV4_OFFSET)     /* Compare Value Register */

int px4_arch_adc_init(uint32_t base_address)
{
	static bool once = false;

	if (!once) {

		once = true;

		/* Input is ADCx_CLK_ROOT_SYS_PLL2_CLK with devide by 6.
		 *  528 Mhz / 6 = 88 Mhz.
		 */


		if (base_address == IMXRT_LPADC1_BASE) {
			imxrt_clockall_adc1();

		} else if (base_address == IMXRT_LPADC2_BASE) {
			imxrt_clockall_adc2();
		}


		irqstate_t flags = px4_enter_critical_section();
		rCTRL(base_address) |= IMXRT_LPADC_CTRL_RST;
		rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_RST;
		rCTRL(base_address) |= IMXRT_LPADC_CTRL_RSTFIFO;
		rCFG(base_address) =  IMXRT_LPADC_CFG_REFSEL_REFSEL_0 | IMXRT_LPADC_CFG_PWREN | IMXRT_LPADC_CFG_PWRSEL_PWRSEL_3 |
				      IMXRT_LPADC_CFG_PUDLY(128);
		rCTRL(base_address) = IMXRT_LPADC_CTRL_ADCEN;

		px4_leave_critical_section(flags);

		/* Read ADC1 vtemp_sensor_plus */

		rCMDL1(base_address) = IMXRT_LPADC_CMDL1_ADCH_ADCH_7;

		rCMDH1(base_address) = IMXRT_LPADC_CMDH1_STS_STS_7 | IMXRT_LPADC_CMDH1_AVGS_AVGS_0;
		rTCTRL0(base_address) = IMXRT_LPADC_TCTRL0_TCMD_TCMD_1;
		rSTAT(base_address) = IMXRT_LPADC_STAT_FOF;

		/* kick off a sample and wait for it to complete */
		hrt_abstime now = hrt_absolute_time();

		rSWTRIG(base_address) = IMXRT_LPADC_SWTRIG_SWT0;

		while (!(rSTAT(base_address) & IMXRT_LPADC_STAT_RDY)) {

			/* don't wait for more than 100us, since that means something broke -
			 * should reset here if we see this
			 */

			if ((hrt_absolute_time() - now) > 100) {
				rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_ADCEN;
				return -4;
			}
		}

		int32_t r = (rRESFIFO(base_address) & IMXRT_LPADC_RESFIFO_D_MASK) >> 3;
		UNUSED(r);
		rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_ADCEN;
	} // once

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_ADCEN;

	if (base_address == IMXRT_LPADC1_BASE) {
		imxrt_clockoff_adc1();

	} else if (base_address == IMXRT_LPADC2_BASE) {
		imxrt_clockoff_adc2();
	}
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{

	uint32_t absel = (channel & 1) ? IMXRT_LPADC_CMDL1_ABSEL : 0;
	channel >>= 1;

	irqstate_t flags = px4_enter_critical_section();

	/* clear any previous results */

	rCTRL(base_address) |= IMXRT_LPADC_CTRL_RSTFIFO;

	rCMDL1(base_address) = absel | (channel & IMXRT_LPADC_CMDL1_ADCH_MASK);
	rCMDH1(base_address) = IMXRT_LPADC_CMDH1_STS_STS_7 | IMXRT_LPADC_CMDH1_AVGS_AVGS_0;
	rTCTRL0(base_address) = IMXRT_LPADC_TCTRL0_TCMD_TCMD_1;
	rSTAT(base_address) = IMXRT_LPADC_STAT_FOF;
	rCTRL(base_address) = IMXRT_LPADC_CTRL_ADCEN;

	up_udelay(1);
	rSWTRIG(base_address) = IMXRT_LPADC_SWTRIG_SWT0;

	/* wait for the conversion to complete */
	hrt_abstime now = hrt_absolute_time();

	while (!(rSTAT(base_address) & IMXRT_LPADC_STAT_RDY)) {
		/* don't wait for more than 30us, since that means something broke
		 *  should reset here if we see this
		 */
		if ((hrt_absolute_time() - now) > 30) {
			rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_ADCEN;
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
	}

	/* read the result and clear  COCO0 */

	uint32_t result = (rRESFIFO(base_address) & IMXRT_LPADC_RESFIFO_D_MASK) >> 3;
	rCTRL(base_address) &= ~IMXRT_LPADC_CTRL_ADCEN;
	px4_leave_critical_section(flags);

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount(void)
{
	return 1 << 12; // 12 bit ADC
}
