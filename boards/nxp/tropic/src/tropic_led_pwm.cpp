/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
* @file tropic_led_pwm.cpp
*/

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include <chip.h>
#include "imxrt_xbar.h"
#include "imxrt_periphclks.h"
#include "hardware/imxrt_tmr.h"
#include "hardware/imxrt_flexpwm.h"

#define LED_PWM_FREQ    1000
#define FLEXPWM_FREQ    1000000
#define QTMR_FREQ       (144000000/128)

#define SM_SPACING (IMXRT_FLEXPWM_SM1CNT_OFFSET-IMXRT_FLEXPWM_SM0CNT_OFFSET)

#define FLEXPWM_TIMER_BASE  IMXRT_FLEXPWM2_BASE

/* Register accessors */
#define _REG(_addr)	(*(volatile uint16_t *)(_addr))
#define _REG16(_base, _reg)	(*(volatile uint16_t *)(_base + _reg))
#define FLEXPWMREG(_tmr, _sm, _reg)		_REG16(FLEXPWM_TIMER_BASE + ((_sm) * SM_SPACING), (_reg))
#define QTMRREG(_reg, _chn)                     _REG16(IMXRT_QTIMER4_BASE + ((_chn) * IMXRT_TMR_CHANNEL_SPACING),(_reg))

/* FlexPWM Registers for LED_G */
#define rINIT(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* Initial Count Register */
#define rCTRL(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* Control Register */
#define rCTRL2(_tim, _sm)      FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* Control 2 Register */
#define rFSTS0(_tim)           FLEXPWMREG(_tim,  0, IMXRT_FLEXPWM_FSTS0_OFFSET)         /* Fault Status Register */
#define rVAL0(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* Value Register 0 */
#define rVAL1(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* Value Register 1 */
#define rVAL2(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* Value Register 2 */
#define rVAL3(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* Value Register 3 */
#define rVAL4(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* Value Register 4 */
#define rVAL5(_tim, _sm)       FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* Value Register 5 */
#define rFFILT0(_tim)          FLEXPWMREG(_tim,  0, IMXRT_FLEXPWM_FFILT0_OFFSET)        /* Fault Filter Register */
#define rDISMAP0(_tim, _sm)    FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* Fault Disable Mapping Register 0 */
#define rDISMAP1(_tim, _sm)    FLEXPWMREG(_tim, _sm,IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* Fault Disable Mapping Register 1 */
#define rOUTEN(_tim)           FLEXPWMREG(_tim,  0, IMXRT_FLEXPWM_OUTEN_OFFSET)         /* Output Enable Register */
#define rDTSRCSEL(_tim)        FLEXPWMREG(_tim,  0, IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* PWM Source Select Register */
#define rMCTRL(_tim)           FLEXPWMREG(_tim,  0, IMXRT_FLEXPWM_MCTRL_OFFSET)         /* Master Control Register */

#define QTMR_rCOMP1(_chn)       QTMRREG(IMXRT_TMR_COMP1_OFFSET, _chn)
#define QTMR_rCOMP2(_chn)       QTMRREG(IMXRT_TMR_COMP2_OFFSET, _chn)
#define QTMR_rCAPT(_chn)       QTMRREG(IMXRT_TMR_CAPT_OFFSET, _chn)
#define QTMR_rLOAD(_chn)       QTMRREG(IMXRT_TMR_LOAD_OFFSET, _chn)
#define QTMR_rHOLD(_chn)       QTMRREG(IMXRT_TMR_HOLD_OFFSET, _chn)
#define QTMR_rCNTR(_chn)       QTMRREG(IMXRT_TMR_CNTR_OFFSET, _chn)
#define QTMR_rCTRL(_chn)       QTMRREG(IMXRT_TMR_CTRL_OFFSET, _chn)
#define QTMR_rSCTRL(_chn)       QTMRREG(IMXRT_TMR_SCTRL_OFFSET, _chn)
#define QTMR_rCMPLD1(_chn)       QTMRREG(IMXRT_TMR_CMPLD1_OFFSET, _chn)
#define QTMR_rCMPLD2(_chn)       QTMRREG(IMXRT_TMR_CMPLD2_OFFSET, _chn)
#define QTMR_rCSCTRL(_chn)       QTMRREG(IMXRT_TMR_CSCTRL_OFFSET, _chn)
#define QTMR_rFILT(_chn)       QTMRREG(IMXRT_TMR_FILT_OFFSET, _chn)
#define QTMR_rDMA(_chn)       QTMRREG(IMXRT_TMR_DMA_OFFSET, _chn)
#define QTMR_rENBL(_chn)       QTMRREG(IMXRT_TMR_ENBL_OFFSET, _chn)

#define FLEXPWM_TIMER 2
#define FLEXPWM_SM 3

#define FREQ

static void flexpwm_led_green(uint16_t cvalue)
{
	if (cvalue == 0) {
		rMCTRL(FLEXPWM_TIMER) &= ~MCTRL_RUN(1 << FLEXPWM_SM);
		px4_arch_configgpio(GPIO_nLED_GREEN);

	} else {
		rMCTRL(FLEXPWM_TIMER) |= (1 << (FLEXPWM_SM + MCTRL_CLDOK_SHIFT));
		rVAL1(FLEXPWM_TIMER, FLEXPWM_SM) = (FLEXPWM_FREQ / LED_PWM_FREQ) - 1;
		rVAL5(FLEXPWM_TIMER, FLEXPWM_SM) = (FLEXPWM_FREQ / LED_PWM_FREQ) - (cvalue * 3);
		rMCTRL(FLEXPWM_TIMER) |= MCTRL_LDOK(1 << FLEXPWM_SM) | MCTRL_RUN(1 << FLEXPWM_SM);
		px4_arch_configgpio(PWM_LED_GREEN);
	}
}

static void init_qtimer(unsigned channel)
{
	QTMR_rCNTR(channel) = 0; /* Reset counter */
	QTMR_rSCTRL(channel) = (TMR_SCTRL_OEN | TMR_SCTRL_FORCE); /* Enable output */
	QTMR_rCSCTRL(channel) = (TMR_CSCTRL_CL1_COMP1);
	QTMR_rCOMP1(channel) = 0x1; /* Store initial value to the duty-compare register */
	QTMR_rCMPLD1(channel) = 0x1; /* Store initial value to the duty-compare register */
	QTMR_rCOMP2(channel) = 0x1; /* Store initial value to the duty-compare register */
	QTMR_rCMPLD2(channel) = 0x1; /* Store initial value to the duty-compare register */
	QTMR_rCTRL(channel) = (TMR_CTRL_PCS_DIV32 | TMR_CTRL_OUTMODE_TOG_ALT | TMR_CTRL_DIR |
			       TMR_CTRL_CM_MODE1); /* Run counter */
}

int
led_pwm_servo_set(unsigned channel, uint8_t cvalue)
{
	if (channel == 2) {

		if (cvalue == 0) {
			px4_arch_configgpio(GPIO_nLED_RED);

		} else {
			px4_arch_configgpio(PWM_LED_RED);
			QTMR_rCMPLD1(0) = (uint16_t)cvalue * 256;
		}

	} else if (channel == 1) {
		flexpwm_led_green(cvalue);

	} else if (channel == 0) {

		if (cvalue == 0) {
			px4_arch_configgpio(GPIO_nLED_BLUE);

		} else {
			px4_arch_configgpio(PWM_LED_BLUE);
			QTMR_rCMPLD1(1) = (uint16_t)cvalue * 256;
		}
	}

	return 0;
}

int led_pwm_servo_init()
{
	/* PWM_LED_GREEN - FLEXPWM2_PWMB03 */
	imxrt_clockall_pwm2();


	/* PWM_LED_RED PWM_LED_BLUE - QTIMER4 */
	imxrt_clockall_timer4();

	/* Clear all Faults */
	rFSTS0(FLEXPWM_TIMER) = FSTS_FFLAG_MASK;
	rMCTRL(FLEXPWM_TIMER) |= (1 << (FLEXPWM_SM + MCTRL_CLDOK_SHIFT));

	rCTRL2(FLEXPWM_TIMER, FLEXPWM_SM) = SMCTRL2_CLK_SEL_EXT_CLK | SMCTRL2_DBGEN | SMCTRL2_INDEP;
	rCTRL(FLEXPWM_TIMER, FLEXPWM_SM)  = SMCTRL_PRSC_DIV16 | SMCTRL_FULL;
	/* Edge aligned at 0 */
	rINIT(FLEXPWM_TIMER, FLEXPWM_SM) = 0;
	rVAL0(FLEXPWM_TIMER, FLEXPWM_SM) = 0;
	rVAL2(FLEXPWM_TIMER, FLEXPWM_SM) = 0;
	rVAL4(FLEXPWM_TIMER, FLEXPWM_SM) = 0;
	rFFILT0(FLEXPWM_TIMER) &= ~FFILT_FILT_PER_MASK;
	rDISMAP0(FLEXPWM_TIMER, FLEXPWM_SM) = 0xf000;
	rDISMAP1(FLEXPWM_TIMER, FLEXPWM_SM) = 0xf000;

	rOUTEN(FLEXPWM_TIMER) |= OUTEN_PWMB_EN(1 << FLEXPWM_SM);

	rDTSRCSEL(FLEXPWM_TIMER) = 0;
	rMCTRL(FLEXPWM_TIMER) |= MCTRL_LDOK(1 << FLEXPWM_SM);

	/* QTMR */
	init_qtimer(0);
	init_qtimer(1);

	/* Red - QTIMER4_TMR0 */
	imxrt_xbar_connect(IMXRT_XBARA1_OUT_IOMUX_XBAR_IO16_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER4_TMR0_OUT);

	/* Blue - QTIMER4_TMR1 */
	imxrt_xbar_connect(IMXRT_XBARA1_OUT_IOMUX_XBAR_IO17_SEL_OFFSET, IMXRT_XBARA1_IN_QTIMER4_TMR1_OUT);

	/* Set XBAR 16 and 17 as an output */
	putreg32((1 << 28) | (1 << 29), IMXRT_IOMUXC_GPR_GPR6);

	return OK;
}
