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
#define FLEXPWMREG(_tmr, _sm, _reg)		_REG16(_tmr + ((_sm) * SM_SPACING), (_reg))

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

#define OUTEN_A_MASK 0x1
#define OUTEN_B_MASK 0x2

#define FREQ

static void flexpwm_led(uint32_t timer, uint32_t sm, uint16_t cvalue, uint32_t gpio_mux, uint32_t pwm_mux,
			uint32_t out_mask)
{
	if (cvalue == 0) {
		//rMCTRL(timer) &= ~MCTRL_RUN(1 << sm);
		px4_arch_configgpio(gpio_mux);

	} else {
		rMCTRL(timer) |= (1 << (sm + MCTRL_CLDOK_SHIFT));
		rVAL1(timer, sm) = (FLEXPWM_FREQ / LED_PWM_FREQ) - 1;

		if (out_mask & OUTEN_A_MASK) {
			rVAL3(timer, sm) = (FLEXPWM_FREQ / LED_PWM_FREQ) - (cvalue);

		} else if (out_mask & OUTEN_B_MASK) {
			rVAL5(timer, sm) = (FLEXPWM_FREQ / LED_PWM_FREQ) - (cvalue);
		}

		rMCTRL(timer) |= MCTRL_LDOK(1 << sm) | MCTRL_RUN(1 << sm);
		px4_arch_configgpio(pwm_mux);
	}
}

int
led_pwm_servo_set(unsigned channel, uint8_t cvalue)
{
	if (channel == 0) {
		flexpwm_led(IMXRT_FLEXPWM3_BASE, 2, cvalue, GPIO_nLED_RED, PWM_LED_RED, OUTEN_A_MASK);

	} else if (channel == 1) {
		flexpwm_led(IMXRT_FLEXPWM1_BASE, 3, cvalue, GPIO_nLED_GREEN, PWM_LED_GREEN, OUTEN_A_MASK);

	} else if (channel == 2) {
		flexpwm_led(IMXRT_FLEXPWM3_BASE, 2, cvalue * 3, GPIO_nLED_BLUE, PWM_LED_BLUE, OUTEN_B_MASK);
	}

	return 0;
}

void flexpwm_init(uint32_t timer, uint32_t sm, uint32_t out_mask)
{
	/* Clear all Faults */
	rFSTS0(timer) = FSTS_FFLAG_MASK;
	rMCTRL(timer) |= (1 << (sm + MCTRL_CLDOK_SHIFT));

	rCTRL2(timer, sm) = SMCTRL2_CLK_SEL_EXT_CLK | SMCTRL2_DBGEN | SMCTRL2_INDEP;
	rCTRL(timer, sm)  = SMCTRL_PRSC_DIV16 | SMCTRL_FULL;
	/* Edge aligned at 0 */
	rINIT(timer, sm) = 0;
	rVAL0(timer, sm) = 0;
	rVAL2(timer, sm) = 0;
	rVAL4(timer, sm) = 0;
	rFFILT0(timer) &= ~FFILT_FILT_PER_MASK;
	rDISMAP0(timer, sm) = 0xf000;
	rDISMAP1(timer, sm) = 0xf000;


	if (out_mask & OUTEN_A_MASK) {
		rOUTEN(timer) |= OUTEN_PWMA_EN(1 << sm);
	}

	if (out_mask & OUTEN_B_MASK) {
		rOUTEN(timer) |= OUTEN_PWMB_EN(1 << sm);
	}

	rDTSRCSEL(timer) = 0;
	rMCTRL(timer) |= MCTRL_LDOK(1 << sm);
}

int led_pwm_servo_init()
{
	/* PWM_LED_GREEN - FLEXPWM2_PWMB03 */
	imxrt_clockall_pwm1();
	imxrt_clockall_pwm3();

	flexpwm_init(IMXRT_FLEXPWM3_BASE, 2, OUTEN_A_MASK | OUTEN_B_MASK); // GPIO_FLEXPWM3_PWMA02_1
	flexpwm_init(IMXRT_FLEXPWM1_BASE, 3, OUTEN_A_MASK); // GPIO_FLEXPWM1_PWMA03_2



	return OK;
}
