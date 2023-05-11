/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

#include <nuttx/analog/adc.h>

#include <hardware/s32k3xx_adc.h>
#include <hardware/s32k344_pinmux.h>

int px4_arch_adc_init(uint32_t base_address)
{
	uint32_t regval;

	/* Configure and perform calibration */
	putreg32(ADC_MCR_ADCLKSEL_DIV4, S32K3XX_ADC2_MCR);

	regval = getreg32(S32K3XX_ADC2_AMSIO);
	regval |= ADC_AMSIO_HSEN_MASK;
	putreg32(regval, S32K3XX_ADC2_AMSIO);

	regval = getreg32(S32K3XX_ADC2_CAL2);
	regval &= ~ADC_CAL2_ENX;
	putreg32(regval, S32K3XX_ADC2_CAL2);

	regval = getreg32(S32K3XX_ADC2_CALBISTREG);
	regval &= ~(ADC_CALBISTREG_TEST_EN | ADC_CALBISTREG_AVG_EN | ADC_CALBISTREG_NR_SMPL_MASK |
		    ADC_CALBISTREG_CALSTFUL | ADC_CALBISTREG_TSAMP_MASK | ADC_CALBISTREG_RESN_MASK);
	regval |= ADC_CALBISTREG_TEST_EN | ADC_CALBISTREG_AVG_EN | ADC_CALBISTREG_NR_SMPL_4SMPL |
		  ADC_CALBISTREG_CALSTFUL | ADC_CALBISTREG_RESN_14BIT;
	putreg32(regval, S32K3XX_ADC2_CALBISTREG);

	while (getreg32(S32K3XX_ADC2_CALBISTREG) & ADC_CALBISTREG_C_T_BUSY) {};

	putreg32(ADC_MCR_PWDN, S32K3XX_ADC2_MCR);

	putreg32(22, S32K3XX_ADC2_CTR0);

	putreg32(22, S32K3XX_ADC2_CTR1);

	putreg32(0, S32K3XX_ADC2_DMAE);

	putreg32(ADC_MCR_ADCLKSEL_DIV4 | ADC_MCR_AVGS_32CONV | ADC_MCR_AVGEN | ADC_MCR_BCTU_MODE | ADC_MCR_MODE,
		 S32K3XX_ADC2_MCR);

	putreg32(0x10, S32K3XX_ADC2_NCMR0);

	putreg32(0x10, S32K3XX_ADC2_NCMR1);

	regval = getreg32(S32K3XX_ADC2_MCR);

	regval |= ADC_MCR_NSTART;

	putreg32(regval, S32K3XX_ADC2_MCR);

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	uint32_t result = 0;

	if (channel == 0) {
		result = getreg32(S32K3XX_ADC2_PCDR4);

		if ((result & ADC_PCDR_VALID) == ADC_PCDR_VALID) {
			result = result & 0xFFFF;

		} else {
			result = 0;
		}
	}

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0; // No temp sensor
}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 15; // 15 bit conversion data
}
