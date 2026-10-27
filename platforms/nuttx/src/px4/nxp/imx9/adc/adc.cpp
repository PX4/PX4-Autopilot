/****************************************************************************
 *
 *   Copyright (C) 2019-2026 PX4 Development Team. All rights reserved.
 *   Copyright (C) 2024 Technology Innovation Institute. All rights reserved.
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
#include <stdint.h>
#include <drivers/drv_adc.h>

#if defined(CONFIG_IMX9_SAR_ADC)
#include <imx9_sar_adc.h>
#endif

int px4_arch_adc_init(uint32_t base_address)
{
	(void)base_address;

#if defined(CONFIG_IMX9_SAR_ADC)
	return imx9_sar_adc_init(ADC_CHANNELS);
#else
	return 0;
#endif
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	(void)base_address;

#if defined(CONFIG_IMX9_SAR_ADC)
	imx9_sar_adc_deinit();
#endif
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	(void)base_address;

#if defined(CONFIG_IMX9_SAR_ADC)
	uint32_t value = 0;

	irqstate_t flags = px4_enter_critical_section();
	int ret = imx9_sar_adc_read_channel(channel, &value);
	px4_leave_critical_section(flags);

	if (ret < 0) {
		return UINT32_MAX;
	}

	return value;
#else
	(void)channel;
	return UINT32_MAX;
#endif
}

float px4_arch_adc_reference_v()
{
#if defined(CONFIG_IMX9_SAR_ADC)
	return 1.8f;
#else
	return 0.0f;
#endif
}

uint32_t px4_arch_adc_temp_sensor_mask()
{
	return 0;
}

uint32_t px4_arch_adc_dn_fullcount()
{
#if defined(CONFIG_IMX9_SAR_ADC)
	return 1 << 12;
#else
	return 0;
#endif
}
