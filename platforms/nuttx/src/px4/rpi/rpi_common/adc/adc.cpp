/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

// #include <rp2040_adc.h> Nuttx doesn't have this file in arch yet.
#include <rp2040_gpio.h>

/*
 * Register accessors.
 * For now, no reason not to just use ADC1.
 */
#define REG(base, _reg) (*(volatile uint32_t *)((base) + (_reg)))

#define rCS(base)	REG((base), 0x00)	// ADC Control and Status
#define rRESULT(base)	REG((base), 0x04)	// Result of most recent ADC conversion
#define rFCS(base)	REG((base), 0x08)	// FIFO control and status
#define rFIFO(base)	REG((base), 0x0c)	// Conversion result FIFO
#define rDIV(base)	REG((base), 0x10)	// Clock divider
#define rINTR(base)	REG((base), 0x14)	// Raw Interrupts
#define rINTE(base)	REG((base), 0x18)	// Interrupt Enable
#define rINTF(base)	REG((base), 0x1c)	// Interrupt Force
#define rINTS(base)	REG((base), 0x20)	// Interrupt status after masking & forcing

int px4_arch_adc_init(uint32_t base_address)
{
	/* Perform ADC init once per ADC */

	static uint32_t once[SYSTEM_ADC_COUNT] {};

	uint32_t *free = nullptr;

	for (uint32_t i = 0; i < SYSTEM_ADC_COUNT; i++) {
		if (once[i] == base_address) {

			/* This one was done already */

			return OK;
		}

		/* Use first free slot */

		if (free == nullptr && once[i] == 0) {
			free = &once[i];
		}
	}

	if (free == nullptr) {

		/* ADC misconfigured SYSTEM_ADC_COUNT too small */;

		PANIC();
	}

	*free = base_address;

	// Assuming that the ADC gpio is configured correctly,
	// all that is left to do is divide 48MHz clock if
	// necessary and then enable the ADC. (One reading
	// requires about 100 clocks.) Also enable the temp
	// sensor channel.

	// Divide incoming 48MHz clock
	// (Trigger adc once per n+1 cycles)
	// So, n >= 96. Because, 1 sample = 96 clocks
	rDIV(base_address) = 0 | (0 << 8); // 8-bit fraction value |  16-bit int value => n = int + frac/256

	// Enable temperature sensor and enable ADC
	rCS(base_address) = 1 | (1 << 1);
	px4_usleep(10);

	// Select temperature channel and kick off a sample and wait for it to complete
	rCS(base_address) &= ~(0b111 << 12);	// Clear AINSEL
	rCS(base_address) |= PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL << 12;	// Choose temperature channel
	hrt_abstime now = hrt_absolute_time();
	rCS(base_address) |= 1 << 2;	// Start a single conversion

	while (!(rCS(base_address) & (1 << 8))) {	// Check if the sample is ready

		/* don't wait for more than 500us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 500) {
			return -1;
		}
	}

	/* Read out result */
	(void) rRESULT(base_address);

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// Disable ADC
	rCS(base_address) = 0;
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* run a single conversion right now - should take about 96 cycles (a few microseconds) max */
	rCS(base_address) &= ~(0b111 << 12);	// Clear AINSEL
	rCS(base_address) |= channel << 12;	// Choose temperature channel
	rCS(base_address) |= 1 << 2;	// Start a single conversion

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rCS(base_address) & (1 << 8))) {	// Check if the sample is ready

		/* don't wait for more than 50us, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > 50) {
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
	}

	/* read the result and clear EOC */
	uint32_t result = rRESULT(base_address);

	px4_leave_critical_section(flags);

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{

	return 1 << PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL;

}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
