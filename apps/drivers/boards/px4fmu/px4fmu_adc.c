/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4fmu_adc.c
 *
 * Board-specific ADC functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_adc.h"
#include "px4fmu_internal.h"

#define ADC3_NCHANNELS 4

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* The PX4FMU board has four ADC channels: ADC323 IN10-13
 */

/* Identifying number of each ADC channel: Variable Resistor. */

#ifdef CONFIG_STM32_ADC3
static const uint8_t  g_chanlist[ADC3_NCHANNELS] = {10, 11};// , 12, 13}; ADC12 and 13 are used by MPU on v1.5 boards

/* Configurations of pins used byte each ADC channels */
static const uint32_t g_pinlist[ADC3_NCHANNELS]  = {GPIO_ADC3_IN10, GPIO_ADC3_IN11}; // ADC12 and 13 are used by MPU on v1.5 boards, GPIO_ADC3_IN12, GPIO_ADC3_IN13};
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: adc_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

int adc_devinit(void)
{
	static bool initialized = false;
	struct adc_dev_s *adc[ADC3_NCHANNELS];
	int ret;
	int i;

	/* Check if we have already initialized */

	if (!initialized) {
		char name[11];

		for (i = 0; i < ADC3_NCHANNELS; i++) {
			stm32_configgpio(g_pinlist[i]);
		}

		for (i = 0; i < 1; i++) {
			/* Configure the pins as analog inputs for the selected channels */
			//stm32_configgpio(g_pinlist[i]);

			/* Call stm32_adcinitialize() to get an instance of the ADC interface */
			//multiple channels only supported with dma!
			adc[i] = stm32_adcinitialize(3, (g_chanlist), 4);

			if (adc == NULL) {
				adbg("ERROR: Failed to get ADC interface\n");
				return -ENODEV;
			}


			/* Register the ADC driver at "/dev/adc0" */
			sprintf(name, "/dev/adc%d", i);
			ret = adc_register(name, adc[i]);

			if (ret < 0) {
				adbg("adc_register failed for adc %s: %d\n", name, ret);
				return ret;
			}
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}
