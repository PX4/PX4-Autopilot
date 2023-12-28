/****************************************************************************
 *
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

#include <px4_arch/tvs.h>

#include "mpfs.h"

/*
 * TVS Readout Register base address is 0x4B00_D000.
 *
 * Register     | Offset | Width  | Description
 *              |        | (bits) |
 * ------------------------------------------------------------------------------------
 * Voltage 1.0  | 0x0    | 32     | 1.0/1.05V Core voltage measurement value. Channel 0
 * Voltage 1.8V | 0x4    | 32     | 1.8V Core voltage measurement value. Channel 1
 * Voltage 2.5V | 0x8    | 32     | 2.5V Core voltage measurement value, Channel 2
 * Temperature  | 0xC    | 32     | Temperature measurement value. Channel 3
*/

#define MPFS_TVS_VOLT_1_0	0x4B00D000 /* MPFS TVS base address */
#define MPFS_TVS_VOLT_1_8	0x4B00D004
#define MPFS_TVS_VOLT_2_5	0x4B00D008
#define MPFS_TVS_TEMP		0x4B00D00C

int tvs_get_voltage(int idx, float *voltage)
{
	uint32_t reg = 0;
	uint16_t volt = 0;

	if (!voltage) {
		return -EINVAL;
	}

	switch (idx) {
	case 0:
		reg = getreg32(MPFS_TVS_VOLT_1_0);
		break;

	case 1:
		reg = getreg32(MPFS_TVS_VOLT_1_8);
		break;

	case 2:
		reg = getreg32(MPFS_TVS_VOLT_2_5);
		break;

	default:
		return -ENODEV;
	}

	/* MPFS TVS Voltage Register:
	 *   [31:16]  Unused bits
	 *   15       Signed bit
	 *   [14:3]   Integer value of the voltage (mV)
	 *   [2:0]    Fractional value of the voltage
	 */

	volt = reg & 0x7FFF;
	*voltage = ((float)volt) / 8;

	/* signed bit */
	if (reg & (1 << 15)) {
		*voltage *= -1;
	}

	return 0;
}

int tvs_get_temperature(float *temperature)
{
	uint16_t reg = 0;

	if (!temperature) {
		return -EINVAL;
	}

	/* MPFS TVS Temperature Register:
	 *   [31:16]  Unused bits
	 *   15       Reserved
	 *   [14:4]   Integer value of the temperature (K)
	 *   [3:0]    Fractional value of the temperature
	 */

	reg = getreg32(MPFS_TVS_TEMP) & 0x7FFF;
	*temperature = ((float)reg) / 16;
	*temperature -= 273.15f; /* Kelvin -> Celcius */

	return 0;
}
