/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mcu_version.c
 * 
 * Read out the microcontroller version from the board
 *
 * @author Lorenz Meier <lorenz@px4.io>
 *
 */

#include "mcu_version.h"

#include <nuttx/config.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#include <up_arch.h>

#define DBGMCU_IDCODE	0xE0042000

#define STM32F40x_41x	0x413
#define STM32F42x_43x	0x419

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

#endif



int mcu_version(char* rev, char** revstr)
{
#ifdef CONFIG_ARCH_CHIP_STM32
	uint32_t abc = getreg32(DBGMCU_IDCODE);

	int32_t chip_version = abc & DEVID_MASK;
	enum MCU_REV revid = (abc & REVID_MASK) >> 16;

	switch (chip_version) {
	case STM32F40x_41x:
		*revstr = "STM32F40x";
		break;
	case STM32F42x_43x:
		*revstr = "STM32F42x";
		break;
	default:
		*revstr = "STM32F???";
		break;
	}

	switch (revid) {

		case MCU_REV_STM32F4_REV_A:
			*rev = 'A';
			break;
		case MCU_REV_STM32F4_REV_Z:
			*rev = 'Z';
			break;
		case MCU_REV_STM32F4_REV_Y:
			*rev = 'Y';
			break;
		case MCU_REV_STM32F4_REV_1:
			*rev = '1';
			break;
		case MCU_REV_STM32F4_REV_3:
			*rev = '3';
			break;
		default:
			*rev = '?';
			revid = -1;
			break;
	}

	return revid;
#else
	return -1;
#endif
}
