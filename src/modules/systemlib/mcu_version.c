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

#include <px4_config.h>
#include <px4_defines.h>

#if defined(CONFIG_ARCH_CHIP_STM32) || defined(CONFIG_ARCH_CHIP_STM32F7)

# define ARCH_CHIP_STM

//STM DocID018909 Rev 8 Sect 38.18 and DocID026670 Rev 5 40.6.1 (MCU device ID code)
# define REVID_MASK    0xFFFF0000
# define DEVID_MASK    0xFFF


# define STM32F74xxx_75xxx  0x449
# define STM32F76xxx_77xxx  0x451
# define STM32F40x_41x      0x413
# define STM32F42x_43x      0x419
# define STM32F103_LD       0x412
# define STM32F103_MD       0x410
# define STM32F103_HD       0x414
# define STM32F103_XLD      0x430
# define STM32F103_CON      0x418

#endif

/** Copy the 96bit MCU Unique ID into the provided pointer */
void mcu_unique_id(uint32_t *uid_96_bit)
{
#if defined(__PX4_NUTTX)

#if defined(ARCH_CHIP_STM)
	uid_96_bit[0] = getreg32(STM32_SYSMEM_UID);
	uid_96_bit[1] = getreg32(STM32_SYSMEM_UID + 4);
	uid_96_bit[2] = getreg32(STM32_SYSMEM_UID + 8);
#elif defined(ARCH_CHIP_KINETIS)
	uid_96_bit[0] = 0xdeadbeef;
	uid_96_bit[1] = 0xbadf00d;
	uid_96_bit[2] = 0xdeadbeef;
#endif
#else
	uid_96_bit[0] = 0;
	uid_96_bit[1] = 1;
	uid_96_bit[2] = 2;
#endif
}

int mcu_version(char *rev, char **revstr)
{
#if defined(_PX4_NUTTX) && defined(ARCH_CHIP_STM)

	uint32_t abc = getreg32(STM32_DEBUGMCU_BASE);

	int32_t chip_version = abc & DEVID_MASK;
	enum MCU_REV revid = (abc & REVID_MASK) >> 16;

	switch (chip_version) {

	case STM32F74xxx_75xxx:
		*revstr = "STM32F74xxx";
		break;

	case STM32F76xxx_77xxx:
		*revstr = "STM32F76xxx";
		break;

	case STM32F42x_43x:
		*revstr = "STM32F42x";
		break;

	case STM32F103_LD:
		*revstr = "STM32F1xx Low";
		break;

	case STM32F103_MD:
		*revstr = "STM32F1xx Med";
		break;

	case STM32F103_HD:
		*revstr = "STM32F1xx Hi";
		break;

	case STM32F103_XLD:
		*revstr = "STM32F1xx XL";
		break;

	case STM32F103_CON:
		*revstr = "STM32F1xx Con";
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
		// todo add rev for 103 - if needed
		*rev = '?';
		revid = -1;
		break;
	}

	return revid;
#else
	return -1;
#endif
}
