/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
#pragma once


#include "../../../nxp_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_IMX95

#define PX4_NUMBER_I2C_BUSES        6

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <imx9_lpspi.h>
#include <imx9_lpi2c.h>

/* The i.MX95 exposes the 128 bit ECID as four 32 bit words in the
 *  Fuse Status Block, read by board_identity.c
 *
 *  PX4 uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]    [2]   [3]
 *   bits 127:96  95-64  63-32, 31-00,
 */
#define PX4_CPU_UUID_BYTE_LENGTH                16
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* define common formating across all commands */

#define PX4_CPU_UUID_WORD32_FORMAT              "%08x"
#define PX4_CPU_UUID_WORD32_SEPARATOR           ":"

#define PX4_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            2 /* Middle High significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_L            1 /* Middle Low significant digits */
#define PX4_CPU_UUID_WORD32_UNIQUE_N            0 /* Most significant digits change the least */

/*                                               Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#define PX4_BUS_OFFSET       0                  /* imx9 buses are 1 based, no adjustment needed */

#define px4_spibus_initialize(bus_num_1based)   imx9_lpspibus_initialize(bus_num_1based)

#define px4_i2cbus_initialize(bus_num_1based)   imx9_i2cbus_initialize(bus_num_1based)
#define px4_i2cbus_uninitialize(pdev)           imx9_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             imx9_config_gpio(pinset)
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               imx9_gpio_read(pinset)
#define px4_arch_gpiowrite(pinset, value)       imx9_gpio_write(pinset, value)

/* imx9_gpiosetevent is added at PX4 level */

int imx9_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  imx9_gpiosetevent(pinset,r,f,e,fp,a)

__END_DECLS
