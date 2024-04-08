/****************************************************************************
 *
 *   Copyright (c) 2024 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_NXPIMX93
#define PX4_NUMBER_I2C_BUSES        5

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <chip.h>
#include "imx9_gpio.h"

/*
 */
#define PX4_CPU_UUID_BYTE_LENGTH                8
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

#define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
#define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Most significant digits change the least */

#define PX4_BUS_OFFSET       1                  /* imx9 buses are 0 based so adjustment needed */

#define px4_savepanic(fileno, context, length)  imx9_dump_savepanic(fileno, context, length)

#define px4_spibus_initialize(bus_num_1based)   NULL //imx9_lpspibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)   NULL //imx9_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           (-1)//imx9_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             imx9_config_gpio(pinset)
#define px4_arch_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               imx9_gpio_read(pinset)
#define px4_arch_gpiowrite(pinset, value)       imx9_gpio_write(pinset, value)

int imx9_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);

#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  imx9_gpiosetevent(pinset,r,f,e,fp,a)

/* Code to access HRT timer counter directly from uer space TODO */

#if 0
#define PX4_USERSPACE_HRT
static inline uintptr_t hrt_absolute_time_usr_base(void)
{
	return USRIO_START + (XXX & MMU_PAGE_MASK);
}
#endif

// TODO!
#  define px4_cache_aligned_data()
#  define px4_cache_aligned_alloc malloc

__END_DECLS
