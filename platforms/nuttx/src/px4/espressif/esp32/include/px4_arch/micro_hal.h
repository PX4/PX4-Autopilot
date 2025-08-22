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


#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#include <esp32_tim.h>
#include <esp32_spi.h>
#include <esp32_i2c.h>
#include <nuttx/mtd/mtd.h>
#include <esp32_spiflash.h>

#include <esp32_gpio.h>

# define PX4_CPU_UUID_WORD32_UNIQUE_H            1 /* Most significant digits change the least */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */

#define PX4_CPU_UUID_BYTE_LENGTH                8
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

#define px4_enter_critical_section()       enter_critical_section()
#define px4_leave_critical_section(flags)  leave_critical_section(flags)

#define px4_udelay(usec) up_udelay(usec)
#define px4_mdelay(msec) up_mdelay(msec)

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

// bits		Function
// 0-5		GPIO number. 0-40 is valid.
// 6		input
// 7		output
// 8		fun
// 9		pull up
// 10		pull down
// 11		open drain
// 12-14	GPIO function select
// 15-31	Unused

#define GPIO_SET_SHIFT  6
#define GPIO_INPUT	(1 << 6)
#define GPIO_OUTPUT	(1 << 7)
#define GPIO_FUNCTION	(1 << 8)

#define GPIO_PULLUP	(1 << 9)
#define GPIO_PULLDOWN	(1 << 10)
#define GPIO_OPEN_DRAIN	(1 << 11)

#define FUNCTION_SHIFT 6
#define FUNCTION_1        (1 << FUNCTION_SHIFT)
#define FUNCTION_2        (2 << FUNCTION_SHIFT)
#define FUNCTION_3        (3 << FUNCTION_SHIFT)
#define FUNCTION_4        (4 << FUNCTION_SHIFT)
#define FUNCTION_5        (5 << FUNCTION_SHIFT)
#define FUNCTION_6        (6 << FUNCTION_SHIFT)

#define GPIO_FUN(func)	(func << 12)	// Function select

#define GPIO_NUM_MASK		0x3f
#define	GPIO_INPUT_MASK		0x40
#define	GPIO_OUTPUT_MASK	0x80
#define	GPIO_FUNCTION_MASK	0x100
#define	GPIO_PULLUP_MASK	0x200
#define	GPIO_PULLDOWN_MASK	0x400
#define	GPIO_OPEN_DRAIN_MASK	0x800
#define	GPIO_FUN_SELECT_MASK	0x7000

int px4_esp32_configgpio(uint32_t pinset);
int px4_esp32_unconfiggpio(uint32_t pinset);
int esp32_gpiosetevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg);
#define PX4_BUS_OFFSET       1
#define px4_arch_configgpio(pinset)			px4_esp32_configgpio(pinset)
#define px4_arch_unconfiggpio(pinset)			px4_esp32_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)			esp32_gpioread(pinset & GPIO_NUM_MASK)
#define px4_arch_gpiowrite(pinset, value)		esp32_gpiowrite(pinset & GPIO_NUM_MASK, value)
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)	esp32_gpiosetevent(pinset,r,f,e,fp,a)

#define px4_arch_mtd_dev()				esp32_spiflash_get_mtd()

#define px4_spibus_initialize(bus_num_1based)   esp32_spibus_initialize(bus_num_1based)

#define px4_i2cbus_initialize(bus_num_1based)   esp32_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           esp32_i2cbus_uninitialize(pdev)

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_UNUSED
#define PX4_FLASH_BASE  	    ESP32_FLASH_BASE
#define PX4_NUMBER_I2C_BUSES 		2
#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 0



#undef DISABLED

__END_DECLS
