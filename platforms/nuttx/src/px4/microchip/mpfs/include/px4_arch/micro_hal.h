/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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


#include "../../../microchip_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_MPFS

/* Some of the boards have 3+ I2C buses on FPGA. Others have always just 2 as SoC hard-ip blocks */
#ifdef CONFIG_MPFS_COREI2C
#  define PX4_NUMBER_I2C_BUSES      CONFIG_MPFS_COREI2C_INSTANCES
#else
#  if defined(CONFIG_MPFS_I2C0) && defined(CONFIG_MPFS_I2C1)
#    define PX4_NUMBER_I2C_BUSES    2
#  else
#    define PX4_NUMBER_I2C_BUSES    1
#  endif
#endif

#define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#include <chip.h>
#include <mpfs_i2c.h>
#include <mpfs_spi.h>
#include <mpfs_corespi.h>
#include <arch/board/board_memorymap.h>

#include "riscv_mmu.h"

/*
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

/* Battery backed up SRAM definitions. TODO: check what memory can actually be used */
#define PX4_BBSRAM_SIZE             2048


// TODO: Use some proper UUID which can be obtained from the HW
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

static inline struct spi_dev_s *mpfs_spibus_initialize_wrap(int port)
{
#if defined(CONFIG_MPFS_CORESPI)
	return mpfs_corespibus_initialize(port);
#else
	return mpfs_spibus_initialize(port);
#endif
}

#define PX4_BUS_OFFSET       1                  /* MPFS buses are 0 based, so adjustment needed */
#define px4_savepanic(fileno, context, length)  mpfs_bbsram_savepanic(fileno, context, length)
#define px4_spibus_initialize(bus_num_1based)   mpfs_spibus_initialize_wrap(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#define px4_i2cbus_initialize(bus_num_1based)   mpfs_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#define px4_i2cbus_uninitialize(pdev)           mpfs_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             mpfs_configgpio(pinset)
#define px4_arch_unconfiggpio(pinset)           mpfs_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               mpfs_gpioread(pinset)
#define px4_arch_gpiowrite(pinset, value)       mpfs_gpiowrite(pinset, value)
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  mpfs_gpiosetevent(pinset,r,f,false,false,e,fp,a)

#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (~GPIO_OUTPUT | GPIO_BANK_MASK | GPIO_PIN_MASK | GPIO_BUFFER_MASK)) | GPIO_INPUT)

#define PX4_MAKE_GPIO_OUTPUT(gpio) (((gpio) & (~GPIO_INPUT | GPIO_BANK_MASK | GPIO_PIN_MASK | GPIO_BUFFER_MASK)) | GPIO_OUTPUT)

#define PX4_GPIO_PIN_OFF(gpio) (gpio & (GPIO_BANK_MASK | GPIO_PIN_MASK))

/* MPFS can access HRT timer counter directly from user space in
 * CONFIG_BUILD_PROTECTED and CONFIG_BUILD_KERNEL
 */

#define PX4_USERSPACE_HRT
static inline uintptr_t hrt_absolute_time_usr_base(void)
{
	return USRIO_START + (MPFS_CLINT_MTIME & RV_MMU_PAGE_MASK);
}

// TODO!
#  define px4_cache_aligned_data()
#  define px4_cache_aligned_alloc malloc

__END_DECLS
