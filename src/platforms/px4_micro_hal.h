/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
/*
 * This file is a shim to bridge to nuttx_v3
 */
#ifdef __PX4_NUTTX
__BEGIN_DECLS
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#  define px4_enter_critical_section()       enter_critical_section()
#  define px4_leave_critical_section(flags)  leave_critical_section(flags)

#  if defined(CONFIG_ARCH_CHIP_STM32) || defined(CONFIG_ARCH_CHIP_STM32F7)
#    if defined(CONFIG_ARCH_CHIP_STM32)
#      include <stm32.h>
#      define PX4_BBSRAM_SIZE STM32_BBSRAM_SIZE
#      define PX4_BBSRAM_GETDESC_IOCTL STM32_BBSRAM_GETDESC_IOCTL
#      define PX4_FLASH_BASE  STM32_FLASH_BASE
#    endif
#    if defined(CONFIG_ARCH_CHIP_STM32F7)
#      include <chip.h>
#      define PX4_BBSRAM_SIZE STM32F7_BBSRAM_SIZE
#      define PX4_BBSRAM_GETDESC_IOCTL STM32F7_BBSRAM_GETDESC_IOCTL
#      define PX4_FLASH_BASE  0x08000000
#    endif
#    include <stm32_tim.h>
#    include <stm32_spi.h>
#    include <stm32_i2c.h>

/* STM32/32F7 defines the 96 bit UUID as
 *  init32_t[3] that can be read as bytes/half-words/words
 *  init32_t[0] PX4_CPU_UUID_ADDRESS[0] bits 31:0  (offset 0)
 *  init32_t[1] PX4_CPU_UUID_ADDRESS[1] bits 63:32 (offset 4)
 *  init32_t[2] PX4_CPU_UUID_ADDRESS[3] bits 96:64 (offset 8)
 *
 * The original PX4 stm32 (legacy) based implementation **displayed** the
 * UUID as: ABCD EFGH IJKL
 * Where:
 *       A was bit 31 and D was bit 0
 *       E was bit 63 and H was bit 32
 *       I was bit 95 and L was bit 64
 *
 * Since the string was used by some manufactures to identify the units
 * it must be preserved.
 *
 * For new targets moving forward we will use
 *      IJKL EFGH ABCD
 */
#    define PX4_CPU_UUID_BYTE_LENGTH                12
#    define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It wil be conferted to a string with the MSD on left and LSD on the right most position.
 */
#    define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* By not defining PX4_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
 * used for selection of significant digits of the UUID in the PX4 code base.
 * This is done to avoid the ripple effects changing the IDs used on STM32 base platforms
 */
#  if defined(PX4_CPU_UUID_CORRECT_CORRELATION)
#    define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
#    define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
#    define PX4_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
#  else
/* Legacy incorrect ordering */
#    define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
#    define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
#    define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#  endif

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#    define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#    define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#    define px4_spibus_initialize(port_1based)       stm32_spibus_initialize(port_1based)

#    define px4_i2cbus_initialize(bus_num_1based)    stm32_i2cbus_initialize(bus_num_1based)
#    define px4_i2cbus_uninitialize(pdev)            stm32_i2cbus_uninitialize(pdev)

#    define px4_arch_configgpio(pinset)              stm32_configgpio(pinset)
#    define px4_arch_unconfiggpio(pinset)            stm32_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)                stm32_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value)        stm32_gpiowrite(pinset, value)
#    define px4_arch_gpiosetevent(pinset,r,f,e,fp)   stm32_gpiosetevent(pinset,r,f, e,fp)
#endif
#include <arch/board/board.h>
__END_DECLS
#endif
