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
 * This file is a shim to bridge to the many SoC architecture supported by PX4
 */

#ifdef __PX4_NUTTX
__BEGIN_DECLS
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

/* For historical reasons (NuttX STM32 numbering) PX4 bus numbering is 1 based
 * All PX4 code, including, board code is written to assuming 1 based numbering.
 * The following macros are used to allow the board config to define the bus
 * numbers in terms of the NuttX driver numbering. 1,2,3 for one based numbering
 * schemes or 0,1,2 for zero based schemes.
 */

#define PX4_BUS_NUMBER_TO_PX4(x)        ((x)+PX4_BUS_OFFSET)  /* Use to define Zero based to match Nuttx Driver but provide 1 based to PX4 */
#define PX4_BUS_NUMBER_FROM_PX4(x)      ((x)-PX4_BUS_OFFSET)  /* Use to map PX4 1 based to NuttX driver 0 based */

#  define px4_enter_critical_section()       enter_critical_section()
#  define px4_leave_critical_section(flags)  leave_critical_section(flags)

#  if defined(CONFIG_ARCH_CHIP_STM32) || defined(CONFIG_ARCH_CHIP_STM32F7)

#    if defined(CONFIG_ARCH_CHIP_STM32)
#      include <stm32.h>
#      define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_STM32F4
#      define PX4_FLASH_BASE  STM32_FLASH_BASE
#      if defined(CONFIG_STM32_STM32F4XXX)
#        include <stm32_bbsram.h>
#        define PX4_BBSRAM_SIZE STM32_BBSRAM_SIZE
#        define PX4_BBSRAM_GETDESC_IOCTL STM32_BBSRAM_GETDESC_IOCTL
#      endif
#      define PX4_NUMBER_I2C_BUSES STM32_NI2C
#    endif

#    if defined(CONFIG_ARCH_CHIP_STM32F7)
#      define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_STM32F7
#      include <chip.h>
#include <chip/stm32_flash.h>
#      include <up_internal.h> //include up_systemreset() which is included on stm32.h
#      include <stm32_bbsram.h>
#      define PX4_BBSRAM_SIZE STM32F7_BBSRAM_SIZE
#      define PX4_BBSRAM_GETDESC_IOCTL STM32F7_BBSRAM_GETDESC_IOCTL
#      define PX4_FLASH_BASE  0x08000000
#      define PX4_NUMBER_I2C_BUSES STM32F7_NI2C
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
 * It will be converted to a string with the MSD on left and LSD on the right most position.
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

#    define px4_savepanic(fileno, context, length)  stm32_bbsram_savepanic(fileno, context, length)

#    define PX4_BUS_OFFSET       0                  /* STM buses are 1 based no adjustment needed */
#    define px4_spibus_initialize(bus_num_1based)   stm32_spibus_initialize(bus_num_1based)

#    define px4_i2cbus_initialize(bus_num_1based)   stm32_i2cbus_initialize(bus_num_1based)
#    define px4_i2cbus_uninitialize(pdev)           stm32_i2cbus_uninitialize(pdev)

#    define px4_arch_configgpio(pinset)             stm32_configgpio(pinset)
#    define px4_arch_unconfiggpio(pinset)           stm32_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)               stm32_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value)       stm32_gpiowrite(pinset, value)
#    define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  stm32_gpiosetevent(pinset,r,f,e,fp,a)
#endif // defined(CONFIG_ARCH_CHIP_STM32) || defined(CONFIG_ARCH_CHIP_STM32F7)

#if defined(CONFIG_ARCH_CHIP_KINETIS)
#    define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_KINETISK66

#    // Fixme: using ??
#    define PX4_BBSRAM_SIZE             2048
#    define PX4_BBSRAM_GETDESC_IOCTL    0
#    define PX4_NUMBER_I2C_BUSES        KINETIS_NI2C

#    define GPIO_OUTPUT_SET             GPIO_OUTPUT_ONE
#    define GPIO_OUTPUT_CLEAR           GPIO_OUTPUT_ZERO

#    include <chip.h>
#    include <kinetis_spi.h>
#    include <kinetis_i2c.h>
#    include <kinetis_uid.h>

/* Kinetis defines the 128 bit UUID as
 *  init32_t[4] that can be read as words
 *  init32_t[0] PX4_CPU_UUID_ADDRESS[0] bits 127:96 (offset 0)
 *  init32_t[1] PX4_CPU_UUID_ADDRESS[1] bits 95:64  (offset 4)
 *  init32_t[2] PX4_CPU_UUID_ADDRESS[1] bits 63:32  (offset 8)
 *  init32_t[3] PX4_CPU_UUID_ADDRESS[3] bits 31:0   (offset C)
 *
 *  PX4 uses the words in bigendian order MSB to LSB
 *   word  [0]    [1]    [2]   [3]
 *   bits 127:96  95-64  63-32, 31-00,
 */
#    define PX4_CPU_UUID_BYTE_LENGTH                KINETIS_UID_SIZE
#    define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#    define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* define common formating across all commands */

#    define PX4_CPU_UUID_WORD32_FORMAT              "%08x"
#    define PX4_CPU_UUID_WORD32_SEPARATOR           ":"

#    define PX4_CPU_UUID_WORD32_UNIQUE_H            3 /* Least significant digits change the most */
#    define PX4_CPU_UUID_WORD32_UNIQUE_M            2 /* Middle High significant digits */
#    define PX4_CPU_UUID_WORD32_UNIQUE_L            1 /* Middle Low significant digits */
#    define PX4_CPU_UUID_WORD32_UNIQUE_N            0 /* Most significant digits change the least */

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#    define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#    define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

#    define kinetis_bbsram_savepanic(fileno, context, length) (0) // todo:Not implemented yet

#    define px4_savepanic(fileno, context, length)   kinetis_bbsram_savepanic(fileno, context, length)

/* bus_num is zero based on kinetis and must be translated from the legacy one based */

#    define PX4_BUS_OFFSET       1                  /* Kinetis buses are 0 based and adjustment is needed */

#    define px4_spibus_initialize(bus_num_1based)   kinetis_spibus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))

#    define px4_i2cbus_initialize(bus_num_1based)   kinetis_i2cbus_initialize(PX4_BUS_NUMBER_FROM_PX4(bus_num_1based))
#    define px4_i2cbus_uninitialize(pdev)           kinetis_i2cbus_uninitialize(pdev)

#    define px4_arch_configgpio(pinset)             kinetis_pinconfig(pinset)
#    define px4_arch_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)               kinetis_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value)       kinetis_gpiowrite(pinset, value)

/* kinetis_gpiosetevent is not implemented and will need to be added */

#    define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  kinetis_gpiosetevent(pinset,r,f,e,fp,a)
#  endif

#include <arch/board/board.h>

__END_DECLS
#endif
