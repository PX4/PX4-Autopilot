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
#    endif
#    if defined(CONFIG_ARCH_CHIP_STM32F7)
#      include <chip.h>
#      define PX4_BBSRAM_SIZE STM32F7_BBSRAM_SIZE
#      define PX4_BBSRAM_GETDESC_IOCTL STM32F7_BBSRAM_GETDESC_IOCTL
#    endif
#    include <stm32_tim.h>
#    include <stm32_spi.h>
#    include <stm32_i2c.h>

#    define px4_spibus_initialize(port_1based)       stm32_spibus_initialize(port_1based)

#    define px4_i2cbus_initialize(bus_num_1based)    stm32_i2cbus_initialize(bus_num_1based)
#    define px4_i2cbus_uninitialize(pdev)            stm32_i2cbus_uninitialize(pdev)

#    define px4_arch_configgpio(pinset)             stm32_configgpio(pinset)
#    define px4_arch_unconfiggpio(pinset)           stm32_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)               stm32_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value)       stm32_gpiowrite(pinset, value)
#    define px4_arch_gpiosetevent(pinset,r,f,e,fp)  stm32_gpiosetevent(pinset,r,f, e,fp)
#endif

#if defined(CONFIG_ARCH_CHIP_KINETIS)

#    define STM32_SYSMEM_UID         0x40048054 // Fixme: using board crtrl
#    define PX4_BBSRAM_SIZE          2048
#    define PX4_BBSRAM_GETDESC_IOCTL 0
#    define GPIO_OUTPUT_SET          GPIO_OUTPUT_ONE
#    define GPIO_OUTPUT_CLEAR        GPIO_OUTPUT_ZER0

#    include <chip.h>
#    include <kinetis_spi.h>
#    include <kinetis_i2c.h>

/* bus_num is zero based on kinetis and must be translated from the legacy one based */

#    define px4_spibus_initialize(port_1based)       kinetis_spibus_initialize(port_1based-1)

#    define px4_i2cbus_initialize(bus_num_1based)    kinetis_i2cbus_initialize(bus_num_1based-1)
#    define px4_i2cbus_uninitialize(pdev)            kinetis_i2cbus_uninitialize(pdev)

#    define px4_arch_configgpio(pinset)             kinetis_pinconfig(pinset)
#    define px4_arch_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)               kinetis_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value)       kinetis_gpiowrite(pinset, value)
#    define px4_arch_gpiosetevent(pinset,r,f,e,fp)  kinetis_gpiosetevent(pinset,r,f, e,fp)
#  endif
#include <arch/board/board.h>
__END_DECLS
#endif
