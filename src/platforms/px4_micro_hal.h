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

#  define px4_enter_critical_section()       irqsave()
#  define px4_leave_critical_section(flags)  irqrestore(flags)

#  define px4_spibus_initialize(port_1based)       up_spiinitialize(port_1based)

#  define px4_i2cbus_initialize(bus_num_1based)    up_i2cinitialize(bus_num_1based)
#  define px4_i2cbus_uninitialize(pdev)            up_i2cuninitialize(pdev)

#  if defined(CONFIG_STM32_VALUELINE) || defined(CONFIG_STM32_STM32F40XX)
#    define px4_arch_configgpio(pinset)      stm32_configgpio(pinset)
#    define px4_arch_unconfiggpio(pinset)     stm32_unconfiggpio(pinset)
#    define px4_arch_gpioread(pinset)         stm32_gpioread(pinset)
#    define px4_arch_gpiowrite(pinset, value) stm32_gpiowrite(pinset, value)
#  endif
#endif
