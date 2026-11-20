/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <nuttx/irq.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_ERBIUM
#define PX4_FLASH_BASE              0x40000000
#define PX4_NUMBER_I2C_BUSES        1
#define PX4_BUS_OFFSET              0

/* Erbium has no battery-backed SRAM */

#define PX4_BBSRAM_SIZE             2048
#define PX4_HF_GETDESC_IOCTL        0

/* Erbium has no unique-ID register; board_identity.c returns a fixed ID */

#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

#define PX4_CPU_UUID_WORD32_UNIQUE_H            2
#define PX4_CPU_UUID_WORD32_UNIQUE_M            1
#define PX4_CPU_UUID_WORD32_UNIQUE_L            0

#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

/* No GPIO driver yet */

static inline int px4_arch_configgpio(uint32_t pinset) { (void)pinset; return 0; }
static inline int px4_arch_unconfiggpio(uint32_t pinset) { (void)pinset; return 0; }
static inline bool px4_arch_gpioread(uint32_t pinset) { (void)pinset; return false; }
static inline void px4_arch_gpiowrite(uint32_t pinset, bool value) { (void)pinset; (void)value; }
static inline int px4_arch_gpiosetevent(uint32_t pinset, bool r, bool f, bool e, xcpt_t func, void *arg)
{
	(void)pinset; (void)r; (void)f; (void)e; (void)func; (void)arg;
	return 0;
}

#define px4_cache_aligned_data()
#define px4_cache_aligned_alloc malloc

__END_DECLS
