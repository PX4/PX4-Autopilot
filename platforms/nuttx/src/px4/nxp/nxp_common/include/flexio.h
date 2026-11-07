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

/* FlexIO1 register access and pin setup shared by the i.MX RT and i.MX9 FlexIO drivers */

#pragma once

#include <px4_arch/io_timer.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"

#if defined(CONFIG_ARCH_CHIP_IMXRT)
#include <imxrt_flexio.h>
#include <imxrt_periphclks.h>

#if defined(IOMUX_PULL_UP_47K)
#define IOMUX_PULL_UP IOMUX_PULL_UP_47K
#endif

#define FLEXIO_BASE			IMXRT_FLEXIO1_BASE
#define FLEXIO_OFFSET(reg)		IMXRT_FLEXIO_##reg##_OFFSET
#define FLEXIO_IRQ			IMXRT_IRQ_FLEXIO1

static inline void flexio_clock_enable(void) { imxrt_clockall_flexio1(); }
static inline void flexio_pin_config(const dshot_conf_t *conf) { imxrt_config_gpio(conf->pinmux | IOMUX_PULL_UP); }
static inline bool flexio_pin_unused(const dshot_conf_t *conf) { return conf->pinmux == 0; }
#else /* i.MX9 */
#include <imx9_flexio.h>
#include <imx9_clockconfig.h>

#define FLEXIO_BASE			IMX9_FLEXIO1_BASE
#define FLEXIO_OFFSET(reg)		IMX9_FLEXIO_##reg##_OFFSET
#define FLEXIO_IRQ			IMX9_IRQ_FLEXIO1

static inline void flexio_clock_enable(void) { imx9_configure_clock(FLEXIO1_CLK_ROOT_SYS_PLL1_DFS1_DIV2_CLK | CLOCK_DIV(3), true); }
static inline void flexio_pin_config(const dshot_conf_t *conf) { imx9_iomux_configure(conf->pinmux); }
/* Pad control registers never sit at offset 0, so a zero padregoff marks an unset pinmux */
static inline bool flexio_pin_unused(const dshot_conf_t *conf) { return conf->pinmux.padcfg.padregoff == 0; }
#endif

/* RT117x and i.MX9 (FlexIO 2.x) have a 3-bit TIMCFG[TIMDEC] (22:20) with FlexIO clock /16 and /256
 * decrement sources; RT106x (FlexIO 1.1) has 2 bits and no prescaler. The NuttX headers carry the
 * 2-bit mask for every part. */
#if defined(CONFIG_ARCH_FAMILY_IMXRT117x) || !defined(CONFIG_ARCH_CHIP_IMXRT)
#define FLEXIO_HAS_CLOCK_PRESCALER	1
#else
#define FLEXIO_HAS_CLOCK_PRESCALER	0
#endif

#define FLEXIO_TIMCFG_TIMDEC3(x)	((((uint32_t)(x)) << FLEXIO_TIMCFG_TIMDEC_SHIFT) & 0x700000u)
#define FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_DIV16	4u

static inline uint32_t flexio_getreg32(uint32_t offset)
{
	return getreg32(FLEXIO_BASE + offset);
}

static inline void flexio_modifyreg32(unsigned int offset,
				      uint32_t clearbits,
				      uint32_t setbits)
{
	modifyreg32(FLEXIO_BASE + offset, clearbits, setbits);
}

static inline void flexio_putreg32(uint32_t value, uint32_t offset)
{
	putreg32(value, FLEXIO_BASE + offset);
}
