/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

/**
 * @file bl_update.cpp
 *
 * STM32F4 & STM32F7 bootloader update tool.
 */

#include <lib/bl_update/bl_update_flash.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <inttypes.h>
#include <string.h>

#if defined (CONFIG_STM32_STM32F4XXX) || defined (CONFIG_ARCH_CHIP_STM32F7) || \
    defined (CONFIG_ARCH_CHIP_STM32H7)

static int setopt(void);

static void print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Utility to flash the bootloader from a file");

	PRINT_MODULE_USAGE_NAME_SIMPLE("bl_update", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("setopt", "Set option bits to unlock the FLASH (only needed if in locked state)");

	PRINT_MODULE_USAGE_COMMAND_DESCR("<file>", "Bootloader bin file");
}

#endif // defined (CONFIG_STM32_STM32F4XXX) || defined (CONFIG_ARCH_CHIP_STM32F7)


extern "C" __EXPORT int bl_update_main(int argc, char *argv[])
{
#if !(defined (CONFIG_STM32_STM32F4XXX) || defined (CONFIG_ARCH_CHIP_STM32F7) \
    || defined (CONFIG_ARCH_CHIP_STM32H7))
	PX4_ERR("Not supported on this HW");
	return 1;
}
#else

	if (argc != 2)
	{
		print_usage("missing firmware filename or command");
		return 1;
	}

	if (!strcmp(argv[1], "setopt"))
	{
		return setopt();
	}

	const bl_update::Result result = bl_update::flash(argv[1]);

	if (result == bl_update::Result::Updated || result == bl_update::Result::Unchanged)
	{
		PX4_INFO("%s", bl_update::result_str(result));
		return 0;
	}

	PX4_ERR("%s: %s", argv[1], bl_update::result_str(result));
	return 1;
}

static int
setopt(void)
{
	volatile uint32_t *optcr = (volatile uint32_t *)0x40023c14;

	const uint16_t opt_mask = (3 << 2);		/* BOR_LEV bitmask */
	const uint16_t opt_bits = (0 << 2);		/* BOR = 0, setting for 2.7-3.6V operation */

	if ((*optcr & opt_mask) == opt_bits) {
		PX4_INFO("option bits are already set as required");
		return 0;
	}

	/* unlock the control register */
	volatile uint32_t *optkeyr = (volatile uint32_t *)0x40023c08;
	*optkeyr = 0x08192a3bU;
	*optkeyr = 0x4c5d6e7fU;

	if (*optcr & 1) {
		PX4_ERR("option control register unlock failed");
		return 1;
	}

	/* program the new option value */
	*optcr = (*optcr & ~opt_mask) | opt_bits | (1 << 1);

	px4_usleep(1000);

	if ((*optcr & opt_mask) == opt_bits) {
		PX4_INFO("option bits set");
		return 0;
	}

	PX4_ERR("option bits setting failed; readback 0x%04" PRIx32, *optcr);
	return 1;
}
#endif // defined (CONFIG_STM32_STM32F4XXX) || defined (CONFIG_ARCH_CHIP_STM32F7)
