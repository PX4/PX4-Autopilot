/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file bootloader_main.c
 *
 * FMU-specific early startup code for bootloader
*/

#include "board_config.h"
#include "bl.h"

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include <px4_platform_common/init.h>

#ifdef CONFIG_USBDEV_COMPOSITE
/* Defined in usb.c. Declared here because the prototypes in nuttx/board.h are
 * gated behind CONFIG_BOARDCTL_USBDEVCTRL, which the bootloader does not use -
 * it calls board_composite_initialize()/connect() directly. */
int board_composite_initialize(int port);
FAR void *board_composite_connect(int port, int configid);
#endif

__EXPORT void board_on_reset(int status) {}

__EXPORT void stm32_boardinitialize(void)
{
	/* configure USB interfaces */
	stm32_configgpio(GPIO_OTGFS_VBUS);

	/* Power up the peripheral rails so the external QSPI flash is powered when
	 * the bootloader programs it. These default to their "on" state in the pin
	 * definitions (PG0 high = 3V3 sensors on, PF2 low = 5V periph on). */
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_configgpio(GPIO_nVDD_5V_PERIPH_EN);

	/* Configure the QSPI external-flash pins here, early in board bring-up
	 * (like ArduPilot's __early_init). On the dual-core STM32H757, configuring
	 * these later from the bootloader's CM7 context (e.g. from the NuttX QSPI
	 * driver's stm32h7_qspi_initialize(), whose own stm32_configgpio() calls
	 * are compiled out for this board) fails to take effect for GPIOB
	 * (CLK=PB2, CS=PB10). The pull-ups keep /WP and /HOLD high during the
	 * single-line commands used to talk to the flash. */
	stm32_configgpio(GPIO_QSPI_SCK);
	stm32_configgpio(GPIO_QSPI_CS);
	stm32_configgpio(GPIO_QSPI_IO0);
	stm32_configgpio(GPIO_QSPI_IO1);
	stm32_configgpio(GPIO_QSPI_IO2);
	stm32_configgpio(GPIO_QSPI_IO3);
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	return 0;
}

void board_late_initialize(void)
{
	/* Bring up the composite USB device: two CDC/ACM ports - ttyACM0 for the
	 * bootloader protocol on this MCU, ttyACM1 for transparent passthrough to
	 * the secondary MCU's bootloader (see usb.c and the passthrough pump). */
	board_composite_initialize(0);
	board_composite_connect(0, 0);
}

extern void sys_tick_handler(void);
void board_timerhook(void)
{
	sys_tick_handler();
}
