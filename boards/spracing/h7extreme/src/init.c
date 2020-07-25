/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file init.c
 *
 * Board-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "up_internal.h"

#include <px4_arch/io_timer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/*
 * Ideally we'd be able to get these from up_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
}

/************************************************************************************
 * Name: board_on_reset
 *
 * Description:
 * Optionally provided function called on entry to board_system_reset
 * It should perform any house keeping prior to the rest.
 *
 * status - 1 if resetting to boot loader
 *          0 if just resetting
 *
 ************************************************************************************/
__EXPORT void board_on_reset(int status)
{
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_as_pwm_input(i)));
	}

	if (status >= 0) {
		up_mdelay(6);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	board_on_reset(-1); /* Reset PWM first thing */

	/* configure LEDs */

	board_autoled_initialize();

	/* configure pins */

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */

	stm32_spiinitialize();

	/* configure USB interfaces */

	stm32_usbinitialize();

	/* configure external memory*/
	flash_w25q128_init();

}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/


__EXPORT int board_app_initialize(uintptr_t arg)
{
	/* Need hrt running before using the ADC */

	px4_platform_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#if 0 // serial DMA is not yet implemented in NuttX for stm32h7
	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);
#endif


	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}


#ifdef CONFIG_MMCSD
	int ret = stm32_sdio_initialize();

	if (ret != OK) {
		led_on(LED_RED);
		return ret;
	}

#endif

	up_udelay(20);

	/* W25128 external flash memory:
	 * 0x90030000 - 0x9003FFFF -> 64KB for FlashFS
	 * 0x90100000 - 0x902FFFFF -> 2MB for PX4 firmware
	 */

	/* Page calculator for W25Q128:
	 * page = (address - 0x90000000)/0x1000 (sector size is 0x1000 (4096 bytes))
	 * examples:
	 * 1) address = 0x90001000, page = (0x90001000 - 0x90000000)/0x1000 = 0x1
	 * 2) address = 0x90200000, page = (0x90200000 - 0x90000000)/0x1000 = 0x200
	 * 3) address = 0x90200000, page = (0x90200100 - 0x90000000)/0x1000 = 0x201
	 * 4) address = 0x90030000, page = (0x90030000 - 0x90000000)/0x1000 = 0x30
	 */

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{0x30, 4096, 0x90030000},
		{0x31, 4096, 0x90031000},
		{0x32, 4096, 0x90032000},
		{0x33, 4096, 0x90033000},
		{0x34, 4096, 0x90034000},
		{0x35, 4096, 0x90035000},
		{0x36, 4096, 0x90036000},
		{0x37, 4096, 0x90037000},
		{0x38, 4096, 0x90038000},
		{0x39, 4096, 0x90039000},
		{0x3A, 4096, 0x9003A000},
		{0x3B, 4096, 0x9003B000},
		{0x3C, 4096, 0x9003C000},
		{0x3D, 4096, 0x9003D000},
		{0x3E, 4096, 0x9003E000},
		{0x3F, 4096, 0x9003F000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_AMBER);
		return -ENODEV;
	}

#endif

	return OK;
}
