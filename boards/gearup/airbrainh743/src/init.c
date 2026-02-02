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

/**
 * @file init.c
 *
 * AirBrainH743-specific early startup code.
 */

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

#if defined(FLASH_BASED_PARAMS)
#include <parameters/flashparams/flashfs.h>
#endif

#ifdef CONFIG_MTD_W25N
extern FAR struct mtd_dev_s *w25n_initialize(FAR struct spi_dev_s *dev, uint32_t spi_devid);
#endif

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
		up_mdelay(100);
	}
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.
 *
 ************************************************************************************/
__EXPORT void stm32_boardinitialize(void)
{
	/* Reset PWM first thing */
	board_on_reset(-1);

	/* configure LEDs */
	board_autoled_initialize();

	/* configure pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure USB interfaces */
	stm32_usbinitialize();
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.
 *
 ****************************************************************************/
__EXPORT int board_app_initialize(uintptr_t arg)
{
	/* Need hrt running before using the ADC */
	px4_platform_init();

	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure the DMA allocator */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}

#ifdef CONFIG_MTD_W25N
	/* Initialize W25N01GV NAND Flash on SPI2 */
	struct spi_dev_s *spi2 = stm32_spibus_initialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI2 for W25N\n");
		led_on(LED_RED);

	} else {
		struct mtd_dev_s *mtd = w25n_initialize(spi2, 0);

		if (!mtd) {
			syslog(LOG_ERR, "[boot] FAILED to initialize W25N MTD driver\n");
			led_on(LED_RED);

		} else {
			int ret = register_mtddriver("/dev/mtd0", mtd, 0755, NULL);

			if (ret < 0) {
				syslog(LOG_ERR, "[boot] FAILED to register MTD driver: %d\n", ret);
				led_on(LED_RED);

			} else {
				syslog(LOG_INFO, "[boot] W25N MTD registered at /dev/mtd0\n");

#ifdef CONFIG_FS_LITTLEFS
				ret = nx_mount("/dev/mtd0", CONFIG_BOARD_ROOT_PATH, "littlefs", 0, NULL);

				if (ret == 0) {
					/* Verify the filesystem is usable by creating a test file */
					int fd = open(CONFIG_BOARD_ROOT_PATH "/.mount_test", O_CREAT | O_WRONLY | O_TRUNC);

					if (fd >= 0) {
						close(fd);
						unlink(CONFIG_BOARD_ROOT_PATH "/.mount_test");

					} else {
						syslog(LOG_WARNING, "[boot] littlefs mounted but not usable, reformatting\n");
						nx_umount2(CONFIG_BOARD_ROOT_PATH, 0);
						ret = -1;
					}
				}

				if (ret < 0) {
					ret = nx_mount("/dev/mtd0", CONFIG_BOARD_ROOT_PATH, "littlefs", 0, "forceformat");
				}

				if (ret < 0) {
					syslog(LOG_ERR, "[boot] FAILED to mount littlefs: %d\n", ret);
					led_on(LED_RED);

				} else {
					syslog(LOG_INFO, "[boot] LittleFS mounted at %s\n", CONFIG_BOARD_ROOT_PATH);
				}

#endif
			}
		}
	}

#endif

#if defined(FLASH_BASED_PARAMS)
	/* Initialize parameters in internal flash (sector 15, 128KB at 0x081E0000) */
	static sector_descriptor_t params_sector_map[] = {
		{15, 128 * 1024, 0x081E0000},
		{0, 0, 0},
	};

	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_RED);
	}

#endif

	/* Configure the HW based on the manifest */
	px4_platform_configure();

	return OK;
}
