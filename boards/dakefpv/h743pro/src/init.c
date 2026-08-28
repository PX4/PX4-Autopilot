/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * FMU-specific early startup code. This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

#include "board_config.h"

#include <syslog.h>
#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>

#if defined(BOARD_TEL3_SWAP_RXTX)
/* TIOCSSWAP/SER_SWAP_ENABLED are not reachable via termios.h or sys/ioctl.h */
#  include <errno.h>
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#  include <nuttx/serial/tioctl.h>
#endif

#include <nuttx/config.h>
#include <nuttx/board.h>
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
#  include <parameters/flashparams/flashfs.h>
#endif

#if defined(CONFIG_MTD_W25) || defined(CONFIG_MTD_M25P)
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <sys/mount.h>
#endif

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Name: boot_log
 *
 * Description:
 *   Emit a board bring-up message to both logging paths.
 *
 *   syslog() reaches the serial console but never PX4's console buffer.
 *   printf() reaches that buffer - px4_platform_init() dup2's stdout onto it
 *   - and so shows up in dmesg, but the buffer does not forward to the
 *   console. Neither path alone is enough: a developer on USB sees only
 *   dmesg, one on the UART console sees only syslog. Write to both, or
 *   bring-up failures stay invisible to whoever is actually looking.
 *
 ****************************************************************************/
static void boot_log(int priority, const char *fmt, ...)
__attribute__((format(printf, 2, 3)));

static void boot_log(int priority, const char *fmt, ...)
{
	char msg[128];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(msg, sizeof(msg), fmt, ap);
	va_end(ap);

	syslog(priority, "%s", msg);
	fputs(msg, stdout);
	fflush(stdout);
}

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

	/*
	 * On resets invoked from system (not boot) ensure we establish a low
	 * output state on PWM pins to disarm the ESC and prevent the reset from potentially
	 * spinning up the motors.
	 */
	if (status >= 0) {
		up_mdelay(100);
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
__EXPORT void stm32_boardinitialize(void)
{
	/* Reset PWM first thing */
	board_on_reset(-1);

	/* configure LEDs */
	board_autoled_initialize();

	/* configure pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure USB interfaces */
	stm32_usbinitialize();

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
 *         meaning to NuttX;
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
		boot_log(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_BLUE);
	}

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{15, 128 * 1024, 0x081E0000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		boot_log(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_on(LED_BLUE);
		return -ENODEV;
	}

#endif

#if defined(CONFIG_MTD_W25) || defined(CONFIG_MTD_M25P)
	/* Mount W25Q128 SPI NOR flash (SPI3, CS=PA15) at /fs/microsd */
	struct spi_dev_s *spi3 = stm32_spibus_initialize(3);

	if (!spi3) {
		boot_log(LOG_INFO, "[boot] flash: SPI3 init failed\n");

	} else {
		/* Read the JEDEC ID (RDID) before probing, so a failure can report
		 * what the part actually answered. "chip not recognised" on its own
		 * gives nothing to act on, and these boards have shipped with more
		 * than one flash part. All-zero or all-ff means the SPI transaction
		 * itself failed rather than the chip being unknown.
		 */
		uint8_t jedec[3] = {0, 0, 0};

		SPI_LOCK(spi3, true);
		SPI_SETMODE(spi3, SPIDEV_MODE0);
		SPI_SETBITS(spi3, 8);
		SPI_SETFREQUENCY(spi3, 1000000);
		SPI_SELECT(spi3, SPIDEV_FLASH(0), true);
		SPI_SEND(spi3, 0x9f);
		jedec[0] = SPI_SEND(spi3, 0xff);
		jedec[1] = SPI_SEND(spi3, 0xff);
		jedec[2] = SPI_SEND(spi3, 0xff);
		SPI_SELECT(spi3, SPIDEV_FLASH(0), false);
		SPI_LOCK(spi3, false);

		/* These boards ship with more than one flash part. Both have been
		 * seen on H743 Pro hardware:
		 *
		 *   ef 40 18  Winbond W25Q128        -> w25 driver
		 *   20 ba 18  Micron MT25Q/N25Q128   -> m25p driver
		 *
		 * Neither driver accepts the other's manufacturer ID, so try each in
		 * turn rather than committing the board to one batch.
		 */
		struct mtd_dev_s *mtd = w25_initialize(spi3);

		if (!mtd) {
			mtd = m25p_initialize(spi3);
		}

		if (!mtd) {
			boot_log(LOG_ERR, "[boot] flash: chip not recognised (JEDEC %02x %02x %02x)\n",
				 jedec[0], jedec[1], jedec[2]);

		} else {
			boot_log(LOG_INFO, "[boot] flash: chip ok (JEDEC %02x %02x %02x), registering MTD...\n",
				 jedec[0], jedec[1], jedec[2]);
			int ret = register_mtddriver("/dev/mtd0", mtd, 0755, NULL);

			if (ret < 0 && ret != -EEXIST) {
				boot_log(LOG_INFO, "[boot] flash: MTD register failed %d\n", ret);

			} else {
				ret = nx_mount("/dev/mtd0", "/fs/microsd", "littlefs", 0, NULL);

				if (ret < 0) {
					boot_log(LOG_INFO, "[boot] flash: first mount failed %d, formatting...\n", ret);
					ret = nx_mount("/dev/mtd0", "/fs/microsd", "littlefs", 0, "forceformat");
				}

				if (ret == 0) {
					boot_log(LOG_INFO, "[boot] flash: mounted at /fs/microsd\n");

					/* Seed extras.txt on first boot if not present */
					mkdir("/fs/microsd/etc", 0755);
					const char *extras = "/fs/microsd/etc/extras.txt";
					FILE *ef = fopen(extras, "r");

					if (!ef) {
						ef = fopen(extras, "w");

						if (ef) {
							fputs("# DAKEFPV H743 Pro extras -- runs at every boot\n", ef);
							fputs("# Add driver start commands here.\n", ef);
							fclose(ef);
						}

					} else {
						fclose(ef);
					}

				} else {
					boot_log(LOG_INFO, "[boot] flash: mount failed %d\n", ret);
				}
			}
		}
	}

#endif

#if defined(BOARD_TEL3_SWAP_RXTX)
	/* The HD VTX connector is wired pin 3 -> PB8 and pin 4 -> PB9, i.e. reversed
	 * with respect to the SoC's UART4 pinout (TX is only available on PB9, RX
	 * only on PB8). Swap the peripheral so pin 3 carries TX as a standard 6-pin
	 * DJI/OpenIPC harness expects.
	 *
	 * Done here rather than in a driver: USART_CR2_SWAP survives up_setup(),
	 * up_set_format() and up_shutdown() -- all of which read-modify-write CR2 --
	 * so a single ioctl at boot covers every consumer of the port, whether
	 * TELEM3 is later used for msp_osd, mavlink or anything else.
	 */
	int tel3 = open(CONFIG_BOARD_SERIAL_TEL3, O_RDWR | O_NONBLOCK);

	if (tel3 < 0) {
		boot_log(LOG_ERR, "[boot] TELEM3 open for RX/TX swap failed: %d\n", errno);

	} else {
		if (ioctl(tel3, TIOCSSWAP, SER_SWAP_ENABLED) < 0) {
			boot_log(LOG_ERR, "[boot] TELEM3 RX/TX swap failed: %d\n", errno);

		} else {
			boot_log(LOG_INFO, "[boot] TELEM3 RX/TX swapped\n");
		}

		close(tel3);
	}

#endif

	/* Configure the HW based on the manifest */
	px4_platform_configure();

	return OK;
}
