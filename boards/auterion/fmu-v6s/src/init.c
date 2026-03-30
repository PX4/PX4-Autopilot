/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * Auterion FMU-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
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
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mm/gran.h>
#include <chip.h>
#include <stm32_uart.h>
#include <arch/board/board.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/px4_manifest.h>
#include <px4_platform_common/px4_mtd.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>
#include <px4_platform/board_hw_eeprom_rev_ver.h>
#include <px4_platform/gpio.h>
#include <lib/crc/crc.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/*
 * Ideally we'd be able to get these from arm_internal.h,
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
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);
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
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
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

	/* configure Ethernet Reference Clock */
	/* PLLQ1 is 100MHz, we need 50MHz on MCO1 => /2 */
	/* stm32_mco1config(RCC_CFGR_MCO1_PLL1Q, RCC_CFGR_MCO1PRE(2)); */

	/* configure pins */

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));
}

#if !defined(BOOTLOADER)
/****************************************************************************
 * Name: eeprom_read_and_check_mft
 *
 * Description:
 *   Read an mtd_mft_v0_t from the EEPROM at the given byte offset and
 *   validate its CRC.  Returns true if the record has a recognised version
 *   field and a matching CRC16.
 ****************************************************************************/

static bool eeprom_read_and_check_mft(struct i2c_master_s *i2c, uint16_t address)
{
	uint8_t addr_write[2] = { (uint8_t)(address >> 8), (uint8_t)(address & 0xFF) };
	mtd_mft_v0_t mft = {};
	struct i2c_msg_s msgs[2] = {
		{ .frequency = 400000, .addr = 0x50, .flags = 0, .buffer = addr_write, .length = sizeof(addr_write) },
		{ .frequency = 400000, .addr = 0x50, .flags = I2C_M_READ, .buffer = (uint8_t *) &mft, .length = sizeof(mft) },
	};

	int retries = 5;

	while (I2C_TRANSFER(i2c, msgs, 2) != OK) {
		if (--retries == 0) {
			syslog(LOG_WARNING, "[boot] EEPROM I2C comm failed\n");
			return false;
		}
	}

	if (mft.version.id != MTD_MFT_v0) { return false; }

	uint16_t computed = crc16_signature(CRC16_INITIAL, sizeof(mft) - sizeof(mft.crc), (const uint8_t *)&mft);
	return computed == mft.crc;
}

/****************************************************************************
 * Name: detect_layout_is_fram
 *
 * Description:
 *   Determine which EEPROM layout is present by reading MTD_MFT_VER from
 *   the two possible byte offsets and validating its CRC.
 ****************************************************************************/

static bool detect_layout_is_fram(void)
{
	struct i2c_master_s *i2c = px4_i2cbus_initialize(4);
	bool ret;

	if (!i2c) {
		syslog(LOG_WARNING, "[boot] EEPROM I2C init failed, defaulting to FRAM layout\n");
		ret = true;
		goto out;
	}

	/* FRAM-variant: MTD_MFT_VER after MTD_CALDATA (224 blocks x 32 B). */
	if (eeprom_read_and_check_mft(i2c, 224u * 32u)) {
		ret = true;
		goto out;
	}

	/* EEPROM-only: MTD_MFT_VER is the first partition (0 blocks x 32 B)*/
	if (eeprom_read_and_check_mft(i2c, 0u)) {
		ret = false;
		goto out;
	}

	/* Neither offset contains a valid record, default to FRAM layout. */
	ret = true;

out:
	px4_i2cbus_uninitialize(i2c);
	return ret;
}
#endif /* !defined(BOOTLOADER) */

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
#if !defined(BOOTLOADER)
	/* Need hrt running before using the ADC */
	px4_platform_init();

	/* First SPI init. HW version is not enabled yet, so only always-enabled
	 * buses/devices can be used. */
	stm32_spiinitialize();

	bool fram_available = detect_layout_is_fram();

	if (fram_available) {
		board_configure_fram();
	}

	/* Configure the HW based on the manifest */
	px4_platform_configure();

	if (OK == board_determine_hw_info()) {
		syslog(LOG_INFO, "[boot] Rev 0x%1x : Ver 0x%1x %s\n", board_get_hw_revision(), board_get_hw_version(),
		       board_get_hw_type_name());

	} else {
		syslog(LOG_ERR, "[boot] Failed to read HW revision and version\n");
	}

	/* EEPROM-only boards use a 128Kbit chip: 512 pages × 32 B = 16 KB.
	 * FRAM boards use a 64Kbit EEPROM: 256 pages × 32 B = 8 KB.
	 * Always use 32-byte page writes (safe for 64-byte page chips too). */
	unsigned int mtd_count = 0;
	mtd_instance_s **instances = px4_mtd_get_instances(&mtd_count);
	uint16_t eeprom_npages = fram_available ? 256 : 512;

	/* instances[0] is always EEPROM on both board variants. */
	if (mtd_count > 0 && instances[0]->mtd_dev != NULL) {
		px4_at24c_set_npages(instances[0]->mtd_dev, eeprom_npages);
	}

	/* Second SPI init. Now HW version is determined and complete init can be done. */
	stm32_spiinitialize();

	board_spi_reset(10, 0xffff);

	/* Configure the DMA allocator */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#  if defined(SERIAL_HAVE_RXDMA)
	// set up the serial DMA polling at 1ms intervals for received bytes that have not triggered a DMA event.
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#  endif

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_on(LED_GREEN); // Indicate Power.
	led_off(LED_BLUE);

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}

	/* Mount LittleFS as /fs/microsd:
	 * FRAM boards:  EEPROM->mtdblock0-4, FRAM->mtdblock5 (LittleFS).
	 * EEPROM-only: EEPROM->mtdblock0-2, FTL on free EEPROM region mtdblock3 (LittleFS). */
	const char *lfs_dev;

	if (fram_available) {
		lfs_dev = "/dev/mtdblock5";

	} else {
		if (mtd_count > 0 && instances[0]->mtd_dev != NULL) {
			FAR struct mtd_dev_s *eeprom_fs = mtd_partition(instances[0]->mtd_dev, 3, eeprom_npages - 3);

			if (!eeprom_fs || ftl_initialize(3, eeprom_fs) != 0) {
				syslog(LOG_ERR, "[boot] EEPROM: FTL init failed\n");
				led_on(LED_RED);
			}

		} else {
			syslog(LOG_ERR, "[boot] EEPROM not found\n");
			led_on(LED_RED);
		}

		lfs_dev = "/dev/mtdblock3";
	}

	if (nx_mount(lfs_dev, "/fs/microsd", "littlefs", 0, "autoformat") != 0) {
		syslog(LOG_ERR, "[boot] failed to mount /fs/microsd\n");
		led_on(LED_RED);
	}

#endif /* !defined(BOOTLOADER) */

	return OK;
}
