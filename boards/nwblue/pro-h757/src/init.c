/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * board-specific early startup code. This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

#include "board_config.h"

#include <syslog.h>

#include <nuttx/config.h>
#include <drivers/bootloaders/boot_app_shared.h>

/****************************************************************************
 * Application firmware descriptor for CAN bootloader
 *
 * This structure is used by the CAN bootloader to validate the firmware.
 * The image_crc and image_size fields are filled in by a post-processing
 * script (make_can_boot_descriptor.py) after the firmware is built.
 ****************************************************************************/
boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = APP_DESCRIPTOR_SIGNATURE,
	{
		0,  /* image_crc - filled by post-processing */
	},
	.image_size = 0,  /* filled by post-processing */
	.git_hash  = 0,   /* filled by post-processing */
	.major_version = 0,
	.minor_version = 1,
	.board_id = 5730,  /* NWBlue Pro H757 board ID */
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
};
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/usb/usbhost.h>
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

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <nuttx/fs/fs.h>
#include <mpu.h>
#include <fcntl.h>

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
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
	/* Power off Interfaces */
	//stm32_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, true);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms\n", ms);

	/* re-enable power */
	//stm32_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, false);
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
	//for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
	//	px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_as_pwm_input(i)));
	//}

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
__EXPORT void stm32_boardinitialize(void)
{
	/* Reset PWM first thing */
	//board_on_reset(-1);

	/* TODO: re-enable watchdog_init() once an STM32H7 implementation of
	 * arch_watchdog_iwdg lands in mainline (the bootloader starts the
	 * IWDG before jumping to the app and we must keep petting it).
	 */

	/* configure pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;

	/* Only call px4_gpio_init if there are GPIOs to initialize */
	if (sizeof(gpio) > 0) {
		px4_gpio_init(gpio, arraySize(gpio));
	}

	//board_control_spi_sensors_power_configgpio();

	/* configure LEDs */
	//board_autoled_initialize();
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
	/* Power on Interfaces */
	//stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, true);
	//stm32_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, false);
	//board_control_spi_sensors_power(true, 0xffff);

	px4_platform_init();

	stm32_spiinitialize();

	/* configure the DMA allocator */
	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	/* initial LED state */
	drv_led_start();
	led_init();

	if (board_hardfault_init(2, true) != 0) {
		//led_on(LED_AMBER);
	}

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{15, 128 * 1024, 0x081E0000},
		{0, 0, 0},
	};

	/* Initialize the flashfs layer to use heap allocated memory */
	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
	}

#endif // FLASH_BASED_PARAMS

#ifdef CONFIG_MMCSD
	/* Mount the SDIO-based MMC/SD block driver */
	/* First, get an instance of the SDIO interface */
	struct sdio_dev_s *sdio_dev = sdio_initialize(0); // SDIO_SLOTNO 0 Only one slot

	if (!sdio_dev) {
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n", 0);
	}

	if (mmcsd_slotinitialize(0, sdio_dev) != OK) {
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver\n");
	}

	/* Assume that the SD card is inserted.  What choice do we have? */
	sdio_mediachange(sdio_dev, true);
#endif /* CONFIG_MMCSD */

#ifdef CONFIG_USBHOST
	// Initialize USB host
	int ret = stm32_usbhost_initialize();

	if (ret < 0) {
		// Handle error
		syslog(LOG_ERR, "[boot] usbhost init failed: %d\n", ret);
	}

#endif

	//int fd = open("/dev/sda", O_RDONLY);
	//if (fd >= 0) {
	//    // Try to force a partition scan
	//    int ret = ioctl(fd, BIOC_PARTINFO, 0);
	//    syslog(LOG_INFO, "Partition scan result: %d (errno: %d)\n", ret, errno);
	//    close(fd);
	//}

	//struct stat st;
	//if (stat("/mnt", &st) < 0) {
	//    // Create the mount point if it doesn't exist
	//    mkdir("/mnt", 0777);
	//}

	//// Mount the USB storage
	//ret = mount("/dev/sda", "/mnt", "vfat", 0, NULL);
	//if (ret < 0) {
	//    // Handle mount error
	//	syslog(LOG_ERR, "[boot] mount failed: %d\n", ret);
	//syslog(LOG_ERR, "Mount failed: %d, errno: %d (%s)\n",
	//   ret, errno, strerror(errno));
	//}

	/* Configure the HW based on the manifest */
	px4_platform_configure();

	return OK;
}
