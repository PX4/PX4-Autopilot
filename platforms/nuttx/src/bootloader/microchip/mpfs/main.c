/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

/*
 * MPFS board support for the bootloader.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_defines.h>
#include <stdbool.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>

#include "hw_config.h"

#include "bl.h"
#include "uart.h"
#include "lib/flash_cache.h"

#include <nuttx/mtd/mtd.h>
#include "riscv_arch.h"

#include "image_toc.h"

#define MK_GPIO_INPUT(def) (((def) & (~(GPIO_OUTPUT)))  | GPIO_INPUT)

#define APP_SIZE_MAX			BOARD_FLASH_SIZE

// Reads/writes to flash are done in this size chunks
#define FLASH_RW_BLOCK 4096

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	INTERFACE_USART_CONFIG
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  INTERFACE_USB_CONFIG
#endif

#if defined(CONFIG_MTD_M25P)
static struct mtd_dev_s *mtd = 0;
static struct spi_dev_s *spinor = 0;

static bool device_flashed = false;
static uintptr_t end_address = 0;
static uintptr_t first_unwritten = 0;
static struct mtd_geometry_s geo;
#endif

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 480,
};

static void board_init(void);

// TODO
#define BOOT_RTC_SIGNATURE          0xb007b007
#define BOOT_RTC_REG                MMIO32(RTC_BASE + 0x50)

/* LED_ACTIVITY == 1, LED_BOOTLOADER == 2 */
static bool g_led_state[3];

/* State of an inserted USB cable */
static bool usb_connected = false;

static uint32_t board_get_rtc_signature(void)
{
	return 0;
}

static void
board_set_rtc_signature(uint32_t sig)
{
}

static bool board_test_force_pin(void)
{
	return false;
}

uint32_t
board_get_devices(void)
{
	uint32_t devices = BOOT_DEVICES_SELECTION;

	if (usb_connected) {
		devices &= BOOT_DEVICES_FILTER_ONUSB;
	}

	return devices;
}

#if defined(CONFIG_MPFS_SPI0)
#define SPI_NOR_CS_FUNC mpfs_spi0_select
#elif defined(CONFIG_MPFS_SPI1)
#define SPI_NOR_CS_FUNC mpfs_spi1_select
#endif

#if defined(BOARD_PIN_CS_SPINOR)
void
SPI_NOR_CS_FUNC(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	px4_arch_gpiowrite(BOARD_PIN_CS_SPINOR, !selected);
}
#endif

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_POWER_PIN_OUT)
	/* Configure the Power pins */
	//	px4_arch_configgpio(BOARD_POWER_PIN_OUT);
	//	px4_arch_gpiowrite(BOARD_POWER_PIN_OUT, BOARD_POWER_ON);
#endif

#if INTERFACE_USB
#if !defined(BOARD_USB_VBUS_SENSE_DISABLED)
	/* enable configured GPIO to sample VBUS */
#  if defined(USE_VBUS_PULL_DOWN)
	//	px4_arch_configgpio((GPIO_OTGFS_VBUS & GPIO_PUPD_MASK) | GPIO_PULLDOWN);
#  else
	//	px4_arch_configgpio((GPIO_OTGFS_VBUS & GPIO_PUPD_MASK) | GPIO_FLOAT);
#  endif
#endif
#endif

#if INTERFACE_USART
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	//	px4_arch_configgpio(BOARD_FORCE_BL_PIN_IN);
	//	px4_arch_configgpio(BOARD_FORCE_BL_PIN_OUT);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	//	px4_arch_configgpio(BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_ACTIVITY);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_BOOTLOADER);
#endif

#if defined(CONFIG_MMCSD)
	mpfs_board_emmcsd_init();
#endif

#if defined(CONFIG_MTD_M25P)
	px4_arch_configgpio(BOARD_PIN_CS_SPINOR);
	spinor = mpfs_spibus_initialize(1);
	mtd = m25p_initialize(spinor);

	if (mtd) {
		if (mtd->ioctl(mtd, MTDIOC_GEOMETRY,
			       (unsigned long)((uintptr_t)&geo))) {
		}
	} else {
		_alert("ERROR: MTD initialization failure!\n");
	}

#endif
}

void
board_deinit(void)
{

#if INTERFACE_USART
	up_disable_irq(MPFS_IRQ_MMUART0);
	up_disable_irq(MPFS_IRQ_MMUART1);
#endif

#if INTERFACE_USB
	//	px4_arch_configgpio(MK_GPIO_INPUT(GPIO_OTGFS_VBUS));
	//	putreg32(RCC_AHB1RSTR_OTGFSRST, STM32_RCC_AHB1RSTR);
#endif

#ifdef CONFIG_MTD_M25P
	mpfs_spibus_uninitialize(spinor);
#endif

#if defined(CONFIG_MMCSD)
	/* deinitialise the MMC/SD interrupt */
	up_disable_irq(MPFS_IRQ_MMC_MAIN);
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	//	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_IN));
	//	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_OUT));
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	//	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN));
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	//	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_POWER_PIN_OUT));
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	/* Initialize LEDs */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_ACTIVITY));
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	/* Initialize LEDs */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_BOOTLOADER));
#endif

#if defined(BOARD_PIN_CS_SPINOR)
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_CS_SPINOR));
#endif

}

static inline void
clock_init(void)
{
	// Done by Nuttx
}

void
clock_deinit(void)
{
	up_disable_irq(MPFS_IRQ_MTIMER);
}

void arch_flash_lock(void)
{
}

void arch_flash_unlock(void)
{
}

inline void arch_setvtor(const uint32_t *address)
{
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	size_t erasesize = mtd ? geo.erasesize : 4096;

	if (sector * erasesize < BOARD_FLASH_SIZE) {
		return erasesize;

	} else {
		return 0;
	}
}

void
flash_func_erase_sector(unsigned sector)
{

	unsigned ss = flash_func_sector_size(sector);

#ifdef CONFIG_MTD_M25P
	int ret = MTD_ERASE(mtd, sector, 1);

	if (ret < 0) {
		ferr("ERROR: Erase block=%u failed: %d\n",
		     sector, ret);
	}

#endif

	uint32_t *addr = (uint32_t *)((uint64_t)APP_LOAD_ADDRESS + sector * ss);
	memset(addr, 0xFFFFFFFF, ss);
}

#ifdef CONFIG_MTD_M25P
static void flash_write_pages(off_t start, unsigned n_pages, uint8_t *src)
{
	size_t ret = MTD_BWRITE(mtd, start, n_pages, src);

	if (ret != n_pages) {
		_alert("SPI NOR write error in pages %d-%d ret %d\n", start, start + n_pages, ret);
	}
}
#endif

void
flash_func_write_word(uintptr_t address, uint32_t word)
{
	/* Also copy it directly to load address for booting */
	uint32_t *app_load_addr = (uint32_t *)(address + APP_LOAD_ADDRESS);

	*app_load_addr = word;

#ifdef CONFIG_MTD_M25P

	// Write a single page every time we got a full one

	unsigned pgs_per_block = (FLASH_RW_BLOCK / geo.blocksize);

	if (address > 0 &&
	    ((address + sizeof(uint32_t)) % FLASH_RW_BLOCK) == 0) {

		// start of this block in memory
		uintptr_t block_start = ((uintptr_t)app_load_addr + sizeof(uint32_t)) - FLASH_RW_BLOCK;

		// first page to be written
		off_t write_page = (address / FLASH_RW_BLOCK) * pgs_per_block;

		// write pages
		flash_write_pages(write_page, pgs_per_block, (uint8_t *)block_start);

		device_flashed = true;

		first_unwritten = address + sizeof(uint32_t);
	}

	// update the end of the image
	if (address + sizeof(uint32_t) > end_address) {
		end_address = address + sizeof(uint32_t);
	}

#endif

}

uint32_t flash_func_read_word(uintptr_t address)
{
	uint32_t word;

	address += APP_LOAD_ADDRESS;
	word = *(uint32_t *)((uintptr_t)address);

	return word;
}


uint32_t
flash_func_read_otp(uintptr_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	return 0;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	revstr[0] = 1;
	revstr[1] = ',';
	revstr[2] = 0;
	return 3;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uintptr_t address)
{
	return 0                                        ;
}

void
led_on(unsigned led)
{
	g_led_state[led] = true;

	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, BOARD_LED_ON);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, BOARD_LED_ON);
#endif
		break;
	}
}

void
led_off(unsigned led)
{
	g_led_state[led] = false;

	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, BOARD_LED_OFF);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, BOARD_LED_OFF);
#endif
		break;
	}
}

void
led_toggle(unsigned led)
{
	g_led_state[led] ^= 1;

	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)

		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, g_led_state[led]);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, g_led_state[led]);
#endif
		break;
	}
}


/* Make the actual jump to app */
void
arch_do_jump(const uint32_t *app_base)
{

	/* Boot PX4 on hart 1 */
	*(volatile uint32_t *)MPFS_CLINT_MSIP1 = 0x01U;

	// TODO. monitor?
	while (1) {
		usleep(1000000);
	}
}

static size_t get_image_size(void)
{
	const image_toc_entry_t *toc_entries;
	const void *end = (const void *)APP_LOAD_ADDRESS;
	uint8_t len;

	if (find_toc(&toc_entries, &len)) {
		_alert("Found TOC\n");

		// find the largest end address

		for (int i = 0; i < len; i++) {
			if (toc_entries[i].end > end) {
				end = toc_entries[i].end;
			}
		}
	}

	// image size is end address - app start
	return (uintptr_t)end - APP_LOAD_ADDRESS;
}

int
bootloader_main(void)
{
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

#if defined(BOARD_POWER_PIN_OUT)

	/* Here we check for the app setting the POWER_DOWN_RTC_SIGNATURE
	 * in this case, we reset the signature and wait to die
	 */
	if (board_get_rtc_signature() == POWER_DOWN_RTC_SIGNATURE) {
		board_set_rtc_signature(0);

		while (1);
	}

#endif
	/* do board-specific initialisation */
	board_init();

	/* configure the clock for bootloader activity */
	clock_init();

	/*
	 * Check the force-bootloader register; if we find the signature there, don't
	 * try booting.
	 */
	if (board_get_rtc_signature() == BOOT_RTC_SIGNATURE) {

		/*
		 * Don't even try to boot before dropping to the bootloader.
		 */
		try_boot = false;

		/*
		 * Don't drop out of the bootloader until something has been uploaded.
		 */
		timeout = 0;

		/*
		 * Clear the signature so that if someone resets us while we're
		 * in the bootloader we'll try to boot next time.
		 */
		board_set_rtc_signature(0);
	}

	size_t image_sz = 0;

#ifdef CONFIG_MMCSD
	/*
	 * Mount the sdcard and check if the image is present
	 */
	int ret = mount("/dev/mmcsd0", "/sdcard/", "vfat", 0, NULL);

	if (ret >= 0) {
		int mmc_fd = open("/sdcard/boot/" IMAGE_FN, O_RDWR | O_CREAT);

		if (mmc_fd) {
			ret = read(mmc_fd, (void *)APP_LOAD_ADDRESS, BOARD_FLASH_SIZE);
			close(mmc_fd);

			if (ret > 0) {
				image_sz = get_image_size();

				if (image_sz > 0) {
					_alert("Booting from SD card\n");
				}
			}
		}

		umount("/sdcard");
	}

#endif

#ifdef CONFIG_MTD_M25P
	/* If loading from sdcard didn't succeed, use SPI-NOR (normal boot media) */

	// Read first FLASH_RW_BLOCK size data, search if TOC exists

	if (image_sz == 0) {
		const unsigned pgs_per_block = FLASH_RW_BLOCK / geo.blocksize;
		size_t pages = MTD_BREAD(mtd, 0, pgs_per_block, (uint8_t *)APP_LOAD_ADDRESS);

		if (pages == pgs_per_block) {
			image_sz = get_image_size();

			if (image_sz > 0) {
				_alert("Booting from NOR flash\n");
				unsigned reads_left = image_sz / FLASH_RW_BLOCK - 1;

				if (image_sz % FLASH_RW_BLOCK) {
					reads_left += 1;
				}

				// read the rest in FLASH_RW_BLOCK blocks
				for (unsigned i = 1; i < reads_left + 1; i++) {
					MTD_BREAD(mtd, i * pgs_per_block, pgs_per_block, ((uint8_t *)APP_LOAD_ADDRESS) + (i * FLASH_RW_BLOCK));
				}
			}
		}
	}

#endif

	if (image_sz == 0) {
		_alert("No boot image found\n");
	}

#ifdef BOOT_DELAY_ADDRESS
	{
		/*
		  if a boot delay signature is present then delay the boot
		  by at least that amount of time in seconds. This allows
		  for an opportunity for a companion computer to load a
		  new firmware, while still booting fast by sending a BOOT
		  command
		 */
		uint32_t sig1 = flash_func_read_word(BOOT_DELAY_ADDRESS);
		uint32_t sig2 = flash_func_read_word(BOOT_DELAY_ADDRESS + 4);

		if (sig2 == BOOT_DELAY_SIGNATURE2 &&
		    (sig1 & 0xFFFFFF00) == (BOOT_DELAY_SIGNATURE1 & 0xFFFFFF00)) {
			unsigned boot_delay = sig1 & 0xFF;

			if (boot_delay <= BOOT_DELAY_MAX) {
				try_boot = false;

				if (timeout < boot_delay * 1000) {
					timeout = boot_delay * 1000;
				}
			}
		}
	}
#endif

	/*
	 * Check if the force-bootloader pins are strapped; if strapped,
	 * don't try booting.
	 */
	if (board_test_force_pin()) {
		try_boot = false;
	}

#if INTERFACE_USB

	/*
	 * Check for USB connection - if present, don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
#if defined(BOARD_VBUS)

	if (px4_arch_gpioread(BOARD_VBUS) != 0) {
		usb_connected = true;
		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#else
	try_boot = false;

#endif
#endif

	/* Try to boot the app if we think we should just go straight there */
	if (try_boot) {
		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif
		/* try to boot immediately */
		jump_to_app();

		// If it failed to boot, reset the boot signature and stay in bootloader
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);

		/* booting failed, stay in the bootloader forever */
		timeout = 0;
	}


	/* start the interface */
#if INTERFACE_USART
	cinit(BOARD_INTERFACE_CONFIG_USART, USART);
#endif
#if INTERFACE_USB
	cinit(BOARD_INTERFACE_CONFIG_USB, USB);
#endif

	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

		/* set the boot-to-bootloader flag so that if boot fails on reset we will stop here */
#ifdef BOARD_BOOT_FAIL_DETECT
		board_set_rtc_signature(BOOT_RTC_SIGNATURE);
#endif

#ifdef CONFIG_MTD_M25P
		/* If device was just flashed, finalize flashing */

		if (device_flashed) {

			/* Write the residue */
			unsigned bytes = end_address - first_unwritten;
			unsigned n_pages = bytes / geo.blocksize;

			if (bytes % geo.blocksize) {
				n_pages += 1;
			}

			if (n_pages) {
				flash_write_pages(first_unwritten / geo.blocksize, n_pages,
						  (uint8_t *)(APP_LOAD_ADDRESS + first_unwritten));
			}

			/* Write first page again to update first word */
			flash_write_pages(0, 1, (uint8_t *)(APP_LOAD_ADDRESS));
		}

#endif

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
