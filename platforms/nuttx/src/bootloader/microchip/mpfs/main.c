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
#include <stdio.h>

#include "hw_config.h"
#include "board_type.h"

#include "bl.h"
#include "uart.h"
#include "lib/flash_cache.h"
#include "riscv_internal.h"

#include <nuttx/mtd/mtd.h>
#include <nuttx/board.h>

#include <mpfs_entrypoints.h>

#include <px4_arch/device_info.h>

#include "image_toc.h"
#include "crypto.h"

extern int sercon_main(int c, char **argv);

#if defined(CONFIG_OPENSBI)
extern void mpfs_opensbi_relocate_from_envm(void);
#endif

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

/* Reset reasons */

/* Reset was caused by the SCB periphery reset signal*/
#define RESET_SR_SCB_PERIPH_RESET_MASK                       (0x01 << 0x0)

/* Reset was caused by the SCB MSS reset register*/
#define RESET_SR_SCB_MSS_RESET_MASK                          (0x01 << 0x1)

/* Reset was caused by the SCB CPU reset register*/
#define RESET_SR_SCB_CPU_RESET_MASK                          (0x01 << 0x2)

/* Reset was caused by the Risc-V Debugger*/
#define RESET_SR_DEBUGGER_RESET_MASK                          (0x01 << 0x3)

/* Reset was caused by the fabric*/
#define RESET_SR_FABRIC_RESET_MASK                           (0x01 << 0x4)

/* Reset was caused by the watchdog*/
#define RESET_SR_WDOG_RESET_MASK                             (0x01 << 0x5)

/* Indicates that fabric asserted the GPIO reset inputs*/
#define RESET_SR_GPIO_RESET_MASK                             (0x01 << 0x6)

/* Indicates that SCB bus reset occurred (which causes warm reset of MS
   S)*/
#define RESET_SR_SCB_BUS_RESET_MASK                          (0x01 << 0x7)

/* Indicates that CPU soft reset occured */
#define RESET_SR_CPU_SOFT_RESET_MASK                         (0x01 << 0x8)

#ifdef CONFIG_MMCSD
static int px4_fd = -1;
static bool sdcard_mounted;
#endif


#if BOOTLOADER_VERIFY_UBOOT

#define UBOOT_BINARY		"u-boot_signed.bin"
#define UBOOT_SIGNATURE_SIZE	64
#else

#define UBOOT_BINARY		"u-boot.bin"
#endif

static struct mtd_dev_s *mtd = 0;
static struct mtd_geometry_s geo;

#if defined(CONFIG_MTD_M25P)
static struct spi_dev_s *spinor = 0;
#endif

static int loader_task = -1;
typedef enum {
	UNINITIALIZED = -1,
	IN_PROGRESS,
	DONE,
	SW_UPDATED,
	INTERRUPTED,
	LOAD_FAIL
} image_loading_status_t;

static image_loading_status_t loading_status;
static bool u_boot_loaded = false;

/* board definition */
struct boardinfo board_info = {
	.board_type	= BOARD_TYPE,
	.board_rev	= 0,
	.fw_size	= 0,
	.systick_mhz	= 600,
};

static void board_init(void);

/* LED_ACTIVITY == 1, LED_BOOTLOADER == 2 */
static bool g_led_state[3];

/* State of an inserted USB cable */
static bool usb_connected = false;

devinfo_t device_info __attribute__((section(".deviceinfo")));

/* PX4 image TOC 'reserved' field for vendor specific info_bits
 * Bit 0 marks for whether SBI should be used or not
*/

static uint8_t info_bits;

static uint32_t board_get_reset_reason(void)
{
	return getreg32(MPFS_SYSREG_BASE + MPFS_SYSREG_RESET_SR_OFFSET);
}

static void board_set_reset_reason(uint32_t reason)
{
	putreg32(reason, MPFS_SYSREG_BASE + MPFS_SYSREG_RESET_SR_OFFSET);
}

static bool board_test_force_pin(void)
{
#ifdef BOARD_FORCE_BL_PIN

	if (px4_arch_gpioread(BOARD_FORCE_BL_PIN)) {
		return true;
	}

#endif
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

#ifdef CONFIG_MMCSD

static ssize_t load_sdcard_images(const char *name, uint64_t loadaddr)
{
	struct stat file_stat;

	int mmcsd_fd = open(name, O_RDONLY);

	if (mmcsd_fd > 0) {
		fstat(mmcsd_fd, &file_stat);
		size_t got = read(mmcsd_fd, (void *)loadaddr, file_stat.st_size);

		if (got > 0 && got == (size_t)file_stat.st_size) {
			_alert("Loading %s OK\n", name);
			close(mmcsd_fd);
			return got;
		}
	}

	_alert("Loading %s failed\n", name);
	close(mmcsd_fd);
	return -1;
}
#endif

static void
board_init(void)
{
	/* fix up the max firmware size, we have to read memory to get this */
	board_info.fw_size = APP_SIZE_MAX;

#if defined(BOARD_FORCE_BL_PIN)
	/* configure the force BL pins */
	px4_arch_configgpio(BOARD_FORCE_BL_PIN);
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_ACTIVITY);
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	/* Initialize LEDs */
	px4_arch_configgpio(BOARD_PIN_LED_BOOTLOADER);
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

#if defined(CONFIG_USBDEV)
	mpfs_usbinitialize();
#  if defined(CONFIG_SYSTEM_CDCACM)
	sercon_main(0, NULL);
#  endif
#endif

#if defined(CONFIG_MMCSD)

	if (mpfs_board_emmcsd_init() == OK) {

#  if defined(CONFIG_USBMSC_COMPOSITE)

		if (board_composite_initialize(0) == OK) {
			if (board_composite_connect(0, 0) == NULL) {
				_alert("Failed to connect composite\n");
			}

		} else {
			_alert("Failed to initialize composite\n");
		}

#  endif

		if (mpfs_board_register_partition(0) == OK) {

			/* Mount the sdcard/eMMC */
			sdcard_mounted = mount("/dev/mmcsd0p0", "/sdcard/", "vfat", 0, NULL) == 0 ? true : false;

			if (!sdcard_mounted) {
				_alert("SD/eMMC mount failed\n");

			} else {
				_alert("SD/eMMC mounted\n");
			}
		}

	} else {
		_alert("ERROR: Failed to initialize SD card");
	}

#endif

}

void
board_deinit(void)
{

#if INTERFACE_USART
#ifdef CONFIG_MPFS_FPGA_UART
	up_disable_irq(MPFS_IRQ_FABRIC_F2H_12);
	up_disable_irq(MPFS_IRQ_FABRIC_F2H_13);
#else
	up_disable_irq(MPFS_IRQ_MMUART0);
	up_disable_irq(MPFS_IRQ_MMUART1);
#endif
#endif

#ifdef CONFIG_MMCSD

	/* Umount the sdcard now if mounted */
	if (sdcard_mounted) {
		umount("/sdcard");
	}

	/* If this is left open from flashing, close it now */
	close(px4_fd);

	/* deinitialise the MMC/SD interrupt */
	up_disable_irq(MPFS_IRQ_MMC_MAIN);
#endif

#ifdef CONFIG_MTD_M25P
	mpfs_spibus_uninitialize(spinor);
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN));
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

#if defined(CONFIG_USBDEV)
	up_disable_irq(MPFS_IRQ_USB_MC);
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
	up_disable_irq(RISCV_IRQ_MTIMER);
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

inline static uint32_t
flash_func_block_size(void)
{
	return mtd ? geo.blocksize : 4096;
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

#ifdef CONFIG_MMCSD
static void create_px4_file(void)
{
	if (sdcard_mounted) {
		px4_fd = open("/sdcard/boot/" IMAGE_FN, O_RDWR | O_CREAT | O_TRUNC);

		/* Couldn't open the file, make sure that the directory exists and try to re-open */

		if (px4_fd < 0) {
			int ret = mkdir("/sdcard/boot", S_IRWXU | S_IRWXG | S_IRWXO);

			if (ret < 0) {
				_alert("boot directory creation failed %d\n", ret);
			}

			px4_fd = open("/sdcard/boot/" IMAGE_FN,  O_RDWR | O_CREAT | O_TRUNC, 0644);
		}

		if (px4_fd < 0) {
			_alert("FATAL: Not able to create px4 fw image!\n");

		}
	}
}
#endif

void
flash_func_erase_sector(unsigned sector)
{

	unsigned ss = flash_func_sector_size(sector);
	uint64_t *addr = (uint64_t *)((uint64_t)APP_LOAD_ADDRESS + sector * ss);

	/**
	 * Break any loading process
	 *
	 * Wait for loading to stop. Loading always ends up with either
	 * LOAD_FAIL or DONE, and any other values are intermediate
	 */

	irqstate_t flags = px4_enter_critical_section();

	if (loading_status != LOAD_FAIL && loading_status != DONE) {
		loading_status = INTERRUPTED;
	}

	px4_leave_critical_section(flags);

	while (loading_status != LOAD_FAIL && loading_status != DONE) {
		usleep(1000);
	}

#ifdef CONFIG_MTD_M25P
	/* Flash into NOR flash */

	int ret = MTD_ERASE(mtd, sector, 1);

	if (ret < 0) {
		ferr("ERROR: Erase block=%u failed: %d\n",
		     sector, ret);
	}

#endif

#if defined(CONFIG_MMCSD)
	/* Flashing into eMMC/SD */

	/* Create the file when erasing the first sector */
	if (sector == 0) {
		create_px4_file();
	}

#endif

	/* Erase the RAM contents */

	memset(addr, 0xFFFFFFFF, ss);

}

#if defined(CONFIG_MTD_M25P) || defined(CONFIG_MMCSD)
static ssize_t flash_write_pages(off_t start, unsigned n_pages, uint8_t *src)
{
	ssize_t ret = 0;
#ifdef CONFIG_MTD_M25P

	ret = MTD_BWRITE(mtd, start, n_pages, src);

	if (ret != n_pages) {
		_alert("SPI NOR write error in pages %d-%d\n", start, start + n_pages);
		ret = -errno;
	}

#elif defined(CONFIG_MMCSD)

	if (!sdcard_mounted) {
		return -EBADF;
	}

	/* Write to file, from the app_load_address */
	ret = (ssize_t)lseek(px4_fd, start * flash_func_block_size(), SEEK_SET);

	if (ret >= 0) {
		ssize_t bytes = n_pages * flash_func_block_size();

		ret = write(px4_fd, (void *)src, bytes);

		if (ret != bytes) {
			ret = -errno;
			_alert("eMMC write error at 0x%lx-0x%lx\n", start * flash_func_block_size(),
			       (start + n_pages) * flash_func_block_size());
		}

	} else {
		_alert("File lseek fail\n");
		ret = -errno;
	}

#endif
	return ret;
}
#endif

void
flash_func_write_word(uintptr_t address, uint32_t word)
{
	/* Also copy it directly to load address for booting */
	uint32_t *app_load_addr = (uint32_t *)(address + APP_LOAD_ADDRESS);

	*app_load_addr = word;

#if defined(CONFIG_MTD_M25P) || defined(CONFIG_MMCSD)

	static uintptr_t end_address = 0;
	static uintptr_t first_unwritten = 0;

	// start of this block in memory
	uint8_t *block_start;

	// total bytes to be written
	unsigned bytes;

	int ret = 0;

	if (address > 0 &&
	    ((address + sizeof(uint32_t)) % FLASH_RW_BLOCK) == 0) {
		// Every time a full FLASH_RW_BLOCK is received, store it to disk

		block_start = ((uint8_t *)app_load_addr + sizeof(uint32_t)) - FLASH_RW_BLOCK;
		bytes = FLASH_RW_BLOCK;

		// first page to be written
		off_t write_page = address / flash_func_block_size();
		// total pages to be written
		unsigned n_pages = (FLASH_RW_BLOCK / flash_func_block_size());
		// write pages
		ret = flash_write_pages(write_page, n_pages, (uint8_t *)block_start);
		// store the first unwritten address for the end of image handling
		first_unwritten = address + sizeof(uint32_t);
	}

	// the address 0 is written last, in the end of flashing
	if (address == 0 && word != 0xffffffffu) {
		// Write the last incomplete block
		bytes = end_address - first_unwritten;
		unsigned n_pages = bytes / flash_func_block_size();

		if (bytes % flash_func_block_size()) {
			n_pages += 1;
		}

		if (n_pages) {
			block_start = (uint8_t *)(APP_LOAD_ADDRESS + first_unwritten);
			ret = flash_write_pages(first_unwritten / flash_func_block_size(), n_pages,
						block_start);
		}

		// re-write the first page
		if (ret >= 0) {
			block_start = (uint8_t *)APP_LOAD_ADDRESS;
			bytes = flash_func_block_size();
			ret = flash_write_pages(0, 1, block_start);
		}
	}

	// if the writing failed, erase the written data in DRAM, and also
	// the first word, in case this was the last write
	if (ret < 0) {
		memset((uint8_t *)block_start, 0xff, bytes);
		*(uint32_t *)APP_LOAD_ADDRESS = 0xffffffffu;
	}

	end_address = address + sizeof(uint32_t);
#endif

	/* After the last word has been written, update the loading_status */
	if (address == 0 && word != 0xffffffffu) {
#ifdef CONFIG_MMCSD
		close(px4_fd);
		px4_fd = -1;
#endif
		loading_status = SW_UPDATED;
	}
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

	/* PX4 on hart 2 */
#if CONFIG_MPFS_HART2_ENTRYPOINT != 0xFFFFFFFFFFFFFFFF
	bool use_sbi = info_bits & INFO_BIT_USE_SBI ? true : false;
	mpfs_set_use_sbi(2, use_sbi);
	mpfs_set_entrypt(2, (uintptr_t)app_base);
	*(volatile uint32_t *)MPFS_CLINT_MSIP2 = 0x01U;
#endif

	/* Linux on harts 1, 3 and 4 */
	if (u_boot_loaded) {
#if CONFIG_MPFS_HART3_ENTRYPOINT != 0xFFFFFFFFFFFFFFFF
		*(volatile uint32_t *)MPFS_CLINT_MSIP3 = 0x01U;
#endif

// If SMP is used on Linux, the primary core (Hart 3) will boot the secondary
// cores (Hart 1 and 4), so they should not be booted here
#if BOOTLOADER_BOOT_HART_1
#if CONFIG_MPFS_HART1_ENTRYPOINT != 0xFFFFFFFFFFFFFFFF
		*(volatile uint32_t *)MPFS_CLINT_MSIP1 = 0x01U;
#endif
#endif

#if BOOTLOADER_BOOT_HART_4
#if CONFIG_MPFS_HART4_ENTRYPOINT != 0xFFFFFFFFFFFFFFFF
		*(volatile uint32_t *)MPFS_CLINT_MSIP4 = 0x01U;
#endif
#endif

	}

	// TODO. monitor?
	while (1) {
		usleep(1000000);
	}
}

#if defined(CONFIG_MMCSD) || defined(CONFIG_MTD_M25P)
static size_t get_image_size(void)
{
	const image_toc_entry_t *toc_entries;
	const void *end = (const void *)APP_LOAD_ADDRESS;
	uint8_t len;

	if (find_toc(&toc_entries, &len)) {
		_alert("Found TOC\n");

		// find the boot image vendor flags and the largest end address
		const uint32_t sig = ('B' << 0) | ('O' << 8) | ('O' << 16) | ('T' << 24);

		for (int i = 0; i < len; i++) {
			if (*(uint32_t *)(void *)toc_entries[i].name == sig) {
				info_bits = toc_entries[i].flags2;
			}

			if (toc_entries[i].end > end) {
				end = toc_entries[i].end;
			}
		}
	}

	// image size is end address - app start
	return (uintptr_t)end - APP_LOAD_ADDRESS;
}
#endif

#if BOOTLOADER_VERIFY_UBOOT
bool verify_image(void *image_start, size_t image_size, size_t signature_size)
{
	uint8_t signature_idx = 1;
	uint16_t index = 0;

	uint8_t *image_end = (uint8_t *)image_start + image_size - signature_size;
	uint8_t *signature_start = (uint8_t *)image_start + image_size - signature_size;
	uint8_t *signature_end = (uint8_t *)image_start + image_size;

	image_toc_entry_t toc_entries[2] = {
		{"IMG ", image_start, image_end, 0, signature_idx, 0, 0, 0},
		{"SIG ", signature_start, signature_end, 0, 0, 0, 0, 0}
	};

	return verify_app(index, toc_entries);
}
#endif

static int loader_main(int argc, char *argv[])
{
	ssize_t image_sz = 0;
	loading_status = IN_PROGRESS;

#if defined(CONFIG_MMCSD)
	ssize_t ret = 0;

	if (sdcard_mounted) {
		ret = load_sdcard_images("/sdcard/boot/" IMAGE_FN, APP_LOAD_ADDRESS);

		if (ret > 0) {
			image_sz = get_image_size();

			if (image_sz > 0) {
				_alert("PX4 load success\n");
			}
		}
	}

#endif

#if defined(CONFIG_MTD_M25P)
	/* If loading from sdcard didn't succeed, use SPI-NOR */

	// Read first FLASH_RW_BLOCK size data, search if TOC exists

	if (image_sz == 0) {
		const unsigned pgs_per_block = FLASH_RW_BLOCK / flash_func_block_size();
		size_t pages = MTD_BREAD(mtd, 0, pgs_per_block, (uint8_t *)APP_LOAD_ADDRESS);

		if (pages == pgs_per_block) {
			image_sz = get_image_size();

			if (image_sz > 0) {
				_alert("Loading from NOR flash\n");
				unsigned reads_left = image_sz / FLASH_RW_BLOCK - 1;

				if (image_sz % FLASH_RW_BLOCK) {
					reads_left += 1;
				}

				// read the rest in FLASH_RW_BLOCK blocks
				for (unsigned i = 1; i < reads_left + 1; i++) {
					if (loading_status == INTERRUPTED) {
						image_sz = -1;
						break;
					}

					MTD_BREAD(mtd, i * pgs_per_block, pgs_per_block, ((uint8_t *)APP_LOAD_ADDRESS) + (i * FLASH_RW_BLOCK));
				}
			}
		}
	}

#endif

#if defined(CONFIG_OPENSBI) && defined(CONFIG_MMCSD)
	ssize_t uboot_size = 0;

	/* Relocate code from eNVM into L2 zero device */

	mpfs_opensbi_relocate_from_envm();

	if (sdcard_mounted) {
		uboot_size = load_sdcard_images("/sdcard/boot/"UBOOT_BINARY, CONFIG_MPFS_HART3_ENTRYPOINT);

		if (uboot_size > 0) {
			u_boot_loaded = true;
#if BOOTLOADER_VERIFY_UBOOT

			if (!verify_image((void *)CONFIG_MPFS_HART3_ENTRYPOINT, uboot_size, UBOOT_SIGNATURE_SIZE)) {
				u_boot_loaded = false;
				/* Wipe the memory */
				memset((void *)CONFIG_MPFS_HART3_ENTRYPOINT, 0, uboot_size);
				_alert("u-boot Authentication Failed\n");
			}

#endif

		} else {
			_alert("u-boot loading failed\n");
			u_boot_loaded = false;
		}
	}

#endif

	/* image_sz < 0 means that the load was interrupted due to flashing in progress */
	if (image_sz == 0) {
		_alert("No boot image found\n");
		loading_status = LOAD_FAIL;

	} else if (loading_status == IN_PROGRESS) {
		_alert("Image loaded succesfully, size %ld\n", image_sz);
		loading_status = DONE;

	} else {
		_alert("Image loading interrupted\n");
		loading_status = LOAD_FAIL;
	}

	return 0;
}

static void start_image_loading(void)
{
	/* Mark that we are waiting for the task to start */
	loading_status = UNINITIALIZED;

	/* create the task */
	loader_task = task_create("loader", SCHED_PRIORITY_MAX - 6, 8192, loader_main, (char *const *)0);

	/* wait for the task to start */
	while (loading_status == UNINITIALIZED) {
		usleep(1000);
	}
}

int
bootloader_main(int argc, char *argv[])
{
	unsigned timeout = BOOTLOADER_DELAY;	 /* if nonzero, drop out of the bootloader after this time */
	bool try_boot;
	snprintf(device_info.bl_version, sizeof(device_info.bl_version), VERSION);
	_alert("Version: %s\n", device_info.bl_version);

	/* do board-specific initialisation */
	board_init();

	/* configure the clock for bootloader activity */
	clock_init();

	/* check the bootloader force pin status */
	try_boot = !board_test_force_pin();

	start_image_loading();

	/*
	 * Check the boot reason. In case we came here with SW system reset,
	 * we stay in bootloader.
	 * SW system reset is issued in PX4 with "reboot -b"
	 */

	uint32_t reset_reason = board_get_reset_reason();

	/* Is not FABRIC reset and not caused by WDOG? FABRIC reset bit is only
	 * set in POR, since it is not even connected in FPGA.
	 * If also CPU_SOFT_RESET is set, we know that this is soft reset from PX4 (reboot -b)
	 */
	if ((reset_reason & RESET_SR_FABRIC_RESET_MASK) == 0 &&
	    (reset_reason & RESET_SR_WDOG_RESET_MASK) == 0 &&
	    (reset_reason & RESET_SR_CPU_SOFT_RESET_MASK) == RESET_SR_CPU_SOFT_RESET_MASK) {

		/* Don't drop out of the bootloader until something has been uploaded */
		timeout = 0;
	}

	/* Clear the reset reason */
	board_set_reset_reason(0);

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

		/* if the sw was just re-flashed, load the image again */
		if (loading_status == SW_UPDATED) {
			timeout = BOOTLOADER_DELAY;
			start_image_loading();
		}

		/* look to see if we can boot the app */

		if (try_boot && loading_status == DONE) {
			jump_to_app();
		}
	}
}
