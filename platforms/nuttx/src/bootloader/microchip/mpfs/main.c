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

#include "hw_config.h"

#include "bl.h"
#include "uart.h"
#include "lib/flash_cache.h"


#define MK_GPIO_INPUT(def) (((def) & (~(GPIO_OUTPUT)))  | GPIO_INPUT)

//#define BOOTLOADER_RESERVATION_SIZE	(128 * 1024)


#define APP_SIZE_MAX			BOARD_FLASH_SIZE

/* context passed to cinit */
#if INTERFACE_USART
# define BOARD_INTERFACE_CONFIG_USART	INTERFACE_USART_CONFIG
#endif
#if INTERFACE_USB
# define BOARD_INTERFACE_CONFIG_USB  INTERFACE_USB_CONFIG
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

ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen)
{
	//TODO:
	memcpy((void *)address, buffer, buflen);
	return 0;
}
inline void arch_setvtor(const uint32_t *address)
{
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	const uint32_t ss = 64 * 1024; // MT25QL01GBBB8E12-0AAT

	if (sector * ss < BOARD_FLASH_SIZE) {
		return ss;

	} else {
		return 0;
	}
}

void
flash_func_erase_sector(unsigned sector)
{
	//TODO
	unsigned ss = flash_func_sector_size(sector);
	uint64_t *addr = (uint64_t *)((uint64_t)APP_LOAD_ADDRESS + sector * ss);
	memset(addr, 0xFFFFFFFF, ss);
}

void
flash_func_write_word(uintptr_t address, uint32_t word)
{
	//TODO
	address += APP_LOAD_ADDRESS;
	*(uint32_t *)(uintptr_t)address = word;
	//	fc_write(address, word);
}

uint32_t flash_func_read_word(uintptr_t address)
{
	//TODO
	if (address & 3) {
		return 0;
	}

	address += APP_LOAD_ADDRESS;
	return *(uint32_t *)((uintptr_t)address);
	//	return fc_read(address + APP_LOAD_ADDRESS);
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

		/* look to see if we can boot the app */
		jump_to_app();

		/* launching the app failed - stay in the bootloader forever */
		timeout = 0;
	}
}
