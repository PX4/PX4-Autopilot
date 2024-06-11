/*
 * imxrt board support for the bootloader.
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_defines.h>

#include "hw_config.h"
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#include <hardware/rt117x/imxrt117x_ocotp.h>
#include <hardware/rt117x/imxrt117x_anadig.h>
#include <hardware/rt117x/imxrt117x_snvs.h>
#include <hardware/imxrt_usb_analog.h>
#include "imxrt_clockconfig.h"

#include <nvic.h>
#include <mpu.h>
#include <lib/systick.h>
#include <lib/flash_cache.h>

#include "bl.h"
#include "uart.h"
#include "arm_internal.h"

#define MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK | GPIO_MODE_MASK)) | (GPIO_INPUT))

#define BOOTLOADER_RESERVATION_SIZE	(128 * 1024)

#define APP_SIZE_MAX			(BOARD_FLASH_SIZE - (BOOTLOADER_RESERVATION_SIZE + APP_RESERVATION_SIZE))

#define CHIP_TAG     "i.MX RT11?0,r??"
#define CHIP_TAG_LEN sizeof(CHIP_TAG)-1

#define SI_REV(n)             ((n & 0x7000000) >> 24)
#define DIFPROG_TYPE(n)       ((n & 0xF000) >> 12)
#define DIFPROG_REV_MAJOR(n)  ((n & 0xF0) >> 4)
#define DIFPROG_REV_MINOR(n)  ((n & 0xF))


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

#define BOOT_RTC_SIGNATURE          0xb007b007
#define PX4_IMXRT_RTC_REBOOT_REG    IMXRT_SNVS_LPGPR3

/* State of an inserted USB cable */
static bool usb_connected = false;

static uint32_t board_get_rtc_signature(void)
{
	uint32_t result = getreg32(PX4_IMXRT_RTC_REBOOT_REG);

	return result;
}

static void
board_set_rtc_signature(uint32_t sig)
{
	modifyreg32(IMXRT_SNVS_LPCR, 0, SNVS_LPCR_GPR_Z_DIS);

	putreg32(sig, PX4_IMXRT_RTC_REBOOT_REG);

}

static bool board_test_force_pin(void)
{
#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* two pins strapped together */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (volatile unsigned cycles = 0; cycles < 10; cycles++) {
		px4_arch_gpiowrite(BOARD_FORCE_BL_PIN_OUT, 1);

		for (unsigned count = 0; count < 20; count++) {
			if (px4_arch_gpioread(BOARD_FORCE_BL_PIN_IN) != 0) {
				vote++;
			}

			samples++;
		}

		px4_arch_gpiowrite(BOARD_FORCE_BL_PIN_OUT, 0);

		for (unsigned count = 0; count < 20; count++) {
			if (px4_arch_gpioread(BOARD_FORCE_BL_PIN_IN) == 0) {
				vote++;
			}

			samples++;
		}
	}

	/* the idea here is to reject wire-to-wire coupling, so require > 90% agreement */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
#if defined(BOARD_FORCE_BL_PIN)
	/* single pin pulled up or down */
	volatile unsigned samples = 0;
	volatile unsigned vote = 0;

	for (samples = 0; samples < 200; samples++) {
		if ((px4_arch_gpioread(BOARD_FORCE_BL_PIN) ? 1 : 0) == BOARD_FORCE_BL_STATE) {
			vote++;
		}
	}

	/* reject a little noise */
	if ((vote * 100) > (samples * 90)) {
		return true;
	}

#endif
	return false;
}

#if INTERFACE_USART
static bool board_test_usart_receiving_break(void)
{
#if !defined(SERIAL_BREAK_DETECT_DISABLED)
	/* (re)start the SysTick timer system */
	systick_interrupt_disable(); // Kill the interrupt if it is still active
	systick_counter_disable(); // Stop the timer
	systick_set_clocksource(CLKSOURCE_PROCESOR);

	/* Set the timer period to be half the bit rate
	 *
	 * Baud rate = 115200, therefore bit period = 8.68us
	 * Half the bit rate = 4.34us
	 * Set period to 4.34 microseconds (timer_period = timer_tick / timer_reset_frequency = 168MHz / (1/4.34us) = 729.12 ~= 729)
	 */
	systick_set_reload(729);  /* 4.3us tick, magic number */
	systick_counter_enable(); // Start the timer

	uint8_t cnt_consecutive_low = 0;
	uint8_t cnt = 0;

	/* Loop for 3 transmission byte cycles and count the low and high bits. Sampled at a rate to be able to count each bit twice.
	 *
	 * One transmission byte is 10 bits (8 bytes of data + 1 start bit + 1 stop bit)
	 * We sample at every half bit time, therefore 20 samples per transmission byte,
	 * therefore 60 samples for 3 transmission bytes
	 */
	while (cnt < 60) {
		// Only read pin when SysTick timer is true
		if (systick_get_countflag() == 1) {
			if (gpio_get(BOARD_PORT_USART_RX, BOARD_PIN_RX) == 0) {
				cnt_consecutive_low++;	// Increment the consecutive low counter

			} else {
				cnt_consecutive_low = 0; // Reset the consecutive low counter
			}

			cnt++;
		}

		// If 9 consecutive low bits were received break out of the loop
		if (cnt_consecutive_low >= 18) {
			break;
		}

	}

	systick_counter_disable(); // Stop the timer

	/*
	 * If a break is detected, return true, else false
	 *
	 * Break is detected if line was low for 9 consecutive bits.
	 */
	if (cnt_consecutive_low >= 18) {
		return true;
	}

#endif // !defined(SERIAL_BREAK_DETECT_DISABLED)

	return false;
}
#endif

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
	px4_arch_configgpio(BOARD_POWER_PIN_OUT);
	px4_arch_gpiowrite(BOARD_POWER_PIN_OUT, BOARD_POWER_ON);
#endif

#if INTERFACE_USB
#endif

#if INTERFACE_USART
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* configure the force BL pins */
	px4_arch_configgpio(BOARD_FORCE_BL_PIN_IN);
	px4_arch_configgpio(BOARD_FORCE_BL_PIN_OUT);
#endif

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
}

void
board_deinit(void)
{

#if INTERFACE_USART
#endif

#if INTERFACE_USB
#endif

#if defined(BOARD_FORCE_BL_PIN_IN) && defined(BOARD_FORCE_BL_PIN_OUT)
	/* deinitialise the force BL pins */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_IN));
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN_OUT));
#endif

#if defined(BOARD_FORCE_BL_PIN)
	/* deinitialise the force BL pin */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_FORCE_BL_PIN));
#endif

#if defined(BOARD_POWER_PIN_OUT) && defined(BOARD_POWER_PIN_RELEASE)
	/* deinitialize the POWER pin - with the assumption the hold up time of
	 * the voltage being bleed off by an inupt pin impedance will allow
	 * enough time to boot the app
	 */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_POWER_PIN_OUT));
#endif

#if defined(BOARD_PIN_LED_ACTIVITY)
	/* Initialize LEDs */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_ACTIVITY));
#endif
#if defined(BOARD_PIN_LED_BOOTLOADER)
	/* Initialize LEDs */
	px4_arch_configgpio(MK_GPIO_INPUT(BOARD_PIN_LED_BOOTLOADER));
#endif

	const uint32_t dnfw[] = {
		CCM_CR_M7,
		CCM_CR_BUS,
		CCM_CR_BUS_LPSR,
		CCM_CR_SEMC,
		CCM_CR_CSSYS,
		CCM_CR_CSTRACE,
		CCM_CR_FLEXSPI1,
		CCM_CR_FLEXSPI2
	};

	for (unsigned int i = 0; i < IMXRT_CCM_CR_COUNT; i++) {
		bool ok = true;

		for (unsigned int d = 0; ok && d < arraySize(dnfw); d++) {
			ok = dnfw[d] != i;
		}

		if (ok) {
			putreg32(CCM_CR_CTRL_OFF, IMXRT_CCM_CR_CTRL(i));
		}
	}
}

inline void arch_systic_init(void)
{
	// Done in NuttX
}

inline void arch_systic_deinit(void)
{
	/* kill the systick interrupt */
	irq_attach(IMXRT_IRQ_SYSTICK, NULL, NULL);
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_CLKSOURCE, 0);
}

/**
  * @brief  Initializes the RCC clock configuration.
  *
  * @param  clock_setup : The clock configuration to set
  */
static inline void
clock_init(void)
{
	// Done by Nuttx
}

/**
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  * @note   This function doesn't modify the configuration of the
  */
void
clock_deinit(void)
{
}

void arch_flash_lock(void)
{
}

void arch_flash_unlock(void)
{
	fc_reset();
}

ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen)
{
	struct flexspi_nor_config_s *pConfig = &g_bootConfig;
	irqstate_t flags = enter_critical_section();

	static volatile  int j = 0;
	j++;

	if (j == 6) {
		j++;
	}

	uintptr_t offset = ((uintptr_t) address) - IMXRT_FLEXSPI1_CIPHER_BASE;

	volatile uint32_t status = ROM_FLEXSPI_NorFlash_ProgramPage(1, pConfig, offset, (const uint32_t *)buffer);
	up_invalidate_dcache((uintptr_t)address,
			     (uintptr_t)address + buflen);


	leave_critical_section(flags);

	if (status == 100) {
		return buflen;
	}

	return 0;
}

inline void arch_setvtor(const uint32_t *address)
{
	putreg32((uint32_t)address, NVIC_VECTAB);
}

uint32_t
flash_func_sector_size(unsigned sector)
{
	if (sector <= BOARD_FLASH_SECTORS) {
		return 4 * 1024;
	}

	return 0;
}

/* imxRT uses Flash lib, not up_progmem so let's stub it here */
ssize_t up_progmem_ispageerased(unsigned sector)
{
	const uint32_t bytes_per_sector =  flash_func_sector_size(sector);
	uint32_t *address = (uint32_t *)(IMXRT_FLEXSPI1_CIPHER_BASE + (sector * bytes_per_sector));
	const uint32_t uint32_per_sector =  bytes_per_sector / sizeof(*address);

	int blank = 0; /* Assume it is Bank */

	for (uint32_t i = 0; i < uint32_per_sector; i++) {
		if (address[i] != 0xffffffff) {
			blank = -1;  /* It is not blank */
			break;
		}
	}

	return blank;
}

/*!
 * @name Configuration Option
 * @{
 */
/*! @brief Serial NOR Configuration Option. */


/*@}
 * */
locate_code(".ramfunc")
void
flash_func_erase_sector(unsigned sector, bool force)
{
	if (sector > BOARD_FLASH_SECTORS || (int)sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE) {
		return;
	}

	if (force || up_progmem_ispageerased(sector) != 0) {

		struct flexspi_nor_config_s *pConfig = &g_bootConfig;

		const uint32_t bytes_per_sector =  flash_func_sector_size(sector);
		uint32_t *address = (uint32_t *)(IMXRT_FLEXSPI1_CIPHER_BASE + (sector * bytes_per_sector));

		uintptr_t offset = ((uintptr_t) address) - IMXRT_FLEXSPI1_CIPHER_BASE;
		irqstate_t flags;
		flags = enter_critical_section();
		volatile uint32_t  status = ROM_FLEXSPI_NorFlash_Erase(1, pConfig, (uintptr_t) offset, bytes_per_sector);
		leave_critical_section(flags);
		UNUSED(status);
	}
}

void
flash_func_write_word(uintptr_t address, uint32_t word)
{
	address += APP_LOAD_ADDRESS;
	fc_write(address, word);
}

uint32_t flash_func_read_word(uintptr_t address)
{

	if (address & 3) {
		return 0;
	}

	return fc_read(address + APP_LOAD_ADDRESS);

}


uint32_t
flash_func_read_otp(uintptr_t address)
{
	return 0;
}

uint32_t get_mcu_id(void)
{
	// ??? is DEBUGMCU get able
	return *(uint32_t *) IMXRT_ANADIG_MISC_MISC_DIFPROG;
}

int get_mcu_desc(int max, uint8_t *revstr)
{
	uint32_t info = getreg32(IMXRT_ANADIG_MISC_MISC_DIFPROG);
	// CHIP_TAG     "i.MX RT11?0,r??"
	static uint8_t chip[sizeof(CHIP_TAG) + 1] = CHIP_TAG;
	chip[CHIP_TAG_LEN - 6] = '0' + DIFPROG_TYPE(info);
	chip[CHIP_TAG_LEN - 2] = 'A' + (DIFPROG_REV_MAJOR(info) - 10);
	chip[CHIP_TAG_LEN - 1] = '0' + DIFPROG_REV_MINOR(info);

	uint8_t *endp = &revstr[max - 1];
	uint8_t *strp = revstr;
	uint8_t *des = chip;

	while (strp < endp && *des) {
		*strp++ = *des++;
	}

	return  strp - revstr;
}


int check_silicon(void)
{
	return 0;
}

uint32_t
flash_func_read_sn(uintptr_t address)
{
	// Bootload has to uses 12 byte ID (3 Words)
	// but this IC has only 2 words
	// Address will be 0 4 8 - 3 words
	// so dummy up the last word....
	if (address > 4) {
		return 0x31313630;
	}

	return *(uint32_t *)((address * 4) + IMXRT_OCOTP_UNIQUE_ID_MSB);
}

void
led_on(unsigned led)
{
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
	switch (led) {
	case LED_ACTIVITY:
#if defined(BOARD_PIN_LED_ACTIVITY)
		px4_arch_gpiowrite(BOARD_PIN_LED_ACTIVITY, px4_arch_gpioread(BOARD_PIN_LED_ACTIVITY) ^ 1);
#endif
		break;

	case LED_BOOTLOADER:
#if defined(BOARD_PIN_LED_BOOTLOADER)
		px4_arch_gpiowrite(BOARD_PIN_LED_BOOTLOADER, px4_arch_gpioread(BOARD_PIN_LED_BOOTLOADER) ^ 1);
#endif
		break;
	}
}

/* we should know this, but we don't */
#ifndef SCB_CPACR
# define SCB_CPACR (*((volatile uint32_t *) (((0xE000E000UL) + 0x0D00UL) + 0x088)))
#endif

/* Make the actual jump to app */
void
arch_do_jump(const uint32_t *app_base)
{
	/* The MPU configuration after booting has ITCM set to MPU_RASR_AP_RORO
	 * We add this overlaping region to allow the Application to copy code into
	 * the ITCM when it is booted. With CONFIG_ARM_MPU_RESET defined. The mpu
	 * init will clear any added regions (after the copy)
	 */

	mpu_configure_region(IMXRT_ITCM_BASE, 256 * 1024,
			     /* Instruction access Enabled */
			     MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
			     MPU_RASR_TEX_NOR    /* Normal                     */
			     /* Not Cacheable              */
			     /* Not Bufferable             */
			     /* Not Shareable              */
			     /* No Subregion disable       */
			    );

	/* extract the stack and entrypoint from the app vector table and go */
	uint32_t stacktop = app_base[APP_VECTOR_OFFSET_WORDS];
	uint32_t entrypoint = app_base[APP_VECTOR_OFFSET_WORDS + 1];

	asm volatile(
		"msr msp, %0  \n"
		"bx %1  \n"
		: : "r"(stacktop), "r"(entrypoint) :);

	// just to keep noreturn happy
	for (;;) ;
}

int
bootloader_main(void)
{
	bool try_boot = true;			/* try booting before we drop to the bootloader */
	unsigned timeout = BOOTLOADER_DELAY;	/* if nonzero, drop out of the bootloader after this time */

	/* Enable the FPU before we hit any FP instructions */
	SCB_CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 Full Access and set CP11 Full Access */

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
	/************************************************************************************
	 * Name: board_read_VBUS_state
	 *
	 * Description:
	 *   All boards must provide a way to read the state of VBUS, this my be simple
	 *   digital input on a GPIO. Or something more complicated like a Analong input
	 *   or reading a bit from a USB controller register.
	 *
	 * Returns -  0 if connected.
	 *
	 ************************************************************************************/
#undef IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT
#define USB1_VBUS_DET_STAT_OFFSET               0xd0
#define IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT (IMXRT_USBPHY1_BASE + USB1_VBUS_DET_STAT_OFFSET)

	if ((getreg32(IMXRT_USB_ANALOG_USB1_VBUS_DETECT_STAT) & USB_ANALOG_USB_VBUS_DETECT_STAT_VBUS_3V_VALID) != 0) {
		usb_connected = true;
		/* don't try booting before we set up the bootloader */
		try_boot = false;
	}

#endif

#if INTERFACE_USART

	/*
	 * Check for if the USART port RX line is receiving a break command, or is being held low. If yes,
	 * don't try to boot, but set a timeout after
	 * which we will fall out of the bootloader.
	 *
	 * If the force-bootloader pins are tied, we will stay here until they are removed and
	 * we then time out.
	 */
	if (board_test_usart_receiving_break()) {
		try_boot = false;
	}

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


#if 0
	// MCO1/02
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF0, GPIO9);
#endif


	while (1) {
		/* run the bootloader, come back after an app is uploaded or we time out */
		bootloader(timeout);

		/* if the force-bootloader pins are strapped, just loop back */
		if (board_test_force_pin()) {
			continue;
		}

#if INTERFACE_USART

		/* if the USART port RX line is still receiving a break, just loop back */
		if (board_test_usart_receiving_break()) {
			continue;
		}

#endif

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
