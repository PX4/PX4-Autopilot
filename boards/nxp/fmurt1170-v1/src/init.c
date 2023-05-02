/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * NXP imxrt1170-v1 specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <barriers.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include "arm_internal.h"
#include "imxrt_flexspi_nor_boot.h"
#include "imxrt_flexspi_nor_flash.h"
#include "imxrt_romapi.h"
#include "imxrt_iomuxc.h"
#include "imxrt_flexcan.h"
#include "imxrt_enet.h"
#include <chip.h>
#include "board_config.h"

#include <hardware/imxrt_lpuart.h>
#undef FLEXSPI_LUT_COUNT
#include <hardware/imxrt_flexspi.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_platform/board_dma_alloc.h>

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

extern uint32_t _srodata;            /* Start of .rodata */
extern uint32_t _erodata;            /* End of .rodata */
extern const uint64_t _fitcmfuncs;   /* Copy source address in FLASH */
extern uint64_t _sitcmfuncs;         /* Copy destination start address in ITCM */
extern uint64_t _eitcmfuncs;         /* Copy destination end address in ITCM */
extern uint64_t _sdtcm;              /* Copy destination start address in DTCM */
extern uint64_t _edtcm;              /* Copy destination end address in DTCM */
__END_DECLS

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* set the peripheral rails off */

	VDD_5V_PERIPH_EN(false);
	VDD_5V_HIPOWER_EN(false);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	syslog(LOG_DEBUG, "reset done, %d ms", ms);

	/* re-enable power */

	/* switch the peripheral rail back on */
	VDD_5V_HIPOWER_EN(true);
	VDD_5V_PERIPH_EN(true);

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
		px4_arch_configgpio(PX4_MAKE_GPIO_INPUT(io_timer_channel_get_gpio_output(i)));
	}

	if (status >= 0) {
		up_mdelay(6);
	}
}

#if defined(CONFIG_BOARD_BOOTLOADER_FIXUP)
/****************************************************************************
 * Name: imxrt_octl_flash_initialize
 *
 * Description:
 *
 ****************************************************************************/

locate_code(".ramfunc")
void imxrt_octl_flash_initialize(void)
{
	const uint32_t instance =  1;

	volatile struct flexspi_nor_config_s bootConfig;

	memcpy((struct flexspi_nor_config_s *)&bootConfig, &g_flash_fast_config,
	       sizeof(struct flexspi_nor_config_s));
	bootConfig.memConfig.tag = FLEXSPI_CFG_BLK_TAG;

	ROM_API_Init();

	ROM_FLEXSPI_NorFlash_Init(instance, (struct flexspi_nor_config_s *)&bootConfig);

	ARM_DSB();
	ARM_ISB();
	ARM_DMB();
}
#endif

locate_code(".ramfunc")
void imxrt_flash_setup_prefetch_partition(void)
{
	putreg32((uint32_t)&_srodata, IMXRT_FLEXSPI1_AHBBUFREGIONSTART0);
	putreg32((uint32_t)&_erodata, IMXRT_FLEXSPI1_AHBBUFREGIONEND0);
	putreg32((uint32_t)&_stext, IMXRT_FLEXSPI1_AHBBUFREGIONSTART1);
	putreg32((uint32_t)&_etext, IMXRT_FLEXSPI1_AHBBUFREGIONEND1);

	struct flexspi_type_s *g_flexspi = (struct flexspi_type_s *)IMXRT_FLEXSPIC_BASE;
	/* RODATA */
	g_flexspi->AHBRXBUFCR0[0] = FLEXSPI_AHBRXBUFCR0_BUFSZ(128) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(7) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(1);


	/* All Text */
	g_flexspi->AHBRXBUFCR0[1] = FLEXSPI_AHBRXBUFCR0_BUFSZ(380) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(7) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(1);
	/* Reset CR7 from rom init */
	g_flexspi->AHBRXBUFCR0[7] = FLEXSPI_AHBRXBUFCR0_BUFSZ(0) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(0);

	ARM_DSB();
	ARM_ISB();
	ARM_DMB();
}
/****************************************************************************
 * Name: imxrt_ocram_initialize
 *
 * Description:
 *   Called off reset vector to reconfigure the flexRAM
 *   and finish the FLASH to RAM Copy.
 *
 ****************************************************************************/

__EXPORT void imxrt_ocram_initialize(void)
{
	uint32_t regval;
	register uint64_t *src;
	register uint64_t *dest;

	/* Reallocate
	 * Final Configuration is
	 *    No DTCM
	 *    512k  OCRAM M7 (FlexRAM)          (2038:0000-203f:ffff)
	*    128k  OCRAMM7 FlexRAM ECC         (2036:0000-2037:ffff)
	*    64k   OCRAM2 ECC parity           (2035:0000-2035:ffff)
	*    64k   OCRAM1 ECC parity           (2034:0000-2034:ffff)
	*    512k  FlexRAM OCRAM2              (202C:0000-2033:ffff)
	 *    512k  FlexRAM OCRAM1              (2024:0000-202B:ffff)
	 *    256k  System  OCRAM M4            (2020:0000-2023:ffff)
	 */

	putreg32(0x0000FFAA, IMXRT_IOMUXC_GPR_GPR17);
	putreg32(0x0000FFAA, IMXRT_IOMUXC_GPR_GPR18);
	regval = getreg32(IMXRT_IOMUXC_GPR_GPR16);
	putreg32(regval | GPR_GPR16_FLEXRAM_BANK_CFG_SEL_REG, IMXRT_IOMUXC_GPR_GPR16);

	/* Copy any necessary code sections from FLASH to ITCM. The process is the
	* same as the code copying from FLASH to RAM above. */
	for (src = (uint64_t *)&_fitcmfuncs, dest = (uint64_t *)&_sitcmfuncs;
	     dest < (uint64_t *)&_eitcmfuncs;) {
		*dest++ = *src++;
	}

	/* Clear .dtcm.  We'll do this inline (vs. calling memset) just to be
	* certain that there are no issues with the state of global variables.
	*/

	for (dest = &_sdtcm; dest < &_edtcm;) {
		*dest++ = 0;
	}

#if defined(CONFIG_BOOT_RUNFROMISRAM)
	const uint32_t *src;
	uint32_t *dest;

	for (src = (uint32_t *)(LOCATE_IN_SRC(g_boot_data.start) + g_boot_data.size),
	     dest = (uint32_t *)(g_boot_data.start + g_boot_data.size);
	     dest < (uint32_t *) &_etext;) {
		*dest++ = *src++;
	}

#endif
}

/****************************************************************************
 * Name: imxrt_boardinitialize
 *
 * Description:
 *   All i.MX RT architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ****************************************************************************/

__EXPORT void imxrt_boardinitialize(void)
{

#if defined(CONFIG_BOARD_BOOTLOADER_FIXUP)
	imxrt_octl_flash_initialize();
#endif

	imxrt_flash_setup_prefetch_partition();

	board_on_reset(-1); /* Reset PWM first thing */

	/* configure LEDs */

	board_autoled_initialize();

	/* configure pins */

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	imxrt_usb_initialize();

	fmurt107x_timer_initialize();
	VDD_3V3_ETH_POWER_EN(true);
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
volatile bool g_debug_loop_on_fault = true;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	VDD_3V3_SD_CARD_EN(true);
	VDD_5V_PERIPH_EN(true);
	VDD_5V_HIPOWER_EN(true);
	VDD_3V3_SPEKTRUM_POWER_EN(true);

	/*
	 *  We have BOARD_I2C_LATEINIT Defined to hold off the I2C init
	 * To enable SE050 driveHW_VER_REV_DRIVE low. But we have to ensure the
	 * EEROM version can be read first.
	 * Power on sequence:
	 * 1) Drive I2C4 lines to output low (avoid backfeeding SE050)
	 * 2) DoHWversioning withVDD_3V3_SENSORS4 off. LeaveHW_VER_REV_DRIVE high (SE050 disabled) on exit.
	 * 3) Then set HW_VER_REV_DRIVE low (SE050 enabled).
	 * 4) Then power onVDD_3V3_SENSORS4.
	 * 5) HW_VER_REV_DRIVE can be used to toggle SE050_ENAlater if needed.
	 */


	/* Step 1 */

	px4_arch_gpiowrite(GPIO_LPI2C3_SCL, 0);
	px4_arch_gpiowrite(GPIO_LPI2C3_SDA, 0);
	px4_arch_gpiowrite(GPIO_HW_VER_REV_DRIVE, 1);
	VDD_3V3_SENSORS4_EN(true);

	int ret = OK;

	/* Need hrt running before using the ADC */

	px4_platform_init();

	// Use the default HW_VER_REV(0x0,0x0) for Ramtron

	imxrt_spiinitialize();

	/* Configure the HW based on the manifest
	 * This will use I2C busses so VDD_3V3_SENSORS4_EN
	 * needs to be up.
	 */

	px4_platform_configure();

	/* Step 2 */

	if (OK == board_determine_hw_info()) {
		syslog(LOG_INFO, "[boot] Rev 0x%1x : Ver 0x%1x %s\n", board_get_hw_revision(), board_get_hw_version(),
		       board_get_hw_type_name());

	} else {
		syslog(LOG_ERR, "[boot] Failed to read HW revision and version\n");
	}

	/* Step 3 reset the SE550
	 * Power it down, prevetn back feeding
	 * and let it settle
	 */

	VDD_3V3_SENSORS4_EN(false);
	px4_arch_gpiowrite(GPIO_LPI2C3_SCL, 0);
	px4_arch_gpiowrite(GPIO_LPI2C3_SDA, 0);
	px4_arch_gpiowrite(GPIO_HW_VER_REV_DRIVE, 1);

	usleep(125000);

	/* Step 4 */

	VDD_3V3_SENSORS4_EN(true);
	px4_arch_configgpio(GPIO_LPI2C3_SCL);
	px4_arch_configgpio(GPIO_LPI2C3_SDA);

	/* Enable the SE550 */

	px4_arch_gpiowrite(GPIO_HW_VER_REV_DRIVE, 0);


	/* Do the I2C init late BOARD_I2C_LATEINIT */

	px4_platform_i2c_init();

	/* Configure the Actual SPI interfaces (after we determined the HW version)  */

	imxrt_spiinitialize();

	board_spi_reset(10, 0xffff);

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#if defined(SERIAL_HAVE_RXDMA) // Needed???
	// set up the serial DMA polling at 1ms intervals for received bytes that have not triggered a DMA event.
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)imxrt_serial_dma_poll, NULL);
#endif

	/* initial LED state */
	drv_led_start();

	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

#if defined(CONFIG_IMXRT_USDHC)
	ret = fmurt1170_usdhc_initialize();

	if (ret != OK) {
		led_on(LED_RED);
	}

#endif

#ifdef CONFIG_IMXRT_ENET
	imxrt_netinitialize(0);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN1
	imxrt_caninitialize(1);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN2
	imxrt_caninitialize(2);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
	imxrt_caninitialize(3);
#endif

	return ret;
}
