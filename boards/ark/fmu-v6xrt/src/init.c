/****************************************************************************
 *
 *   Copyright (c) 2018-2019, 2023 PX4 Development Team. All rights reserved.
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
 * ARK V6X-RT specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "board_config.h"

#include <arch/barriers.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include "arm_internal.h"
#include "imxrt_flexspi_nor_boot.h"
#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include "imxrt_iomuxc.h"
#include "imxrt_flexcan.h"
#include "imxrt_enet.h"
#include <chip.h>

#include <hardware/imxrt_lpuart.h>
#undef FLEXSPI_LUT_COUNT
#include <hardware/imxrt_flexspi.h>

#include <arch/board/board.h>


#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_arch/imxrt_romapi.h>
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

	/*
	 * On resets invoked from system (not boot) ensure we establish a low
	 * output state on PWM pins to disarm the ESC and prevent the reset from potentially
	 * spinning up the motors.
	 */
	if (status >= 0) {
		up_mdelay(100);
	}
}

#if defined(CONFIG_BOARD_BOOTLOADER_FIXUP)
/****************************************************************************
 * Name: imxrt_octl_flash_initialize
 *
 * Description:
 *
 ****************************************************************************/
struct flexspi_nor_config_s g_bootConfig;


locate_code(".ramfunc")
void imxrt_octl_flash_initialize(void)
{
	const uint32_t instance =  1;


	memcpy((struct flexspi_nor_config_s *)&g_bootConfig, &g_flash_fast_config,
	       sizeof(struct flexspi_nor_config_s));
	g_bootConfig.memConfig.tag = FLEXSPI_CFG_BLK_TAG;

	ROM_API_Init();

	ROM_FLEXSPI_NorFlash_Init(instance, (struct flexspi_nor_config_s *)&g_bootConfig);
	ROM_FLEXSPI_NorFlash_ClearCache(1);

	arm_dsb();
	arm_isb();
	arm_dmb();
}
#endif

/****************************************************************************
 * FlexSPI DLL / DQS read-strobe calibration
 *
 * Corrects the ROM-provided DLL delay by finding the valid DQS sampling range
 * and selecting its midpoint for reliable flash reads.
 ****************************************************************************/

#define DLL_SPIN     200000u
#define DLL_WORDS    128u                         /* 512 B reference pattern */
#define DLL_CHUNK    16u                          /* 64 B per read command, < 128 B RX FIFO */
#define DLL_LOCK     (FLEXSPI_STS2_ASLVLOCK_MASK | FLEXSPI_STS2_AREFLOCK_MASK)
#define DLL_IDLE     (FLEXSPI_STS0_ARBIDLE_MASK | FLEXSPI_STS0_SEQIDLE_MASK)
#define DLL_SETTLE   4000u                        /* post-lock settle; ERR011377 needs >=100 NOPs, this is well beyond */
#define DLL_CODE_LO  1u                           /* skip SLVDLYTARGET=0 (sub-cell, degenerate) */
#define DLL_PAT(i)   ((uint32_t)(((uint32_t)(i) * 2654435761u) ^ 0xa5a55a5aul))
#define DLL_P8(i)    DLL_PAT(i), DLL_PAT((i) + 1), DLL_PAT((i) + 2), DLL_PAT((i) + 3), \
	DLL_PAT((i) + 4), DLL_PAT((i) + 5), DLL_PAT((i) + 6), DLL_PAT((i) + 7)
#define DLL_P64(i)   DLL_P8(i), DLL_P8((i) + 8), DLL_P8((i) + 16), DLL_P8((i) + 24), \
	DLL_P8((i) + 32), DLL_P8((i) + 40), DLL_P8((i) + 48), DLL_P8((i) + 56)

/* Known reference pattern, kept in flash so reading it back tests each delay */
static const uint32_t g_dll_train[DLL_WORDS] __attribute__((aligned(32))) = {
	DLL_P64(0), DLL_P64(64)
};

struct dll_cal_result_s {
	uint32_t magic;         /* 'DLLC' when populated */
	uint32_t sts2_entry;    /* STS2 as left by the ROM (pre-calibration snapshot) */
	uint16_t passmask;      /* bit N set => delay code N read back correctly */
	int8_t   chosen;        /* selected delay code, or -1 on ROM fallback */
};

struct dll_cal_result_s g_dll_cal;

/* Write DLLCR[0] (port A) using the NXP DLL-update sequence and return true
 * once it locks. Runs from RAM: it disables the flash we execute from, so no
 * flash access may occur across the call.
 */
locate_code(".ramfunc")
static bool imxrt_dll_write(struct flexspi_type_s *flexspi, uint32_t dllcr)
{
	uint32_t spin = DLL_SPIN;

	while (spin-- && (flexspi->STS0 & DLL_IDLE) != DLL_IDLE) {
	}

	flexspi->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;   /* stop mode before touching DLLCR */
	arm_dsb();
	arm_isb();
	flexspi->DLLCR[0] = dllcr;
	flexspi->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;  /* exit stop -> DLL re-locks (no SWRESET) */

	if (dllcr & FLEXSPI_DLLCR_DLLEN_MASK) {
		for (spin = DLL_SPIN; spin-- && (flexspi->STS2 & DLL_LOCK) != DLL_LOCK;) {
		}

		if ((flexspi->STS2 & DLL_LOCK) != DLL_LOCK) {
			return false;   /* never locked */
		}
	}

	for (volatile uint32_t d = DLL_SETTLE; d--;) {  /* settle delay line (ERR011377) */
	}

	return true;
}

/* Read the reference pattern back with a direct command (not the execute-in-
 * place fetch path) and return true only if every word matches.
 */
locate_code(".ramfunc")
static bool imxrt_dll_read_matches(struct flexspi_type_s *flexspi, uint32_t sfar_base)
{
	for (uint32_t word = 0; word < DLL_WORDS; word += DLL_CHUNK) {
		const uint32_t bytes = DLL_CHUNK * 4u;
		uint32_t spin;

		flexspi->INTR = FLEXSPI_INTR_IPCMDDONE_MASK | FLEXSPI_INTR_IPCMDERR_MASK |
				FLEXSPI_INTR_IPCMDGE_MASK | FLEXSPI_INTR_AHBCMDERR_MASK;
		flexspi->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
		flexspi->IPCR0 = sfar_base + word * 4u;
		flexspi->IPCR1 = FLEXSPI_IPCR1_IDATSZ(bytes) | FLEXSPI_IPCR1_ISEQID(0) |
				 FLEXSPI_IPCR1_ISEQNUM(0);            /* read seq at LUT index 0 */
		flexspi->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;

		bool done = false;

		for (spin = DLL_SPIN; spin--;) {
			uint32_t intr = flexspi->INTR;

			if (intr & FLEXSPI_INTR_IPCMDERR_MASK) {
				return false;
			}

			if (intr & FLEXSPI_INTR_IPCMDDONE_MASK) {
				done = true;
				break;
			}
		}

		if (!done) {
			return false;
		}

		for (spin = DLL_SPIN; spin-- &&
		     ((flexspi->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK) >> FLEXSPI_IPRXFSTS_FILL_SHIFT) * 8u < bytes;) {
		}

		for (uint32_t j = 0; j < DLL_CHUNK; j++) {
			if (flexspi->RFDR[j] != DLL_PAT(word + j)) {
				return false;
			}
		}

		flexspi->INTR = FLEXSPI_INTR_IPRXWA_MASK;   /* pop the FIFO */
	}

	return true;
}

/* Middle of the widest run of passing delay codes, or -1 if none. RAM-resident. */
locate_code(".ramfunc")
static int imxrt_dll_center(uint16_t mask)
{
	int best_start = -1, best_len = 0, run = 0;

	for (int i = 0; mask >> i; i++) {
		if (mask & (1u << i)) {
			if (++run > best_len) {
				best_len = run;
				best_start = i - run + 1;
			}

		} else {
			run = 0;
		}
	}

	return (best_start < 0) ? -1 : best_start + best_len / 2;
}

/* Best-effort DLL calibration; on anything unexpected it keeps the ROM DLL. */
locate_code(".ramfunc")
static void imxrt_flexspi_dll_calibrate(struct flexspi_type_s *flexspi)
{
	irqstate_t flags = up_irq_save();

	const uint32_t rom_dllcr = flexspi->DLLCR[0];
	const uint32_t rxclksrc = (flexspi->MCR0 & FLEXSPI_MCR0_RXCLKSRC_MASK) >> FLEXSPI_MCR0_RXCLKSRC_SHIFT;

	g_dll_cal.magic = 0x444c4c43ul;   /* 'DLLC' */
	g_dll_cal.sts2_entry = flexspi->STS2;
	g_dll_cal.passmask = 0;
	g_dll_cal.chosen = -1;

	/* Only calibrate when reads are sampled by the flash's DQS strobe */
	if (rxclksrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) {
		const uint32_t sfar = (uint32_t)(uintptr_t)&g_dll_train[0] - IMXRT_FLEXSPI1_CIPHER_BASE;

		for (uint32_t code = DLL_CODE_LO; code < 16u; code++) {
			if (imxrt_dll_write(flexspi, FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(code)) &&
			    imxrt_dll_read_matches(flexspi, sfar)) {
				g_dll_cal.passmask |= (uint16_t)(1u << code);
			}
		}

		int chosen = imxrt_dll_center(g_dll_cal.passmask);

		if (chosen >= 0 &&
		    imxrt_dll_write(flexspi, FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET((uint32_t)chosen))) {
			g_dll_cal.chosen = (int8_t)chosen;

		} else {
			imxrt_dll_write(flexspi, rom_dllcr);   /* no window: restore ROM */
		}
	}

	up_irq_restore(flags);
}

locate_code(".ramfunc")
void imxrt_flash_setup_prefetch_partition(void)
{
	struct flexspi_type_s *flexspi = (struct flexspi_type_s *)IMXRT_FLEXSPIC_BASE;

	imxrt_flexspi_dll_calibrate(flexspi);

	putreg32((uint32_t)&_srodata, IMXRT_FLEXSPI1_AHBBUFREGIONSTART0);
	putreg32((uint32_t)&_erodata, IMXRT_FLEXSPI1_AHBBUFREGIONEND0);
	putreg32((uint32_t)&_stext, IMXRT_FLEXSPI1_AHBBUFREGIONSTART1);
	putreg32((uint32_t)&_etext, IMXRT_FLEXSPI1_AHBBUFREGIONEND1);

	/* RODATA */
	flexspi->AHBRXBUFCR0[0] = FLEXSPI_AHBRXBUFCR0_BUFSZ(128) |
				  FLEXSPI_AHBRXBUFCR0_MSTRID(7) |
				  FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				  FLEXSPI_AHBRXBUFCR0_REGIONEN(1);


	/* All Text */
	flexspi->AHBRXBUFCR0[1] = FLEXSPI_AHBRXBUFCR0_BUFSZ(380) |
				  FLEXSPI_AHBRXBUFCR0_MSTRID(7) |
				  FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				  FLEXSPI_AHBRXBUFCR0_REGIONEN(1);
	/* Reset CR7 from rom init */
	flexspi->AHBRXBUFCR0[7] = FLEXSPI_AHBRXBUFCR0_BUFSZ(0) |
				  FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				  FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				  FLEXSPI_AHBRXBUFCR0_REGIONEN(0);

	arm_dsb();
	arm_isb();
	arm_dmb();
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

	arkv6xrt_timer_initialize();
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
__EXPORT int board_app_initialize(uintptr_t arg)
{
	int ret = OK;

#if !defined(BOOTLOADER)

	VDD_3V3_SD_CARD_EN(true);
	VDD_3V3_SPEKTRUM_POWER_EN(true);

	/*
	 * SE051 (U17) is on SENSORS3 / I2C3; ENA is HW_VER_REV_DRIVE (active low).
	 * Hold I2C3 low and ENA off until after HW versioning so the SE cannot
	 * backfeed the bus.
	 *
	 * BMP390 analog VDD is SENSORS4; VDDIO is always-on FMU_3V3. POR only
	 * runs when both rails are up, so do not drop SENSORS4 after VDDIO is live.
	 */
	px4_arch_gpiowrite(GPIO_LPI2C3_SCL, 0);
	px4_arch_gpiowrite(GPIO_LPI2C3_SDA, 0);
	px4_arch_gpiowrite(GPIO_HW_VER_REV_DRIVE, 1);

	VDD_3V3_SENSORS4_EN(true);
	usleep(3000); /* BMP390 tstartup after VDD */

	/* Need hrt running before using the ADC */

	px4_platform_init();

	// Use the default HW_VER_REV(0x0,0x0) for Ramtron

	imxrt_spiinitialize();

	if (OK == board_determine_hw_info()) {
		syslog(LOG_INFO, "[boot] Rev 0x%1x : Ver 0x%1x %s\n", board_get_hw_revision(), board_get_hw_version(),
		       board_get_hw_type_name());

	} else {
		syslog(LOG_ERR, "[boot] Failed to read HW revision and version\n");
	}

	/* Report the FlexSPI DLL calibration result captured during early board
	 * init (see imxrt_flexspi_dll_calibrate()). Logged here, not there, so it
	 * does not corrupt the early console before syslog is serialized.
	 */
	if (g_dll_cal.magic == 0x444c4c43ul) {
		struct flexspi_type_s *flexspi = (struct flexspi_type_s *)IMXRT_FLEXSPIC_BASE;
		uint32_t dllcr = flexspi->DLLCR[0];
		uint32_t sts2 = flexspi->STS2;
		uint32_t delay = (sts2 & FLEXSPI_STS2_ASLVSEL_MASK) >> FLEXSPI_STS2_ASLVSEL_SHIFT;
		uint32_t rom_delay = (g_dll_cal.sts2_entry & FLEXSPI_STS2_ASLVSEL_MASK) >> FLEXSPI_STS2_ASLVSEL_SHIFT;
		bool locked = (sts2 & DLL_LOCK) == DLL_LOCK;
		syslog(LOG_INFO,
		       "[boot] FlexSPI DLL: chosen code %d, passmask 0x%04x, sample delay %" PRIu32
		       " cells (ROM %" PRIu32 "), %s [dllcr 0x%04" PRIx32 "]\n",
		       g_dll_cal.chosen, g_dll_cal.passmask, delay, rom_delay,
		       g_dll_cal.chosen < 0 ? "ROM fallback" : (locked ? "locked" : "NO LOCK"),
		       dllcr);
	}

	VDD_5V_PERIPH_EN(true);
	VDD_5V_HIPOWER_EN(true);

	usleep(75000);

	px4_arch_configgpio(GPIO_LPI2C3_SCL);
	px4_arch_configgpio(GPIO_LPI2C3_SDA);
	px4_arch_gpiowrite(GPIO_HW_VER_REV_DRIVE, 0);

	/* CTS had been treated as inputs pulled high
	 * to avoid radios from enteriong bootloader
	 * Set them up as CTS inputs
	 */

	px4_arch_configgpio(GPIO_LPUART4_CTS);
	px4_arch_configgpio(GPIO_LPUART8_CTS);
	px4_arch_configgpio(GPIO_LPUART10_CTS);

	/* Do the I2C init late BOARD_I2C_LATEINIT */

	px4_platform_i2c_init();

	/* PAB 24LC64T is on I2C3_BASE = LPI2C6 (X1_8/10). Manifest MTD talks I2C, so it
	 * runs after those pins are I2C and VDD_5V_PERIPH is on. FlexSPI FRAM does not care. */
	px4_platform_configure();

	/* Configure the Actual SPI interfaces (after we determined the HW version)  */

	imxrt_spiinitialize();

	board_spi_reset(10, 0xffff);

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#if 0 // defined(SERIAL_HAVE_RXDMA)
	// set up the serial DMA polling at 1ms intervals for received bytes that have not triggered a DMA event.
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)imxrt_serial_dma_poll, NULL);
#endif

	/* initial LED state */
	drv_led_start();

	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

#ifdef CONFIG_BOARD_CRASHDUMP_CUSTOM

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}

#endif

#if defined(CONFIG_IMXRT_USDHC)
	ret = arkv6xrt_usdhc_initialize();

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

#endif /* !defined(BOOTLOADER) */

	return ret;
}
