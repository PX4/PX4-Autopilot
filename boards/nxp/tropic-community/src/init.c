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
 * NXP imxrt1062-v1 specific early startup code.  This file implements the
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
#include "arm_internal.h"
#include "imxrt_flexspi_nor_boot.h"
#include "imxrt_iomuxc.h"
#include "imxrt_flexcan.h"
#include "imxrt_enet.h"
#include <chip.h>
#include "board_config.h"

#include <hardware/imxrt_lpuart.h>
#undef FLEXSPI_LUT_COUNT
#include <hardware/imxrt_flexspi.h>
#include <hardware/imxrt_ccm.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>
#include <px4_platform/board_dma_alloc.h>

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


locate_code(".ramfunc")
void imxrt_flash_setup_prefetch_partition(void)
{
//Prefetch tuning to be determined
#if 0
	putreg32((uint32_t)&_srodata, IMXRT_FLEXSPI1_AHBBUFREGIONSTART0);
	putreg32((uint32_t)&_erodata, IMXRT_FLEXSPI1_AHBBUFREGIONEND0);
	/*putreg32((uint32_t)&_stext, IMXRT_FLEXSPI1_AHBBUFREGIONSTART1);
	putreg32((uint32_t)&_etext, IMXRT_FLEXSPI1_AHBBUFREGIONEND1);
	putreg32((uint32_t)&_stext, IMXRT_FLEXSPI1_AHBBUFREGIONSTART2);
	putreg32((uint32_t)&_etext, IMXRT_FLEXSPI1_AHBBUFREGIONEND2);
	putreg32((uint32_t)&_stext, IMXRT_FLEXSPI1_AHBBUFREGIONSTART3);
	putreg32((uint32_t)&_etext, IMXRT_FLEXSPI1_AHBBUFREGIONEND3);*/
#endif
#if 0
	struct flexspi_type_s *g_flexspi = (struct flexspi_type_s *)IMXRT_FLEXSPIC_BASE;
	/* RODATA */
	g_flexspi->AHBRXBUFCR0[0] = FLEXSPI_AHBRXBUFCR0_BUFSZ(48) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(1);


	/* All Text */
	g_flexspi->AHBRXBUFCR0[1] = FLEXSPI_AHBRXBUFCR0_BUFSZ(80) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(1);

	/* Disable 2 */
	g_flexspi->AHBRXBUFCR0[2] = FLEXSPI_AHBRXBUFCR0_BUFSZ(0) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(0);

	/* Disable 3 */
	g_flexspi->AHBRXBUFCR0[3] = FLEXSPI_AHBRXBUFCR0_BUFSZ(0) |
				    FLEXSPI_AHBRXBUFCR0_MSTRID(0) |
				    FLEXSPI_AHBRXBUFCR0_PREFETCHEN(1) |
				    FLEXSPI_AHBRXBUFCR0_REGIONEN(0);
#endif

	ARM_DSB();
	ARM_ISB();
	ARM_DMB();
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
	imxrt_flash_setup_prefetch_partition();

	board_on_reset(-1); /* Reset PWM first thing */

	/* configure LEDs */

	board_autoled_initialize();

	/* configure pins */

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* configure SPI interfaces */

	imxrt_spidev_initialize();

	imxrt_usb_initialize();

	fmurt1062_timer_initialize();
}

/****************************************************************************
 * Function: imxrt_phy_boardinitialize
 *
 * Description:
 *   Some boards require specialized initialization of the PHY before it can
 *   be used.  This may include such things as configuring GPIOs, resetting
 *   the PHY, etc.
 *   If CONFIG_IMXRT_ENET_PHYINIT is defined in the configuration then the
 *   board specific logic must provide imxrt_phyinitialize();
 *   The i.MX RT Ethernet driver will call this function one time before it
 *   first uses the PHY.
 *
 * Input Parameters:
 *   intf - Always zero for now.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int imxrt_phy_boardinitialize(int intf)
{
#ifdef CONFIG_IMXRT_GPIO1_0_15_IRQ
	/* Configure the PHY interrupt pin */

	printf("Configuring interrupt: %08x\n", GPIO_ENET_INT);
	imxrt_config_gpio(GPIO_ENET_INT);
#endif

	/* Configure the PHY reset pin.
	 *
	 * The #RST uses inverted logic.  The initial value of zero will put the
	 * PHY into the reset state.
	 */

	printf("Configuring reset: %08x\n", GPIO_ENET_RST);
	imxrt_config_gpio(GPIO_ENET_RST);

	/* Take the PHY out of reset. */

	imxrt_gpio_write(GPIO_ENET_RST, true);
	return OK;
}

void imxrt_flexio_clocking(void)
{
	uint32_t reg;

	/* Init USB PLL3 PFD2 */

	reg = getreg32(IMXRT_CCM_ANALOG_PFD_480);

	while ((getreg32(IMXRT_CCM_ANALOG_PLL_USB1) &
		CCM_ANALOG_PLL_USB1_LOCK) == 0) {
	}

	reg &= ~CCM_ANALOG_PFD_480_PFD2_FRAC_MASK;

	/* Set PLL3 PFD2 to 480 * 18 / CONFIG_PLL3_PFD2_FRAC */

	reg |= ((uint32_t)(CONFIG_PLL3_PFD2_FRAC) << CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT);

	putreg32(reg, IMXRT_CCM_ANALOG_PFD_480);

	reg = getreg32(IMXRT_CCM_CDCDR);
	reg &= ~(CCM_CDCDR_FLEXIO1_CLK_SEL_MASK |
		 CCM_CDCDR_FLEXIO1_CLK_PODF_MASK |
		 CCM_CDCDR_FLEXIO1_CLK_PRED_MASK);
	reg |= CCM_CDCDR_FLEXIO1_CLK_SEL(CONFIG_FLEXIO1_CLK);
	reg |= CCM_CDCDR_FLEXIO1_CLK_PODF
	       (CCM_PODF_FROM_DIVISOR(CONFIG_FLEXIO1_PODF_DIVIDER));
	reg |= CCM_CDCDR_FLEXIO1_CLK_PRED
	       (CCM_PRED_FROM_DIVISOR(CONFIG_FLEXIO1_PRED_DIVIDER));
	putreg32(reg, IMXRT_CCM_CDCDR);
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
	imxrt_flexio_clocking();

	/* Power on Interfaces */

	board_spi_reset(10, 0xffff);

	px4_platform_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

#if defined(SERIAL_HAVE_RXDMA)
	// set up the serial DMA polling at 1ms intervals for received bytes that have not triggered a DMA event.
	static struct hrt_call serial_dma_call;
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)imxrt_serial_dma_poll, NULL);
#endif

	int ret = OK;
#if defined(CONFIG_IMXRT_USDHC)
	ret = fmurt1062_usdhc_initialize();
#endif

	/* Configure SPI-based devices */

	ret = imxrt1062_spi_bus_initialize();

#ifdef CONFIG_IMXRT_ENET
	imxrt_netinitialize(0);
#endif

#ifdef CONFIG_IMXRT_FLEXCAN3
	imxrt_caninitialize(3);
#endif

#ifdef CONFIG_IMXRT_FLEXSPI
	ret = imxrt_flexspi_storage_initialize();

	if (ret < 0) {
		syslog(LOG_ERR,
		       "ERROR: imxrt_flexspi_nor_initialize failed: %d\n", ret);
	}

#endif

	return ret;
}
