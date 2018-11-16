/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file px4same70xplained_init.c
 *
 * PX4_SAME70XPLAINED_V1 specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>
#include <px4_tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include "platform/cxxinitialize.h"
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <sam_lowputc.h>
#include <sam_gpio.h>
#include <sam_spi.h>
#include <sam_hsmci.h>
#include <sam_pck.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>
#include <systemlib/cpuload.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <parameters/param.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(LOG_NOTICE, __VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#  else
#    define message printf
#  endif
#endif

/*
 * Ideally we'd be able to get these from up_internal.h,
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

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAMV7 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after clocking and
 *   memory have been configured but before caches have been enabled and
 *   before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
sam_boardinitialize(void)
{
#ifdef CONFIG_SCHED_TICKLESS
	uint32_t frequency;
	uint32_t actual;

	/* If Tickless mode is selected then enabled PCK6 as a possible clock
	 * source for the timer/counters.  The ideal frequency could be:
	 *
	 *  frequency = 1,000,000 / CONFIG_USEC_PER_TICK
	 *
	 * The main crystal is selected as the frequency source.  The maximum
	 * prescaler value is 256 so the minimum frequency is 46,875 Hz which
	 * corresponds to a period of 21.3 microseconds.  A value of
	 * CONFIG_USEC_PER_TICK=20, or 50KHz, would give an exact solution with
	 * a divider of 240.
	 */

	frequency = USEC_PER_SEC / CONFIG_USEC_PER_TICK;
	DEBUGASSERT(frequency >= (BOARD_MAINOSC_FREQUENCY / 256));

	actual = sam_pck_configure(PCK6, PCKSRC_MAINCK, frequency);

	/* We expect to achieve this frequency exactly */

	DEBUGASSERT(actual == frequency);
	UNUSED(actual);

	/* Enable PCK6 */

	(void)sam_pck_enable(PCK6, true);
#endif

	/* Lets bring the clock out for a sanity check*/
#ifdef GPIO_PCK1
	sam_configgpio(GPIO_PCK1);
	volatile uint32_t actual = sam_pck_configure(PCK1, PCKSRC_MCK, BOARD_MCK_FREQUENCY / 2); // Out 1/2 Clock
	UNUSED(actual);

	(void)sam_pck_enable(PCK1, true);
#endif
#ifdef CONFIG_SAMV7_SDRAMC
	/* Configure SDRAM if it has been enabled in the NuttX configuration.
	 * Here we assume, of course, that we are not running out SDRAM.
	 */

	sam_sdram_config();
#endif

#ifdef CONFIG_SAMV7_SPI

	/* configure SPI interfaces */

	board_spi_initialize();
#endif

#ifdef CONFIG_ARCH_LEDS

	/* configure LEDs */

	board_autoled_initialize();
#endif
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

static struct spi_dev_s *spi0;
#if defined(CONFIG_SAMV7_SPI1_MASTER)
static struct spi_dev_s *spi1;
#endif
static struct sdio_dev_s *sdio;

__EXPORT int board_app_initialize(uintptr_t arg)
{

	/* configure ADC pins */

	/* configure power supply control/sense pins */

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)

	/* run C++ ctors before we go any further */

	up_cxxinitialize();

#	if defined(CONFIG_EXAMPLES_NSH_CXXINITIALIZE)
#  		error CONFIG_EXAMPLES_NSH_CXXINITIALIZE Must not be defined! Use CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE.
#	endif

#else
#  error platform is dependent on c++ both CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE must be defined.
#endif

	/* configure the high-resolution time/callout interface */
	hrt_init();

	param_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		message("DMA alloc FAILED");
	}

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif


	/* initial LED state */
	drv_led_start();
	led_on(LED_AMBER);
	led_off(LED_AMBER);

	/* Configure SPI-based devices */

	spi0 = px4_spibus_initialize(PX4_SPI_BUS_SENSORS);

	if (!spi0) {
		message("[boot] FAILED to initialize SPI port %d\n", PX4_SPI_BUS_SENSORS);
		board_autoled_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi0, 10000000);
	SPI_SETBITS(spi0, 8);
	SPI_SETMODE(spi0, SPIDEV_MODE3);
	SPI_SELECT(spi0, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi0, PX4_SPIDEV_ACCEL_MAG, false);
	SPI_SELECT(spi0, PX4_SPIDEV_BARO, false);
	SPI_SELECT(spi0, PX4_SPIDEV_MPU, false);
	up_udelay(20);

#if defined(CONFIG_SAMV7_SPI1_MASTER)
	spi1 = px4_spibus_initialize(PX4_SPI_BUS_MEMORY);

	/* Default SPI4 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, PX4_SPIDEV_EXT0, false);
	SPI_SELECT(spi1, PX4_SPIDEV_EXT1, false);
#endif

#ifdef CONFIG_MMCSD
	/* First, get an instance of the SDIO interface */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		message("[boot] Failed to initialize SDIO slot %d\n",
			CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		message("[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	/* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
	sdio_mediachange(sdio, true);

#endif

	return OK;
}

#if defined(CONFIG_BOARDCTL_RESET)
int board_reset(int status)
{
	up_systemreset();
	return 0;
}
#endif

//FIXME: Stubs  -----v
/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

int up_rtc_getdatetime(FAR struct tm *tp);
int up_rtc_getdatetime(FAR struct tm *tp)
{
	tp->tm_sec = 0;
	tp->tm_min = 0;
	tp->tm_hour = 0;
	tp->tm_mday = 30;
	tp->tm_mon = 10;
	tp->tm_year = 116;
	tp->tm_wday = 1;    /* Day of the week (0-6) */
	tp->tm_yday = 0;    /* Day of the year (0-365) */
	tp->tm_isdst = 0;   /* Non-0 if daylight savings time is in effect */
	return 0;
}

int up_rtc_initialize(void)
{
	return 0;
}

int up_rtc_settime(FAR const struct timespec *tp)
{
	return 0;
}
//FIXME: Stubs  -----^
