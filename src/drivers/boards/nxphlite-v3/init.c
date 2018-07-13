/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file nxphlite_init.c
 *
 * NXPHLITEV1v2-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>

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

#include <kinetis.h>
#include <chip/kinetis_uart.h>
#include "board_config.h"

#include "up_arch.h"
#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>
#include <systemlib/cpuload.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>

#if defined(CONFIG_KINETIS_BBSRAM) //fixme:Need BBSRAM
#include <systemlib/hardfault_log.h>
#endif


/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
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

void board_on_reset(int status)
{
	/* configure the GPIO pins to outputs and keep them low */

	const uint32_t gpio[] = PX4_GPIO_PWM_INIT_LIST;
	board_gpio_init(gpio, arraySize(gpio));

	if (status >= 0) {
		up_mdelay(6);
	}
}

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

int board_read_VBUS_state(void)
{
	return BOARD_ADC_USB_CONNECTED ? 0 : 1;
}

/************************************************************************************
 * Name: board_rc_input
 *
 * Description:
 *   All boards my optionally provide this API to invert the Serial RC input.
 *   This is needed on SoCs that support the notion RXINV or TXINV as opposed to
 *   and external XOR controlled by a GPIO
 *
 ************************************************************************************/

__EXPORT void board_rc_input(bool invert_on, uint32_t uxart_base)
{

	irqstate_t irqstate = px4_enter_critical_section();

	uint8_t s2 =  getreg8(KINETIS_UART_S2_OFFSET + uxart_base);
	uint8_t c3 =  getreg8(KINETIS_UART_C3_OFFSET + uxart_base);

	/* {R|T}XINV bit fields can written any time */

	if (invert_on) {
		s2 |= (UART_S2_RXINV);
		c3 |= (UART_C3_TXINV);

	} else {
		s2 &= ~(UART_S2_RXINV);
		c3 &= ~(UART_C3_TXINV);
	}

	putreg8(s2, KINETIS_UART_S2_OFFSET + uxart_base);
	putreg8(c3, KINETIS_UART_C3_OFFSET + uxart_base);

	leave_critical_section(irqstate);
}

/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* set the peripheral rails off */

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the peripheral rail back on */
}

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All Kinetis architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
kinetis_boardinitialize(void)
{
	board_on_reset(-1); /* Reset PWM first thing */

	/* configure LEDs */
	board_autoled_initialize();

	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	board_gpio_init(gpio, arraySize(gpio));

	nxphlite_timer_initialize();

	/* Power on Spektrum */

	VDD_3V3_SPEKTRUM_POWER_EN(true);

}

//FIXME: Stubs  -----v
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

static void kinetis_serial_dma_poll(void)
{
	// todo:Stubbed
}
//FIXME: Stubs  -----v


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

	VDD_3V3_SD_CARD_EN(true);
	VDD_3V3_SENSORS_EN(true);

	/* configure SPI interfaces */

	nxphlite_spidev_initialize();

	VDD_ETH_EN(true);

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)

	/* run C++ ctors before we go any further */

	up_cxxinitialize();

#	if defined(CONFIG_EXAMPLES_NSH_CXXINITIALIZE)
#  		error CONFIG_EXAMPLES_NSH_CXXINITIALIZE Must not be defined! Use CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE.
#	endif

#else
#  error platform is dependent on c++ both CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE must be defined.
#endif

	param_init();

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure the DMA allocator */

	if (board_dma_alloc_init() < 0) {
		message("DMA alloc FAILED");
	}

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)kinetis_serial_dma_poll,
		       NULL);

#if defined(CONFIG_KINETIS_BBSRAM)

	/* NB. the use of the console requires the hrt running
	 * to poll the DMA
	 */

	/* Using Battery Backed Up SRAM */

	int filesizes[CONFIG_KINETIS_BBSRAM_FILES + 1] = BSRAM_FILE_SIZES;

	stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

#if defined(CONFIG_KINETIS_SAVE_CRASHDUMP)

	/* Panic Logging in Battery Backed Up Files */

	/*
	 * In an ideal world, if a fault happens in flight the
	 * system save it to BBSRAM will then reboot. Upon
	 * rebooting, the system will log the fault to disk, recover
	 * the flight state and continue to fly.  But if there is
	 * a fault on the bench or in the air that prohibit the recovery
	 * or committing the log to disk, the things are too broken to
	 * fly. So the question is:
	 *
	 * Did we have a hard fault and not make it far enough
	 * through the boot sequence to commit the fault data to
	 * the SD card?
	 */

	/* Do we have an uncommitted hard fault in BBSRAM?
	 *  - this will be reset after a successful commit to SD
	 */
	int hadCrash = hardfault_check_status("boot");

	if (hadCrash == OK) {

		message("[boot] There is a hard fault logged. Hold down the SPACE BAR," \
			" while booting to halt the system!\n");

		/* Yes. So add one to the boot count - this will be reset after a successful
		 * commit to SD
		 */

		int reboots = hardfault_increment_reboot("boot", false);

		/* Also end the misery for a user that holds for a key down on the console */

		int bytesWaiting;
		ioctl(fileno(stdin), FIONREAD, (unsigned long)((uintptr_t) &bytesWaiting));

		if (reboots > 2 || bytesWaiting != 0) {

			/* Since we can not commit the fault dump to disk. Display it
			 * to the console.
			 */

			hardfault_write("boot", fileno(stdout), HARDFAULT_DISPLAY_FORMAT, false);

			message("[boot] There were %d reboots with Hard fault that were not committed to disk - System halted %s\n",
				reboots,
				(bytesWaiting == 0 ? "" : " Due to Key Press\n"));


			/* For those of you with a debugger set a break point on up_assert and
			 * then set dbgContinue = 1 and go.
			 */

			/* Clear any key press that got us here */

			static volatile bool dbgContinue = false;
			int c = '>';

			while (!dbgContinue) {

				switch (c) {

				case EOF:


				case '\n':
				case '\r':
				case ' ':
					continue;

				default:

					putchar(c);
					putchar('\n');

					switch (c) {

					case 'D':
					case 'd':
						hardfault_write("boot", fileno(stdout), HARDFAULT_DISPLAY_FORMAT, false);
						break;

					case 'C':
					case 'c':
						hardfault_rearm("boot");
						hardfault_increment_reboot("boot", true);
						break;

					case 'B':
					case 'b':
						dbgContinue = true;
						break;

					default:
						break;
					} // Inner Switch

					message("\nEnter B - Continue booting\n" \
						"Enter C - Clear the fault log\n" \
						"Enter D - Dump fault log\n\n?>");
					fflush(stdout);

					if (!dbgContinue) {
						c = getchar();
					}

					break;

				} // outer switch
			} // for

		} // inner if
	} // outer if

#endif // CONFIG_KINETIS_SAVE_CRASHDUMP
#endif // CONFIG_KINETIS_BBSRAM

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	int ret = nxphlite_sdhc_initialize();

	if (ret != OK) {
		board_autoled_on(LED_RED);
		return ret;
	}

#ifdef HAVE_AUTOMOUNTER
	/* Initialize the auto-mounter */

	nxphlite_automount_initialize();
#endif

	/* Configure SPI-based devices */

#ifdef CONFIG_SPI
	ret = nxphlite_spi_bus_initialize();

	if (ret != OK) {
		board_autoled_on(LED_RED);
		return ret;
	}

#endif

	return OK;
}
