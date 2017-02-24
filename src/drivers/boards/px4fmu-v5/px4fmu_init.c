/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4fmu_init.c
 *
 * PX4FMU-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <chip.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>
#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <systemlib/hardfault_log.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "up_internal.h"
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
 * Name: board_rc_input
 *
 * Description:
 *   All boards my optionally provide this API to invert the Serial RC input.
 *   This is needed on SoCs that support the notion RXINV or TXINV as apposed to
 *   and external XOR controlled by a GPIO
 *
 ************************************************************************************/

__EXPORT void board_rc_input(bool invert_on)
{

	irqstate_t irqstate = px4_enter_critical_section();

	uint32_t cr1 =  getreg32(STM32_USART_CR1_OFFSET + RC_UXART_BASE);
	uint32_t cr2 =  getreg32(STM32_USART_CR2_OFFSET + RC_UXART_BASE);
	uint32_t regval = cr1;

	/* {R|T}XINV bit fields can only be written when the USART is disabled (UE=0). */

	regval &= ~USART_CR1_UE;

	putreg32(regval, STM32_USART_CR1_OFFSET + RC_UXART_BASE);

	if (invert_on) {
		cr2 |= (USART_CR2_RXINV | USART_CR2_TXINV);

	} else {
		cr2 &= ~(USART_CR2_RXINV | USART_CR2_TXINV);
	}

	putreg32(cr2, STM32_USART_CR1_OFFSET + RC_UXART_BASE);
	putreg32(cr1, STM32_USART_CR1_OFFSET + RC_UXART_BASE);

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

	stm32_gpiowrite(GPIO_PERIPH_5V_EN, 0);
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	bool last = stm32_gpioread(GPIO_SPEKTRUM_POWER_EN);
	/* Keep Spektum on to discharge rail*/
	stm32_gpiowrite(GPIO_SPEKTRUM_POWER_EN, 1);

	/* wait for the peripheral rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the peripheral rail back on */
	stm32_gpiowrite(GPIO_SPEKTRUM_POWER_EN, last);
	stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);
	stm32_gpiowrite(GPIO_PERIPH_5V_EN, 1);

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

__EXPORT void
stm32_boardinitialize(void)
{
	/* configure LEDs */
	board_autoled_initialize();

	/* configure the GPIO pins to outputs and keep them low */
	stm32_configgpio(GPIO_GPIO0_OUTPUT);
	stm32_configgpio(GPIO_GPIO1_OUTPUT);
	stm32_configgpio(GPIO_GPIO2_OUTPUT);
	stm32_configgpio(GPIO_GPIO3_OUTPUT);
	stm32_configgpio(GPIO_GPIO4_OUTPUT);
	stm32_configgpio(GPIO_GPIO5_OUTPUT);

	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN0);	/* ADC_BATTERY_VOLTAGE_CHANNEL  */
	stm32_configgpio(GPIO_ADC1_IN1);	/* ADC_BATTERY_CURRENT_CHANNEL */
	stm32_configgpio(GPIO_ADC1_IN2);	/* ADC_BATTERY1_VOLTAGE_CHANNEL */
	stm32_configgpio(GPIO_ADC1_IN3);	/* ADC_BATTERY1_CURRENT_CHANNEL */
	stm32_configgpio(GPIO_ADC1_IN4);	/* ADC_5V_RAIL_SENSE */
	stm32_configgpio(GPIO_ADC1_IN8);	/* ADC_RC_RSSI_CHANNEL */
	stm32_configgpio(GPIO_ADC1_IN10);	/* ADC_INT_1 */
	stm32_configgpio(GPIO_ADC1_IN11);	/* ADC_INT_2 */
	stm32_configgpio(GPIO_ADC1_IN12);	/* ADC_INT_3 */
	stm32_configgpio(GPIO_ADC1_IN13);	/* ADC_INT_4 */
	stm32_configgpio(GPIO_ADC1_IN14);	/* ADC_INT_5 */

	/* Configure the HEATER off */

	stm32_configgpio(GPIO_HEATER);


	/* Configure the CAN Silent Control pins and keep them low */

	stm32_configgpio(GPIO_CAN1_SILENCE);
	stm32_configgpio(GPIO_CAN2_SILENCE);
	stm32_configgpio(GPIO_CAN3_SILENCE);

	/* configure power supply control/sense pins */

	stm32_configgpio(GPIO_POWER_IN_A);
	stm32_configgpio(GPIO_POWER_IN_B);
	stm32_configgpio(GPIO_POWER_IN_C);

	stm32_configgpio(GPIO_VDD_BRICK_VALID);
	stm32_configgpio(GPIO_PERIPH_5V_EN);
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_configgpio(GPIO_VDD_3V3V_SD_CARD_EN);
	stm32_configgpio(GPIO_VDD_5V_RC_EN);
	stm32_configgpio(GPIO_VDD_5V_WIFI_EN);
	stm32_configgpio(GPIO_SPEKTRUM_POWER_EN);

	stm32_configgpio(GPIO_LED_SAFETY);
	stm32_configgpio(GPIO_BTN_SAFETY);

#ifdef GPIO_RC_OUT
	stm32_configgpio(GPIO_RC_OUT);      /* Serial RC output pin */
#endif

	/* configure SPI interfaces */
	stm32_spiinitialize();

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
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

#if defined(CONFIG_STM32F7_BBSRAM)

	/* NB. the use of the console requires the hrt running
	 * to poll the DMA
	 */

	/* Using Battery Backed Up SRAM */

	int filesizes[CONFIG_STM32F7_BBSRAM_FILES + 1] = BSRAM_FILE_SIZES;

	stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

#if defined(CONFIG_STM32F7_SAVE_CRASHDUMP)

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

#endif // CONFIG_STM32F7_SAVE_CRASHDUMP
#endif // CONFIG_STM32F7_BBSRAM

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

#ifdef CONFIG_SPI
	int ret = stm32_spi_bus_initialize();

	if (ret != OK) {
		board_autoled_on(LED_RED);
		return ret;
	}

#endif

#ifdef CONFIG_MMCSD
	ret = stm32_sdio_initialize();

	if (ret != OK) {
		board_autoled_on(LED_RED);
		return ret;
	}

#endif

	return OK;
}
