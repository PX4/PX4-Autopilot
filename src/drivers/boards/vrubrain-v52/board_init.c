/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file board_init.c
 *
 * VRBRAIN-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
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
	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure LEDs */
	up_ledinit();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;

#include <math.h>

__EXPORT int nsh_archinitialize(void)
{
	int result;

	message("\r\n");

	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN10);	/* BATT_VOLTAGE_SENS */


	/* configure power supply control/sense pins */










#ifdef GPIO_RC_OUT
	stm32_configgpio(GPIO_RC_OUT);      /* Serial RC output pin */
	stm32_gpiowrite(GPIO_RC_OUT, 1);    /* set it high to pull RC input up */
#endif

	/* configure the GPIO pins to outputs and keep them low */
	stm32_configgpio(GPIO_GPIO0_OUTPUT);
	stm32_configgpio(GPIO_GPIO1_OUTPUT);
	stm32_configgpio(GPIO_GPIO2_OUTPUT);
	stm32_configgpio(GPIO_GPIO3_OUTPUT);
	stm32_configgpio(GPIO_GPIO4_OUTPUT);
	stm32_configgpio(GPIO_GPIO5_OUTPUT);
	stm32_configgpio(GPIO_GPIO6_OUTPUT);
	stm32_configgpio(GPIO_GPIO7_OUTPUT);
	stm32_configgpio(GPIO_GPIO8_OUTPUT);
	stm32_configgpio(GPIO_GPIO9_OUTPUT);
	stm32_configgpio(GPIO_GPI10_OUTPUT);

    stm32_configgpio(GPIO_GPI12_OUTPUT);
    stm32_configgpio(GPIO_GPI13_OUTPUT);
    stm32_configgpio(GPIO_GPI14_OUTPUT);

	/* configure the high-resolution time/callout interface */
	hrt_init();

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

	/* initial LED state */
	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);
	led_off(LED_BLUE);

	/* Configure SPI-based devices */

	message("[boot] Initializing SPI port 1\r\n");
	spi1 = up_spiinitialize(1);

	if (!spi1) {
		message("[boot] FAILED to initialize SPI port 1\r\n");
		up_ledon(LED_RED);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, SPIDEV_MS5611, false);



	up_udelay(20);

	message("[boot] Successfully initialized SPI port 1\r\n");

	message("[boot] Initializing SPI port 2\r\n");
	spi2 = up_spiinitialize(2);

	if (!spi2) {
		message("[boot] FAILED to initialize SPI port 2\r\n");
		up_ledon(LED_RED);
		return -ENODEV;
	}

	/* Default SPI2 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, SPIDEV_MPU6000, false);




	message("[boot] Successfully initialized SPI port 2\r\n");

	/* Get the SPI port for the microSD slot */

#ifdef CONFIG_MMCSD
	message("[boot] Initializing SPI port 3\r\n");
	spi3 = up_spiinitialize(3);

	if (!spi3) {
		message("[boot] FAILED to initialize SPI port 3\r\n");
		up_ledon(LED_RED);
		return -ENODEV;
	}

	/* Default SPI3 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi3, 10000000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);
	SPI_SELECT(spi3, SPIDEV_MMCSD, false);
	SPI_SELECT(spi3, SPIDEV_FLASH, false);

	message("[boot] Successfully initialized SPI port 3\r\n");

	/* Now bind the SPI interface to the MMCSD driver */
	result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi3);

	if (result != OK) {
		message("[boot] FAILED to bind SPI port 3 to the MMCSD driver\r\n");
		led_on(LED_AMBER);
		return -ENODEV;
	}

	message("[boot] Successfully bound SPI port 3 to the MMCSD driver\r\n");
#endif

	return OK;
}
