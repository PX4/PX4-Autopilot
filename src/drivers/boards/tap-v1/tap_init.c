/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *         Author: David Sidrane <david_s5@nscdg.com>
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
 * @file tap-v1_init.c
 *
 * tap-v1-specific early startup code.  This file implements the
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
#include <nuttx/i2c.h>
#include <nuttx/analog/adc.h>



#include "stm32.h"
#include "board_config.h"
#include "stm32_uart.h"

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/cpuload.h>

# if defined(FLASH_BASED_PARAMS)
#  include <systemlib/flashparams/flashfs.h>
#endif

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
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	/* configure always-on ADC pins */
	stm32_configgpio(GPIO_ADC1_IN10);

	board_pwr_init(0);

	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure LEDs (empty call to NuttX' ledinit) */
	up_ledinit();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

__EXPORT int nsh_archinitialize(void)
{
	int result;

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

	board_pwr_init(1);

	/* initial LED state */
	drv_led_start();
	led_off(LED_AMBER);
	led_off(LED_BLUE);

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t  sector_map[] = {
		{1, 16 * 1024, 0x08004000},
		{2, 16 * 1024, 0x08008000},
		{0, 0, 0},
	};

	/* Initalizee the flashfs layer to use heap allocated memory */

	result = parameter_flashfs_init(sector_map, NULL, 0);

	if (result != OK) {
		message("[boot] FAILED to init params in FLASH %d\n", result);
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

#endif

	/* Init the microSD slot */

	result = board_sdio_initialize();

	if (result != OK) {
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	return OK;
}
