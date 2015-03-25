/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4cannode_init.c
 *
 * PX4CANNODE-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include "board_config.h"
#include "stm32_uart.h"

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/cpuload.h>

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
#include <systemlib/systemlib.h>
#endif

/* todo: This is constant but not proper */
__BEGIN_DECLS
extern void led_off(int led);
__END_DECLS


/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

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

__EXPORT void stm32_boardinitialize(void)
{
    /* configure LEDs */
    board_led_initialize();
    board_button_initialize();
    board_spiinitialize();

}


__EXPORT void board_initialize(void)
{
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/


#include <math.h>

#ifdef __cplusplus
__EXPORT int matherr(struct __exception *e)
{
    return 1;
}
#else
__EXPORT int matherr(struct exception *e)
{
    return 1;
}
#endif

__EXPORT int nsh_archinitialize(void)
{
    int result = OK;

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

    led_off(LED_AMBER);
    led_off(LED_BLUE);

    return result;
}
