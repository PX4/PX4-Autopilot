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

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/gran.h>

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

#if defined(CONFIG_FAT_DMAMEMORY)
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif

static GRAN_HANDLE dma_allocator;

/* 
 * The DMA heap size constrains the total number of things that can be 
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file. 
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */
static uint8_t g_dma_heap[8192] __attribute__((aligned(64)));
static perf_counter_t g_dma_perf;

static void
dma_alloc_init(void)
{
	dma_allocator = gran_initialize(g_dma_heap,
					sizeof(g_dma_heap),
					7,  /* 128B granule - must be > alignment (XXX bug?) */
					6); /* 64B alignment */
	if (dma_allocator == NULL) {
		message("[boot] DMA allocator setup FAILED");
	} else {
		g_dma_perf = perf_alloc(PC_COUNT, "DMA allocations");
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * DMA-aware allocator stubs for the FAT filesystem.
 */

__EXPORT void *fat_dma_alloc(size_t size);
__EXPORT void fat_dma_free(FAR void *memory, size_t size);

void *
fat_dma_alloc(size_t size)
{
	perf_count(g_dma_perf);
	return gran_alloc(dma_allocator, size);
}

void
fat_dma_free(FAR void *memory, size_t size)
{
	gran_free(dma_allocator, memory, size);
}

#else

# define dma_alloc_init()

#endif

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
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
static struct spi_dev_s *spi4;
static struct sdio_dev_s *sdio;

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

	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN2);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN3);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	// stm32_configgpio(GPIO_ADC1_IN10);	/* used by VBUS valid */
	// stm32_configgpio(GPIO_ADC1_IN11);	/* unused */
	// stm32_configgpio(GPIO_ADC1_IN12);	/* used by MPU6000 CS */
	stm32_configgpio(GPIO_ADC1_IN13);	/* FMU_AUX_ADC_1 */
	stm32_configgpio(GPIO_ADC1_IN14);	/* FMU_AUX_ADC_2 */
	stm32_configgpio(GPIO_ADC1_IN15);	/* PRESSURE_SENS */

	/* configure power supply control/sense pins */
	stm32_configgpio(GPIO_VDD_5V_PERIPH_EN);
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_configgpio(GPIO_VDD_BRICK_VALID);
	stm32_configgpio(GPIO_VDD_SERVO_VALID);
	stm32_configgpio(GPIO_VDD_5V_HIPOWER_OC);
	stm32_configgpio(GPIO_VDD_5V_PERIPH_OC);

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure the DMA allocator */
	dma_alloc_init();

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
	led_off(LED_AMBER);

	/* Configure SPI-based devices */

	spi1 = up_spiinitialize(1);

	if (!spi1) {
		message("[boot] FAILED to initialize SPI port 1\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_ACCEL_MAG, false);
	SPI_SELECT(spi1, PX4_SPIDEV_BARO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_MPU, false);
	up_udelay(20);

	/* Get the SPI port for the FRAM */

	spi2 = up_spiinitialize(2);

	if (!spi2) {
		message("[boot] FAILED to initialize SPI port 2\n");
		up_ledon(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	 * and de-assert the known chip selects. */

	// XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, SPIDEV_FLASH, false);

	spi4 = up_spiinitialize(4);

	/* Default SPI4 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi4, 10000000);
	SPI_SETBITS(spi4, 8);
	SPI_SETMODE(spi4, SPIDEV_MODE3);
	SPI_SELECT(spi4, PX4_SPIDEV_EXT0, false);
	SPI_SELECT(spi4, PX4_SPIDEV_EXT1, false);

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
