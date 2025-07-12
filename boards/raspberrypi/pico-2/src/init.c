/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * board specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#if defined(CONFIG_NSH_MMCSDSPIPORTNO)
	#include <nuttx/mmcsd.h>
#endif
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include "board_config.h"
//platforms/nuttx/src/px4/rpi/rpi_common/include/px4_arch/micro_hal.h
#include <px4_arch/micro_hal.h>
#include <rp23xx_uart.h>
#include <rp23xx_gpio.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

# if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

#ifdef CONFIG_RP23XX_FLASH_FILE_SYSTEM
#  include "rp23xx_flash_mtd.h"
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/**
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
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);
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
	// Configure the GPIO pins to outputs and keep them low.
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	/*
	 * On resets invoked from system (not boot) insure we establish a low
	 * output state (discharge the pins) on PWM pins before they become inputs.
	 */

	if (status >= 0) {
		up_mdelay(400);
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
    // FIXME: it reads "USB/VBUS not connected" now !?
    return BOARD_ADC_USB_CONNECTED ? 0 : 1; // FIXME:  ?
    // return 0;
}

/****************************************************************************
 * Name: rp23xx_boardearlyinitialize
 *
 * Description:
 *
 * This function is taken directly from nuttx's rp23xx_boardinitialize.c
 ****************************************************************************/

void rp23xx_boardearlyinitialize(void)
{

	// reset all GPIO functions!
	rp23xx_gpio_initialize();

	/* Set default UART pin */
#if defined(CONFIG_RP23XX_UART0) && CONFIG_RP23XX_UART0_GPIO >= 0
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_GPIO, RP23XX_GPIO_FUNC_UART);     /* TX */
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_GPIO + 1, RP23XX_GPIO_FUNC_UART);     /* RX */
# ifdef CONFIG_SERIAL_OFLOWCONTROL
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_GPIO + 2, RP23XX_GPIO_FUNC_UART);     /* CTS */
# endif /* CONFIG_SERIAL_OFLOWCONTROL */
# ifdef CONFIG_SERIAL_IFLOWCONTROL
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART0_GPIO + 3, RP23XX_GPIO_FUNC_UART);     /* RTS */
# endif /* CONFIG_SERIAL_IFLOWCONTROL */
#endif

#if defined(CONFIG_RP23XX_UART1) && CONFIG_RP23XX_UART1_GPIO >= 0
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_GPIO, RP23XX_GPIO_FUNC_UART);     /* TX */
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_GPIO + 1, RP23XX_GPIO_FUNC_UART);     /* RX */
# ifdef CONFIG_SERIAL_OFLOWCONTROL
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_GPIO + 2, RP23XX_GPIO_FUNC_UART);     /* CTS */
# endif /* CONFIG_SERIAL_OFLOWCONTROL */
# ifdef CONFIG_SERIAL_IFLOWCONTROL
	rp23xx_gpio_set_function(CONFIG_RP23XX_UART1_GPIO + 3, RP23XX_GPIO_FUNC_UART);     /* RTS */
# endif /* CONFIG_SERIAL_IFLOWCONTROL */
#endif
}

/************************************************************************************
 * Name: rp23xx_boardinitialize
 *
 * Description:
 *   All architectures must provide the following entry point. This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void _setup_i2c_pins(uint32_t gpio){
	// platforms/nuttx/NuttX/nuttx/arch/arm/src/rp23xx/rp23xx_gpio.c
	uint32_t gpio_sda = gpio;
	uint32_t gpio_scl = gpio + 1;
	rp23xx_gpio_set_function(gpio_sda, RP23XX_GPIO_FUNC_I2C);      /* SDA */
	rp23xx_gpio_set_function(gpio_scl, RP23XX_GPIO_FUNC_I2C);      /* SCL */
	rp23xx_gpio_set_pulls(gpio_sda, true, false);  /* Pull up */
	rp23xx_gpio_set_pulls(gpio_scl, true, false);
}

__EXPORT void
rp23xx_boardinitialize(void)
{
	// FIXME: something is very bad in this method somewhere, breaking UART0 TX
	// /* Reset all PWM to Low outputs */
	board_on_reset(-1);

	// /* configure LEDs */
	board_autoled_initialize();

	// Disable IE and enable OD on GPIO 26-29 (These are ADC Pins)
	// Do this only for the channels configured in board_config.h
	rp23xx_gpioconfig(27 | GPIO_FUN(RP23XX_GPIO_FUNC_NULL));		/* BATT_VOLTAGE_SENS */
	clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(27));	/* BATT_VOLTAGE_SENS */
	setbits_reg32(RP23XX_PADS_BANK0_GPIO_OD, RP23XX_PADS_BANK0_GPIO(27));	/* BATT_VOLTAGE_SENS */
	rp23xx_gpioconfig(28 | GPIO_FUN(RP23XX_GPIO_FUNC_NULL));		/* BATT_VOLTAGE_SENS */
	clrbits_reg32(RP23XX_PADS_BANK0_GPIO_IE, RP23XX_PADS_BANK0_GPIO(28));	/* BATT_CURRENT_SENS */
	setbits_reg32(RP23XX_PADS_BANK0_GPIO_OD, RP23XX_PADS_BANK0_GPIO(28));	/* BATT_CURRENT_SENS */

	/* Set default I2C pin */
#if defined(CONFIG_RP23XX_I2C0) && CONFIG_RP23XX_I2C0_GPIO >= 0
	_setup_i2c_pins(CONFIG_RP23XX_I2C0_GPIO);
#endif

#if defined(CONFIG_RP23XX_I2C1) &&  CONFIG_RP23XX_I2C1_GPIO >= 0
	_setup_i2c_pins(CONFIG_RP23XX_I2C1_GPIO);
#endif


	// // TODO: power peripherals

	// configure VBUS sense GPIO
	rp23xx_usbinitialize();

	// ///* configure power supply control/sense pins */
	// FIXME: stm32_configgpio(GPIO_PERIPH_3V3_EN);
	// //stm32_configgpio(GPIO_VDD_BRICK_VALID);
	// //stm32_configgpio(GPIO_VDD_USB_VALID);

	// // TODO: 3v3 Sensor?
	// ///* Start with Sensor voltage off We will enable it
	// // * in board_app_initialize
	// // */
	// //stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);

	// // TODO: SBUS inversion? SPEK power?
	// //stm32_configgpio(GPIO_SBUS_INV);
	// //stm32_configgpio(GPIO_SPEKTRUM_PWR_EN);

	// // TODO: $$$ Unused?
	// //stm32_configgpio(GPIO_8266_GPIO0);
	// //stm32_configgpio(GPIO_8266_PD);
	// //stm32_configgpio(GPIO_8266_RST);

	// /* Safety - led don in led driver */

	// // TODO: unused?
	// //stm32_configgpio(GPIO_BTN_SAFETY);

	// // TODO: RSSI
	// //stm32_configgpio(GPIO_RSSI_IN);

	// stm32_configgpio(GPIO_PPM_IN);

	/* configure SPI all interfaces GPIO */
	// FIXME - UART messed up here!
	// platforms/nuttx/src/px4/rpi/rpi_common/spi/spi.cpp
	rp23xx_spiinitialize();

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
#if defined(CONFIG_NSH_MMCSDSPIPORTNO)
static struct spi_dev_s *spi1;
#endif

static struct spi_dev_s *spi2;

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();

	/* configure the DMA allocator */				// Needs to be figured out

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

	/* set up the serial DMA polling */	// RP2040 nuttx implementation doesn't have serial_dma_poll function yet.
	// static struct hrt_call serial_dma_call;
	// struct timespec ts;

	// /*
	//  * Poll at 1ms intervals for received bytes that have not triggered
	//  * a DMA event.
	//  */
	// ts.tv_sec = 0;
	// ts.tv_nsec = 1000000;

	// hrt_call_every(&serial_dma_call,
	// 	       ts_to_abstime(&ts),
	// 	       ts_to_abstime(&ts),
	// 	       (hrt_callout)stm32_serial_dma_poll,
	// 	       NULL);

	/* initial LED state */
	drv_led_start();
	led_on(LED_BLUE);

	// if (board_hardfault_init(2, true) != 0) {		// Needs to be figured out as RP2040 doesn't have BBSRAM.
	// 	led_off(LED_BLUE);
	// }


	/* Configure SPI-based devices */

	#if defined(CONFIG_NSH_MMCSDSPIPORTNO)
		// // SPI1: SDCard					// Will be configured later
		// /* Get the SPI port for the microSD slot */
		 spi1 = rp23xx_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO); // PX4_BUS_NUMBER_FROM_PX4(1)

		 if (!spi1) {
			syslog(LOG_ERR, "[boot] FAILED to initialize SPI port %d\n", CONFIG_NSH_MMCSDSPIPORTNO);
			led_off(LED_BLUE);
		 }

		// /* Now bind the SPI interface to the MMCSD driver */
		 int result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi1);

		 if (result != OK) {
			led_off(LED_BLUE);
			syslog(LOG_ERR, "[boot] FAILED to bind SPI port 1 to the MMCSD driver\n");
		 }
	  #endif

	 up_udelay(20);

	// SPI2: sensors
	#if defined(CONFIG_NSH_MMCSDSPIPORTNO)
  		// SPI2
        #define SENSOR_SPI_BUS    2
    #else
  	  #define SENSOR_SPI_BUS    2
    #endif

    #if defined(CONFIG_RP23XX_SPI0)
    	int sensors_spi_rp_port_num = PX4_BUS_NUMBER_FROM_PX4(SENSOR_SPI_BUS);
    	syslog(LOG_ERR, "[boot] initializing sensors on SPI%d\n", sensors_spi_rp_port_num);
	spi2 = rp23xx_spibus_initialize(sensors_spi_rp_port_num);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		led_off(LED_BLUE);
	} else {
		syslog(LOG_INFO, "[boot] initialized SPI sensors port 1\n");
	}

	/* Default SPI2 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	up_udelay(20);
	syslog(LOG_INFO, "[boot] initialized SPI sensors port 1\n");
    #else
    	syslog(LOG_ERR, "[boot] SKIPPED initializing sensors on SPI0 - disabled\n");
    #endif

 #if defined(FLASH_BASED_PARAMS)					// This probably doesn't relate to RP2040 right now.
 	static sector_descriptor_t params_sector_map[] = {
 		// FIXME: maybe we want to hide this memory from script.ld ?
 		// The W25Q128JV array is organized into 65,536 programmable pages of 256-bytes each. Up to 256 bytes
                //   can be programmed at a time. Pages can be erased in groups of 16 (4KB sector erase), groups of 128
                //   (32KB block erase), groups of 256 (64KB block erase) or the entire chip (chip erase). The W25Q128JV
                //    has 4,096 erasable sectors and 256 erasable blocks respectively. The small 4KB sectors allow for greater
                //    flexibility in applications that require data and parameter storage. (See Figure 2.)
 		// flash start + 4MB offset for code = 0x10000000 + 4mb = 268435456 + 4194304 = 272629760 = 0x10400000
 		{1, 32 * 1024, 0x10400000},
 		{0, 0, 0},
 	};

 	/* Initialize the flashfs layer to use heap allocated memory */
 	int result = parameter_flashfs_init(params_sector_map, (uint8_t *)NULL, (uint16_t)0);

 	if (result != OK) {
 		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
 		led_off(LED_AMBER);
 	} else {
 		syslog(LOG_INFO, "[boot] initialized params in FLASH\n");
 	}

 #endif


 #ifdef CONFIG_RP23XX_FLASH_FILE_SYSTEM
   syslog(LOG_INFO, "[boot] initializing params in FLASH - rp23xx_flash_mtd_initialize\n");
   up_udelay(300000);
   struct mtd_dev_s *mtd_dev;
   mtd_dev = rp23xx_flash_mtd_initialize();
   up_udelay(300000);
   if (mtd_dev == NULL)
     {
       syslog(LOG_ERR, "ERROR: flash_mtd_initialize failed: %d\n", errno);
       up_udelay(200);
     }
   else
     {
       syslog(LOG_INFO, "[boot] initializing params in FLASH - smart_initialize\n");
       up_udelay(2000);
       int ret = smart_initialize(0, mtd_dev, NULL);

       if (ret < 0)
         {
           syslog(LOG_ERR, "ERROR: smart_initialize failed: %d\n", -ret);
           up_udelay(200);
         }
       else if (sizeof(CONFIG_RP23XX_FLASH_MOUNT_POINT) > 1)
         {
           syslog(LOG_INFO, "[boot] initializing params in FLASH - nx_mount\n");
           up_udelay(2000);
           mkdir(CONFIG_RP23XX_FLASH_MOUNT_POINT, 0777);

           /* Mount the file system */

           ret = nx_mount("/dev/smart0",
                         CONFIG_RP23XX_FLASH_MOUNT_POINT,
                         "smartfs",
                         0,
                         NULL);
           if (ret < 0)
             {
               syslog(LOG_ERR,
                     "ERROR: nx_mount(\"/dev/smart0\", \"%s\", \"smartfs\","
                     " 0, NULL) failed: %d\n",
                     CONFIG_RP23XX_FLASH_MOUNT_POINT,
                     ret);
               up_udelay(2000);
             }
         }
       syslog(LOG_INFO, "[boot] initializing params in FLASH - done\n");
       up_udelay(2000);
     }
 #endif

	/* Configure the HW based on the manifest */
        syslog(LOG_INFO, "[boot] px4_platform_configure - starting\n");
        up_udelay(200);
	px4_platform_configure();
	syslog(LOG_INFO, "[boot] init done\n");
        up_udelay(200);

	return OK;
}
