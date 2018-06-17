/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4SAME70_XPLAINEDV1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <chip.h>
#include <sam_gpio.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4_SAME70XPLAINED_V1 GPIO Pin Definitions *************************************************/

/* Ethernet MAC.
 *
 * KSZ8081RNACA Connections
 * ------------------------
 *
 *   ------ --------- ---------
 *   SAME70 SAME70    Ethernet
 *   Pin    Function  Functio
 *   ------ --------- ---------
 *   PD0    GTXCK     REF_CLK
 *   PD1    GTXEN     TXEN
 *   PD2    GTX0      TXD0
 *   PD3    GTX1      TXD1
 *   PD4    GRXDV     CRS_DV
 *   PD5    GRX0      RXD0
 *   PD6    GRX1      RXD1
 *   PD7    GRXER     RXER
 *   PD8    GMDC      MDC
 *   PD9    GMDIO     MDIO
 *   PA14   GPIO      INTERRUPT
 *   PC10   GPIO      RESET
 *   ------ --------- ---------
 */

#define GPIO_EMAC0_INT    (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
			   GPIO_INT_FALLING | GPIO_PORT_PIOA | GPIO_PIN14)
#define GPIO_EMAC0_RESET  (GPIO_OUTPUT | GPIO_CFG_PULLUP | GPIO_OUTPUT_SET | \
			   GPIO_PORT_PIOC | GPIO_PIN10)

#define IRQ_EMAC0_INT     SAM_IRQ_PA14

/* LEDs
 *
 * A single LED is available driven by PC8.
 */

#define GPIO_LED1     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
		       GPIO_PORT_PIOC | GPIO_PIN8)

/* Buttons
 *
 * SAM E70 Xplained contains two mechanical buttons. One button is the RESET
 * button connected to the SAM E70 reset line and the other, PA11, is a generic
 * user configurable button. When a button is pressed it will drive the I/O
 * line to GND.
 *
 * NOTE: There are no pull-up resistors connected to the generic user buttons
 * so it is necessary to enable the internal pull-up in the SAM E70 to use the
 * button.
 */

#define GPIO_SW0      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
		       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN11)
#define IRQ_SW0       SAM_IRQ_PA11

/* HSMCI SD Card Detect
 *
 * The SAM E70 Xplained has one standard SD card connector that is connected
 * to the High Speed Multimedia Card Interface (HSMCI) of the SAM E70. SD
 * card connector:
 *
 *   ------ ----------------- ---------------------
 *   SAME70 SAME70            Shared functionality
 *   Pin    Function
 *   ------ ----------------- ---------------------
 *   PA30   MCDA0 (DAT0)
 *   PA31   MCDA1 (DAT1)
 *   PA26   MCDA2 (DAT2)
 *   PA27   MCDA3 (DAT3)      Camera
 *   PA25   MCCK (CLK)        Shield
 *   PA28   MCCDA (CMD)
 *   PC16   Card Detect (C/D) Shield
 *   ------ ----------------- ---------------------
 */

#define GPIO_MCI0_CD (GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		      GPIO_INT_BOTHEDGES | GPIO_PORT_PIOC | GPIO_PIN16)
#define IRQ_MCI0_CD   SAM_IRQ_PC16

/* USB Host
 *
 * The SAM E70 Xplained has a Micro-USB connector for use with the SAM E70 USB
 * module labeled as TARGET USB on the kit. In USB host mode VBUS voltage is
 * provided by the kit and has to be enabled by setting the "VBUS Host Enable"
 * pin (PC16) low.
 */

#define GPIO_VBUSON (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
		     GPIO_PORT_PIOC | GPIO_PIN16)



/* External interrupts */
//todo:DBS fix all these
#define GPIO_EXTI_GYRO_DRDY		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_INT_RISING | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_EXTI_MAG_DRDY		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_INT_RISING | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_EXTI_ACCEL_DRDY	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_INT_RISING | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_EXTI_MPU_DRDY		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_INT_RISING | GPIO_PORT_PIOD | GPIO_PIN30)

/* Data ready pins off */
#define GPIO_GYRO_DRDY_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_MAG_DRDY_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_ACCEL_DRDY_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN30)
#define GPIO_EXTI_MPU_DRDY_OFF	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN30)

/* SPI1 off */
#define GPIO_SPI1_SCK_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN22)
#define GPIO_SPI1_MISO_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN20)
#define GPIO_SPI1_MOSI_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN21)

/* SPI1 chip selects off */
#define GPIO_SPI_CS_GYRO_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOC | GPIO_PIN13)
#define GPIO_SPI_CS_ACCEL_MAG_OFF	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOC | GPIO_PIN17 )
#define GPIO_SPI_CS_BARO_OFF		(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN11)
#define GPIO_SPI_CS_MPU_OFF			(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
		GPIO_CFG_PULLDOWN | GPIO_PORT_PIOD | GPIO_PIN27)

/* SPI chip selects */
#define GPIO_SPI_CS_GYRO		(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOC | GPIO_PIN13)
#define GPIO_SPI_CS_ACCEL_MAG	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOC | GPIO_PIN17 )
#define GPIO_SPI_CS_BARO		(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOD | GPIO_PIN11)
#define GPIO_SPI_CS_MPU			(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOD | GPIO_PIN27)

#define PX4_SPI_BUS_SENSORS PX4_BUS_NUMBER_TO_PX4(0)
#define PX4_SPI_BUS_BARO	PX4_SPI_BUS_SENSORS

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_GYRO		1
#define PX4_SPIDEV_ACCEL_MAG	2
#define PX4_SPIDEV_BARO		3
#define PX4_SPIDEV_MPU		4


/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	PX4_BUS_NUMBER_TO_PX4(0)
/* No Onboard Sensors #define PX4_I2C_BUS_ONBOARD		0 */
#define PX4_I2C_BUS_LED			PX4_I2C_BUS_EXPANSION

/* Define the follwoing to output the clock on J500-1 */
//#define GPIO_PCK1			(GPIO_PERIPHB | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | GPIO_PIN17)


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15

/* r GPIOs
 *
 * GPIO0-3 are the PWM servo outputs.
 */


/* User GPIOs
 *
 * GPIO0-3 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_CFG_PULLDOWN | GPIO_PORT_PIOC | GPIO_PIN19)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_CFG_PULLDOWN | GPIO_PORT_PIOC | GPIO_PIN26)
#define GPIO_GPIO2_INPUT	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_CFG_PULLDOWN | GPIO_PORT_PIOA | GPIO_PIN2)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT | GPIO_CFG_DEFAULT | GPIO_CFG_DEGLITCH | \
				 GPIO_CFG_PULLDOWN | GPIO_PORT_PIOA | GPIO_PIN24)
#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOC | GPIO_PIN19)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOC | GPIO_PIN26)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOA | GPIO_PIN2)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | GPIO_PORT_PIOA | GPIO_PIN24)


/* Tone alarm output */
#define TONE_ALARM_CHANNEL	5	/* channel TC 1 Chan 5 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | GPIO_PORT_PIOC | GPIO_PIN30)
#define GPIO_TONE_ALARM			GPIO_TC5_TIOB

#if 0

/* PWM
 *
 * Six PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE14 : TIM1_CH4
 * CH2 : PE13 : TIM1_CH3
 * CH3 : PE11 : TIM1_CH2
 * CH4 : PE9  : TIM1_CH1
 * CH5 : PD13 : TIM4_CH2
 * CH6 : PD14 : TIM4_CH3
 */
#define GPIO_TIM1_CH1OUT	GPIO_TIM1_CH1OUT_2
#define GPIO_TIM1_CH2OUT	GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH3OUT	GPIO_TIM1_CH3OUT_2
#define GPIO_TIM1_CH4OUT	GPIO_TIM1_CH4OUT_2
#define GPIO_TIM4_CH2OUT	GPIO_TIM4_CH2OUT_2
#define GPIO_TIM4_CH3OUT	GPIO_TIM4_CH3OUT_2
#endif
#define DIRECT_PWM_OUTPUT_CHANNELS	1
#define DIRECT_INPUT_TIMER_CHANNELS  1

/* High-resolution timer */
/* sam timers are usually addressed by channel number 0-11
 * Timer 0 has Channel 0,1,2, Timer 1 has  channels 3, 4, 5,
 * Timer 2 has Channel 6,7,8, Timer 3 has  channels 9, 10, 11,
 */
#define HRT_TIMER_CHANNEL	2	/* use channel 2 */

#if 0
/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_2
#endif

#define BOARD_NAME "PX4_SAME70XPLAINED_V1"

/*
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
/*
#define BOARD_ADC_USB_CONNECTED board_read_VBUS_state()
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)
*/

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, }

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
# define GPIO_SERVO_1          (1<<0)          /**< servo 1 output */
# define GPIO_SERVO_2          (1<<1)          /**< servo 2 output */
# define GPIO_SERVO_3          (1<<2)          /**< servo 3 output */
# define GPIO_SERVO_4          (1<<3)          /**< servo 4 output */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/************************************************************************************
 * Name: sam_sdram_config
 *
 * Description:
 *   Configures the on-board SDRAM.  SAME70 Xplained features one external
 *   IS42S16100E-7BLI, 512Kx16x2, 10ns, SDRAM. SDRAM0 is connected to chip select
 *   NCS1.
 *
 *  Input Parameters:
 *     None
 *
 *  Assumptions:
 *    The DDR memory regions is configured as strongly ordered memory.  When we
 *    complete initialization of SDRAM and it is ready for use, we will make DRAM
 *    into normal memory.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_SDRAMC
void sam_sdram_config(void);
#else
#  define sam_sdram_config(t)
#endif

/************************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Bring up board features
 *
 ************************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)
int sam_bringup(void);
#endif

/************************************************************************************
 * Name: sam_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAME70-XPLD board.
 *
 ************************************************************************************/

#ifdef CONFIG_SAMV7_SPI
#define board_spi_reset(ms)
void board_spi_initialize(void);
#endif

#define board_peripheral_reset(ms)

/************************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int slot, int minor);
#else
# define sam_hsmci_initialize(s,m) (-ENOSYS)
#endif

/************************************************************************************
 * Name:  board_usb_initialize
 *
 * Description:
 *   Called from stm32_boardinitialize very early in initialization to setup USB-
 *   related GPIO pins for the SAME70-XPLD board.
 *
 ************************************************************************************/

void board_usb_initialize(void);

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

#ifdef HAVE_NETWORK
void sam_netinitialize(void);
#endif

/************************************************************************************
 * Name: sam_emac0_setmac
 *
 * Description:
 *   Read the Ethernet MAC address from the AT24 FLASH and configure the Ethernet
 *   driver with that address.
 *
 ************************************************************************************/

#ifdef HAVE_MACADDR
int sam_emac0_setmac(void);
#endif

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#else
#  define sam_cardinserted(slotno) (false)
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#endif

/************************************************************************************
 * Name:  sam_automount_initialize
 *
 * Description:
 *   Configure auto-mounters for each enable and so configured HSMCI
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_initialize(void);
#endif

/************************************************************************************
 * Name:  sam_automount_event
 *
 * Description:
 *   The HSMCI card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
 *   slotno - Identifies the HSMCI0 slot: HSMCI0 or HSMCI1_SLOTNO.  There is a
 *      terminology problem here:  Each HSMCI supports two slots, slot A and slot B.
 *      Only slot A is used.  So this is not a really a slot, but an HSCMI peripheral
 *      number.
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void sam_automount_event(int slotno, bool inserted);
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

/************************************************************************************
 * Name: sam_at24config
 *
 * Description:
 *   Create an AT24xx-based MTD configuration device for storage device configuration
 *   information.
 *
 ************************************************************************************/

#ifdef HAVE_MTDCONFIG
int sam_at24config(void);
#endif

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
