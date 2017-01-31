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
 * nxphlite-v3 internal definitions
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
#include <kinetis.h>
#include <chip/kinetis_pinmux.h>
#include <arch/board/board.h>

/* NXPHLITE GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 * TBD (no makring on schematic)
 *   LED    K66
 *   ------ -------------------------------------------------------
 *   RED    FB_CS0_b/ UART2_CTS_b / ADC0_SE5b / SPI0_SCK / FTM3_CH1/ PTD1
 *   GREEN  FTM2_FLT0/ CMP0_IN3/ FB_AD6 / I2S0_RX_BCLK/ FTM3_CH5/ ADC1_SE5b/ PTC9
 *   BLUE   CMP0_IN2/ FB_AD7 / I2S0_MCLK/ FTM3_CH4/ ADC1_SE4b/ PTC8
 */

#define GPIO_LED_R         (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTD | PIN1)
#define GPIO_LED_G         (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTC | PIN9)
#define GPIO_LED_B         (GPIO_HIGHDRIVE | GPIO_OUTPUT_ONE | PIN_PORTC | PIN8)

/* High-resolution timer */
#define HRT_TIMER          3  /* use timer 2 for the HRT */
#define HRT_TIMER_CHANNEL  1  /* use capture/compare channel */

/* PPM IN
 *
 */

#define HRT_PPM_CHANNEL     0   /* use capture/compare channel */
#define GPIO_PPM_IN         PIN_FTM2_CH0_1    /* PTA10 220R to P4-1 */


/* IR transmitter & receiver
 *
 * Rx could be on UART0 RX
 * TX is On PTA24
 * Usage TBD - both left as inputs for now.
 *
 */
#define GPIO_IR_TRANSMITTER                (GPIO_PULLUP | PIN_PORTA | PIN24)
#define GPIO_IR_RRCEIVER                   (GPIO_PULLUP | PIN_PORTA | PIN21)

/* SBUS Control
 *
 * SBUS is brought out on P4-4 and controlled buy the following.
 * */

#define GPIO_SBUS_EN                       (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTA | PIN2)
#define GPIO_SBUS_IN  /* Not usable 0 needs UART */  (GPIO_PULLUP            | PIN_PORTA | PIN8)
#define GPIO_SBUS_OUT /* Not usable 0 needs UART */  (GPIO_INPUT             | PIN_PORTA | PIN9)
#define GPIO_RSSI_IN  /* Needs s/b on ADC/Timer  */  (GPIO_INPUT             | PIN_PORTA | PIN25)

/* Ethernet Control
 *
 * Unintalized to Reset Disabled and Inhibited
 */

#define GPIO_E_RST                         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN28)
#define GPIO_E_EN                          (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTA | PIN29)
#define GPIO_E_INH                         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN2)


/* CAN Control
 * Control pin S allows two operating modes to be selected:
 * high-speed mode (Low) or silent mode (high)
 */

#define GPIO_CAN0_STB                     (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN4)


/* URF Connector
 * TBD
 */

#define GPIO_ECH                           (GPIO_PULLUP | PIN_PORTC | PIN8)
#define GPIO_TRI                           (GPIO_PULLUP | PIN_PORTC | PIN9)

/* GPIO on UART P10
 * TBD
 */

#define GPIO_PTD11                        (GPIO_PULLUP | PIN_PORTD | PIN11)
#define GPIO_PTD12                        (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTD | PIN11)

/* GPIO on UART P14
 * TBD
 */

#define GPIO_PTD15                        (GPIO_PULLUP | PIN_PORTD | PIN15)
#define GPIO_PTE4                         (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTE | PIN4)

/* Safety Switch
 * TBD
 */
#define GPIO_LED_SAFETY                   (GPIO_HIGHDRIVE | GPIO_OUTPUT_ZER0  | PIN_PORTD | PIN13)
#define GPIO_BTN_SAFETY                   (GPIO_PULLUP | PIN_PORTD | PIN14)

/* NXPHlite-v3 GPIOs ****************************************************************/

/*	SDIO
 *
 *				NOT IN Current HW t uses SPI
 * A micro Secure Digital (SD) card slot is available on the board connected to
 * the SD Host Controller (SDHC) signals of the MCU. This slot will accept micro
 * format SD memory cards. The SD card detect pin (PTE6) is an open switch that
 * shorts with VDD when card is inserted.
 *
 *   ------------ ------------- --------
 *    SD Card Slot Board Signal  K66 Pin
 *    ------------ ------------- --------
 *    DAT0         SDHC0_D0      PTE0
 *    DAT1         SDHC0_D1      PTE1
 *    DAT2         SDHC0_D2      PTE5
 *    CD/DAT3      SDHC0_D3      PTE4
 *    CMD          SDHC0_CMD     PTE3
 *    CLK          SDHC0_DCLK    PTE2
 *    SWITCH       D_CARD_DETECT PTE6
 *    ------------ ------------- --------
 *
 * There is no Write Protect pin available to the K66 Or Card detect.
 */

/* SPI
 *
 *  SD Card is on SPI 0
 *	FXOS8700CQ Accelerometer & Magnetometer is on SPI 1
 *	FXAS21002CQ Gyro is on SPI 2
 */

/* SPI Bus assignments */

#define PX4_SPI_BUS_SDCARD                  0
#define PX4_SPI_BUS_ACCEL_MAG               1
#define PX4_SPI_BUS_GYRO                    2


/* SPI chip selects */

#define GPIO_SPI_CS_FXAS21002CQ_GYRO        (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN20)
#define GPIO_SPI_CS_FXOS8700CQ_ACCEL_MAG    (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN10)
#define GPIO_SPI_CS_SDCARD                  (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTC | PIN4)

/* SPI device reset signals
 * In Inactive state
 */

#define GPIO_GM_RST                         (GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTC | PIN0)
#define GPIO_A_RST                          (GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTB | PIN7)

/* Sensor interrupts */

#define GPIO_EXTI_GYRO_INT1                 (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTC | PIN1)
#define GPIO_EXTI_GYRO_INT2                 (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTC | PIN2)
#define GPIO_EXTI_ACCEL_MAG_INT1            (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTB | PIN8)
#define GPIO_EXTI_ACCEL_MAG_INT2            (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTB | PIN9)
#define GPIO_EXTI_BARO_INT1                 (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN26)
#define GPIO_EXTI_BARO_INT2                 (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTE | PIN27)

#define PX4_MK_SPI_SEL(b,d)      ((((b) & 0xf) << 4) + ((d) & 0xf))
#define PX4_SPI_BUS_ID(bd)       (((bd) >> 4) & 0xf)
#define PX4_SPI_DEV_ID(bd)       ((bd) & 0xf)

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */

#define PX4_SPIDEV_SDCARD        PX4_MK_SPI_SEL(PX4_SPI_BUS_SDCARD,0)
#define PX4_SDCARD_BUS_CS_GPIO   {GPIO_SPI_CS_SDCARD}
#define PX4_SDCARD_BUS_FIRST_CS  PX4_SPIDEV_SDCARD
#define PX4_SDCARD_BUS_LAST_CS   PX4_SPIDEV_SDCARD

#define PX4_SPIDEV_ACCEL_MAG        PX4_MK_SPI_SEL(PX4_SPI_BUS_ACCEL_MAG,0)
#define PX4_ACCEL_MAG_BUS_CS_GPIO   {GPIO_SPI_CS_FXOS8700CQ_ACCEL_MAG}
#define PX4_ACCEL_MAG_BUS_FIRST_CS  PX4_SPIDEV_ACCEL_MAG
#define PX4_ACCEL_MAG_BUS_LAST_CS   PX4_SPIDEV_ACCEL_MAG

#define PX4_SPIDEV_GYRO          PX4_MK_SPI_SEL(PX4_SPI_BUS_GYRO,0)
#define PX4_GYRO_BUS_CS_GPIO     {GPIO_SPI_CS_FXAS21002CQ_GYRO}
#define PX4_GYRO_BUS_FIRST_CS    PX4_SPIDEV_GYRO
#define PX4_GYRO_BUS_LAST_CS     PX4_SPIDEV_GYRO


/* I2C busses */

#define PX4_I2C_BUS_ONBOARD     1
#define PX4_I2C_BUS_EXPANSION   2

#define PX4_I2C_BUS_LED         PX4_I2C_BUS_EXPANSION

#define PX4_I2C_OBDEV_LED       0x55
#define PX4_I2C_OBDEV_HMC5883   0x1e
#define PX4_I2C_OBDEV_LIS3MDL   0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS 1 << 0 // TBD

// ADC defines to be used in sensors.cpp to read from a particular channel

// BAT_VSENS ADC1_SE16 or ADC0_SE22
// BAT_ISENS ADC0_SE16 or ADC0_SE21
// V5_VSENS  ADC1_SE18

// AD1  ADC1_SE5a
// AD2  ADC1_SE7a
// AD3  ADC0_SE10
// AD4  ADC1_SE12

#define ADC_BATTERY_VOLTAGE_CHANNEL 22
#define ADC_BATTERY_CURRENT_CHANNEL 21
#define ADC_5V_RAIL_SENSE           18

#define BOARD_BATTERY1_V_DIV   (10.177939394f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)

/* User GPIOs
 *
 * GPIO-
 * Define as GPIO input / GPIO outputs and timers IO
 */

#define PX4_MK_GPIO(pin_ftmx, io)    ((pin_ftmx) & (((uint32_t) ~(_PIN_MODE_MASK | _PIN_OPTIONS_MASK)) | (uint32_t)(io)))
#define PX4_MK_GPIO_INPUT(pin_ftmx)    PX4_MK_GPIO(pin_ftmx, GPIO_PULLUP)
#define PX4_MK_GPIO_OUTPUT(pin_ftmx)   PX4_MK_GPIO(pin_ftmx, GPIO_HIGHDRIVE)

#define GPIO_GPIO0_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH0OUT)
#define GPIO_GPIO1_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH1OUT)
#define GPIO_GPIO2_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH2OUT)
#define GPIO_GPIO3_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH3OUT)
#define GPIO_GPIO4_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH4OUT)
#define GPIO_GPIO5_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH5OUT)
#define GPIO_GPIO6_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH6OUT)
#define GPIO_GPIO7_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM3_CH0OUT)
#define GPIO_GPIO8_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM0_CH4OUT)
#define GPIO_GPIO9_INPUT    PX4_MK_GPIO_INPUT(GPIO_FTM0_CH5OUT)
#define GPIO_GPIO10_INPUT   PX4_MK_GPIO_INPUT(GPIO_FTM0_CH6OUT)
#define GPIO_GPIO11_INPUT   PX4_MK_GPIO_INPUT(GPIO_FTM0_CH7OUT)
#define GPIO_GPIO12_INPUT   PX4_MK_GPIO_INPUT(GPIO_FTM0_CH3OUT)
#define GPIO_GPIO13_INPUT   PX4_MK_GPIO_INPUT(GPIO_FTM0_CH2OUT)

#define GPIO_GPIO0_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH0OUT)
#define GPIO_GPIO1_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH1OUT)
#define GPIO_GPIO2_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH2OUT)
#define GPIO_GPIO3_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH3OUT)
#define GPIO_GPIO4_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH4OUT)
#define GPIO_GPIO5_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH5OUT)
#define GPIO_GPIO6_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH6OUT)
#define GPIO_GPIO7_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM3_CH0OUT)
#define GPIO_GPIO8_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH4OUT)
#define GPIO_GPIO9_OUTPUT   PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH5OUT)
#define GPIO_GPIO10_OUTPUT  PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH6OUT)
#define GPIO_GPIO11_OUTPUT  PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH7OUT)
#define GPIO_GPIO12_OUTPUT  PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH3OUT)
#define GPIO_GPIO13_OUTPUT  PX4_MK_GPIO_OUTPUT(GPIO_FTM0_CH2OUT)

/* Timer I/O PWM and capture
 *
 * 14 PWM outputs are configured.
 * 14 Timer inputs are configured.
 *
 * Pins:
 *      Defined in board.h
 */
// todo:Desine this!

#define DIRECT_PWM_OUTPUT_CHANNELS	14
#define DIRECT_INPUT_TIMER_CHANNELS 14

/* Power supply control and monitoring GPIOs */
// None

#define GPIO_PERIPH_3V3_EN  0

#define GPIO_SBUS_INV       0
#define INVERT_RC_INPUT(_s) px4_arch_gpiowrite(GPIO_SBUS_INV, _s)


/* Tone alarm output is On E28 */
#define TONE_ALARM_TIMER    2   /* timer */
#define TONE_ALARM_CHANNEL  3   /* channel  */
#define GPIO_TONE_ALARM_IDLE	(GPIO_LOWDRIVE | GPIO_OUTPUT_ZER0 | PIN_PORTE | PIN28)
#define GPIO_TONE_ALARM			(GPIO_LOWDRIVE | GPIO_OUTPUT_ONE  | PIN_PORTE | PIN28)

/* USB
 *
 *  Note No external VBUD detection
 */


/* PWM input driver. Use FMU PWM14 pin
 * todo:desing this
 */
#define PWMIN_TIMER		0
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_FTM0_CH2IN

#define BOARD_NAME "NXPHLITE_V3"

/* Not Supported in V1 HW
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (0)
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,       0}, \
		{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,       0}, \
		{GPIO_GPIO8_INPUT,       GPIO_GPIO8_OUTPUT,       0}, \
		{GPIO_GPIO9_INPUT,       GPIO_GPIO9_OUTPUT,       0}, \
		{GPIO_GPIO10_INPUT,      GPIO_GPIO10_OUTPUT,      0}, \
		{GPIO_GPIO11_INPUT,      GPIO_GPIO11_OUTPUT,      0}, \
		{GPIO_GPIO12_INPUT,      GPIO_GPIO12_OUTPUT,      0}, \
		{GPIO_GPIO13_INPUT,      GPIO_GPIO13_OUTPUT,      0}, \
	}
/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1          (1<<0)  /**< servo 1 output */
#define GPIO_SERVO_2          (1<<1)  /**< servo 2 output */
#define GPIO_SERVO_3          (1<<2)  /**< servo 3 output */
#define GPIO_SERVO_4          (1<<3)  /**< servo 4 output */
#define GPIO_SERVO_5          (1<<4)  /**< servo 5 output */
#define GPIO_SERVO_6          (1<<5)  /**< servo 6 output */
#define GPIO_SERVO_7          (1<<6)  /**< servo 7 output */
#define GPIO_SERVO_8          (1<<7)  /**< servo 8 output */
#define GPIO_SERVO_9          (1<<8)  /**< servo 9 output */
#define GPIO_SERVO_10         (1<<9)  /**< servo 9 output */
#define GPIO_SERVO_11         (1<<10) /**< servo 10 output */
#define GPIO_SERVO_12         (1<<11) /**< servo 11 output */
#define GPIO_SERVO_13         (1<<12) /**< servo 12 output */
#define GPIO_SERVO_14         (1<<13) /**< servo 13 output */

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120


/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: nxphlite_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXPHlite-v3 board.
 *
 ************************************************************************************/

void nxphlite_spidev_initialize(void);

/************************************************************************************
 * Name: nxphlite_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

int  nxphlite_spi_bus_initialize(void);

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/
void board_spi_reset(int ms);
void board_peripheral_reset(int ms);

/************************************************************************************
 * Name: nxphlite_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the NXPHlite-v3 board.
 *
 ************************************************************************************/

void nxphlite_usbinitialize(void);

/************************************************************************************
 * Name: nxphlite_bringup
 *
 * Description:
 *   Bring up board features
 *
 ************************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_INITIALIZE)
int nxphlite_bringup(void);
#endif

/****************************************************************************
 * Name: nxphlite_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int nxphlite_sdhc_initialize(void);
#else
#  define nxphlite_sdhc_initialize() (OK)
#endif

/****************************************************************************
 * Name: nxphlite_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

int nxphlite_sdio_initialize(void);

/************************************************************************************
 * Name: nxphlite_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool nxphlite_cardinserted(void);
#else
#  define nxphlite_cardinserted() (false)
#endif

/************************************************************************************
 * Name: nxphlite_writeprotected
 *
 * Description:
 *   Check if the card in the MMC/SD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
bool nxphlite_writeprotected(void);
#else
#  define nxphlite_writeprotected() (false)
#endif

/************************************************************************************
 * Name:  nxphlite_automount_initialize
 *
 * Description:
 *   Configure auto-mounter for the configured SDHC slot
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ************************************************************************************/

#ifdef HAVE_AUTOMOUNTER
void nxphlite_automount_initialize(void);
#endif

/************************************************************************************
 * Name:  nxphlite_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal event.  It
 *   has already scheduled the MMC/SD block driver operations.  Now we need to
 *   schedule the auto-mount event which will occur with a substantial delay to make
 *   sure that everything has settle down.
 *
 * Input Parameters:
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
void nxphlite_automount_event(bool inserted);
#endif

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
