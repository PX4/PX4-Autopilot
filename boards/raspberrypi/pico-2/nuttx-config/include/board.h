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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "rp23xx_i2cdev.h"
#include "rp23xx_spidev.h"
#include "rp23xx_i2sdev.h"
#include "rp23xx_spisd.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
#define BOARD_PLL_SYS_FREQ      (150 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (150 * MHZ)
#define BOARD_PERI_FREQ         (150 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_HSTX_FREQ         (150 * MHZ)

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* definitions for pico-sdk */

/* GPIO definitions *********************************************************/

#define BOARD_GPIO_LED_PIN      25
#define BOARD_NGPIOOUT          1
#define BOARD_NGPIOIN           1
#define BOARD_NGPIOINT          1

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way. The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

#define BOARD_LED_GREEN   BOARD_LED1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* This LED is not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/rp23xx_autoleds.c. The LED is used to encode
 * OS-related events as follows:
 *
 *   -------------------- ----------------------------- ------
 *   SYMBOL                   Meaning                   LED
 *   -------------------- ----------------------------- ------
 */

#define LED_STARTED       0  /* NuttX has been started  OFF    */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated OFF    */
#define LED_IRQSENABLED   0  /* Interrupts enabled      OFF    */
#define LED_STACKCREATED  1  /* Idle stack created      ON     */
#define LED_INIRQ         2  /* In an interrupt         N/C    */
#define LED_SIGNAL        2  /* In a signal handler     N/C    */
#define LED_ASSERTION     2  /* An assertion failed     N/C    */
#define LED_PANIC         3  /* The system has crashed  FLASH  */
#undef  LED_IDLE             /* Not used                       */

/* Thus if the LED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  If the LED is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* BUTTON definitions *******************************************************/

#define NUM_BUTTONS       0

#define BUTTON_USER1      0
#define BUTTON_USER2      1
#define BUTTON_USER1_BIT  (1 << BUTTON_USER1)
#define BUTTON_USER2_BIT  (1 << BUTTON_USER2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardearlyinitialize(void);

/****************************************************************************
 * Name: rp23xx_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

/* Alternate function pin selections ************************************************/

/*
 * UARTs.
 * UART0TX: GPIO0
 * UART0RX: GPIO1
 * UART1TX: GPIO8
 * UART1RX: GPIO9
 */
#define CONFIG_RP23XX_UART0_GPIO	0	/* TELEM */

#define CONFIG_RP23XX_UART1_GPIO	8	/* GPS */

/*
 * I2C (external)
 *
 * I2C1SCL: GPIO7
 * I2C1SDA: GPIO6
 *
 * TODO:
 *   The optional _GPIO configurations allow the I2C driver to manually
 *   reset the bus to clear stuck slaves.  They match the pin configuration,
 *   but are normally-high GPIOs.
 */
#define CONFIG_RP23XX_I2C1_GPIO		6

/* SPI0:
 *  SPIDEV_FLASH (probably micro sd card)
 *  CS: GPIO5 -- should be configured in sec/spi.cpp (probably)
 *  CLK: GPIO2
  * MOSI: GPIO3
 *  MISO: GPIO4
 */


#define GPIO_SPI0_SCLK  ( 2 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MISO ( 4 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MOSI ( 3 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )

/* SPI1:
 *  MPU9250 and BMP280
 *  CS: GPIO13 for MPU9250, GPIO14 for BMP280 -- should be configured in sec/spi.cpp (probably)
 *  CLK: GPIO10
 *  MISO: GPIO12
 *  MOSI: GPIO11
 */

#define GPIO_SPI1_SCLK	( 10 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MISO	( 12 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MOSI	( 11 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )


#endif  /* __ARCH_BOARD_BOARD_H */
