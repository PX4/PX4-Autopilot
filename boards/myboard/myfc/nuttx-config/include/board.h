#ifndef __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H
#define __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
#include <nuttx/config.h>
// #include <px4_arch/micro_hal.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
#define BOARD_PLL_SYS_FREQ      (125 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (125 * MHZ)
#define BOARD_PERI_FREQ         (125 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_RTC_FREQ          46875

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * omnibusf4sd.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 */
#define LED_STACKCREATED  3  /* LED1 + LED2 */
#define LED_INIRQ         4  /* LED1 */
#define LED_SIGNAL        5  /* LED2 */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED1 + LED2 */

/* SPI0:
 *  Sensor not decided
 *  CS: not decided -- should be configured in sec/spi.cpp (probably)
 *  CLK: not decided
 *  MISO: not decided
 *  MOSI: not decided
 * Setup is done on pins 0-3, take a look at rp2040 datasheet pg. 259
 */

#define GPIO_SPI0_SCK  ( 2 | RP2040_GPIO_FUNC_SPI )
#define GPIO_SPI0_MISO ( 0 | RP2040_GPIO_FUNC_SPI )
#define GPIO_SPI0_MOSI ( 3 | RP2040_GPIO_FUNC_SPI )

/* SPI1:
 *  Sensor not decided
 *  CS: not decided -- should be configured in sec/spi.cpp (probably)
 *  CLK: not decided
 *  MISO: not decided
 *  MOSI: not decided
 * Setup is done on pins 8-11, take a look at rp2040 datasheet pg. 259
 */

#define GPIO_SPI1_SCK	( 10 | RP2040_GPIO_FUNC_SPI )
#define GPIO_SPI1_MISO	( 8 | RP2040_GPIO_FUNC_SPI )
#define GPIO_SPI1_MOSI	( 11 | RP2040_GPIO_FUNC_SPI )

#endif  /* __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H */
