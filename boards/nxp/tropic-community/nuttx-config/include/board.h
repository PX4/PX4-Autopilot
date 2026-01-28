/************************************************************************************
 * nuttx-configs/nxp/nxp_tropic-community/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __NUTTX_CONFIG_NXP_TROPIC_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_NXP_TROPIC_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Set VDD_SOC to 1.25V */

#define IMXRT_VDD_SOC (0x12)

/* Set Arm PLL (PLL1) to  fOut    = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        576Mhz  = (24Mhz * ARM_PLL_DIV_SELECT/2) / ARM_PODF_DIVISOR
 *                        ARM_PLL_DIV_SELECT = 96
 *                        ARM_PODF_DIVISOR   = 2
 *                        576Mhz  = (24Mhz * 96/2) / 2
 *
 *     AHB_CLOCK_ROOT             = PLL1fOut / IMXRT_AHB_PODF_DIVIDER
 *     1Hz to 600 Mhz             = 576Mhz / IMXRT_ARM_CLOCK_DIVIDER
 *                        IMXRT_ARM_CLOCK_DIVIDER = 1
 *                        576Mhz  = 576Mhz / 1
 *
 *     PRE_PERIPH_CLK_SEL         = PRE_PERIPH_CLK_SEL_PLL1
 *     PERIPH_CLK_SEL             = 1 (0 select PERIPH_CLK2_PODF, 1 select PRE_PERIPH_CLK_SEL_PLL1)
 *     PERIPH_CLK                 = 576Mhz
 *
 *     IPG_CLOCK_ROOT             = AHB_CLOCK_ROOT / IMXRT_IPG_PODF_DIVIDER
 *                       IMXRT_IPG_PODF_DIVIDER = 4
 *                       144Mhz = 576Mhz / 4
 *
 *     PRECLK_CLOCK_ROOT          = IPG_CLOCK_ROOT / IMXRT_PERCLK_PODF_DIVIDER
 *                       IMXRT_PERCLK_PODF_DIVIDER = 1
 *                       16Mhz = 144Mhz / 9
 *
 *     SEMC_CLK_ROOT              = 576Mhz / IMXRT_SEMC_PODF_DIVIDER (labeled AIX_PODF in 18.2)
 *                       IMXRT_SEMC_PODF_DIVIDER = 8
 *                       72Mhz    = 576Mhz / 8
 *
 * Set Sys PLL (PLL2) to  fOut    = (24Mhz * (20+(2*(DIV_SELECT)))
 *                        528Mhz  = (24Mhz * (20+(2*(1)))
 *
 * Set USB1 PLL (PLL3) to fOut    = (24Mhz * 20)
 *                         480Mhz = (24Mhz * 20)
 *
 * Set LPSPI PLL3 PFD0 to fOut    = (480Mhz / 12 * 18)
 *                        720Mhz  = (480Mhz / 12 * 18)
 *                         90Mhz  = (720Mhz / LSPI_PODF_DIVIDER)
 *
 * Set LPI2C PLL3 / 8 to   fOut   = (480Mhz / 8)
 *                         60Mhz  = (480Mhz / 8)
 *                         12Mhz  = (60Mhz / LSPI_PODF_DIVIDER)
 *
 * Set USDHC1 PLL2 PFD2 to fOut   = (528Mhz / 24 * 18)
 *                        396Mhz  = (528Mhz / 24 * 18)
 *                        198Mhz  = (396Mhz / IMXRT_USDHC1_PODF_DIVIDER)
 */

#define BOARD_XTAL_FREQUENCY      24000000
#define IMXRT_PRE_PERIPH_CLK_SEL  CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1
#define IMXRT_PERIPH_CLK_SEL      CCM_CBCDR_PERIPH_CLK_SEL_PRE_PERIPH
#define IMXRT_ARM_PLL_DIV_SELECT  96
#define IMXRT_ARM_PODF_DIVIDER    2
#define IMXRT_AHB_PODF_DIVIDER    1
#define IMXRT_IPG_PODF_DIVIDER    4
#define IMXRT_PERCLK_CLK_SEL      CCM_CSCMR1_PERCLK_CLK_SEL_IPG_CLK_ROOT
#define IMXRT_PERCLK_PODF_DIVIDER 9
#define IMXRT_SEMC_PODF_DIVIDER   8
#define IMXRT_CAN_CLK_SELECT       CCM_CSCMR2_CAN_CLK_SEL_PLL3_SW_80
#define IMXRT_CAN_PODF_DIVIDER    1

#define IMXRT_LPSPI_CLK_SELECT    CCM_CBCMR_LPSPI_CLK_SEL_PLL3_PFD0
#define IMXRT_LSPI_PODF_DIVIDER   8

#define IMXRT_LPI2C_CLK_SELECT    CCM_CSCDR2_LPI2C_CLK_SEL_PLL3_60M
#define IMXRT_LSI2C_PODF_DIVIDER  5

#define IMXRT_USDHC1_CLK_SELECT    CCM_CSCMR1_USDHC1_CLK_SEL_PLL2_PFD0
#define IMXRT_USDHC1_PODF_DIVIDER 2

#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20

#define IMXRT_SYS_PLL_SELECT      CCM_ANALOG_PLL_SYS_DIV_SELECT_22

#define IMXRT_USB1_PLL_DIV_SELECT  CCM_ANALOG_PLL_USB1_DIV_SELECT_20

#define BOARD_CPU_FREQUENCY \
	(BOARD_XTAL_FREQUENCY * (IMXRT_ARM_PLL_DIV_SELECT / 2)) / IMXRT_ARM_PODF_DIVIDER

#define BOARD_GPT_FREQUENCY \
	(BOARD_CPU_FREQUENCY / IMXRT_IPG_PODF_DIVIDER) / IMXRT_PERCLK_PODF_DIVIDER


/* 108Mhz clock for FlexIO using PLL3 PFD2 @ 520 */
#define CONFIG_FLEXIO1_CLK          1
#define CONFIG_FLEXIO1_PRED_DIVIDER 5
#define CONFIG_FLEXIO1_PODF_DIVIDER 1
#define CONFIG_PLL3_PFD2_FRAC       16
#define BOARD_FLEXIO_PREQ           108000000

/* Define this to enable tracing */
#if CONFIG_USE_TRACE
#  define IMXRT_TRACE_PODF_DIVIDER 1
#  define IMXRT_TRACE_CLK_SELECT   CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0
#endif

/* SDIO *****************************************************************************/

/* Pin drive characteristics */

#define USDHC1_DATAX_IOMUX  (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define USDHC1_CMD_IOMUX    (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)
#define USDHC1_CLK_IOMUX    (IOMUX_SLEW_FAST | IOMUX_DRIVE_130OHM | IOMUX_SPEED_MAX)
#define USDHC1_CD_IOMUX     (IOMUX_PULL_UP_47K | IOMUX_SCHMITT_TRIGGER)

#define PIN_USDHC1_D0       (GPIO_USDHC1_DATA0_1 | USDHC1_DATAX_IOMUX) /* GPIO_SD_B0_02 */
#define PIN_USDHC1_D1       (GPIO_USDHC1_DATA1_1 | USDHC1_DATAX_IOMUX) /* GPIO_SD_B0_03 */
#define PIN_USDHC1_D2       (GPIO_USDHC1_DATA2_1 | USDHC1_DATAX_IOMUX) /* GPIO_SD_B0_04 */
#define PIN_USDHC1_D3       (GPIO_USDHC1_DATA3_1 | USDHC1_DATAX_IOMUX) /* GPIO_SD_B0_05 */
#define PIN_USDHC1_DCLK     (GPIO_USDHC1_CLK_1   | USDHC1_CLK_IOMUX)   /* GPIO_SD_B0_01 */
#define PIN_USDHC1_CMD      (GPIO_USDHC1_CMD_1   | USDHC1_CMD_IOMUX)   /* GPIO_SD_B0_00 */
#define PIN_USDHC1_CD       (PIN_USDHC1_D3)

/* Ideal 400Khz for initial inquiry.
 *  Given input clock 198 Mhz.
 *   386.71875 KHz =  198 Mhz / (256 * 2)
 */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* Ideal 25 Mhz for other modes
 *  Given input clock 198 Mhz.
 *    24.75 MHz =  198 Mhz / (8 * 1)
 */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* ETH Disambiguation *******************************************************/

/* Ethernet Interrupt: GPIO_B0_15
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | \
			 GPIO_PORT2 | GPIO_PIN15)    /* B0_15 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO2_15

/* Ethernet Reset:  GPIO_B0_14
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
			 GPIO_PORT2 | GPIO_PIN14 | IOMUX_ENET_RST_DEFAULT) /* B0_14 */

#define GPIO_ENET_TX_DATA00  (GPIO_ENET_TX_DATA00_1| \
			      IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_07 */
#define GPIO_ENET_TX_DATA01  (GPIO_ENET_TX_DATA01_1| \
			      IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_08 */
#define GPIO_ENET_RX_DATA00  (GPIO_ENET_RX_DATA00_1| \
			      IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_04 */
#define GPIO_ENET_RX_DATA01  (GPIO_ENET_RX_DATA01_1| \
			      IOMUX_ENET_DATA_DEFAULT)                    /* GPIO_B1_05 */
#define GPIO_ENET_MDIO       (GPIO_ENET_MDIO_1|IOMUX_ENET_MDIO_DEFAULT)   /* GPIO_B1_15 */
#define GPIO_ENET_MDC        (GPIO_ENET_MDC_1|IOMUX_ENET_MDC_DEFAULT)     /* GPIO_B1_14 */
#define GPIO_ENET_RX_EN      (GPIO_ENET_RX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_06 */
#define GPIO_ENET_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT) /* GPIO_B1_11 */
#define GPIO_ENET_TX_CLK     (GPIO_ENET_REF_CLK_2|\
			      IOMUX_ENET_TX_CLK_DEFAULT)                  /* GPIO_B1_10 */
#define GPIO_ENET_TX_EN      (GPIO_ENET_TX_EN_1|IOMUX_ENET_EN_DEFAULT)    /* GPIO_B1_09 */

/* LED definitions ******************************************************************/
/* The nxp fmutr1062 board has numerous LEDs but only three, LED_GREEN a Green LED,
 * LED_BLUE a Blue LED and LED_RED a Red LED, that can be controlled by software.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_RED     BOARD_LED1
#define BOARD_LED_GREEN   BOARD_LED2
#define BOARD_LED_BLUE    BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* PIO Disambiguation ***************************************************************/
/* LPUARTs
 */
#define LPUART_IOMUX      (IOMUX_PULL_UP_22K | IOMUX_DRIVE_40OHM | IOMUX_SLEW_SLOW | IOMUX_SPEED_LOW | IOMUX_SCHMITT_TRIGGER)
#define IOMUX_UART_CTS_DEFAULT (IOMUX_PULL_DOWN_100K | IOMUX_DRIVE_40OHM | IOMUX_SLEW_SLOW)

/* Debug */

#define GPIO_LPUART5_RX   (GPIO_LPUART5_RX_1 | LPUART_IOMUX) /* GPIO_B1_13 */
#define GPIO_LPUART5_TX   (GPIO_LPUART5_TX_1 | LPUART_IOMUX) /* GPIO_B1_12 */

/* AUX */

#define GPIO_LPUART4_RX   (GPIO_LPUART4_RX_3 | LPUART_IOMUX) /* GPIO_B1_01 */
#define GPIO_LPUART4_TX   (GPIO_LPUART4_TX_3 | LPUART_IOMUX) /* GPIO_B1_00 */

/* GPS 1 */

#define GPIO_LPUART2_RX   (GPIO_LPUART2_RX_1 | LPUART_IOMUX) /* GPIO_AD_B1_03 */
#define GPIO_LPUART2_TX   (GPIO_LPUART2_TX_1 | LPUART_IOMUX) /* GPIO_AD_B1_02 */

/* Telem 1 */

#define HS_INPUT_IOMUX  (IOMUX_CMOS_INPUT | IOMUX_SLEW_SLOW | IOMUX_DRIVE_HIZ  | IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_47K)
#define HS_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_SLEW_FAST | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_PULL_KEEP)

#define GPIO_LPUART3_RX   (GPIO_LPUART3_RX_1 | LPUART_IOMUX) /* GPIO_AD_B1_07 */
#define GPIO_LPUART3_TX   (GPIO_LPUART3_TX_1 | LPUART_IOMUX) /* GPIO_AD_B1_06 */
#define GPIO_LPUART3_CTS  (GPIO_LPUART3_CTS_1 | IOMUX_UART_CTS_DEFAULT | PADMUX_SION)  /* GPIO_AD_B1_04 GPIO1_IO20 (GPIO only, no HW Flow control) */
#define GPIO_LPUART3_RTS  (GPIO_PORT1 | GPIO_PIN21 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | HS_OUTPUT_IOMUX) /* GPIO_AD_B1_05 GPIO1_IO21 (GPIO only, no HW Flow control) */
//TODO check RT7 partial HW handshake

/* RC INPUT single wire mode on TX, RX is not used */

#define GPIO_LPUART8_RX   (GPIO_LPUART8_RX_2 | LPUART_IOMUX) /* WRONG!! */
#define GPIO_LPUART8_TX   (GPIO_LPUART8_TX_1 | LPUART_IOMUX) /* GPIO_AD_B1_10 */

/* CAN
 *
 * CAN1 is routed to transceiver.
 * CAN2 is routed to transceiver.
 * CAN3 is routed to transceiver.
 */
#define FLEXCAN_IOMUX     (IOMUX_PULL_UP_100K | IOMUX_DRIVE_40OHM | IOMUX_SLEW_FAST | IOMUX_SPEED_MEDIUM)

#define GPIO_FLEXCAN3_RX  (GPIO_FLEXCAN3_RX_3 | FLEXCAN_IOMUX) /* GPIO_EMC_37 */
#define GPIO_FLEXCAN3_TX  (GPIO_FLEXCAN3_TX_3 | FLEXCAN_IOMUX) /* GPIO_EMC_36 */

/* LPSPI */
#define LPSPI_IOMUX       (IOMUX_PULL_UP_100K | IOMUX_DRIVE_33OHM | IOMUX_SLEW_FAST | IOMUX_SPEED_MAX)


#define GPIO_LPSPI3_SCK   (GPIO_LPSPI3_SCK_1 | LPSPI_IOMUX) /* GPIO_AD_B1_15 */
#define GPIO_LPSPI3_MISO  (GPIO_LPSPI3_SDI_1 | LPSPI_IOMUX) /* GPIO_AD_B1_13 */
#define GPIO_LPSPI3_MOSI  (GPIO_LPSPI3_SDO_1 | LPSPI_IOMUX) /* GPIO_AD_B1_14 */

#define GPIO_LPSPI4_SCK   (GPIO_LPSPI4_SCK_2 | LPSPI_IOMUX) /* GPIO_B0_03 */
#define GPIO_LPSPI4_MISO  (GPIO_LPSPI4_SDI_2 | LPSPI_IOMUX) /* GPIO_B0_01 */
#define GPIO_LPSPI4_MOSI  (GPIO_LPSPI4_SDO_2 | LPSPI_IOMUX) /* GPIO_B0_02 */

#define BOARD_SPI_BUS_MAX_BUS_ITEMS 2

/* LPI2Cs */

#define LPI2C_IOMUX    (IOMUX_SPEED_MEDIUM | IOMUX_DRIVE_33OHM  | IOMUX_OPENDRAIN | GPIO_SION_ENABLE)
#define LPI2C_IO_IOMUX (IOMUX_SPEED_MAX | IOMUX_SLEW_FAST | IOMUX_DRIVE_33OHM  | IOMUX_OPENDRAIN | IOMUX_PULL_NONE)

#define GPIO_LPI2C1_SDA   (GPIO_LPI2C1_SDA_2 | LPI2C_IOMUX) /* EVK J24-9 R276  */ /* GPIO_AD_B1_01 */
#define GPIO_LPI2C1_SCL   (GPIO_LPI2C1_SCL_2 | LPI2C_IOMUX) /* EVK J24-10 R277 */ /* GPIO_AD_B1_00 */

#define GPIO_LPI2C1_SDA_RESET (GPIO_PORT1 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B1_01 GPIO1_IO17 */
#define GPIO_LPI2C1_SCL_RESET (GPIO_PORT1 | GPIO_PIN16 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B1_00 GPIO1_IO16 */

#define GPIO_LPI2C4_SDA   (GPIO_LPI2C4_SDA_1 | LPI2C_IOMUX) /* GPIO_AD_B0_13 */
#define GPIO_LPI2C4_SCL   (GPIO_LPI2C4_SCL_1 | LPI2C_IOMUX) /* GPIO_AD_B0_12 */

#define GPIO_LPI2C4_SDA_RESET (GPIO_PORT1 | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B0_13 */
#define GPIO_LPI2C4_SCL_RESET (GPIO_PORT1 | GPIO_PIN12 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | LPI2C_IO_IOMUX) /* GPIO_AD_B0_12 */

/* Board doesn't provide GPIO or other Hardware for signaling to timing analyzer */

#define PROBE_INIT(mask)
#define PROBE(n,s)
#define PROBE_MARK(n)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __NUTTX_CONFIG_NXP_TROPIC_INCLUDE_BOARD_H */
