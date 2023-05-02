/************************************************************************************
 * nuttx-configs/nxp_fmurt1170-v1/include/board.h
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

#ifndef __NUTTX_CONFIG_NXP_FMURT1170_V1_INCLUDE_BOARD_H
#define __NUTTX_CONFIG_NXP_FMURT1170_V1_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* Set VDD_SOC to 1.3V */

#define IMXRT_VDD_SOC (0x14)

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

#define BOARD_GPT_FREQUENCY 24000000

/* Define this to enable tracing */
#if CONFIG_USE_TRACE
#  define IMXRT_TRACE_PODF_DIVIDER 1
#  define IMXRT_TRACE_CLK_SELECT   CCM_CBCMR_TRACE_CLK_SEL_PLL2_PFD0
#endif

/* SDIO *********************************************************************/

/* Pin drive characteristics - drive strength in particular may need tuning
 * for specific boards.  Settings have been copied from i.MX RT 1170 EVK.
 */

#define PIN_USDHC1_D0     (GPIO_USDHC1_DATA0_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* GPIO_SD_B1_02 */
#define PIN_USDHC1_D1     (GPIO_USDHC1_DATA1_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* GPIO_SD_B1_03 */
#define PIN_USDHC1_D2     (GPIO_USDHC1_DATA2_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* GPIO_SD_B1_04 */
#define PIN_USDHC1_D3     (GPIO_USDHC1_DATA3_1 | IOMUX_USDHC1_DATAX_DEFAULT) /* GPIO_SD_B1_05 */
#define PIN_USDHC1_DCLK   (GPIO_USDHC1_CLK_1   | IOMUX_USDHC1_CLK_DEFAULT)   /* GPIO_SD_B1_01 */
#define PIN_USDHC1_CMD    (GPIO_USDHC1_CMD_1   | IOMUX_USDHC1_CMD_DEFAULT)   /* GPIO_SD_B1_00 */
#define PIN_USDHC1_CD     (GPIO_USDHC1_CD_2    | IOMUX_USDHC1_CLK_DEFAULT)   /* GPIO_AD_32    */

/* 386 KHz for initial inquiry stuff */

#define BOARD_USDHC_IDMODE_PRESCALER    USDHC_SYSCTL_SDCLKFS_DIV256
#define BOARD_USDHC_IDMODE_DIVISOR      USDHC_SYSCTL_DVS_DIV(2)

/* 24.8MHz for other modes */

#define BOARD_USDHC_MMCMODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_MMCMODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD1MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD1MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

#define BOARD_USDHC_SD4MODE_PRESCALER   USDHC_SYSCTL_SDCLKFS_DIV8
#define BOARD_USDHC_SD4MODE_DIVISOR     USDHC_SYSCTL_DVS_DIV(1)

/* LED definitions ******************************************************************/
/* The nxp fmutr1170 board has numerous LEDs but only three, LED_GREEN a Green LED,
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

/* Debug */

#define GPIO_LPUART1_RX  (GPIO_LPUART1_RX_2|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART1_RX_DEBUG GPIO_DISP_B1_03 */
#define GPIO_LPUART1_TX  (GPIO_LPUART1_TX_2|IOMUX_UART_DEFAULT)             /* UART1_TX_DEBUG GPIO_DISP_B1_02 */

/* GPS 1 */

#define GPIO_LPUART3_RX  (GPIO_LPUART3_RX_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART3_RX_GPS1 GPIO_AD_31 */
#define GPIO_LPUART3_TX  (GPIO_LPUART3_TX_1|IOMUX_UART_DEFAULT)             /* UART3_TX_GPS1 GPIO_AD_30 */

/* Telem 1 */

#define GPIO_LPUART4_CTS (GPIO_LPUART4_CTS_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART4_CTS_TELEM1 GPIO_DISP_B1_05 */
#define GPIO_LPUART4_RTS (GPIO_LPUART4_RTS_1|IOMUX_UART_DEFAULT)             /* UART4_RTS_TELEM1 GPIO_DISP_B1_07 */
#define GPIO_LPUART4_RX  (GPIO_LPUART4_RX_1|IOMUX_UART_DEFAULT|PADMUX_SION)  /* UART4_RX_TELEM1 GPIO_DISP_B1_04 */
#define GPIO_LPUART4_TX  (GPIO_LPUART4_TX_1|IOMUX_UART_DEFAULT)              /* UART4_TX_TELEM1 GPIO_DISP_B1_06 */

/* GPS 2 */

#define GPIO_LPUART5_RX  (GPIO_LPUART5_RX_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART5_RX_GPS2 GPIO_AD_29 */
#define GPIO_LPUART5_TX  (GPIO_LPUART5_TX_1|IOMUX_UART_DEFAULT)             /* UART5_TX_GPS2 GPIO_AD_28 */

/* PX4IO */

#define GPIO_LPUART6_RX  (GPIO_LPUART6_RX_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART6_RX_FROM_IO__NC GPIO_EMC_B1_41 */
#define GPIO_LPUART6_TX  (GPIO_LPUART6_TX_1|IOMUX_UART_DEFAULT)             /* UART6_TX_TO_IO__RC_INPUT GPIO_EMC_B1_40 */

/* Telem 2 */

#define GPIO_LPUART8_CTS (GPIO_LPUART8_CTS_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART8_CTS_TELEM2 GPIO_AD_04 */
#define GPIO_LPUART8_RTS (GPIO_LPUART8_RTS_1|IOMUX_UART_DEFAULT)             /* UART8_RTS_TELEM2 GPIO_AD_05 */
#define GPIO_LPUART8_RX  (GPIO_LPUART8_RX_2|IOMUX_UART_DEFAULT|PADMUX_SION)  /* UART8_RX_TELEM2 GPIO_AD_03 */
#define GPIO_LPUART8_TX  (GPIO_LPUART8_TX_2|IOMUX_UART_DEFAULT)              /* UART8_TX_TELEM2 GPIO_AD_02 */

/* Telem 3 */

#define GPIO_LPUART10_CTS (GPIO_LPUART10_CTS_1|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART10_CTS_TELEM3 GPIO_AD_34 */
#define GPIO_LPUART10_RTS (GPIO_LPUART10_RTS_1|IOMUX_UART_DEFAULT)             /* UART10_RTS_TELEM3 GPIO_AD_35 */
#define GPIO_LPUART10_RX  (GPIO_LPUART10_RX_2|IOMUX_UART_DEFAULT|PADMUX_SION)  /* UART10_RX_TELEM3 GPIO_AD_33 */
#define GPIO_LPUART10_TX  (GPIO_LPUART10_TX_1|IOMUX_UART_DEFAULT)              /* UART10_TX_TELEM3 GPIO_AD_15 */

/* Ext 2 */

/* No DMA Support at this time for lack of DMA1, DMAMUX1 support */

#define GPIO_LPUART11_RX  (GPIO_LPUART11_RX_2|IOMUX_UART_DEFAULT|PADMUX_SION) /* UART11_RX_EXTERNAL2 GPIO_LPSR_05 */
#define GPIO_LPUART11_TX  (GPIO_LPUART11_TX_2|IOMUX_UART_DEFAULT)             /* UART11_TX_EXTERNAL2 GPIO_LPSR_04 */


/* CAN
 *
 * CAN1 is routed to transceiver.
 * CAN2 is routed to transceiver.
 * CAN3 is routed to transceiver.
 */

#define FLEXCAN_IOMUX     (IOMUX_PULL_UP | IOMUX_SLEW_FAST)

#define GPIO_FLEXCAN1_TX  (GPIO_FLEXCAN1_TX_1 | FLEXCAN_IOMUX) /* GPIO_AD_06 */
#define GPIO_FLEXCAN1_RX  (GPIO_FLEXCAN1_RX_1 | FLEXCAN_IOMUX) /* GPIO_AD_07 */

#define GPIO_FLEXCAN2_TX  (GPIO_FLEXCAN2_TX_1 | FLEXCAN_IOMUX) /* GPIO_AD_00 */
#define GPIO_FLEXCAN2_RX  (GPIO_FLEXCAN2_RX_1 | FLEXCAN_IOMUX) /* GPIO_AD_01 */

#define GPIO_FLEXCAN3_TX  (GPIO_FLEXCAN3_TX_1 | FLEXCAN_IOMUX) /* GPIO_LPSR_00 */
#define GPIO_FLEXCAN3_RX  (GPIO_FLEXCAN3_RX_1 | FLEXCAN_IOMUX) /* GPIO_LPSR_01 */

/* LPSPI */

#define GPIO_LPSPI1_MISO  (GPIO_LPSPI1_SDI_2|IOMUX_LPSPI_DEFAULT) /* SPI1_MISO_SENSOR1 GPIO_EMC_B2_03 */
#define GPIO_LPSPI1_MOSI  (GPIO_LPSPI1_SDO_2|IOMUX_LPSPI_DEFAULT) /* SPI1_MOSI_SENSOR1 GPIO_EMC_B2_02 */
#define GPIO_LPSPI1_SCK   (GPIO_LPSPI1_SCK_2|IOMUX_LPSPI_DEFAULT) /* SPI1_SCK_SENSOR1 GPIO_EMC_B2_00 */

#define GPIO_LPSPI2_MISO  (GPIO_LPSPI2_SDI_1|IOMUX_LPSPI_DEFAULT) /* SPI2_MISO_SENSOR2 GPIO_AD_27 */
#define GPIO_LPSPI2_MOSI  (GPIO_LPSPI2_SDO_1|IOMUX_LPSPI_DEFAULT) /* SPI2_MOSI_SENSOR2 GPIO_AD_26 */
#define GPIO_LPSPI2_SCK   (GPIO_LPSPI2_SCK_1|IOMUX_LPSPI_DEFAULT) /* SPI2_SCK_SENSOR2 GPIO_AD_24 */

#define GPIO_LPSPI3_MISO  (GPIO_LPSPI3_SDI_1|IOMUX_LPSPI_DEFAULT) /* SPI3_MISO_SENSOR3 GPIO_EMC_B2_07 */
#define GPIO_LPSPI3_MOSI  (GPIO_LPSPI3_SDO_1|IOMUX_LPSPI_DEFAULT) /* SPI3_MOSI_SENSOR3 GPIO_EMC_B2_06 */
#define GPIO_LPSPI3_SCK   (GPIO_LPSPI3_SCK_1|IOMUX_LPSPI_DEFAULT) /* SPI3_SCK_SENSOR3 GPIO_EMC_B2_04 */

/* SPI4 Not connected to anything */

//#define GPIO_LPSPI4_MISO  (GPIO_LPSPI4_SDI_2|IOMUX_LPSPI_DEFAULT) /* SPI4_MISO_SENSOR4 GPIO_DISP_B2_13 */
//#define GPIO_LPSPI4_MOSI  (GPIO_LPSPI4_SDO_2|IOMUX_LPSPI_DEFAULT) /* SPI4_MOSI_SENSOR4 GPIO_DISP_B2_14 */
//#define GPIO_LPSPI4_SCK   (GPIO_LPSPI4_SCK_2|IOMUX_LPSPI_DEFAULT) /* SPI4_SCK_SENSOR4 GPIO_DISP_B2_12 */

/* LPSPI6 No DMA Support at this time for lack of DMA1, DMAMUX1 support */


#define GPIO_LPSPI6_MISO  (GPIO_LPSPI6_SDI_1|IOMUX_LPSPI_DEFAULT) /* SPI6_MISO_EXTERNAL1 GPIO_LPSR_12 */
#define GPIO_LPSPI6_MOSI  (GPIO_LPSPI6_SDO_1|IOMUX_LPSPI_DEFAULT) /* SPI6_MOSI_EXTERNAL1 GPIO_LPSR_11 */
#define GPIO_LPSPI6_SCK   (GPIO_LPSPI6_SCK_1|IOMUX_LPSPI_DEFAULT) /* SPI6_SCK_EXTERNAL1 GPIO_LPSR_10 */

/* LPI2Cs */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT3 | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C1_SCL_GPS1 GPIO_AD_08 GPIO_GPIO3_IO07 */
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT3 | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C1_SDA_GPS1 GPIO_AD_09 GPIO_GPIO3_IO08 */

#define GPIO_LPI2C1_SCL        (GPIO_LPI2C1_SCL_2|IOMUX_LPI2C_DEFAULT) /* I2C1_SCL_GPS1 GPIO_AD_08 */
#define GPIO_LPI2C1_SDA        (GPIO_LPI2C1_SDA_2|IOMUX_LPI2C_DEFAULT) /* I2C1_SDA_GPS1 GPIO_AD_09 */

#define GPIO_LPI2C2_SCL_RESET  (GPIO_PORT3 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C2_SCL_GPS2 GPIO_AD_18 GPIO_GPIO3_IO17 */
#define GPIO_LPI2C2_SDA_RESET  (GPIO_PORT3 | GPIO_PIN18 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C2_SDA_GPS2 GPIO_AD_19 GPIO_GPIO3_IO18 */

#define GPIO_LPI2C2_SCL        (GPIO_LPI2C2_SCL_2|IOMUX_LPI2C_DEFAULT) /* I2C2_SCL_GPS2 GPIO_AD_18 */
#define GPIO_LPI2C2_SDA        (GPIO_LPI2C2_SDA_2|IOMUX_LPI2C_DEFAULT) /* I2C2_SDA_GPS2 GPIO_AD_19 */

#define GPIO_LPI2C3_SCL_RESET  (GPIO_PORT5 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C3_SCL_FMU GPIO_DISP_B2_10 GPIO_GPIO5_IO11_1 */
#define GPIO_LPI2C3_SDA_RESET  (GPIO_PORT5 | GPIO_PIN12 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C3_SDA_FMU GPIO_DISP_B2_11 GPIO_GPIO5_IO12_1 */

#define GPIO_LPI2C3_SCL        (GPIO_LPI2C3_SCL_2|IOMUX_LPI2C_DEFAULT) /* I2C3_SCL_FMU GPIO_DISP_B2_10 */
#define GPIO_LPI2C3_SDA        (GPIO_LPI2C3_SDA_2|IOMUX_LPI2C_DEFAULT) /* I2C3_SDA_FMU GPIO_DISP_B2_11 */

#define GPIO_LPI2C6_SCL_RESET  (GPIO_PORT6 | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C6_SCL_EXTERNAL2 GPIO_LPSR_07 GPIO_GPIO6_IO07_1 */
#define GPIO_LPI2C6_SDA_RESET  (GPIO_PORT6 | GPIO_PIN6 | GPIO_OUTPUT | GPIO_OUTPUT_ONE) /* I2C6_SDA_EXTERNAL2 GPIO_LPSR_06 GPIO_GPIO6_IO06_1 */

/* LPI2C6 No DMA Support at this time for lack of DMA1, DMAMUX1 support */

#define GPIO_LPI2C6_SCL        (GPIO_LPI2C6_SCL_1|IOMUX_LPI2C_DEFAULT) /* I2C6_SCL_EXTERNAL2 GPIO_LPSR_07 */
#define GPIO_LPI2C6_SDA        (GPIO_LPI2C6_SDA_1|IOMUX_LPI2C_DEFAULT) /* I2C6_SDA_EXTERNAL2 GPIO_LPSR_06 */

/* ETH Disambiguation *******************************************************/

// This is the ENET_1G interface.

#define BOARD_PHY_ADDR 18

#define GPIO_ENET2_TX_DATA00  (GPIO_ENET_1G_TX_DATA0_1|IOMUX_ENET_DATA_DEFAULT)  /* GPIO_DISP_B1_09 */
#define GPIO_ENET2_TX_DATA01  (GPIO_ENET_1G_TX_DATA1_1|IOMUX_ENET_DATA_DEFAULT)  /* GPIO_DISP_B1_08 */
#define GPIO_ENET2_RX_DATA00  (GPIO_ENET_1G_RX_DATA0_2|IOMUX_ENET_DATA_DEFAULT)  /* GPIO_EMC_B2_15  */
#define GPIO_ENET2_RX_DATA01  (GPIO_ENET_1G_RX_DATA1_2|IOMUX_ENET_DATA_DEFAULT)  /* GPIO_EMC_B2_16  */
#define GPIO_ENET2_MDIO       (GPIO_ENET_1G_MDIO_1|IOMUX_ENET_MDIO_DEFAULT)      /* GPIO_EMC_B2_20  */
#define GPIO_ENET2_MDC        (GPIO_ENET_1G_MDC_1|IOMUX_ENET_MDC_DEFAULT)        /* GPIO_EMC_B2_19  */
#define GPIO_ENET2_RX_EN      (GPIO_ENET_1G_RX_EN_1|IOMUX_ENET_EN_DEFAULT)       /* GPIO_DISP_B1_00 */
#define GPIO_ENET2_RX_ER      (GPIO_ENET_RX_ER_1|IOMUX_ENET_RXERR_DEFAULT)       /* GPIO_DISP_B1_01 */
#define GPIO_ENET2_TX_CLK     (GPIO_ENET_1G_REF_CLK_1|IOMUX_ENET_TX_CLK_DEFAULT) /* GPIO_DISP_B1_11 */
#define GPIO_ENET2_TX_EN      (GPIO_ENET_1G_TX_EN_1|IOMUX_ENET_EN_DEFAULT)       /* GPIO_DISP_B1_10 */


/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>
// add     -I<full path> build/nxp_fmurt1170-v1_default/NuttX/nuttx/arch/arm/src/chip \ to NuttX Makedefs.in
#define PROBE_IOMUX (IOMUX_SPEED_MAX | IOMUX_SLEW_FAST | IOMUX_DRIVE_33OHM  | IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE)
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1 /* GPIO_B0_06    */  (GPIO_PORT2 | GPIO_PIN6  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_2 /* GPIO_EMC_08   */  (GPIO_PORT4 | GPIO_PIN8  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_3 /* GPIO_EMC_10   */  (GPIO_PORT4 | GPIO_PIN10 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_4 /* GPIO_AD_B0_09 */  (GPIO_PORT1 | GPIO_PIN9  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_5 /* GPIO_EMC_33   */  (GPIO_PORT3 | GPIO_PIN19 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_6 /* GPIO_EMC_30   */  (GPIO_PORT4 | GPIO_PIN30 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_7 /* GPIO_EMC_04   */  (GPIO_PORT4 | GPIO_PIN4  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)
# define PROBE_8 /* GPIO_EMC_01   */  (GPIO_PORT4 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | PROBE_IOMUX)

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { imxrt_config_gpio(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { imxrt_config_gpio(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { imxrt_config_gpio(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { imxrt_config_gpio(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { imxrt_config_gpio(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { imxrt_config_gpio(PROBE_6); } \
		if ((mask)& PROBE_N(7)) { imxrt_config_gpio(PROBE_7); } \
		if ((mask)& PROBE_N(8)) { imxrt_config_gpio(PROBE_8); } \
	} while(0)

# define PROBE(n,s)  do {imxrt_gpio_write(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif

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
#endif /* __NUTTX_CONFIG_NXP_FMURT1170_V1_INCLUDE_BOARD_H */