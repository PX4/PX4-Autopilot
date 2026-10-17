/****************************************************************************
 * boards/nxp/mr-navq95/nuttx-config/include/board.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_NXP_MR_NAVQ95_NUTTX_CONFIG_INCLUDE_BOARD_H
#define __BOARDS_NXP_MR_NAVQ95_NUTTX_CONFIG_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_XTAL_FREQUENCY       24000000

#define BOARD_CPU_FREQUENCY        800000000

#define LPUART2_CLK                (LPUART2_CLK_ROOT_SYS_PLL1_DFS0_DIV2_CLK | CLOCK_DIV(8))
#define LPUART5_CLK                (LPUART5_CLK_ROOT_SYS_PLL1_DFS0_DIV2_CLK | CLOCK_DIV(8))
#define LPUART7_CLK                (LPUART7_CLK_ROOT_SYS_PLL1_DFS0_DIV2_CLK | CLOCK_DIV(8))
#define LPI2C1_CLK                 (LPI2C1_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPI2C4_CLK                 (LPI2C4_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPI2C6_CLK                 (LPI2C6_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPSPI1_CLK                 (LPSPI1_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define LPSPI8_CLK                 (LPSPI8_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(1))
#define FLEXCAN1_CLK               (CAN1_CLK_ROOT_SYS_PLL1_DFS1_DIV2_CLK | CLOCK_DIV(5))
#define FLEXCAN2_CLK               (CAN2_CLK_ROOT_SYS_PLL1_DFS1_DIV2_CLK | CLOCK_DIV(5))
#define FLEXCAN3_CLK               (CAN3_CLK_ROOT_SYS_PLL1_DFS1_DIV2_CLK | CLOCK_DIV(5))

#define LED_STARTED       0  /* NuttX has been started   OFF     OFF   */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated  OFF     OFF   */
#define LED_IRQSENABLED   0  /* Interrupts enabled       OFF     OFF   */
#define LED_STACKCREATED  1  /* Idle stack created       ON      OFF   */
#define LED_INIRQ         2  /* In an interrupt         (No change)    */
#define LED_SIGNAL        2  /* In a signal handler     (No change)    */
#define LED_ASSERTION     2  /* An assertion failed     (No change)    */
#define LED_PANIC         3  /* The system has crashed   OFF     FLASH */
#undef  LED_IDLE             /* Not used                (Not used)     */

/* Default PAD configurations */

#define IOMUX_LPI2C_DEFAULT  (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X6)
#define IOMUX_LPI2C1_DEFAULT (IOMUXC_PAD_OD_ENABLE | IOMUXC_PAD_PU_ON | IOMUXC_PAD_FSEL_SFAST | IOMUXC_PAD_DSE_X2)
#define IOMUX_LPSPI_DEFAULT  (IOMUXC_PAD_PU_ON     | IOMUXC_PAD_FSEL_FAST  | IOMUXC_PAD_DSE_X6)
#define IOMUX_GPIO_DEFAULT   (IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X6)

/* UART pin muxings */

#define MUX_LPUART2_RX       IOMUX_CFG(IOMUXC_PAD_UART2_RXD_LPUART2_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART2_TX       IOMUX_CFG(IOMUXC_PAD_UART2_TXD_LPUART2_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

#define MUX_LPUART5_RX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO01_LPUART5_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART5_TX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO00_LPUART5_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

#define MUX_LPUART7_RX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO09_LPUART7_RX, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART7_TX       IOMUX_CFG(IOMUXC_PAD_GPIO_IO08_LPUART7_TX, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)
#define MUX_LPUART7_CTS      IOMUX_CFG(IOMUXC_PAD_GPIO_IO10_LPUART7_CTS_B, 0, IOMUXC_MUX_SION_ON)
#define MUX_LPUART7_RTS      IOMUX_CFG(IOMUXC_PAD_GPIO_IO11_LPUART7_RTS_B, IOMUXC_PAD_FSEL_SLOW | IOMUXC_PAD_DSE_X4, 0)

/* LPI2Cs */

#define MUX_LPI2C1_SCL       IOMUX_CFG(IOMUXC_PAD_I2C1_SCL_LPI2C1_SCL, IOMUX_LPI2C1_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C1_SDA       IOMUX_CFG(IOMUXC_PAD_I2C1_SDA_LPI2C1_SDA, IOMUX_LPI2C1_DEFAULT, IOMUXC_MUX_SION_ON)

#define MUX_LPI2C4_SCL       IOMUX_CFG(IOMUXC_PAD_GPIO_IO31_LPI2C4_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C4_SDA       IOMUX_CFG(IOMUXC_PAD_GPIO_IO30_LPI2C4_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

#define MUX_LPI2C6_SCL       IOMUX_CFG(IOMUXC_PAD_GPIO_IO03_LPI2C6_SCL, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPI2C6_SDA       IOMUX_CFG(IOMUXC_PAD_GPIO_IO02_LPI2C6_SDA, IOMUX_LPI2C_DEFAULT, IOMUXC_MUX_SION_ON)

/* I2C reset functionality */

#define GPIO_LPI2C1_SCL_RESET  (GPIO_PORT1 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPI2C1_SDA_RESET  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* LPSPIs */

#define MUX_LPSPI1_SCK       IOMUX_CFG(IOMUXC_PAD_SAI1_TXD0_LPSPI1_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI1_MOSI      IOMUX_CFG(IOMUXC_PAD_SAI1_RXD0_LPSPI1_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI1_MISO      IOMUX_CFG(IOMUXC_PAD_SAI1_TXC_LPSPI1_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

#define MUX_LPSPI8_SCK       IOMUX_CFG(IOMUXC_PAD_GPIO_IO15_LPSPI8_SCK,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI8_MOSI      IOMUX_CFG(IOMUXC_PAD_GPIO_IO14_LPSPI8_SOUT, IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI8_MISO      IOMUX_CFG(IOMUXC_PAD_GPIO_IO13_LPSPI8_SIN,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

/* SPI CS */

#define MUX_LPSPI1_CS        IOMUX_CFG(IOMUXC_PAD_SAI1_TXFS_GPIO1_IO11,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

#define MUX_LPSPI8_CS        IOMUX_CFG(IOMUXC_PAD_GPIO_IO12_GPIO2_IO12,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)
#define MUX_LPSPI8_CS1       IOMUX_CFG(IOMUXC_PAD_GPIO_IO26_GPIO2_IO26,  IOMUX_LPSPI_DEFAULT, IOMUXC_MUX_SION_ON)

#define GPIO_LPSPI1_CS       (GPIO_PORT1 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

#define GPIO_LPSPI8_CS       (GPIO_PORT2 | GPIO_PIN12 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)
#define GPIO_LPSPI8_CS1      (GPIO_PORT2 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_OUTPUT_ONE)

/* FlexCAN */

#define GPIO_FLEXCAN1_TX     IOMUX_CFG(IOMUXC_PAD_PDM_CLK_CAN1_TX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXCAN1_RX     IOMUX_CFG(IOMUXC_PAD_PDM_BIT_STREAM0_CAN1_RX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)

#define GPIO_FLEXCAN2_TX     IOMUX_CFG(IOMUXC_PAD_GPIO_IO25_CAN2_TX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXCAN2_RX     IOMUX_CFG(IOMUXC_PAD_GPIO_IO27_CAN2_RX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)

#define GPIO_FLEXCAN3_TX     IOMUX_CFG(IOMUXC_PAD_CCM_CLKO3_CAN3_TX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXCAN3_RX     IOMUX_CFG(IOMUXC_PAD_CCM_CLKO4_CAN3_RX,  IOMUX_GPIO_DEFAULT, IOMUXC_MUX_SION_ON)

/* Dshot Disambiguation *******************************************************/


#define BOARD_FLEXIO_PREQ   133333333
#define IOMUX_DSHOT_DEFAULT (IOMUXC_PAD_DSE_X4 | IOMUXC_PAD_FSEL_FAST | IOMUXC_PAD_PU_ON)
#define GPIO_FLEXIO04       IOMUX_CFG(IOMUXC_PAD_GPIO_IO04_FLEXIO1_FLEXIO04, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO20       IOMUX_CFG(IOMUXC_PAD_GPIO_IO20_FLEXIO1_FLEXIO20, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO24       IOMUX_CFG(IOMUXC_PAD_GPIO_IO24_FLEXIO1_FLEXIO24, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO05       IOMUX_CFG(IOMUXC_PAD_GPIO_IO05_FLEXIO1_FLEXIO05, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO21       IOMUX_CFG(IOMUXC_PAD_GPIO_IO21_FLEXIO1_FLEXIO21, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO06       IOMUX_CFG(IOMUXC_PAD_GPIO_IO06_FLEXIO1_FLEXIO06, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO22       IOMUX_CFG(IOMUXC_PAD_GPIO_IO22_FLEXIO1_FLEXIO22, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)
#define GPIO_FLEXIO23       IOMUX_CFG(IOMUXC_PAD_GPIO_IO23_FLEXIO1_FLEXIO23, IOMUX_DSHOT_DEFAULT, IOMUXC_MUX_SION_ON)


/* The System Manager owns the PLLs; SYS_PLL1_PFD1 provides 800 MHz */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_NXP_MR_NAVQ95_NUTTX_CONFIG_INCLUDE_BOARD_H */
