/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/include/board.h
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

/* Copyright 2022 NXP */

#ifndef __BOARDS_ARM_S32K3XX_MR_CANHUBK3_INCLUDE_BOARD_H
#define __BOARDS_ARM_S32K3XX_MR_CANHUBK3_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The MR-CANHUBK3 is fitted with a 16 MHz crystal */

#define BOARD_XTAL_FREQUENCY  16000000

/* The S32K344 will run at 160 MHz */

#define MR_CANHUBK3_SYSCLK_FREQUENCY  160000000

/* MX25L QuadSPI Flash ******************************************************/

#ifdef CONFIG_S32K3XX_QSPI
#  ifdef CONFIG_MTD_MX25RXX
#    define HAVE_MX25L
#      ifdef CONFIG_FS_LITTLEFS
#        define HAVE_MX25L_LITTLEFS
#      else
#        ifdef CONFIG_FS_NXFFS
#          define HAVE_MX25L_NXFFS
#        else
#          define HAVE_MX25L_CHARDEV
#        endif
#      endif
#  endif
#endif

#define MX25L_MTD_MINOR    0
#define MX25L_SMART_MINOR  0

/* LED definitions **********************************************************/

/* The MR-CANHUBK3 has one RGB LED:
 *
 *   RedLED    PTE14  (FXIO D7 / EMIOS0 CH19)
 *   GreenLED  PTA27  (FXIO D5 / EMIOS1 CH10 / EMIOS2 CH10)
 *   BlueLED   PTE12  (FXIO D8 / EMIOS1 CH5)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the MR-CANHUBK3.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *      SYMBOL            Meaning                         LED state
 *                                                        RED    GREEN  BLUE
 *      ----------------  -----------------------------  -------------------
 */

#define LED_STARTED       1 /* NuttX has been started     OFF    OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated    OFF    OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled         OFF    OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created         OFF    ON     OFF */
#define LED_INIRQ         0 /* In an interrupt           (No change)        */
#define LED_SIGNAL        0 /* In a signal handler       (No change)        */
#define LED_ASSERTION     0 /* An assertion failed       (No change)        */
#define LED_PANIC         4 /* The system has crashed     FLASH  OFF    OFF */
#undef  LED_IDLE            /* S32K344 is in sleep mode  (Not used)         */

/* Button definitions *******************************************************/

/* The MR-CANHUBK3 supports two buttons:
 *
 *   SW1  PTD15  (EIRQ31)
 *   SW2  PTA25  (EIRQ5 / WKPU34)
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

/* UART selections **********************************************************/

/* By default, the serial console will be provided on the DCD-LZ UART
 * (available on the 7-pin DCD-LZ debug connector P6):
 *
 *   DCD-LZ UART RX  PTA8  (LPUART2_RX)
 *   DCD-LZ UART TX  PTA9  (LPUART2_TX)
 */

#define PIN_LPUART2_RX    (PIN_LPUART2_RX_1 | PIN_INPUT_PULLUP)   /* PTA8 */
#define PIN_LPUART2_TX    PIN_LPUART2_TX_1   /* PTA9 */

/* LPUART0   P2 UART (with flow control) connector */

#define PIN_LPUART0_CTS   PIN_LPUART0_CTS_1  /* PTA0 */
#define PIN_LPUART0_RTS   (PIN_PTA1 | GPIO_OUTPUT) /* PIN_LPUART0_RTS_1 PTA1 */
#define PIN_LPUART0_RX    (PIN_LPUART0_RX_1 | PIN_INPUT_PULLUP)   /* PTA2 */
#define PIN_LPUART0_TX    PIN_LPUART0_TX_1   /* PTA3 */

/* LPUART1   P5 UART (with flow control) connector */

#define PIN_LPUART1_CTS   PIN_LPUART1_CTS_2  /* PTE2 */
#define PIN_LPUART1_RTS   PIN_LPUART1_RTS_2  /* PTE6 */
#define PIN_LPUART1_RX    (PIN_LPUART1_RX_3 | PIN_INPUT_PULLUP)   /* PTC6 */
#define PIN_LPUART1_TX    PIN_LPUART1_TX_3   /* PTC7 */


/* LPUART4 /dev/ttyS3  P8B 3X7 Pin 3 Single wire RC UART */

#define PIN_LPUART4_RX    PIN_LPUART4_TX_3   /* Dummy since it's Single Wire TX-only */
#define PIN_LPUART4_TX    PIN_LPUART4_TX_3   /* PTE11 */


/* LPUART7 /dev/ttyS4  P8B 3X7 Pin 3 and Pin 8 */

#define PIN_LPUART7_RX    (PIN_LPUART7_RX_3 | PIN_INPUT_PULLUP)   /* PTE0 */
#define PIN_LPUART7_TX    PIN_LPUART7_TX_3   /* PTE1 */

/* LPUART9   P24 UART connector */

#define PIN_LPUART9_RX    (PIN_LPUART9_RX_1 | PIN_INPUT_PULLUP)   /* PTB2 */
#define PIN_LPUART9_TX    PIN_LPUART9_TX_1   /* PTB3 */

/* LPUART10  P24 UART connector */

#define PIN_LPUART10_RX   (PIN_LPUART10_RX_1 | PIN_INPUT_PULLUP)  /* PTC12 */
#define PIN_LPUART10_TX   PIN_LPUART10_TX_1  /* PTC13 */

/* LPUART13  P25 UART connector */

#define PIN_LPUART13_RX   (PIN_LPUART13_RX_1 | PIN_INPUT_PULLUP)  /* PTB19 */
#define PIN_LPUART13_TX   PIN_LPUART13_TX_1  /* PTB18 */

/* LPUART14  P25 UART connector */

#define PIN_LPUART14_RX   (PIN_LPUART14_RX_1 | PIN_INPUT_PULLUP)  /* PTB21 */
#define PIN_LPUART14_TX   PIN_LPUART14_TX_1  /* PTB20 */

/* SPI selections ***********************************************************/

/* LPSPI1  P1A external SPI connector */

#define PIN_LPSPI1_SCK    PIN_LPSPI1_SCK_3   /* PTA28 */
#define PIN_LPSPI1_MISO   PIN_LPSPI1_SOUT_2  /* PTA30 */
#define PIN_LPSPI1_MOSI   PIN_LPSPI1_SIN_3   /* PTA29 */
#define PIN_LPSPI1_PCS0   PIN_LPSPI1_PCS0_2  /* PTA21 */
#define PIN_LPSPI1_PCS1   PIN_LPSPI1_PCS1_6  /* PTE4  */

#define PIN_LPSPI1_PCS    (PIN_PTA21 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)  /* PTA21 */

/* LPSPI2  P1B external SPI connector */

#define PIN_LPSPI2_SCK    PIN_LPSPI2_SCK_1   /* PTB29 */
#define PIN_LPSPI2_MISO   PIN_LPSPI2_SOUT_3  /* PTB27 */
#define PIN_LPSPI2_MOSI   PIN_LPSPI2_SIN_2   /* PTB28 */
#define PIN_LPSPI2_PCS0   PIN_LPSPI2_PCS0_2  /* PTB25 */
#define PIN_LPSPI2_PCS1   PIN_LPSPI2_PCS1_3  /* PTC19 */

#define PIN_LPSPI2_PCS    (PIN_PTB25 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)  /* PTB25 */

/* LPSPI3  FS26 Safety SBC */

#define PIN_LPSPI3_SCK    PIN_LPSPI3_SCK_2   /* PTD1  */
#define PIN_LPSPI3_MISO   PIN_LPSPI3_SOUT_2  /* PTD0  */
#define PIN_LPSPI3_MOSI   PIN_LPSPI3_SIN_3   /* PTE10 */
#define PIN_LPSPI3_PCS    (PIN_PTD17 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE) /* PTD17 */

/* LPSPI4  P8B I/O connector / P26 external IMU connector */

#define PIN_LPSPI4_SCK    PIN_LPSPI4_SCK_1   /* PTB10 */
#define PIN_LPSPI4_MISO   PIN_LPSPI4_SOUT_1  /* PTB9  */
#define PIN_LPSPI4_MOSI   PIN_LPSPI4_SIN_1   /* PTB11 */
#define PIN_LPSPI4_PCS0   PIN_LPSPI4_PCS0_1  /* PTB8  */
#define PIN_LPSPI4_PCS3   PIN_LPSPI4_PCS3_1  /* PTA16 */

#define PIN_LPSPI4_CS_P26 (PIN_PTA16 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)  /* PTA16 */
#define PIN_LPSPI4_CS_P8B (PIN_PTB8 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)   /* PTB8 */

/* LPSPI5  P26 external IMU connector */

#define PIN_LPSPI5_SCK    PIN_LPSPI5_SCK_3   /* PTD26 */
#define PIN_LPSPI5_MISO   PIN_LPSPI5_SOUT_2  /* PTD27 */
#define PIN_LPSPI5_MOSI   PIN_LPSPI5_SIN_3   /* PTD28 */
#define PIN_LPSPI5_PCS1   PIN_LPSPI5_PCS1_1  /* PTA14 */

#define PIN_LPSPI5_PCS    (PIN_PTA14 | GPIO_LOWDRIVE | GPIO_OUTPUT_ONE)  /* PTA14 */

/*      PIN_LPSPI5_PCS2                         PTD29 */

/* I2C selections ***********************************************************/

/* LPI2C0  P4 LCD header / P26 external IMU connector */

#define PIN_LPI2C0_SCL    PIN_LPI2C0_SCL_2   /* PTD14 */
#define PIN_LPI2C0_SDA    PIN_LPI2C0_SDA_2   /* PTD13 */

/* LPI2C1  P3 external I2C connector / SE050 EdgeLock Secure Element */

#define PIN_LPI2C1_SCL    PIN_LPI2C1_SCL_4   /* PTD9 */
#define PIN_LPI2C1_SDA    PIN_LPI2C1_SDA_4   /* PTD8 */

/* CAN selections ***********************************************************/

/* CAN0  TJA1443 CAN transceiver */

#define PIN_CAN0_RX       PIN_CAN0_RX_1      /* PTA6 */
#define PIN_CAN0_TX       PIN_CAN0_TX_1      /* PTA7 */
#define PIN_CAN0_STB      (PIN_PTC21 | GPIO_OUTPUT)
#define CAN0_STB_OUT      1
#define PIN_CAN0_ENABLE   (PIN_PTC24 | GPIO_OUTPUT)
#define CAN0_ENABLE_OUT   1
#define PIN_CAN0_LED      (PIN_PTC18 | GPIO_OUTPUT)
#define CAN0_LED_OUT      0
#define PIN_CAN0_ERRN     (PIN_PTC20 | GPIO_INPUT)

/* CAN1  TJA1443 CAN transceiver */

#define PIN_CAN1_RX       PIN_CAN1_RX_4      /* PTC9 */
#define PIN_CAN1_TX       PIN_CAN1_TX_4      /* PTC8 */
#define PIN_CAN1_STB      (PIN_PTD2  | GPIO_OUTPUT)
#define CAN1_STB_OUT      1
#define PIN_CAN1_ENABLE   (PIN_PTD23 | GPIO_OUTPUT)
#define CAN1_ENABLE_OUT   1
#define PIN_CAN1_LED      (PIN_PTE5  | GPIO_OUTPUT)
#define CAN1_LED_OUT      0
#define PIN_CAN1_ERRN     (PIN_PTD3  | GPIO_INPUT)

/* CAN2  TJA1463 CAN transceiver */

#define PIN_CAN2_RX       PIN_CAN2_RX_5      /* PTE25 */
#define PIN_CAN2_TX       PIN_CAN2_TX_5      /* PTE24 */
#define PIN_CAN2_STB      (PIN_PTD22 | GPIO_OUTPUT)
#define CAN2_STB_OUT      1
#define PIN_CAN2_ENABLE   (PIN_PTD4  | GPIO_OUTPUT)
#define CAN2_ENABLE_OUT   1
#define PIN_CAN2_LED      (PIN_PTD20 | GPIO_OUTPUT)
#define CAN2_LED_OUT      0
#define PIN_CAN2_ERRN     (PIN_PTD21 | GPIO_INPUT)

/* CAN3  TJA1463 CAN transceiver */

#define PIN_CAN3_RX       PIN_CAN3_RX_2      /* PTC29 */
#define PIN_CAN3_TX       PIN_CAN3_TX_2      /* PTC28 */
#define PIN_CAN3_STB      (PIN_PTB1  | GPIO_OUTPUT)
#define CAN3_STB_OUT      1
#define PIN_CAN3_ENABLE   (PIN_PTB0  | GPIO_OUTPUT)
#define CAN3_ENABLE_OUT   1
#define PIN_CAN3_LED      (PIN_PTB24 | GPIO_OUTPUT)
#define CAN3_LED_OUT      0
#define PIN_CAN3_ERRN     (PIN_PTC27 | GPIO_INPUT)

/* CAN4  TJA1153 CAN transceiver */

#define PIN_CAN4_RX       PIN_CAN4_RX_2      /* PTC31 */
#define PIN_CAN4_TX       PIN_CAN4_TX_2      /* PTC30 */
#define PIN_CAN4_STB      (PIN_PTC25 | GPIO_OUTPUT)
#define CAN4_STB_OUT      0
#define PIN_CAN4_ENABLE   (PIN_PTC26 | GPIO_OUTPUT)
#define CAN4_ENABLE_OUT   1
#define PIN_CAN4_LED      (PIN_PTB26 | GPIO_OUTPUT)
#define CAN4_LED_OUT      0
#define PIN_CAN4_ERRN     (PIN_PTC23 | GPIO_INPUT)

/* CAN5  TJA1153 CAN transceiver */

#define PIN_CAN5_RX       PIN_CAN5_RX_1      /* PTC11 */
#define PIN_CAN5_TX       PIN_CAN5_TX_1      /* PTC10 */
#define PIN_CAN5_STB      (PIN_PTE17 | GPIO_OUTPUT)
#define CAN5_STB_OUT      0
#define PIN_CAN5_ENABLE   (PIN_PTD30 | GPIO_OUTPUT)
#define CAN5_ENABLE_OUT   1
#define PIN_CAN5_LED      (PIN_PTD31 | GPIO_OUTPUT)
#define CAN5_LED_OUT      0
#define PIN_CAN5_ERRN     (PIN_PTD24 | GPIO_INPUT)

/* ENET selections **********************************************************/

#define PIN_EMAC_MII_RMII_TXD0    PIN_EMAC_MII_RMII_TXD0_1     /* PTB5  */
#define PIN_EMAC_MII_RMII_TXD1    PIN_EMAC_MII_RMII_TXD1_1     /* PTB4  */
#define PIN_EMAC_MII_RMII_TX_EN   PIN_EMAC_MII_RMII_TX_EN_3    /* PTE9  */
#define PIN_EMAC_MII_RMII_RXD0    PIN_EMAC_MII_RMII_RXD0_1     /* PTC0  */
#define PIN_EMAC_MII_RMII_RXD1    PIN_EMAC_MII_RMII_RXD1_2     /* PTC1  */
#define PIN_EMAC_MII_RMII_RX_DV   PIN_EMAC_MII_RMII_RX_DV_1    /* PTC15 */
#define PIN_EMAC_MII_RMII_RX_ER   PIN_EMAC_MII_RMII_RX_ER_1    /* PTC14 */
#define PIN_EMAC_MII_RMII_MDC     PIN_EMAC_MII_RMII_MDC_3      /* PTC8  */
#define PIN_EMAC_MII_RMII_MDIO    PIN_EMAC_MII_RMII_MDIO_2     /* PTD16 */
#define PIN_EMAC_MII_RMII_TX_CLK  PIN_EMAC_MII_RMII_TX_CLK_2   /* PTD6 */


/* Board provides GPIO or other Hardware for signaling to timing analyzer */

#if defined(CONFIG_BOARD_USE_PROBES)
# include "s32k3xx_pin.h"
# include "hardware/s32k3xx_pinmux.h"
# define PROBE_N(n) (1<<((n)-1))
# define PROBE_1    (PIN_PTB12 | GPIO_OUTPUT)  /* PWM-0 */
# define PROBE_2    (PIN_PTB13 | GPIO_OUTPUT)  /* PWM-1 */
# define PROBE_3    (PIN_PTB14 | GPIO_OUTPUT)  /* PWM-2 */
# define PROBE_4    (PIN_PTB15 | GPIO_OUTPUT)  /* PWM-3 */
# define PROBE_5    (PIN_PTB16 | GPIO_OUTPUT)  /* PWM-4 */
# define PROBE_6    (PIN_PTB17 | GPIO_OUTPUT)  /* PWM-5 */
# define PROBE_7    (PIN_PTA17 | GPIO_OUTPUT)  /* PWM-6 */
# define PROBE_8    (PIN_PTE7  | GPIO_OUTPUT)  /* PWM-7 */

# define PROBE_INIT(mask) \
	do { \
		if ((mask)& PROBE_N(1)) { s32k3xx_pinconfig(PROBE_1); } \
		if ((mask)& PROBE_N(2)) { s32k3xx_pinconfig(PROBE_2); } \
		if ((mask)& PROBE_N(3)) { s32k3xx_pinconfig(PROBE_3); } \
		if ((mask)& PROBE_N(4)) { s32k3xx_pinconfig(PROBE_4); } \
		if ((mask)& PROBE_N(5)) { s32k3xx_pinconfig(PROBE_5); } \
		if ((mask)& PROBE_N(6)) { s32k3xx_pinconfig(PROBE_6); } \
		if ((mask)& PROBE_N(7)) { s32k3xx_pinconfig(PROBE_7); } \
		if ((mask)& PROBE_N(8)) { s32k3xx_pinconfig(PROBE_8); } \
	} while(0)

# define PROBE(n,s)  do {s32k3xx_gpiowrite(PROBE_##n,(s));}while(0)
# define PROBE_MARK(n) PROBE(n,false);PROBE(n,true)
#else
# define PROBE_INIT(mask)
# define PROBE(n,s)
# define PROBE_MARK(n)
#endif


#endif  /* __BOARDS_ARM_S32K3XX_MR_CANHUBK3_INCLUDE_BOARD_H */
