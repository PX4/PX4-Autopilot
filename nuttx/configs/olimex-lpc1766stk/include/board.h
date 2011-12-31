/************************************************************************************
 * configs/olimex-lpc1766stk/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/
/* NOTE:  The following definitions require lpc17_syscon.h.  It is not included here
 * because the including C file may not have that file in its include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20, pre-divider=1
 *   CCLCK = 480MHz / 6 = 80MHz               -> CCLK divider = 6
 */

#define LPC17_CCLK                 80000000 /* 80Mhz */

/* Select the main oscillator as the frequency source.  SYSCLK is then the frequency
 * of the main oscillator.
 */

#undef CONFIG_LPC17_MAINOSC
#define CONFIG_LPC17_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKCFG_DIVIDER      6
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.  PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20 
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC17_PLL0
#define CONFIG_LPC17_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC17_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.  This divider is used when PLL1 is not enabled to get the
 * USB clock from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LP17_FLASH
#define CONFIG_LP17_FLASH          1
#define BOARD_FLASHCFG_VALUE       0x0000303a

/* Ethernet configuration */

//#define ETH_MCFG_CLKSEL_DIV ETH_MCFG_CLKSEL_DIV44
#define ETH_MCFG_CLKSEL_DIV ETH_MCFG_CLKSEL_DIV20

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1            -- Connected to P1[25]
 * LED2            -- Connected to P0[4]
 */

/* LED index values for use with lpc17_setled() */

#define BOARD_LED1                0
#define BOARD_LED2                1
#define BOARD_NLEDS               2

/* LED bits for use with lpc17_setleds() */

#define BOARD_LED1_BIT            (1 << BOARD_LED1)
#define BOARD_LED2_BIT            (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 3 LEDs
 * on board the Olimex LPC1766-STK.  The following definitions
 * describe how NuttX controls the LEDs:
 */
                                      /* LED1 LED2 */
#define LED_STARTED                0  /*  OFF  OFF = Still initializing */
#define LED_HEAPALLOCATE           0  /*  OFF  OFF = Still initializing */
#define LED_IRQSENABLED            0  /*  OFF  OFF = Still initializing */
#define LED_STACKCREATED           1  /*  ON   OFF = Initialization complete */
#define LED_INIRQ                  2  /*  N/C  ON  = In an interrupt handler */
#define LED_SIGNAL                 2  /*  N/C  ON  = In a signal handler (glowing) */
#define LED_ASSERTION              2  /*  N/C  ON  = In an assertion */
#define LED_PANIC                  2  /*  N/C  ON  = Oops! We crashed. (flashing) */
#define LED_IDLE                   3  /*  OFF  N/C = LPC17 in sleep mode (LED1 glowing) */
 
/* Button definitions ***************************************************************/
/* The LPC1766-STK supports several buttons.  All will read "1" when open and "0"
 * when closed
 *
 * BUT1            -- Connected to P0[23]
 * BUT2            -- Connected to P2[13]
 * WAKE-UP         -- Connected to P2[12]
 *
 * And a Joystick
 *
 * CENTER          -- Connected to P0[4]
 * DOWN            -- Connected to P2[1]
 * LEFT            -- Connected to P2[7]
 * RIGHT           -- Connected to P2[8]
 * UP              -- Connected to P2[0]
 */

#define BOARD_BUTTON_1             0
#define BOARD_BUTTON_2             1
#define BOARD_BUTTON_WAKEUP        2

#define BOARD_JOYSTICK_CENTER      3
#define BOARD_JOYSTICK_DOWN        4
#define BOARD_JOYSTICK_LEFT        5
#define BOARD_JOYSTICK_RIGHT       6
#define BOARD_JOYSTICK_UP          7

#define BOARD_NUM_BUTTONS          8

#define BOARD_BUTTON_BUTTON1_BIT   (1 << BOARD_BUTTON_1)
#define BOARD_BUTTON_BUTTON2_BIT   (1 << BOARD_BUTTON_2)
#define BOARD_BUTTON_WAKEUP_BIT    (1 << BOARD_BUTTON_WAKEUP)

#define BOARD_JOYSTICK_CENTER_BIT  (1 << BOARD_JOYSTICK_CENTER)
#define BOARD_JOYSTICK_DOWN_BIT    (1 << BOARD_JOYSTICK_DOWN)
#define BOARD_JOYSTICK_LEFT_BIT    (1 << BOARD_JOYSTICK_LEFT)
#define BOARD_JOYSTICK_RIGHT_BIT   (1 << BOARD_JOYSTICK_RIGHT)
#define BOARD_JOYSTICK_UP_BIT      (1 << BOARD_JOYSTICK_UP)

/* Alternate pin selections *********************************************************/

/* CAN1 GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[0]/RD1/TXD3/SDA1               46  RD1
 * P0[1]/TD1/RXD3/SCL1               47  TD1
 */

#define GPIO_CAN1_RD  GPIO_CAN1_RD_1
#define GPIO_CAN1_TD  GPIO_CAN1_TD_1

/* UART0 GPIO                       PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[2]/TXD0/AD0[7]                 98  TXD0
 * P0[3]/RXD0/AD0[6]                 99  RXD0
 */

/* UART1 GPIO                       PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[15]/TXD1/SCK0/SCK              62  TXD1
 * P0[16]/RXD1/SSEL0/SSEL            63  RXD1
 * P0[17]/CTS1/MISO0/MISO            61  CTS1
 * P0[18]/DCD1/MOSI0/MOSI            60  DCD1
 * P0[19]/DSR1/SDA1                  59  DSR1
 * P0[20]/DTR1/SCL1                  58  DTR1
 * P0[22]/RTS1/TD1                   56  RTS1
 */

#define GPIO_UART1_TXD GPIO_UART1_TXD_1
#define GPIO_UART1_RXD GPIO_UART1_RXD_1
#define GPIO_UART1_CTS GPIO_UART1_CTS_1
#define GPIO_UART1_DCD GPIO_UART1_DCD_1
#define GPIO_UART1_DSR GPIO_UART1_DSR_1
#define GPIO_UART1_DTR GPIO_UART1_DTR_1
#define GPIO_UART1_RTS GPIO_UART1_RTS_1

/* SSP0 GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P1[21]/MCABORT/PWM1[3]/SSEL0      35  SSEL0
 * P1[20]/MCFB0/PWM1[2]/SCK0         34  SCK0
 * P1[23]/MCFB1/PWM1[4]/MISO0        37  MISO0
 * P1[24]/MCFB2/PWM1[5]/MOSI0        38  MOSI0
 */

#define GPIO_SSP0_SSEL GPIO_SSP0_SSEL_2
#define GPIO_SSP0_SCK  GPIO_SSP0_SCK_2
#define GPIO_SSP0_MISO GPIO_SSP0_MISO_2
#define GPIO_SSP0_MOSI GPIO_SSP0_MOSI_2

/* SSP1 GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     79  SSEL1
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      78  SCK1
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      77  MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     76  MOSI1
 */

#define GPIO_SSP1_SCK GPIO_SSP1_SCK_1

/* I2C2 GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[10]/TXD2/SDA2/MAT3[0]          48  SDA2
 * P0[11]/RXD2/SCL2/MAT3[1]          49  SCL2
 */

/* AD GPIO                          PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[24]/AD0[1]/I2SRX_WS/CAP3[1]     8  TEMP
 * P0[25]/AD0[2]/I2SRX_SDA/TXD3       7  MIC IN

/* USB GPIO                         PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[27]/SDA0/USB_SDA               25  USB_SDA
 * P0[28]/SCL0/USB_SCL               24  USB_SCL
 * P0[29]/USB_D+                     29  USB_D+
 * P0[30]/USB_D-                     30  USB_D-
 * P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] 32  USB_UP_LED
 * P1[19]/MC0A/#USB_PPWR/CAP1[1]     33  #USB_PPWR
 * P1[22]/MC0B/USB_PWRD/MAT1[0]      36  USBH_PWRD
 * P1[27]/CLKOUT/#USB_OVRCR/CAP0[1]  43  #USB_OVRCR
 * P1[30]/VBUS/AD0[4]                21  VBUS
 * P2[9]/USB_CONNECT/RXD2            64  USBD_CONNECT
 */

#ifdef GPIO_USB_PPWR  /* We can only redefine this if they have been defined */

/* The Olimex LPC1766-STK has 10K pull-ups on PPWR and OVRCR and a 100k
 * pull-down on PWRD so we should make sure that the outputs float.
 */

#  undef  GPIO_USB_PPWR
#  define GPIO_USB_PPWR    (GPIO_ALT2 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN19)
#  undef  GPIO_USB_OVRCR
#  define GPIO_USB_OVRCR   (GPIO_ALT2 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN27)
#  undef  GPIO_USB_PWRD
#  define GPIO_USB_PWRD    (GPIO_ALT2 | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN22)

/* In host mode (only) there are also 15K pull-downs on D+ and D- */

#  ifdef CONFIG_USBHOST
#    undef  GPIO_USB_DP
#    define GPIO_USB_DP    (GPIO_ALT1 | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN29)
#    undef  GPIO_USB_DM
#    define GPIO_USB_DM    (GPIO_ALT1 | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN30)
#  endif
#endif

/* Ethernet GPIO                    PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P1[0]/ENET_TXD0                   95  E_TXD0
 * P1[1]/ENET_TXD1                   94  E_TXD1
 * P1[4]/ENET_TX_EN                  93  E_TX_EN
 * P1[8]/ENET_CRS                    92  E_CRS
 * P1[9]/ENET_RXD0                   91  E_RXD0
 * P1[10]/ENET_RXD1                  90  E_RXD1
 * P1[14]/ENET_RX_ER                 89  E_RX_ER
 * P1[15]/ENET_REF_CLK               88  E_REF_CLK
 * P1[16]/ENET_MDC                   87  E_MDC
 * P1[17]/ENET_MDIO                  86  E_MDIO
 */

#define GPIO_ENET_MDC  GPIO_ENET_MDC_1
#define GPIO_ENET_MDIO GPIO_ENET_MDIO_1

/* Trace GPIO                       PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P2[2]/PWM1[3]/CTS1/TRACEDATA[3]   73  TRACE_D3
 * P2[3]/PWM1[4]/DCD1/TRACEDATA[2]   70  TRACE_D2
 * P2[4]/PWM1[5]/DSR1/TRACEDATA[1]   69  TRACE_D1
 * P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   68  TRACE_D0
 * P2[6]/PCAP1[0]/RI1/TRACECLK       67  TRACE_CLK
 */

/* EINT GPIO                       PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P2[11]/#EINT1/I2STX_CLK           52  #EINT1
 */

/* ?
 *  P0[26]/AD0[3]/AOUT/RXD3            6  AOUT
 *  P1[31]/SCK1/AD0[5]                20  AIN5
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void lpc17_boardinitialize(void);

/****************************************************************************
 * Name:  lpc17_ledinit and lpc17_setled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void lpc17_ledinit(void);
EXTERN void lpc17_setled(int led, bool ledon);
EXTERN void lpc17_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
