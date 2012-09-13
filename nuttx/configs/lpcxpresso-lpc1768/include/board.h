/************************************************************************************
 * configs/lpcxpresso-lpc1768/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
/* The LPCXpresso LPC1768 board has a single red LED (there are additional LEDs on
 * the base board not considered here).
 */
                                     /* ON      OFF                 */
#define LED_STARTED                0 /* OFF     ON  (never happens) */
#define LED_HEAPALLOCATE           0 /* OFF     ON  (never happens) */
#define LED_IRQSENABLED            0 /* OFF     ON  (never happens) */
#define LED_STACKCREATED           1 /* ON      ON  (never happens) */
#define LED_INIRQ                  2 /* OFF     NC  (momentary) */
#define LED_SIGNAL                 2 /* OFF     NC  (momentary) */
#define LED_ASSERTION              2 /* OFF     NC  (momentary) */
#define LED_PANIC                  0 /* OFF     ON  (1Hz flashing) */

/* Alternate pin selections *********************************************************/
/* Pin Description                  Connector On Board       Base Board
 * -------------------------------- --------- -------------- ---------------------
 * P0[0]/RD1/TXD3/SDA1               J6-9     I2C E2PROM SDA TXD3/SDA1
 * P0[1]/TD1/RXD3/SCL                J6-10                   RXD3/SCL1
 * P0[2]/TXD0/AD0[7]                 J6-21    
 * P0[3]/RXD0/AD0[6]                 J6-22    
 * P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38                   CAN_RX2
 * P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39                   CAN_TX2
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8                    SSEL1, OLED CS
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7                    SCK1, OLED SCK
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6                    MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5                    MOSI1, OLED data in
 * P0[10]                            J6-40                   TXD2/SDA2
 * P0[11]                            J6-41                   RXD2/SCL2
 * P0[15]/TXD1/SCK0/SCK              J6-13                   TXD1/SCK0
 * P0[16]/RXD1/SSEL0/SSEL            J6-14                   RXD1/SSEL0
 * P0[17]/CTS1/MISO0/MISO            J6-12                   MISO0
 * P0[18]/DCD1/MOSI0/MOSI            J6-11                   MOSI0
 * P0[19]/DSR1/SDA1                  PAD17                   N/A
 * P0[20]/DTR1/SCL1                  PAD18    I2C E2PROM SCL N/A
 * P0[21]/RI1/MCIPWR/RD1             J6-23                  
 * P0[22]/RTS1/TD1                   J6-24    LED            
 * P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]   J6-15                   AD0.0
 * P0[24]/AD0[1]/I2SRX_WS/CAP3[1]    J6-16                   AD0.1
 * P0[25]/AD0[2]/I2SRX_SDA/TXD3      J6-17                   AD0.2
 * P0[26]/AD0[3]/AOUT/RXD3           J6-18                   AD0.3/AOUT / RGB LED
 * P0[27]/SDA0/USB_SDA               J6-25                   
 * P0[28]/SCL0                       J6-26                   
 * P0[29]/USB_D+                     J6-37                   USB_D+
 * P0[30]/USB_D-                     J6-36                   USB_D-
 */

#define GPIO_UART3_TXD     GPIO_UART3_TXD_1
#define GPIO_I2C1_SDA      GPIO_I2C1_SDA_1
#define GPIO_UART3_RXD     GPIO_UART3_RXD_1
#define GPIO_I2C1_SCL      GPIO_I2C1_SCL_1
#define GPIO_SSP1_SCK      GPIO_SSP1_SCK_1
#define GPIO_UART2_TXD     GPIO_UART2_TXD_1
#define GPIO_UART2_RXD     GPIO_UART2_RXD_1
#define GPIO_UART1_TXD     GPIO_UART1_TXD_1
#define GPIO_SSP0_SCK      GPIO_SSP0_SCK_1
#define GPIO_UART1_RXD     GPIO_UART1_RXD_1
#define GPIO_SSP0_SSEL     GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO     GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI     GPIO_SSP0_MOSI_1

/* P1[0]/ENET-TXD0                   J6-34?  TXD0            TX-(Ethernet PHY)
 * P1[1]/ENET_TXD1                   J6-35?  TXD1            TX+(Ethernet PHY)
 * P1[4]/ENET_TX_EN                          TXEN            N/A
 * P1[8]/ENET_CRS                            CRS_DV/MODE2    N/A
 * P1[9]/ENET_RXD0                   J6-32?  RXD0/MODE0      RD-(Ethernet PHY)
 * P1[10]/ENET_RXD1                  J6-33?  RXD1/MODE1      RD+(Ethernet PHY)
 * P1[14]/ENET_RX_ER                         RXER/PHYAD0     N/A
 * P1[15]/ENET_REF_CLK                       REFCLK          N/A
 * P1[16]/ENET_MDC                           MDC             N/A
 * P1[17]/ENET_MDIO                          MDIO            N/A
 * P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] PAD1                    N/A
 * P1[19]/MC0A/USB_PPWR/N_CAP1.1     PAD2                    N/A
 * P1[20]/MCFB0/PWM1.2/SCK0          PAD3                    N/A
 * P1[21]/MCABORT/PWM1.3/SSEL0       PAD4                    N/A
 * P1[22]/MC0B/USB-PWRD/MAT1.0       PAD5                    N/A
 * P1[23]/MCFB1/PWM1.4/MISO0         PAD6                    N/A
 * P1[24]/MCFB2/PWM1.5/MOSI0         PAD7                    N/A
 * P1[25]/MC1A/MAT1.1                PAD8                    N/A
 * P1[26]/MC1B/PWM1.6/CAP0.0         PAD9                    N/A
 * P1[27]/CLKOUT/USB-OVRCR-N/CAP0.1  PAD10                   N/A
 * P1[28]/MC2A/PCAP1.0/MAT0.0        PAD11                   N/A
 * P1[29]/MC2B/PCAP1.1/MAT0.1        PAD12                   N/A
 * P1[30]/VBUS/AD0[4]                J6-19                   AD0.4
 * P1[31]/SCK1/AD0[5]                J6-20                   AD0.5
 */

#define GPIO_ENET_MDC      GPIO_ENET_MDC_1
#define GPIO_ENET_MDIO     GPIO_ENET_MDIO_1

/* P2[0]/PWM1.1/TXD1                 J6-42                   PWM1.1 / RGB LED / RS422 RX
 * P2[1]/PWM1.2/RXD1                 J6-43                   PWM1.2 / OLED voltage / RGB LED / RS422 RX
 * P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44                   PWM1.3
 * P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45                   PWM1.4
 * P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46                   PWM1.5
 * P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47                   PWM1.6
 * P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48    
 * P2[7]/RD2/RTS1                    J6-49                   OLED command/data
 * P2[8]/TD2/TXD2                    J6-50    
 * P2[9]/USB_CONNECT/RXD2            PAD19   USB Pullup      N/A
 * P2[10]/EINT0/NMI                  J6-51    
 * P2[11]/EINT1/I2STX_CLK            J6-52    
 * P2[12]/EINT2/I2STX_WS             J6-53    
 * P2[13]/EINT3/I2STX_SDA            J6-27                 
 */
 
#define GPIO_PWM1p1        GPIO_PWM1p1_2
#define GPIO_PWM1p2        GPIO_PWM1p2_2
#define GPIO_PWM1p3        GPIO_PWM1p3_2
#define GPIO_PWM1p4        GPIO_PWM1p4_2
#define GPIO_PWM1p5        GPIO_PWM1p5_2
#define GPIO_PWM1p6        GPIO_PWM1p6_2

/* P3[25]/MAT0.0/PWM1.2              PAD13                   N/A
 * P3[26]/STCLK/MAT0.1/PWM1.3        PAD14                   N/A
 *
 * P4[28]/RX-MCLK/MAT2.0/TXD3        PAD15                   N/A
 * P4[29]/TX-MCLK/MAT2.1/RXD3        PAD16                   N/A
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
