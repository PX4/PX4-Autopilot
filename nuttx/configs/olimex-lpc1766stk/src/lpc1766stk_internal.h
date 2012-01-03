/************************************************************************************
 * configs/olimex-lpc1766stk/src/lpc1766stk_internal.h
 * arch/arm/src/board/lpc1766stk_internal.n
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

#ifndef _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H
#define _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LPC1766-STK GPIO Pin Definitions *************************************************/
/* Board GPIO Usage:
 *
 *  GPIO                             PIN  SIGNAL NAME
 *  -------------------------------- ---- --------------
 *  P0[0]/RD1/TXD3/SDA1               46  RD1
 *  P0[1]/TD1/RXD3/SCL1               47  TD1
 *  P0[2]/TXD0/AD0[7]                 98  TXD0
 *  P0[3]/RXD0/AD0[6]                 99  RXD0
 *  P0[4]/I2SRX_CLK/RD2/CAP2[0]       81  LED2/ACC IRQ
 *  P0[5]/I2SRX_WS/TD2/CAP2[1]        80  CENTER
 *  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     79  SSEL1
 *  P0[7]/I2STX_CLK/SCK1/MAT2[1]      78  SCK1
 *  P0[8]/I2STX_WS/MISO1/MAT2[2]      77  MISO1
 *  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     76  MOSI1
 *  P0[10]/TXD2/SDA2/MAT3[0]          48  SDA2
 *  P0[11]/RXD2/SCL2/MAT3[1]          49  SCL2
 *  P0[15]/TXD1/SCK0/SCK              62  TXD1
 *  P0[16]/RXD1/SSEL0/SSEL            63  RXD1
 *  P0[17]/CTS1/MISO0/MISO            61  CTS1
 *  P0[18]/DCD1/MOSI0/MOSI            60  DCD1
 *  P0[19]/DSR1/SDA1                  59  DSR1
 *  P0[20]/DTR1/SCL1                  58  DTR1
 *  P0[21]/RI1/RD1                    57  MMC PWR
 *  P0[22]/RTS1/TD1                   56  RTS1
 *  P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]    9  BUT1
 *  P0[24]/AD0[1]/I2SRX_WS/CAP3[1]     8  TEMP
 *  P0[25]/AD0[2]/I2SRX_SDA/TXD3       7  MIC IN
 *  P0[26]/AD0[3]/AOUT/RXD3            6  AOUT
 *  P0[27]/SDA0/USB_SDA               25  USB_SDA
 *  P0[28]/SCL0/USB_SCL               24  USB_SCL
 *  P0[29]/USB_D+                     29  USB_D+
 *  P0[30]/USB_D-                     30  USB_D-
 *  P1[0]/ENET_TXD0                   95  E_TXD0
 *  P1[1]/ENET_TXD1                   94  E_TXD1
 *  P1[4]/ENET_TX_EN                  93  E_TX_EN
 *  P1[8]/ENET_CRS                    92  E_CRS
 *  P1[9]/ENET_RXD0                   91  E_RXD0
 *  P1[10]/ENET_RXD1                  90  E_RXD1
 *  P1[14]/ENET_RX_ER                 89  E_RX_ER
 *  P1[15]/ENET_REF_CLK               88  E_REF_CLK
 *  P1[16]/ENET_MDC                   87  E_MDC
 *  P1[17]/ENET_MDIO                  86  E_MDIO
 *  P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] 32  USB_UP_LED
 *  P1[19]/MC0A/#USB_PPWR/CAP1[1]     33  #USB_PPWR
 *  P1[20]/MCFB0/PWM1[2]/SCK0         34  SCK0
 *  P1[21]/MCABORT/PWM1[3]/SSEL0      35  SSEL0
 *  P1[22]/MC0B/USB_PWRD/MAT1[0]      36  USBH_PWRD
 *  P1[23]/MCFB1/PWM1[4]/MISO0        37  MISO0
 *  P1[24]/MCFB2/PWM1[5]/MOSI0        38  MOSI0
 *  P1[25]/MC1A/MAT1[1]               39  LED1
 *  P1[26]/MC1B/PWM1[6]/CAP0[0]       40  CS_UEXT
 *  P1[27]/CLKOUT/#USB_OVRCR/CAP0[1]  43  #USB_OVRCR
 *  P1[28]/MC2A/PCAP1[0]/MAT0[0]      44  P1.28
 *  P1[29]/MC2B/PCAP1[1]/MAT0[1]      45  P1.29
 *  P1[30]/VBUS/AD0[4]                21  VBUS
 *  P1[31]/SCK1/AD0[5]                20  AIN5
 *  P2[0]/PWM1[1]/TXD1                75  UP
 *  P2[1]/PWM1[2]/RXD1                74  DOWN
 *  P2[2]/PWM1[3]/CTS1/TRACEDATA[3]   73  TRACE_D3
 *  P2[3]/PWM1[4]/DCD1/TRACEDATA[2]   70  TRACE_D2
 *  P2[4]/PWM1[5]/DSR1/TRACEDATA[1]   69  TRACE_D1
 *  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   68  TRACE_D0
 *  P2[6]/PCAP1[0]/RI1/TRACECLK       67  TRACE_CLK
 *  P2[7]/RD2/RTS1                    66  LEFT
 *  P2[8]/TD2/TXD2                    65  RIGHT
 *  P2[9]/USB_CONNECT/RXD2            64  USBD_CONNECT
 *  P2[10]/#EINT0/NMI                 53  ISP_E4
 *  P2[11]/#EINT1/I2STX_CLK           52  #EINT1
 *  P2[12]/#EINT2/I2STX_WS            51  WAKE-UP
 *  P2[13]/#EINT3/I2STX_SDA           50  BUT2
 *  P3[25]/MAT0[0]/PWM1[2]            27  LCD_RST
 *  P3[26]/STCLK/MAT0[1]/PWM1[3]      26  LCD_BL
 */

/* LEDs GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P1[25]/MC1A/MAT1[1]               39  LED1
 * P0[4]/I2SRX_CLK/RD2/CAP2[0]       81  LED2/ACC IRQ
 *
 * LEDs are connected to +3.3V through a diode on one side and must be pulled
 * low (through a resistor) on the LPC17xx side in order to illuminuate them.
 */

#define LPC1766STK_LED1       (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN25)
#define LPC1766STK_LED2       (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN4)

/* Buttons GPIO                     PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]    9  BUT1
 * P2[13]/#EINT3/I2STX_SDA           50  BUT2
 * P2[12]/#EINT2/I2STX_WS            51  WAKE-UP
 *
 * NOTES:
 * 1. Pull-ups are not required because the pins are already pulled-up by
 *    through resistors on the board.
 * 2. All buttons are capable of supporting interrupts if up_irqbutton() is
 *    called to attach an interrupt handler.  Interrupts are configured to
 *    occur on both edges.
 */

#define LPC1766STK_BUT1       (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN23)
#define LPC1766STK_BUT2       (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN13)
#define LPC1766STK_WAKEUP     (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN12)

/* Button IRQ numbers */

#define LPC1766STK_BUT1_IRQ   LPC17_IRQ_P0p23
#define LPC1766STK_BUT2_IRQ   LPC17_IRQ_P2p13
#define LPC1766STK_WAKEUP_IRQ LPC17_IRQ_P2p12

/* Joystick GPIO                    PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[5]/I2SRX_WS/TD2/CAP2[1]        80  CENTER
 * P2[0]/PWM1[1]/TXD1                75  UP
 * P2[1]/PWM1[2]/RXD1                74  DOWN
 * P2[7]/RD2/RTS1                    66  LEFT
 * P2[8]/TD2/TXD2                    65  RIGHT
 *
 * NOTES:
 * 1. Pull-ups are not required because the pins are already pulled-up by
 *    through resistors on the board.
 * 2. All buttons are capable of supporting interrupts if up_irqbutton() is
 *    called to attach an interrupt handler.  Interrupts are configured to
 *    occur on both edges.
 */

#define LPC1766STK_CENTER     (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN5)
#define LPC1766STK_UP         (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN0)
#define LPC1766STK_DOWN       (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN1)
#define LPC1766STK_LEFT       (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN7)
#define LPC1766STK_RIGHT      (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN8)

/* Joystick IRQ numbers */

#define LPC1766STK_CENTER_IRQ LPC17_IRQ_P0p5
#define LPC1766STK_UP_IRQ     LPC17_IRQ_P2p0
#define LPC1766STK_DOWN_IRQ   LPC17_IRQ_P2p1
#define LPC1766STK_LEFT_IRQ   LPC17_IRQ_P2p7
#define LPC1766STK_RIGHT_IRQ  LPC17_IRQ_P2p8

/* Nokia LCD GPIO                   PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P1[21]/MCABORT/PWM1[3]/SSEL0      35  SSEL0
 * P1[20]/MCFB0/PWM1[2]/SCK0         34  SCK0
 * P1[23]/MCFB1/PWM1[4]/MISO0        37  MISO0
 * P1[24]/MCFB2/PWM1[5]/MOSI0        38  MOSI0
 * P3[25]/MAT0[0]/PWM1[2]            27  LCD_RST
 * P3[26]/STCLK/MAT0[1]/PWM1[3]      26  LCD_BL (PWM1)
 */

#define LPC1766STK_LCD_CS     (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN21)
#define LPC1766STK_LCD_RST    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT3 | GPIO_PIN25)
#define LPC1766STK_LCD_BL     GPIO_PWM1p3_3
#define GPIO_PWM1p3           GPIO_PWM1p3_3

/* SD/MMC GPIO                      PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     79  SSEL1 (active low)
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      78  SCK1
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      77  MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     76  MOSI1
 * P0[21]/RI1/RD1                    57  MMC PWR (active low)
 */

#define LPC1766STK_MMC_CS     (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN6)
#define LPC1766STK_MMC_PWR    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN21)

/* AD GPIO                          PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P0[24]/AD0[1]/I2SRX_WS/CAP3[1]     8  TEMP
 * P0[25]/AD0[2]/I2SRX_SDA/TXD3       7  MIC IN
 */

#define LPC1766STK_TEMP       GPIO_AD0p1
#define LPC1766STK_MIC_IN     GPIO_AD0p2

/* UEXT GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P1[26]/MC1B/PWM1[6]/CAP0[0]       40  CS_UEXT
 */

#define LPC1766STK_CS_UEXT (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN26)

/* ISP? GPIO                        PIN  SIGNAL NAME
 * -------------------------------- ---- --------------
 * P2[10]/#EINT0/NMI                 53  ISP_E4
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Olimex LPC1766-STK board.
 *
 ************************************************************************************/

void weak_function lpc17_sspinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_OLIMEX_LPC1766STK_SRC_LPC1766STK_INTERNAL_H */

