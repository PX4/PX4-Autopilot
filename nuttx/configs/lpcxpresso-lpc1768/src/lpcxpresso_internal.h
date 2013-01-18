/************************************************************************************
 * configs/lpcxpresso-lpc1768/src/lpcxpresso_internal.h
 * arch/arm/src/board/lpcxpresso_internal.n
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

#ifndef _CONFIGS_LPCXPRESSO_LPC1768_SRC_LPCXPRESSO_INTERNAL_H
#define _CONFIGS_LPCXPRESSO_LPC1768_SRC_LPCXPRESSO_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LPCXpresso LPC1768 board pin usage ***********************************************/
/* Pin Description                  Connector On Board       Base Board
 * -------------------------------- --------- -------------- ---------------------
 * P0[0]/RD1/TXD3/SDA1               J6-9     I2C E2PROM SDA TXD3/SDA1
 * P0[1]/TD1/RXD3/SCL                J6-10                   RXD2/SCL1
 * P0[2]/TXD0/AD0[7]                 J6-21    
 * P0[3]/RXD0/AD0[6]                 J6-22    
 * P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38                   CAN_RX2
 * P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39                   CAN_TX2
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8                    SSEL1
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7                    SCK1
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6                    MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5                    MOSI1
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
 * P0[26]/AD0[3]/AOUT/RXD3           J6-18                   AD0.3/AOUT
 * P0[27]/SDA0/USB_SDA               J6-25                   
 * P0[28]/SCL0                       J6-26                   
 * P0[29]/USB_D+                     J6-37                   USB_D+
 * P0[30]/USB_D-                     J6-36                   USB_D-
 */

#define LPCXPRESSO_I2C1_EPROM_SDA GPIO_I2C1_SDA_1
#define LPCXPRESSO_I2C1_EPROM_SDL GPIO_I2C1_SCL_2
#define LPCXPRESSO_LED (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN22)

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
 *
 * P2[0]/PWM1.1/TXD1                 J6-42                   PWM1.1
 * P2[1]/PWM1.2/RXD1                 J6-43                   PWM1.2
 * P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44                   PWM1.3
 * P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45                   PWM1.4
 * P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46                   PWM1.5
 * P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47                   PWM1.6
 * P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48    
 * P2[7]/RD2/RTS1                    J6-49    
 * P2[8]/TD2/TXD2                    J6-50    
 * P2[9]/USB_CONNECT/RXD2            PAD19   USB Pullup      N/A
 * P2[10]/EINT0/NMI                  J6-51    
 * P2[11]/EINT1/I2STX_CLK            J6-52    
 * P2[12]/EINT2/I2STX_WS             J6-53    
 * P2[13]/EINT3/I2STX_SDA            J6-27                 
 */

#define LPCXPRESSO_USB_PULLUP (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN9)

/* P3[25]/MAT0.0/PWM1.2              PAD13                   N/A
 * P3[26]/STCLK/MAT0.1/PWM1.3        PAD14                   N/A
 *
 * P4[28]/RX-MCLK/MAT2.0/TXD3        PAD15                   N/A
 * P4[29]/TX-MCLK/MAT2.1/RXD3        PAD16                   N/A
 */
 
/* SD Slot
 *
 *      Base-board  J4/J6 LPC1768
 * SD   Signal      Pin   Pin
 * ---  ----------- ----- --------
 * CS   PIO1_11*     55   P2.2        (See LPCXPRESSO_SD_CS)
 * DIN  PIO0_9-MOSI   5   P0.9 MOSI1  (See GPIO_SSP1_MOSI in chip/lpc17_ssp.h)
 * DOUT PIO0_8-MISO   6   P0.8 MISO1  (See GPIO_SSP1_MISO in chip/lpc17_ssp.h)
 * CLK  PIO2_11-SCK   7   P0.9 SCK1   (See GPIO_SSP1_SCK in board.h)
 * CD   PIO2_10      52   P2.11       (See LPCXPRESSO_SD_CD)
 */

#define LPCXPRESSO_SD_CS (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN2)
#ifdef CONFIG_GPIO_IRQ
#  define LPCXPRESSO_SD_CD (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#else
#  define LPCXPRESSO_SD_CD (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#endif

/* USB:
 *
 *   Base-board          J4/J6 LPC1768
 *   Signal              Pin   Pin
 *   ------------------- ----- --------
 *   USB_DM              36    P0.30/USB-D-
 *   USB_DP              37    P0.29/USB-D+
 *   N/C                 N/C   P1.18/USB-UP-LED/PWM1.1/CAP1.0
 *   PIO1_3 (not used)   19    P1.30/VBUS/AD0.4
 *   N/C                 N/C   P2.9/USB-CONNECT/RXD2 (See Notes)
 *   ------------------- ----- --------
 *   PIO0_6-USB_CONNECT* 23    P0.21/RI1/RD1
 *   PIO0_3-VBUS_SENSE   39    P0.5/I2SRX-WS/TD2/CAP2.1
 *
 * Notes:
 * - The standard USB CONNECT (P0.9) provides USB D+ pullup on board the
 *   LPCXpresso card; it should be un-necessary to use the based board
 *   version of the pullup.
 * - No changes to jumper settings are required.  There are few USB-
 *   related jumpers on the based board, but none are required:
 *   - J14 must be set to permit GPIO control of the base board USB
 *     connect pin. NOT USED
 *   - J12 must be set to permit GPIO control of the USB vbus sense pin
 * - The standard VBUS (P1.30) is not connected.  As a consequence, the
 *   USB driver will not correctly respond to USB insertion or removal
 *   events.
 * - The standard USB LED (P1.18) is not connected.
 */

#define LPCXPRESSO_USB_CONNECT (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN21)
#ifdef CONFIG_GPIO_IRQ
#  define LPCXPRESSO_USB_VBUSSENSE (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#else
#  define LPCXPRESSO_USB_VBUSSENSE (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#endif

/* 96x64 White OLED with I2C/SPI interface
 *
 *  ----------------------------+-------+-------------- -----------------------------
 *  LPC1758 Pin                 | J4/6  | Base Board    Description
 *  ----------------------------+-------+-------------- -----------------------------
 *  P2.1/PWM1.2/RXD1            |  43   | PIO1_10       FAN5331 Power Control (SHDN#)
 *  P0.6/I2SRX-SDA/SSEL1/MAT2.0 |   8   | PIO0_2        OLED chip select (CS#)
 *  P2.7/RD2/RTS1               |  49   | PIO2_7        OLED command/data (D/C#)
 *  P0.7/I2STX-CLK/SCK1/MAT2.1  |   7   | PIO2_11-SCK   OLED clock (D0)
 *  P0.9/I2STX-SDA/MOSI1/MAT2.3 |   5   | PIO0_9-MOSI   OLED data in (D1)
 *  ----------------------------+-------+-------------- -----------------------------
 */

#define LPCXPRESSO_OLED_POWER (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN1)
#define LPCXPRESSO_OLED_CS    (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT0 | GPIO_PIN6)
#define LPCXPRESSO_OLED_DC    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN7)

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
 *   Called to configure SPI chip select GPIO pins for the LPCXpresso board.
 *
 ************************************************************************************/

extern void weak_function lpc17_sspinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_LPCXPRESSO_LPC1768_SRC_LPCXPRESSO_INTERNAL_H */

