/************************************************************************************
 * configs/nucleus2g/src/nucleus2g_internal.h
 * arch/arm/src/board/nucleus2g_internal.n
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef _CONFIGS_NUCLEUS2G_SRC_NUCLEUS2G_INTERNAL_H
#define _CONFIGS_NUCLEUS2G_SRC_NUCLEUS2G_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* NUCLEUS-2G GPIO Pin Definitions **************************************************/
/* Board GPIO Usage:
 *
 * P0[0]/RD1/TXD3/SDA1               P0[0]/CAN_RX1
 * P0[1]/TD1/RXD3/SCL                P0[1]/CAN_TX1
 * P0[2]/TXD0/AD0[7]                 TX0
 * P0[3]/RXD0/AD0[6]                 RX0
 * P0[4]                             P0[4]/CAN1_STB
 * P0[5]                             P0[5]/CAN2_STB
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     GPI/O_CS1
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      SCLK1
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     MOSI1
 * P0[10]                            P0[10]/CAN1_TERM
 * P0[11]                            P0[11]/CAN2_TERM
 * P0[15]/TXD1/SCK0/SCK              MMC_CLK
 * P0[16]/RXD1/SSEL0/SSEL            MMC_CD
 * P0[17]/CTS1/MISO0/MISO            MMC_DATA0
 * P0[18]/DCD1/MOSI0/MOSI            MMC_MISO
 * P0[19]/DSR1/SDA1                  GPI/O_CS2
 * P0[20]/DTR1/SCL1                  GPI/O_CS3
 * P0[21]/RI1/MCIPWR/RD1             P0[21]
 * P0[22]/RTS1/TD1                   P0[22]
 * P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]   AD0
 * P0[24]/AD0[1]/I2SRX_WS/CAP3[1]    AD1
 * P0[25]/AD0[2]/I2SRX_SDA/TXD3      AD2
 * P0[26]/AD0[3]/AOUT/RXD3           AD3
 * P0[27]/SDA0/USB_SDA               SDA
 * P0[28]/SCL0                       SCL
 * P0[29]/USB_D+                     USB+
 * P0[30]/USB_D-                     USB-
 *
 * P1[0] - P1[17]                    Not connected
 * P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] USB_LINK
 * P1[19]-P[29]                      P[19]-P[29]
 * P1[30]/VBUS/AD0[4]                USB_+5
 * P1[31]/SCK1/AD0[5]                AD5
 * 
 * P2[0]                             P2[0]/LED1_A
 * P2[1]                             P2[1]/LED1_B
 * P2[2]                             P2[2]/LED2_A
 * P2[3]                             P2[3]/LED2_B
 * P2[4]                             P2[4]
 * P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   232_POWERAVE
 * P2[6]/PCAP1[0]/RI1/TRACECLK       232_VALID
 * P2[7]/RD2/RTS1                    P2[7]/CAN_RX2
 * P2[8]/TD2/TXD2                    P2[8]/CAN_TX2
 * P2[9]/USB_CONNECT/RXD2            USB_CONNECT
 * P2[10]/EINT0/NMI                  BOOTLOADER
 * P2[11]/EINT1/I2STX_CLK            HEARTBEAT
 * P2[12]/EINT2/I2STX_WS             EXTRA_LED
 * P2[13]/EINT3/I2STX_SDA            5V_ENABLE
 *
 * P3[25]-P3[26]                     Not connected
 *
 * P4[28]-P4[29]                     P4[28]-P4[29]
 */

#define NUCLEUS2G_LED1_A             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN0)
#define NUCLEUS2G_LED1_B             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN1)
#define NUCLEUS2G_LED2_A             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN2)
#define NUCLEUS2G_LED2_B             (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN3)
#define NUCLEUS2G_232_ENABLE         (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN5)
#define NUCLEUS2G_232_POWERSAVE      (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN5)
#define NUCLEUS2G_232_VALID          (GPIO_INPUT  | GPIO_PULLUP     | GPIO_PORT2 | GPIO_PIN5)
#define NUCLEUS2G_HEARTBEAT          (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN11)
#define NUCLEUS2G_EXTRA_LED          (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN12)
#define NUCLEUS2G_5V_ENABLE          (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT2 | GPIO_PIN13)
#define NUCLEUS2G_5V_DISABLE         (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN13)

#define NUCLEUS2G_MMCSD_CS           (GPIO_OUTPUT | GPIO_VALUE_ONE  |  GPIO_PORT0 | GPIO_PIN16)

#define NUCLEUS_BMS_RELAY1  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN20)
#define NUCLEUS_BMS_RELAY2  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN21)
#define NUCLEUS_BMS_RELAY3  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN22)
#define NUCLEUS_BMS_RELAY4  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN23)

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
 *   Called to configure SPI chip select GPIO pins for the NUCLEUS-2G board.
 *
 ************************************************************************************/

extern void weak_function lpc17_sspinitialize(void);

extern void up_relayinit(void);

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_NUCLEUS2G_SRC_NUCLEUS2G_INTERNAL_H */

