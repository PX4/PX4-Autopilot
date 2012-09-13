/************************************************************************************
 * configs/ne64badge/src/ne64badge_internal.h
 * arch/arm/src/board/ne64badge_internal.n
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

#ifndef __CONFIGS_NE64BADGE_SRC_NE64BADGE_INTERNAL_H
#define __CONFIGS_NE64BADGE_SRC_NE64BADGE_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "m9s12_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* NE64BADGE Pin Usage **************************************************************/
/* PIN PIN NAME            BOARD SIGNAL   NOTES
 * --- ------------------- -------------- ----------------------
 *  44 RESET               J3 RESET_L     Also to SW3
 *  57 BKGD/MODC/TAGHI_B   BDM BKGD CON6A
 * 
 *  85 PAD0                VR1            Potentiometer
 *  86 PAD1                J3 ANALOG_IN0  Not used on board
 *  87 PAD2                J3 ANALOG_IN1  " " "  " "" "   "
 *  88 PAD3                J3 ANALOG_IN2  " " "  " "" "   "
 *  89 PAD4                J3 ANALOG_IN3  " " "  " "" "   "
 * 
 *  70 PHY_TXP             J7 TD+         RJ45 connector
 *  71 PHY_TXN             J7 TD-         RJ45 connector
 *  73 PHY_RXP             J7 RD+         RJ45 connector
 *  74 PHY_RXN             J7 RD-         RJ45 connector
 * 
 * Ports A,B,E,K managed by the MEBI block
 * ---------------------------------------
 *  60 PA0/ADDR8/DATA8     J3 ADDR_DATA8  Not used on board
 *  61 PA1/ADDR9/DATA9     J3 ADDR_DATA9  " " "  " "" "   "
 *  62 PA2/ADDR10/DATA10   J3 ADDR_DATA10 " " "  " "" "   "
 *  63 PA3/ADDR11/DATA11   J3 ADDR_DATA11 " " "  " "" "   "
 *  77 PA4/ADDR12/DATA12   J3 ADDR_DATA12 " " "  " "" "   "
 *  78 PA5/ADDR13/DATA13   J3 ADDR_DATA13 " " "  " "" "   "
 *  79 PA6/ADDR14/DATA14   J3 ADDR_DATA14 " " "  " "" "   "
 *  80 PA7/ADDR15/DATA15   J3 ADDR_DATA15 " " "  " "" "   "
 * 
 *  10 PB0/ADDR0/DATA0     J3 ADDR_DATA0  Not used on board
 *  11 PB1/ADDR1/DATA1     J3 ADDR_DATA1  " " "  " "" "   "
 *  12 PB2/ADDR2/DATA2     J3 ADDR_DATA2  " " "  " "" "   "
 *  13 PB3/ADDR3/DATA3     J3 ADDR_DATA3  " " "  " "" "   "
 *  16 PB4/ADDR4/DATA4     J3 ADDR_DATA4  " " "  " "" "   "
 *  17 PB5/ADDR5/DATA5     J3 ADDR_DATA5  " " "  " "" "   "
 *  18 PB6/ADDR6/DATA6     J3 ADDR_DATA6  " " "  " "" "   "
 *  19 PB7/ADDR7/DATA7     J3 ADDR_DATA7  " " "  " "" "   "
 * 
 *  56 PE0/XIRQ_B          BUTTON1        SW1
 *  55 PE1/IRQ_B           J3 IRQ         Not used on board
 *  54 PE2/R_W             J3 RW          " " "  " "" "   "
 *  53 PE3/LSTRB_B/TAGLO_B J3 LSTRB       " " "  " "" "   "
 *  41 PE4/ECLK            J3 ECLK        " " "  " "" "   "
 *  40 PE5/IPIPE0/MODA     J3 MODA        " " "  " "" "   "
 *  39 PE6/IPIPE1/MODB     J3 MODB        " " "  " "" "   "
 *  38 PE7/NOACC/XCLKS_B   pulled low     pulled low
 */

#define NE64BADGE_BUTTON1 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT_E | GPIO_PIN_0)

/*  97 PK0/XADR14          N/C            N/C
 *  98 PK1/XADR15          N/C            N/C
 *  99 PK2/XADR16          N/C            N/C
 * 100 PK3/XADR17          N/C            N/C
 * 103 PK4/XADR18          N/C            N/C
 * 104 PK5/XADR19          N/C            N/C
 * 105 PK6/XCS_B           J3 XCS         Not used on board
 * 106 PK7/ECS_B/ROMCTL    J3 ECS         " " "  " "" "   "
 * 
 * Ports T,S,G,H,J,L managed by the PIM Block
 * ------------------------------------------
 * 110 PT4/IOC1_4          J3 GPIO8       Not used on board
 * 109 PT5/IOC1_5          J3 GPIO9       " " "  " "" "   "
 * 108 PT6/IOC1_6          J3 GPIO10      " " "  " "" "   "
 * 107 PT7/IOC1_7          N/C            N/C
 * 
 *  30 PS0/RXD0            RS232_RX       Eventually maps to J2 RXD
 *  31 PS1/TXD0            RS232_TX       Eventually maps to J2 TXD
 *  32 PS2/RXD1            J3&J4 UART_RX  Not used on board
 *  33 PS3/TXD1            J3&J4 UART_TX  " " "  " "" "   "
 *  34 PS4/MISO            J3 SPI_MISO    " " "  " "" "   "
 *  35 PS5/MOSI            J3 SPI_MOSI    " " "  " "" "   "
 *  36 PS6/SCK             J3 SPI_CLOCK   " " "  " "" "   "
 *  37 PS7/SS_B            J3 SPI_SS      " " "  " "" "   "
 * 
 *  22 PG0/RXD0/KWG0       J3 GPIO0       Not used on board
 *  23 PG1/RXD1/KWG1       J3 GPIO1       " " "  " "" "   "
 *  24 PG2/RXD2/KWG2       J3 GPIO2       " " "  " "" "   "
 *  25 PG3/RXD3/KWG3       J3 GPIO3       " " "  " "" "   "
 *  26 PG4/RXCLK/KWG4      J3 GPIO4       " " "  " "" "   "
 *  27 PG5/RXDV/KWG5       J3 GPIO5       " " "  " "" "   "
 *  28 PG6/RXER/KWG6       J3 GPIO6       " " "  " "" "   "
 *  29 PG7/KWG7            J3 GPIO7       " " "  " "" "   "
 * 
 *   7 PH0/TXD0/KWH0       N/C            N/C
 *   6 PH1/TXD1/KWH1       N/C            N/C
 *   5 PH2/TXD2/KWH2       J4 XBEE_RESET  Not used on board
 *   4 PH3/TXD3/KWH3       J4 XBEE_RSSI   Not used on board
 *   3 PH4/TXCLK/KWH4      BUTTON2        SW2
 *   2 PH5/TXDV/KWH5       J5 XBEE_LOAD_H Not used on board
 *   1 PH6/TXER/KWH6       J4 XBEE_LOAD_L Not used on board
 */

#define NE64BADGE_BUTTON2 (GPIO_INPUT | GPIO_PULLUP | GPIO_PORT_H | GPIO_PIN_4)
 
/*   8 PJ0/MDC/KWJ0        LED1           D21, red
 *   9 PJ1/MDIO/KWJ1       LED2           D22, red
 *  20 PJ2/CRS/KWJ2        J3 SPI_CS      Not used on board
 *  21 PJ3/COL/KWJ3        N/C
 * 112 PJ6/SDA/KWJ6        J3 I2C_DATA    Not used on board
 * 111 PJ7/SCL/KWJ7        J3 I2C_CLOCK   " " "  " "" "   "
 */

#define NE64BADGE_LED1 (GPIO_OUTPUT | GPIO_OUTPUT_HIGH | GPIO_PORT_J | GPIO_PIN_0)
#define NE64BADGE_LED2 (GPIO_OUTPUT | GPIO_OUTPUT_HIGH | GPIO_PORT_J | GPIO_PIN_1)

/*  51 PL6/TXER/KWL6       N/C            N/C
 *  52 PL5/TXDV/KWL5       N/C            N/C
 *  58 PL4/COLLED          Collision LED  red
 *  59 PL3/DUPLED          Full Duplex LED yellow
 *  81 PL2/SPDLED          100Mbps Speed LED yellow
 *  83 PL1/LNKLED          Link Good LED  green
 *  84 PL0/ACTLED          Activity LED   yellow
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
 * Name: up_ledinit
 *
 * Description:
 *   Configure and initialize on-board LEDs
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
#endif

/************************************************************************************
 * Name: hcs12_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3210E-EVAL board.
 *
 ************************************************************************************/

extern void weak_function hcs12_spiinitialize(void);


#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_NE64BADGE_SRC_NE64BADGE_INTERNAL_H */

