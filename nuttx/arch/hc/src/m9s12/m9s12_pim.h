/************************************************************************************
 * arch/hc/src/m9s12/m9s12_pim.h
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_PIM_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_PIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/
/* Friendly names for ports */

#define PIM_PORTT                (0)
#define PIM_PORTS                (1)
#define PIM_PORTG                (2)
#define PIM_PORTH                (3)
#define PIM_PORTJ                (4)
#define PIM_PORTL                (5)

/* Port register block offsets */

#define HCS12_PIM_PORT_OFFSET(n) (HCS12_PIM_BASE + ((n) << 3))
#define HCS12_PIM_PORTT_OFFSET   (0x0000)
#define HCS12_PIM_PORTS_OFFSET   (0x0008)
#define HCS12_PIM_PORTG_OFFSET   (0x0010)
#define HCS12_PIM_PORTH_OFFSET   (0x0018)
#define HCS12_PIM_PORTJ_OFFSET   (0x0020)
#define HCS12_PIM_PORTL_OFFSET   (0x0028)

/* Register offsets within a port register block */

#define HCS12_PIM_IO_OFFSET      (0x0000) /* I/O Register (ALL) */
#define HCS12_PIM_INPUT_OFFSET   (0x0001) /* Input Register (ALL) */
#define HCS12_PIM_DDR_OFFSET     (0x0002) /* Data Direction Register (ALL) */
#define HCS12_PIM_RDR_OFFSET     (0x0003) /* Reduced Drive Register (ALL) */
#define HCS12_PIM_PER_OFFSET     (0x0004) /* Pull Device Enable Register (ALL) */
#define HCS12_PIM_PS_OFFSET      (0x0005) /* Polarity Select Register (ALL) */
#define HCS12_PIM_WOM_OFFSET     (0x0006) /* Wired OR Mode Register (PORT S and L) */
#define HCS12_PIM_IE_OFFSET      (0x0006) /* Interrupt Enable Register (PORT G, H, and J) */
#define HCS12_PIM_IF_OFFSET      (0x0007) /* Interrupt Flag Register (PORT G, H, and J) */

/* Register Addresses ***************************************************************/
/* Port register block addresses */

#define HCS12_PIM_PORT_BASE(n)   (HCS12_PIM_BASE + HCS12_PIM_PORT_OFFSET(n))
#define HCS12_PIM_PORTT_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTT_OFFSET)
#define HCS12_PIM_PORTS_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTS_OFFSET)
#define HCS12_PIM_PORTG_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTG_OFFSET)
#define HCS12_PIM_PORTH_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTH_OFFSET)
#define HCS12_PIM_PORTJ_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTJ_OFFSET)
#define HCS12_PIM_PORTL_BASE     (HCS12_PIM_BASE + HCS12_PIM_PORTL_OFFSET)

/* Port register addresses */

#define HCS12_PIM_PORT_IO(n)     (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORT_INPUT(n)  (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORT_DDR(n)    (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORT_RDR(n)    (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORT_PER(n)    (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORT_PS(n)     (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORT_WOM(n)    (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_WOM_OFFSET)
#define HCS12_PIM_PORT_IE(n)     (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_IE_OFFSET)
#define HCS12_PIM_PORT_IF(n)     (HCS12_PIM_PORT_BASE(n) + HCS12_PIM_IF_OFFSET)

/* Port T register addresses */

#define HCS12_PIM_PORTT_IO       (HCS12_PIM_PORTT_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTT_INPUT    (HCS12_PIM_PORTT_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTT_DDR      (HCS12_PIM_PORTT_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTT_RDR      (HCS12_PIM_PORTT_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTT_PER      (HCS12_PIM_PORTT_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTT_PS       (HCS12_PIM_PORTT_BASE + HCS12_PIM_PS_OFFSET)

/* Port S register addresses */

#define HCS12_PIM_PORTS_IO       (HCS12_PIM_PORTS_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTS_INPUT    (HCS12_PIM_PORTS_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTS_DDR      (HCS12_PIM_PORTS_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTS_RDR      (HCS12_PIM_PORTS_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTS_PER      (HCS12_PIM_PORTS_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTS_PS       (HCS12_PIM_PORTS_BASE + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORTS_WOM      (HCS12_PIM_PORTS_BASE + HCS12_PIM_WOM_OFFSET)

/* Port G register addresses */

#define HCS12_PIM_PORTG_IO       (HCS12_PIM_PORTG_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTG_INPUT    (HCS12_PIM_PORTG_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTG_DDR      (HCS12_PIM_PORTG_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTG_RDR      (HCS12_PIM_PORTG_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTG_PER      (HCS12_PIM_PORTG_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTG_PS       (HCS12_PIM_PORTG_BASE + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORTG_IE       (HCS12_PIM_PORTG_BASE + HCS12_PIM_IE_OFFSET)
#define HCS12_PIM_PORTG_IF       (HCS12_PIM_PORTG_BASE + HCS12_PIM_IF_OFFSET)

/* Port H register addresses */

#define HCS12_PIM_PORTH_IO       (HCS12_PIM_PORTH_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTH_INPUT    (HCS12_PIM_PORTH_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTH_DDR      (HCS12_PIM_PORTH_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTH_RDR      (HCS12_PIM_PORTH_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTH_PER      (HCS12_PIM_PORTH_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTH_PS       (HCS12_PIM_PORTH_BASE + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORTH_IE       (HCS12_PIM_PORTH_BASE + HCS12_PIM_IE_OFFSET)
#define HCS12_PIM_PORTH_IF       (HCS12_PIM_PORTH_BASE + HCS12_PIM_IF_OFFSET)

/* Port J register addresses */

#define HCS12_PIM_PORTJ_IO       (HCS12_PIM_PORTJ_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTJ_INPUT    (HCS12_PIM_PORTJ_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTJ_DDR      (HCS12_PIM_PORTJ_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTJ_RDR      (HCS12_PIM_PORTJ_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTJ_PER      (HCS12_PIM_PORTJ_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTJ_PS       (HCS12_PIM_PORTJ_BASE + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORTJ_IE       (HCS12_PIM_PORTJ_BASE + HCS12_PIM_IE_OFFSET)
#define HCS12_PIM_PORTJ_IF       (HCS12_PIM_PORTJ_BASE + HCS12_PIM_IF_OFFSET)

/* Port L register addresses */

#define HCS12_PIM_PORTL_IO       (HCS12_PIM_PORTL_BASE + HCS12_PIM_IO_OFFSET)
#define HCS12_PIM_PORTL_INPUT    (HCS12_PIM_PORTL_BASE + HCS12_PIM_INPUT_OFFSET)
#define HCS12_PIM_PORTL_DDR      (HCS12_PIM_PORTL_BASE + HCS12_PIM_DDR_OFFSET)
#define HCS12_PIM_PORTL_RDR      (HCS12_PIM_PORTL_BASE + HCS12_PIM_RDR_OFFSET)
#define HCS12_PIM_PORTL_PER      (HCS12_PIM_PORTL_BASE + HCS12_PIM_PER_OFFSET)
#define HCS12_PIM_PORTL_PS       (HCS12_PIM_PORTL_BASE + HCS12_PIM_PS_OFFSET)
#define HCS12_PIM_PORTL_WOM      (HCS12_PIM_PORTL_BASE + HCS12_PIM_WOM_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Port register bits (all ports) */

#define PIM_PIN(n)              (1 << (n))
#define PIM_PIN0                (1 << 0)
#define PIM_PIN1                (1 << 1)
#define PIM_PIN2                (1 << 2)
#define PIM_PIN3                (1 << 3)
#define PIM_PIN4                (1 << 4)
#define PIM_PIN5                (1 << 5)
#define PIM_PIN6                (1 << 6)
#define PIM_PIN7                (1 << 7)

/* Port T I/O register aliases */

#define TIM_IOC4                PIM_PIN4
#define TIM_IOC5                PIM_PIN5
#define TIM_IOC6                PIM_PIN6
#define TIM_IOC7                PIM_PIN7

/* Port S I/O register aliases */

#define SCI0_RXD                PIM_PIN0
#define SCI0_TXD                PIM_PIN1
#define SCI1_RXD                PIM_PIN2
#define SCI1_TXD                PIM_PIN3
#define SPI_MISO                PIM_PIN4
#define SPI_MOSI                PIM_PIN5
#define SPI_SCK                 PIM_PIN6
#define SPI_SS                  PIM_PIN7

/* Port G I/O register aliases */

#define MII_RXD0                PIM_PIN0
#define MII_RXD1                PIM_PIN1
#define MII_RXD2                PIM_PIN2
#define MII_RXD3                PIM_PIN3
#define MII_RXCLK               PIM_PIN4
#define MII_RXDV                PIM_PIN5
#define MII_RXER                PIM_PIN6

/* Port H I/O register aliases */

#define MII_TXD0                PIM_PIN0
#define MII_TXD1                PIM_PIN1
#define MII_TXD2                PIM_PIN2
#define MII_TXD3                PIM_PIN3
#define MII_TXCLK               PIM_PIN4
#define MII_TXEN                PIM_PIN5
#define MII_TXER                PIM_PIN6

/* Port J I/O register aliases */

#define MII_MDC                 PIM_PIN0
#define MII_MDIO                PIM_PIN1
#define MII_CRS                 PIM_PIN2
#define MII_COL                 PIM_PIN3
#define IIC_SDA                 PIM_PIN5
#define IIC_SCL                 PIM_PIN6

/* Port L I/O register aliases */

#define PHY_ACTLED              PIM_PIN0
#define PHY_LNKLED              PIM_PIN1
#define PHY_SPDLED              PIM_PIN2
#define PHY_DUPLED              PIM_PIN3
#define PHY_COLLED              PIM_PIN4

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_PIM_H */
