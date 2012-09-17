/************************************************************************************
 * arch/hc/src/m9s12/m9s12_phyv2.h (v2)
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

#ifndef __ARCH_ARM_HC_SRC_M9S12_M9S12_PHY_H
#define __ARCH_ARM_HC_SRC_M9S12_M9S12_PHY_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_PHY_EPHYCTL0_OFFSET      0x0000 /* Ethernet Physical Transceiver Control Register 0 */
#define HCS12_PHY_EPHYCTL1_OFFSET      0x0001 /* Ethernet Physical Transceiver Control Register 1 */
#define HCS12_PHY_EPHYSR_OFFSET        0x0002 /* Ethernet Physical Transceiver Status Register */

/* Register Addresses ***************************************************************/

#define HCS12_PHY_EPHYCTL0             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL0_OFFSET)
#define HCS12_PHY_EPHYCTL1             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL1_OFFSET)
#define HCS12_PHY_EPHYSR               (HCS12_EPHY_BASE+HCS12_PHY_EPHYSR_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Ethernet Physical Transceiver Control Register 0 */

#define PHY_EPHYCTL0_EPHYIEN           (1 << 0)  /* Bit 0: EPHY Interrupt Enable */
#define PHY_EPHYCTL0_EPHYWAI           (1 << 2)  /* Bit 2: EPHY Module Stops While in Wait */
#define PHY_EPHYCTL0_LEDEN             (1 << 3)  /* Bit 3: LED Drive Enable */
#define PHY_EPHYCTL0_DIS10             (1 << 4)  /* Bit 4: Disable 10BASE-T PLL */
#define PHY_EPHYCTL0_DIS100            (1 << 5)  /* Bit 5: Disable 100 BASE-TX PLL */
#define PHY_EPHYCTL0_ANDIS             (1 << 6)  /* Bit 6: Auto Negotiation Disable */
#define PHY_EPHYCTL0_EPHYEN            (1 << 7)  /* Bit 7: EPHY Enable */

/* Ethernet Physical Transceiver Control Register 1 */

#define PHY_EPHYCTL1_PHYADD_SHIFT      (0)       /* Bits 0-4: EPHY Address for MII Requests */
#define PHY_EPHYCTL1_PHYADD_MASK       (0x1f)

/* Ethernet Physical Transceiver Status Register */

#define PHY_EPHYSR_EPHYI               (1 << 0)  /* Bit 0: EPHY Interrupt Flag */
#define PHY_EPHYSR_10DIS               (1 << 4)  /* Bit 4: EPHY Port 10BASE-T mode status */
#define PHY_EPHYSR_100DIS              (1 << 5)  /* Bit 5: EPHY Port 100BASE-TX mode status */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S12_M9S12_PHY_H */
