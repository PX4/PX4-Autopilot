/************************************************************************************
 * arch/hc/src/m9s12/m9s12_iic.h (v2)
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

#ifndef __ARCH_ARM_HC_SRC_M9S124_M9S124_IIC_H
#define __ARCH_ARM_HC_SRC_M9S124_M9S124_IIC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_IIC_IBAD_OFFSET          0x0000 /* IIC Address Register */
#define HCS12_IIC_IBFD_OFFSET          0x0001 /* IIC Frequency Divider Register */
#define HCS12_IIC_IBCR_OFFSET          0x0002 /* IIC Control Register */
#define HCS12_IIC_IBSR_OFFSET          0x0003 /* IIC Status Register */
#define HCS12_IIC_IBDR_OFFSET          0x0004 /* IIC Data I/O Register */

/* Register Addresses ***************************************************************/

#define HCS12_IIC_IBAD                 (HCS12_IIC_BASE+HCS12_IIC_IBAD_OFFSET)
#define HCS12_IIC_IBFD                 (HCS12_IIC_BASE+HCS12_IIC_IBFD_OFFSET)
#define HCS12_IIC_IBCR                 (HCS12_IIC_BASE+HCS12_IIC_IBCR_OFFSET)
#define HCS12_IIC_IBSR                 (HCS12_IIC_BASE+HCS12_IIC_IBSR_OFFSET)
#define HCS12_IIC_IBDR                 (HCS12_IIC_BASE+HCS12_IIC_IBDR_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* IIC Address Register */

#define IIC_IBAD_MASK                  (0xfe)

/* IIC Frequency Divider Register -- 8-bit bus clock rate value */

/* IIC Control Register */

#define IIC_IBCR_IBSWAI                (1 << 0)  /* Bit 0: I Bus Interface Stop in Wait Mode */
#define IIC_IBCR_RSTA                  (1 << 2)  /* Bit 2: Repeat Start */
#define IIC_IBCR_TXAK                  (1 << 3)  /* Bit 3: Transmit Acknowledge Enable— */
#define IIC_IBCR_TX                    (1 << 4)  /* Bit 4: Transmit/Receive Mode Select Bit */
#define IIC_IBCR_MSSL                  (1 << 5)  /* Bit 5: Master/Slave Mode Select Bit— */
#define IIC_IBCR_IBIE                  (1 << 6)  /* Bit 6: I-Bus Interrupt Enable */
#define IIC_IBCR_IBEN                  (1 << 7)  /* Bit 7: I-Bus Enable */

/* IIC Status Register */

#define IIC_IBSR_RXAK                  (1 << 0)  /* Bit 0: Received Acknowledge */
#define IIC_IBSR_IBIF                  (1 << 1)  /* Bit 1: I-Bus Interrupt */
#define IIC_IBSR_SRW                   (1 << 2)  /* Bit 2: Slave Read/Write */
#define IIC_IBSR_AL                    (1 << 4)  /* Bit 4: Arbitration Lost */
#define IIC_IBSR_BB                    (1 << 5)  /* Bit 5: Bus Busy Bit */
#define IIC_IBSR_AAS                   (1 << 6)  /* Bit 6: Addressed as a Slave Bit */
#define IIC_IBSR_TCF                   (1 << 7)  /* Bit 7: Data Transferring Bit */

/* IIC Data I/O Register -- 8-Bit data value */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_M9S124_M9S124_IIC_H */
