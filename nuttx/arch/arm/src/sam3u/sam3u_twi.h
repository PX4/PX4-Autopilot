/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_twi.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_TWI_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_TWI_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* TWI register offsets *****************************************************************/

#define SAM3U_TWI_CR_OFFSET          0x00 /* Control Register */
#define SAM3U_TWI_MMR_OFFSET         0x04 /* Master Mode Register */
#define SAM3U_TWI_SMR_OFFSET         0x08 /* Slave Mode Register */
#define SAM3U_TWI_IADR_OFFSET        0x0c /* Internal Address Register */
#define SAM3U_TWI_CWGR_OFFSET        0x10 /* Clock Waveform Generator Register */
#define SAM3U_TWI_SR_OFFSET          0x20 /* Status Register */
#define SAM3U_TWI_IER_OFFSET         0x24 /* Interrupt Enable Register */
#define SAM3U_TWI_IDR_OFFSET         0x28 /* Interrupt Disable Register */
#define SAM3U_TWI_IMR_OFFSET         0x2c /* Interrupt Mask Register */
#define SAM3U_TWI_RHR_OFFSET         0x30 /* Receive Holding Register */
#define SAM3U_TWI_THR_OFFSET         0x34 /* Transmit Holding Register */
                                          /* 0x38-0xfc: Reserved */
                                          /* 0x100-0x124: Reserved for the PDC */

/* TWI register adresses ****************************************************************/

#define SAM3U_TWI_CR(n)              (SAM3U_TWIN_BASE(n)+SAM3U_TWI_CR_OFFSET)
#define SAM3U_TWI_MMR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_MMR_OFFSET)
#define SAM3U_TWI_SMR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_SMR_OFFSET)
#define SAM3U_TWI_IADR(n)            (SAM3U_TWIN_BASE(n)+SAM3U_TWI_IADR_OFFSET)
#define SAM3U_TWI_CWGR(n)            (SAM3U_TWIN_BASE(n)+SAM3U_TWI_CWGR_OFFSET)
#define SAM3U_TWI_SR(n)              (SAM3U_TWIN_BASE(n)+SAM3U_TWI_SR_OFFSET)
#define SAM3U_TWI_IER(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_IER_OFFSET)
#define SAM3U_TWI_IDR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_IDR_OFFSET)
#define SAM3U_TWI_IMR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_IMR_OFFSET)
#define SAM3U_TWI_RHR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_RHR_OFFSET)
#define SAM3U_TWI_THR(n)             (SAM3U_TWIN_BASE(n)+SAM3U_TWI_THR_OFFSET)

#define SAM3U_TWI0_CR                (SAM3U_TWI0_BASE+SAM3U_TWI_CR_OFFSET)
#define SAM3U_TWI0_MMR               (SAM3U_TWI0_BASE+SAM3U_TWI_MMR_OFFSET)
#define SAM3U_TWI0_SMR               (SAM3U_TWI0_BASE+SAM3U_TWI_SMR_OFFSET)
#define SAM3U_TWI0_IADR              (SAM3U_TWI0_BASE+SAM3U_TWI_IADR_OFFSET)
#define SAM3U_TWI0_CWGR              (SAM3U_TWI0_BASE+SAM3U_TWI_CWGR_OFFSET)
#define SAM3U_TWI0_SR                (SAM3U_TWI0_BASE+SAM3U_TWI_SR_OFFSET)
#define SAM3U_TWI0_IER               (SAM3U_TWI0_BASE+SAM3U_TWI_IER_OFFSET)
#define SAM3U_TWI0_IDR               (SAM3U_TWI0_BASE+SAM3U_TWI_IDR_OFFSET)
#define SAM3U_TWI0_IMR               (SAM3U_TWI0_BASE+SAM3U_TWI_IMR_OFFSET)
#define SAM3U_TWI0_RHR               (SAM3U_TWI0_BASE+SAM3U_TWI_RHR_OFFSET)
#define SAM3U_TWI0_THR               (SAM3U_TWI0_BASE+SAM3U_TWI_THR_OFFSET)

#define SAM3U_TWI1_CR                (SAM3U_TWI1_BASE+SAM3U_TWI_CR_OFFSET)
#define SAM3U_TWI1_MMR               (SAM3U_TWI1_BASE+SAM3U_TWI_MMR_OFFSET)
#define SAM3U_TWI1_SMR               (SAM3U_TWI1_BASE+SAM3U_TWI_SMR_OFFSET)
#define SAM3U_TWI1_IADR              (SAM3U_TWI1_BASE+SAM3U_TWI_IADR_OFFSET)
#define SAM3U_TWI1_CWGR              (SAM3U_TWI1_BASE+SAM3U_TWI_CWGR_OFFSET)
#define SAM3U_TWI1_SR                (SAM3U_TWI1_BASE+SAM3U_TWI_SR_OFFSET)
#define SAM3U_TWI1_IER               (SAM3U_TWI1_BASE+SAM3U_TWI_IER_OFFSET)
#define SAM3U_TWI1_IDR               (SAM3U_TWI1_BASE+SAM3U_TWI_IDR_OFFSET)
#define SAM3U_TWI1_IMR               (SAM3U_TWI1_BASE+SAM3U_TWI_IMR_OFFSET)
#define SAM3U_TWI1_RHR               (SAM3U_TWI1_BASE+SAM3U_TWI_RHR_OFFSET)
#define SAM3U_TWI1_THR               (SAM3U_TWI1_BASE+SAM3U_TWI_THR_OFFSET)

/* TWI register bit definitions *********************************************************/

/* TWI Control Register */

#define TWI_CR_START                 (1 << 0)  /* Bit 0:  Send a START Condition */
#define TWI_CR_STOP                  (1 << 1)  /* Bit 1:  Send a STOP Condition */
#define TWI_CR_MSEN                  (1 << 2)  /* Bit 2:  TWI Master Mode Enabled */
#define TWI_CR_MSDIS                 (1 << 3)  /* Bit 3:  TWI Master Mode Disabled */
#define TWI_CR_SVEN                  (1 << 4)  /* Bit 4:  TWI Slave Mode Enabled */
#define TWI_CR_SVDIS                 (1 << 5)  /* Bit 5:  TWI Slave Mode Disabled */
#define TWI_CR_QUICK                 (1 << 6)  /* Bit 6:  SMBUS Quick Command */
#define TWI_CR_SWRST                 (1 << 7)  /* Bit 7:  Software Reset */

/* TWI Master Mode Register */'

#define TWI_MMR_IADRSZ_SHIFT         (8)      /* Bits 8-9:  Internal Device Address Size */
#define TWI_MMR_IADRSZ_MASK          (3 << TWI_MMR_IADRSZ_SHIFT)
#  define TWI_MMR_IADRSZ_NONE        (0 << TWI_MMR_IADRSZ_SHIFT) /* No internal device address */
#  define TWI_MMR_IADRSZ_1BYTE       (1 << TWI_MMR_IADRSZ_SHIFT) /* One-byte internal device address */
#  define TWI_MMR_IADRSZ_3BYTE       (2 << TWI_MMR_IADRSZ_SHIFT) /* Two-byte internal device address */
#  define TWI_MMR_IADRSZ_3BYTE       (3 << TWI_MMR_IADRSZ_SHIFT) /* Three-byte internal device address */
#define TWI_MMR_MREAD                (1 << 12) /* Bit 12: Master Read Direction */
#define TWI_MMR_DADR_SHIFT           (16)      /* Bits 16-23:  Device Address */
#define TWI_MMR_DADR_MASK            (0xff << TWI_MMR_DADR_SHIFT)

/* TWI Slave Mode Register */

#define TWI_SMR_SADR_SHIFT           (16)      /* Bits 16-23:  Slave Address */
#define TWI_SMR_SADR_MASK            (0xff << TWI_SMR_SADR_SHIFT)

/* TWI Internal Address Register */

#define TWI_IADR_SHIFT               (0)      /* Bits 0-23:  Internal Address */
#define TWI_IADR_MASK                (0x00ffffff << TWI_IADR_SHIFT)

/* TWI Clock Waveform Generator Register */

#define TWI_CWGR_CLDIV_SHIFT         (0)       /* Bits 0-7:  Clock Low Divider */
#define TWI_CWGR_CLDIV_MASK          (0xff << TWI_CWGR_CLDIV_SHIFT)
#define TWI_CWGR_CHDIV_SHIFT         (8)       /* Bits 8-15:  Clock High Divider */
#define TWI_CWGR_CHDIV_MASK          (0xff << TWI_CWGR_CLDIV_SHIFT)
#define TWI_CWGR_CKDIV_SHIFT         (16)      /* Bits 16-18:  Clock Divider */
#define TWI_CWGR_CKDIV_MASK          (7 << TWI_CWGR_CLDIV_SHIFT)

/* TWI Status Register, TWI Interrupt Enable Register, TWI Interrupt Disable
 * Register, and TWI Interrupt Mask Register common bit fields.
 */

#define TWI_INT_TXCOMP               (1 << 0)  /* Bit 0:  Transmission Completed */
#define TWI_INT_RXRDY                (1 << 1)  /* Bit 1:  Receive Holding Register */
#define TWI_INT_TXRDY                (1 << 2)  /* Bit 2:  Transmit Holding Register Ready */
#define TWI_SR_SVREAD                (1 << 3)  /* Bit 3:  Slave Read (SR only) */
#define TWI_INT_SVACC                (1 << 4)  /* Bit 4:  Slave Access */
#define TWI_INT_GACC                 (1 << 5)  /* Bit 5:  General Call Access */
#define TWI_INT_OVRE                 (1 << 6)  /* Bit 6:  Overrun Error */
#define TWI_INT_NACK                 (1 << 8)  /* Bit 8:  Not Acknowledged */
#define TWI_INT_ARBLST               (1 << 9)  /* Bit 9:  Arbitration Lost */
#define TWI_INT_SCLWS                (1 << 10) /* Bit 10: Clock Wait State */
#define TWI_INT_EOSACC               (1 << 11) /* Bit 11: End Of Slave Access */
#define TWI_INT_ENDRX                (1 << 12) /* Bit 12: End of RX buffer */
#define TWI_INT_ENDTX                (1 << 13) /* Bit 13: End of TX buffer */
#define TWI_INT_RXBUFF               (1 << 14) /* Bit 14: RX Buffer */
#define TWI_INT_TXBUFE               (1 << 15) /* Bit 15: TX Buffer Empty */

/* TWI Receive Holding Register */

#define TWI_RHR_RXDATA_SHIFT         (0)       /* Bits 0-7:  Master or Slave Receive Holding Data */
#define TWI_RHR_RXDATA_MASK          (0xff << TWI_RHR_RXDATA_SHIFT)

/* TWI Transmit Holding Register */

#define TWI_THR_TXDATA_SHIFT         (0)       /* Bits 0-7:  Master or Slave Transmit Holding Data */
#define TWI_THR_TXDATA_MASK          (0xff << TWI_THR_TXDATA_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_TWI_H */
