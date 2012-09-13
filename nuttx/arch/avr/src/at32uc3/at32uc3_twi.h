/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_twi.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_TWI_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_TWI_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_TWI_CR_OFFSET       0x00 /* Control Register */
#define AVR32_TWI_MMR_OFFSET      0x04 /* Master Mode Register */
#define AVR32_TWI_SMR_OFFSET      0x08 /* Slave Mode Register */
#define AVR32_TWI_IADR_OFFSET     0x0c /* Internal Address Register */
#define AVR32_TWI_CWGR_OFFSET     0x10 /* Clock Waveform Generator Register */
#define AVR32_TWI_SR_OFFSET       0x20 /* Status Register */
#define AVR32_TWI_IER_OFFSET      0x24 /* Interrupt Enable Register */
#define AVR32_TWI_IDR_OFFSET      0x28 /* Interrupt Disable Register */
#define AVR32_TWI_IMR_OFFSET      0x2c /* Interrupt Mask Register */
#define AVR32_TWI_RHR_OFFSET      0x30 /* Receive Holding Register */
#define AVR32_TWI_THR_OFFSET      0x34 /* Transmit Holding Register */

/* Register Addresses ***************************************************************/

#define AVR32_TWI_CR              (AVR32_TWI_BASE+AVR32_TWI_CR_OFFSET)
#define AVR32_TWI_MMR             (AVR32_TWI_BASE+AVR32_TWI_MMR_OFFSET)
#define AVR32_TWI_SMR             (AVR32_TWI_BASE+AVR32_TWI_SMR_OFFSET)
#define AVR32_TWI_IADR            (AVR32_TWI_BASE+AVR32_TWI_IADR_OFFSET)
#define AVR32_TWI_CWGR            (AVR32_TWI_BASE+AVR32_TWI_CWGR_OFFSET)
#define AVR32_TWI_SR              (AVR32_TWI_BASE+AVR32_TWI_SR_OFFSET)
#define AVR32_TWI_IER             (AVR32_TWI_BASE+AVR32_TWI_IER_OFFSET)
#define AVR32_TWI_IDR             (AVR32_TWI_BASE+AVR32_TWI_IDR_OFFSET)
#define AVR32_TWI_IMR             (AVR32_TWI_BASE+AVR32_TWI_IMR_OFFSET)
#define AVR32_TWI_RHR             (AVR32_TWI_BASE+AVR32_TWI_RHR_OFFSET)
#define AVR32_TWI_THR             (AVR32_TWI_BASE+AVR32_TWI_THR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register Bit-field Definitions */

#define TWI_CR_START              (1 << 0)  /* Bit 0:  Send a START Condition */
#define TWI_CR_STOP               (1 << 1)  /* Bit 1:  Send a STOP Condition */
#define TWI_CR_MSEN               (1 << 2)  /* Bit 2:  TWI Master Mode Enabled */
#define TWI_CR_MSDIS              (1 << 3)  /* Bit 3:  TWI Master Mode Disabled */
#define TWI_CR_SVEN               (1 << 4)  /* Bit 4:  TWI Slave Mode Enabled */
#define TWI_CR_SVDIS              (1 << 5)  /* Bit 5:  TWI Slave Mode Disabled */
#define TWI_CR_SWRST              (1 << 7)  /* Bit 6:  Software Reset */

/* Master Mode Register Bit-field Definitions */

#define TWI_MMR_IADRSZ_SHIFT      (8)       /* Bits 8-9: Internal Device Address Size */
#define TWI_MMR_IADRSZ_MASK       (3 << TWI_MMR_IADRSZ_SHIFT)
#  define TWI_MMR_IADRSZ_ NONE    (0 << TWI_MMR_IADRSZ_SHIFT) /* No internal device address */
#  define TWI_MMR_IADRSZ_1BYTE    (1 << TWI_MMR_IADRSZ_SHIFT) /* One-byte internal device address */
#  define TWI_MMR_IADRSZ_2BYTES   (2 << TWI_MMR_IADRSZ_SHIFT) /* Two-byte internal device address */
#  define TWI_MMR_IADRSZ_3BYTES   (3 << TWI_MMR_IADRSZ_SHIFT) /* Three-byte internal device address */
#define TWI_MMR_MREAD             (1 << 12) /* Bit 12: Master Read Direction */
#define TWI_MMR_DADR_SHIFT        (16)      /* Bits 16-22: Device Address */
#define TWI_MMR_DADR:_MASK        (0x7f << TWI_MMR_DADR_SHIFT)

/* Slave Mode Register Bit-field Definitions */

#define TWI_SMR_SADR_SHIFT       (16)       /* Bits 16-22: Slave Address */
#define TWI_SMR_SADR_MASK        (0x7f << TWI_SMR_SADR_SHIFT)

/* Internal Address Register Bit-field Definitions */

#define TWI_IADR_MASK            (0x00ffffff)

/* Clock Waveform Generator Register Bit-field Definitions */

#define TWI_CWGR_CLDIV_SHIFT     (0)        /* Bits 0-7: Clock Low Divider */
#define TWI_CWGR_CLDIV_MASK      (0xff <<TWI_CWGR_CLDIV_SHIFT)
#define TWI_CWGR_CHDIV_SHIFT     (8)        /* Bits 8-15: Clock High Divider */
#define TWI_CWGR_CHDIV_MASK      (0xff << TWI_CWGR_CHDIV_SHIFT)
#define TWI_CWGR_CKDIV_SHIFT     (16)       /* Bits 16-18: Clock Divider */
#define TWI_CWGR_CKDIV:_MASK     (7 << TWI_CWGR_CKDIV_SHIFT)

/* Status Register Bit-field Definitions */
/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */

#define TWI_INT_TXCOMP            (1 << 0)  /* Bit 0:  Transmission Completed (automatically set / reset) */
#define TWI_INT_RXRDY             (1 << 1)  /* Bit 1:  Receive Holding Register Ready (automatically set / reset) */
#define TWI_INT_TXRDY             (1 << 2)  /* Bit 2:  Transmit Holding Register Ready (automatically set / reset) */
#define TWI_SR_SVREAD             (1 << 3)  /* Bit 3:  Slave Read (automatically set / reset) */
#define TWI_INT_SVACC             (1 << 4)  /* Bit 4:  Slave Access (automatically set / reset) */
#define TWI_INT_GACC              (1 << 5)  /* Bit 5:  General Call Access (clear on read) */
#define TWI_INT_OVRE              (1 << 6)  /* Bit 6:  Overrun Error (clear on read) */
#define TWI_INT_NACK              (1 << 8)  /* Bit 8:  Not Acknowledged (clear on read) */
#define TWI_INT_ARBLST            (1 << 9)  /* Bit 9:  Arbitration Lost (clear on read) */
#define TWI_INT_SCLWS             (1 << 10) /* Bit 10: Clock Wait State (automatically set / reset) */
#define TWI_INT_EOSACC            (1 << 11) /* Bit 11: End Of Slave Access (clear on read) */
#define TWI_INT_ENDRX             (1 << 12) /* Bit 12: End of RX buffer */
#define TWI_INT_ENDTX             (1 << 13) /* Bit 13: End of TX buffer */
#define TWI_INT_RXBUFF            (1 << 14) /* Bit 14: RX Buffer Full */
#define TWI_INT_TXBUFE            (1 << 15) /* Bit 15: TX Buffer Empty */

/* Receive Holding Register Bit-field Definitions */

#define TWI_RHR_MASK              (0xff)

/* Transmit Holding Register Bit-field Definitions */

#define TWI_THR_MASK              (0xff)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_TWI_H */

