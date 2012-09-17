/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_ssc.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_SSC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_SSC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_SSC_CR_OFFSET       0x00 /* Control Register */
#define AVR32_SSC_CMR_OFFSET      0x04 /* Clock Mode Register */
#define AVR32_SSC_RCMR_OFFSET     0x10 /* Receive Clock Mode Register */
#define AVR32_SSC_RFMR_OFFSET     0x14 /* Receive Frame Mode Register */
#define AVR32_SSC_TCMR_OFFSET     0x18 /* Transmit Clock Mode Register */
#define AVR32_SSC_TFMR_OFFSET     0x1c /* Transmit Frame Mode Register */
#define AVR32_SSC_RHR_OFFSET      0x20 /* Receive Holding Register */
#define AVR32_SSC_THR_OFFSET      0x24 /* Transmit Holding Register */
#define AVR32_SSC_RSHR_OFFSET     0x30 /* Receive Synchronization Holding Register */
#define AVR32_SSC_TSHR_OFFSET     0x34 /* Transmit Synchronization Holding Register */
#define AVR32_SSC_RC0R_OFFSET     0x38 /* Receive Compare 0 Register */
#define AVR32_SSC_RC1R_OFFSET     0x3c /* Receive Compare 1 Register */
#define AVR32_SSC_SR_OFFSET       0x40 /* Status Register */
#define AVR32_SSC_IER_OFFSET      0x44 /* Interrupt Enable Register */
#define AVR32_SSC_IDR_OFFSET      0x48 /* Interrupt Disable Register */
#define AVR32_SSC_IMR_OFFSET      0x4c /* Interrupt Mask Register */

/* Register Addresses ***************************************************************/

#define AVR32_SSC_CR              (AVR32_SSC_BASE+AVR32_SSC_CR_OFFSET)
#define AVR32_SSC_CMR             (AVR32_SSC_BASE+AVR32_SSC_CMR_OFFSET)
#define AVR32_SSC_RCMR            (AVR32_SSC_BASE+AVR32_SSC_RCMR_OFFSET)
#define AVR32_SSC_RFMR            (AVR32_SSC_BASE+AVR32_SSC_RFMR_OFFSET)
#define AVR32_SSC_TCMR            (AVR32_SSC_BASE+AVR32_SSC_TCMR_OFFSET)
#define AVR32_SSC_TFMR            (AVR32_SSC_BASE+AVR32_SSC_TFMR_OFFSET)
#define AVR32_SSC_RHR             (AVR32_SSC_BASE+AVR32_SSC_RHR_OFFSET)
#define AVR32_SSC_THR             (AVR32_SSC_BASE+AVR32_SSC_THR_OFFSET)
#define AVR32_SSC_RSHR            (AVR32_SSC_BASE+AVR32_SSC_RSHR_OFFSET)
#define AVR32_SSC_TSHR            (AVR32_SSC_BASE+AVR32_SSC_TSHR_OFFSET)
#define AVR32_SSC_RC0R            (AVR32_SSC_BASE+AVR32_SSC_RC0R_OFFSET)
#define AVR32_SSC_RC1R            (AVR32_SSC_BASE+AVR32_SSC_RC1R_OFFSET)
#define AVR32_SSC_SR              (AVR32_SSC_BASE+AVR32_SSC_SR_OFFSET)
#define AVR32_SSC_IER             (AVR32_SSC_BASE+AVR32_SSC_IER_OFFSET)
#define AVR32_SSC_IDR             (AVR32_SSC_BASE+AVR32_SSC_IDR_OFFSET)
#define AVR32_SSC_IMR             (AVR32_SSC_BASE+AVR32_SSC_IMR_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Control Register Bit-field Definitions */

#define SSC_CR_RXEN               (1 << 0)  /* Bit 0:  Receive Enable */
#define SSC_CR_RXDIS              (1 << 1)  /* Bit 1:  Receive Disable */
#define SSC_CR_TXEN               (1 << 8)  /* Bit 8:  Transmit Enable */
#define SSC_CR_TXDIS              (1 << 9)  /* Bit 9:  Transmit Disable */
#define SSC_CR_SWRST              (1 << 15) /* Bit 15: Software Reset */

/* Clock Mode Register Bit-field Definitions */

#define SSC_CMR_DIV_SHIFT         (0)       /* Bits 0-11: Clock Divider */
#define SSC_CMR_DIV_MASK          (0xfff << SSC_CMR_DIV_SHIFT)

/* Receive Clock Mode Register Bit-field Definitions */

#define SSC_RCMR_CKS_SHIFT        (0)       /* Bits 0-1: Receive Clock Selection */
#define SSC_RCMR_CKS_MASK         (3 << SSC_RCMR_CKS_SHIFT)
#  define SSC_RCMR_CKS_DIVCLK     (0 << SSC_RCMR_CKS_SHIFT) /* Divided clock */
#  define SSC_RCMR_CKS_TXCLK      (1 << SSC_RCMR_CKS_SHIFT) /* TX_CLOCK clock signal */
#  define SSC_RCMR_CKS_RXCLK      (2 << SSC_RCMR_CKS_SHIFT) /* RX_CLOCK pin */
#define SSC_RCMR_CKO_SHIFT        (2)       /* Bits 2-4: Receive Clock Output Mode Selection */
#define SSC_RCMR_CKO_MASK         (7 << SSC_RCMR_CKO_SHIFT)
#  define SSC_RCMR_CKO_NONE       (0 << SSC_RCMR_CKO_SHIFT) /* None (Input-only) */
#  define SSC_RCMR_CKO_CONT       (1 << SSC_RCMR_CKO_SHIFT) /* Continuous receive clock */
#  define SSC_RCMR_CKO_XFR        (2 << SSC_RCMR_CKO_SHIFT) /* Receive clock only during data transfers */
#define SSC_RCMR_CKI              (1 << 5)  /* Bit 5:  Receive Clock Inversion */
#define SSC_RCMR_CKG_SHIFT        (6)       /* Bits 6-7: Receive Clock Gating Selection */
#define SSC_RCMR_CKG_MASK         (3 << SSC_RCMR_CKG_SHIFT)
#  define SSC_RCMR_CKG_NONE       (0 << SSC_RCMR_CKG_SHIFT) /* None, continous clock */
#  define SSC_RCMR_CKG_LOW        (1 << SSC_RCMR_CKG_SHIFT) /* Enable if RX_FRAME_SYNC low */
#  define SSC_RCMR_CKG_HIGH       (2 << SSC_RCMR_CKG_SHIFT) /* Enable if RX_FRAME_SYNC high */
#define SSC_RCMR_START_SHIFT      (8)       /* Bits 8-11: Receive Start Selection */
#define SSC_RCMR_START_MASK       (15 << SSC_RCMR_START_SHIFT)
#  define SSC_RCMR_START_CONT     (0 << SSC_RCMR_START_SHIFT) /* Continuous */
#  define SSC_RCMR_START_XMTSTART (1 << SSC_RCMR_START_SHIFT) /* Transmit start */
#  define SSC_RCMR_START_LOW      (2 << SSC_RCMR_START_SHIFT) /* RX_FRAME_SYNC low */
#  define SSC_RCMR_START_HIGH     (3 << SSC_RCMR_START_SHIFT) /* RX_FRAME_SYNC high */
#  define SSC_RCMR_START_FALLING  (4 << SSC_RCMR_START_SHIFT) /* Falling RX_FRAME_SYNC */
#  define SSC_RCMR_START_RISING   (5 << SSC_RCMR_START_SHIFT) /* Rising RX_FRAME_SYNC */
#  define SSC_RCMR_START_CHANGE   (6 << SSC_RCMR_START_SHIFT) /* RX_FRAME_SYNC change */
#  define SSC_RCMR_START_BOTH     (7 << SSC_RCMR_START_SHIFT) /* Any edge RX_FRAME_SYNC */
#  define SSC_RCMR_START_CMP0     (8 << SSC_RCMR_START_SHIFT) /* Compare 0 */
#define SSC_RCMR_STOP             (1 << 12) /* Bit 12: Receive Stop Selection */
#define SSC_RCMR_STTDLY_SHIFT     (16)      /* Bits 16-23: Receive Start Delay */
#define SSC_RCMR_STTDLY_MASK      (0xff << SSC_RCMR_STTDLY_SHIFT)
#define SSC_RCMR_PERIOD_SHIFT     (24)      /* Bits 24-31: Receive Period Divider Selection */
#define SSC_RCMR_PERIOD_MASK      (0xff << SSC_RCMR_PERIOD_SHIFT)

/* Receive Frame Mode Register Bit-field Definitions */

#define SSC_RFMR_DATLEN_SHIFT     (0)       /* Bits 0-4: Data Length */
#define SSC_RFMR_DATLEN_MASK      (0x1f << SSC_RFMR_DATLEN_SHIFT)
#define SSC_RFMR_LOOP             (1 << 5)  /* Bit 5:  Loop Mode */
#define SSC_RFMR_MSBF             (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_RFMR_DATNB_SHIFT      (8)       /* Bits 8-11: Data Number per Frame */
#define SSC_RFMR_DATNB_MASK       (15 << SSC_RFMR_DATNB_SHIFT)
#define SSC_RFMR_FSLEN_SHIFT      (16)      /* Bits 16-19: Receive Frame Sync Length */
#define SSC_RFMR_FSLEN_MASK       (15 << SSC_RFMR_FSLEN_SHIFT)
#define SSC_RFMR_FSOS_SHIFT       (20)      /* Bits 20-22: Receive Frame Sync Output Selection */
#define SSC_RFMR_FSOS_MASK        (7 << SSC_RFMR_FSOS_SHIFT)
#  define SSC_RFMR_FSOS_NONE      (0 << SSC_RFMR_FSOS_SHIFT) /* None (Input-only )*/
#  define SSC_RFMR_FSOS_NEGP      (1 << SSC_RFMR_FSOS_SHIFT) /* Negative Pulse */
#  define SSC_RFMR_FSOS_POSP      (2 << SSC_RFMR_FSOS_SHIFT) /* Positive Pulse */
#  define SSC_RFMR_FSOS_LOW       (3 << SSC_RFMR_FSOS_SHIFT) /* Driven Low during data transfer */
#  define SSC_RFMR_FSOS_HIGH      (4 << SSC_RFMR_FSOS_SHIFT) /* Driven High during data transfer */
#  define SSC_RFMR_FSOS_TOGGLE    (5 << SSC_RFMR_FSOS_SHIFT) /* Toggling at each start of data transfer */
#define SSC_RFMR_FSEDGE           (1 << 24) /* Bit 24: Receive Frame Sync Edge Detection */
#define SSC_RFMR_FSLENHI_SHIFT    (28)      /* Bits 28-31: Receive Frame Sync Length High Part */
#define SSC_RFMR_FSLENHI_MASK     (15 << SSC_RFMR_FSLENHI_SHIFT)

/* Transmit Clock Mode Register Bit-field Definitions */

#define SSC_TCMR_CKS_SHIFT        (0)       /* Bits 0-1: Transmit Clock Selection */
#define SSC_TCMR_CKS_MASK         (3 << SSC_TCMR_CKS_SHIFT)
#  define SSC_TCMR_CKS_DIVCLK     (0 << SSC_TCMR_CKS_SHIFT) /* Divided clock */
#  define SSC_TCMR_CKS_TXCLK      (1 << SSC_TCMR_CKS_SHIFT) /* RX_CLOCK clock signal */
#  define SSC_TCMR_CKS_RXCLK      (2 << SSC_TCMR_CKS_SHIFT) /* TX_CLOCK pin */
#define SSC_TCMR_CKO_SHIFT        (2)       /* Bits 2-4: Transmit Clock Output Mode Selection */
#define SSC_TCMR_CKO_MASK         (7 << SSC_TCMR_CKO_SHIFT)
#  define SSC_TCMR_CKO_NONE       (0 << SSC_TCMR_CKO_SHIFT) /* None (Input-only) */
#  define SSC_TCMR_CKO_CONT       (1 << SSC_TCMR_CKO_SHIFT) /* Continuous transmit clock Output */
#  define SSC_TCMR_CKO_XFR        (2 << SSC_TCMR_CKO_SHIFT) /* Transmit clock only during data transfers Output */
#define SSC_TCMR_CKI              (1 << 5)  /* Bit 5:  Transmit Clock Inversion */
#define SSC_TCMR_CKG_SHIFT        (6)      /* Bits 6-7: Transmit Clock Gating Selection */
#define SSC_TCMR_CKG_MASK         (3 << SSC_TCMR_CKG_SHIFT)
#  define SSC_TCMR_CKG_NONE       (0 << SSC_TCMR_CKG_SHIFT) /* None, continous clock */
#  define SSC_TCMR_CKG_LOW        (1 << SSC_TCMR_CKG_SHIFT) /* Enable if TX_FRAME_SYNC low */
#  define SSC_TCMR_CKG_HIGH       (2 << SSC_TCMR_CKG_SHIFT) /* Enable if TX_FRAME_SYNC high */
#define SSC_TCMR_START_SHIFT      (8)      /* Bits 8-11: Transmit Start Selection */
#define SSC_TCMR_START_MASK       (15 << SSC_TCMR_START_SHIFT)
#  define SSC_TCMR_START_CONT     (0 << SSC_TCMR_START_SHIFT) /* Continuous */
#  define SSC_TCMR_START_XMTSTART (1 << SSC_TCMR_START_SHIFT) /* Receive start */
#  define SSC_TCMR_START_LOW      (2 << SSC_TCMR_START_SHIFT) /* TX_FRAME_SYNC low */
#  define SSC_TCMR_START_HIGH     (3 << SSC_TCMR_START_SHIFT) /* TX_FRAME_SYNC high */
#  define SSC_TCMR_START_FALLING  (4 << SSC_TCMR_START_SHIFT) /* Falling TX_FRAME_SYNC */
#  define SSC_TCMR_START_RISING   (5 << SSC_TCMR_START_SHIFT) /* Rising TX_FRAME_SYNC */
#  define SSC_TCMR_START_CHANGE   (6 << SSC_TCMR_START_SHIFT) /* TX_FRAME_SYNC change */
#  define SSC_TCMR_START_BOTH     (7 << SSC_TCMR_START_SHIFT) /* Any edge TX_FRAME_SYNC */
#define SSC_TCMR_STTDLY_SHIFT     (16)      /* Bits 16-23: Transmit Start Delay */
#define SSC_TCMR_STTDLY_MASK      (0xff << SSC_TCMR_STTDLY_SHIFT)
#define SSC_TCMR_PERIOD_SHIFT     (24)      /* Bits 24-31: Transmit Period Divider Selection */
#define SSC_TCMR_PERIOD_MASK      (0xff << SSC_TCMR_PERIOD_SHIFT)

/* Transmit Frame Mode Register Bit-field Definitions */

#define SSC_TFMR_DATLEN_SHIFT     (0)       /* Bits 0-4: Data Length */
#define SSC_TFMR_DATLEN_MASK      (0x1f << SSC_TFMR_DATLEN_SHIFT)
#define SSC_TFMR_DATDEF           (1 << 5)  /* Bit 5:  Data Default Value */
#define SSC_TFMR_MSBF             (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_TFMR_DATNB_SHIFT      (8)       /* Bits 8-11: Data Number per Frame */
#define SSC_TFMR_DATNB_MASK       (15 << SSC_TFMR_DATNB_SHIFT)
#define SSC_TFMR_FSLEN_SHIFT      (16)      /* Bits 16-19: Transmit Frame Sync Length */
#define SSC_TFMR_FSLEN_MASK       (15 << SSC_TFMR_FSLEN_SHIFT)
#define SSC_TFMR_FSOS_SHIFT       (20)      /* Bits 20-22: Transmit Frame Sync Data Enable */
#define SSC_TFMR_FSOS_MASK        (7 << SSC_TFMR_FSOS_SHIFT)
#  define SSC_TFMR_FSOS_NONE      (0 << SSC_TFMR_FSOS_SHIFT) /* None (Input-only )*/
#  define SSC_TFMR_FSOS_NEGP      (1 << SSC_TFMR_FSOS_SHIFT) /* Negative Pulse */
#  define SSC_TFMR_FSOS_POSP      (2 << SSC_TFMR_FSOS_SHIFT) /* Positive Pulse */
#  define SSC_TFMR_FSOS_LOW       (3 << SSC_TFMR_FSOS_SHIFT) /* Driven Low during data transfer */
#  define SSC_TFMR_FSOS_HIGH      (4 << SSC_TFMR_FSOS_SHIFT) /* Driven High during data transfer */
#  define SSC_TFMR_FSOS_TOGGLE    (5 << SSC_TFMR_FSOS_SHIFT) /* Toggling at each start of data transfer */
#define SSC_TFMR_FSDEN            (1 << 23) /* Bit 23: Transmit Frame Sync Data Enable */
#define SSC_TFMR_FSEDGE           (1 << 24) /* Bit 24: Transmit Frame Sync Edge Detection */
#define SSC_TFMR_FSLENHI_SHIFT    (28)      /* Bits 28-31: Transmit Frame Sync Length High Part */
#define SSC_TFMR_FSLENHI_MASK     (15 << SSC_TFMR_FSLENHI_SHIFT)

/* Receive Holding Register Bit-field Definitions */
/* Transmit Holding Register Bit-field Definitions */

/* These register hold 32-bit values with no bit-fields */

/* Receive Synchronization Holding Register Bit-field Definitions */

#define SSC_RSHR_MASK             (0xffff)

/* Transmit Synchronization Holding Register Bit-field Definitions */

#define SSC_TSHR_MASK             (0xffff)

/* Receive Compare 0 Register Bit-field Definitions */

#define SSC_RC0R_MASK             (0xffff)

/* Receive Compare 1 Register Bit-field Definitions */

#define SSC_RC1R_MASK             (0xffff)

/* Interrupt Enable Register Bit-field Definitions */
/* Interrupt Disable Register Bit-field Definitions */
/* Interrupt Mask Register Bit-field Definitions */

#define SSC_INT_TXRDY             (1 << 0)  /* Bit 0:  Transmit Ready */
#define SSC_INT_TXEMPTY           (1 << 1)  /* Bit 1:  Transmit Empty */
#define SSC_INT_RXRDY             (1 << 4)  /* Bit 4:  Receive Ready */
#define SSC_INT_OVRUN             (1 << 5)  /* Bit 5:  Receive Overrun */
#define SSC_INT_CP0               (1 << 8)  /* Bit 8:  Compare 0 */
#define SSC_INT_CP1               (1 << 9)  /* Bit 9:  Compare 1 */
#define SSC_INT_TXSYN             (1 << 10) /* Bit 10: Transmit Sync */
#define SSC_INT_RXSYN             (1 << 11) /* Bit 11: Receive Sync */

/* Status Register Bit-field Definitions (Only) */

#define SSC_SR_TXEN               (1 << 16) /* Bit 16: Transmit Enable */
#define SSC_SR_RXEN               (1 << 17) /* Bit 17: Receive Enable */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_SSC_H */

