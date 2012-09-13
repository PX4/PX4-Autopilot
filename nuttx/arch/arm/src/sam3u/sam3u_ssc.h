/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_ssc.h
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_SSC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_SSC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* SSC register offsets *****************************************************************/

#define SAM3U_SSC_CR_OFFSET          0x000 /* Control Register */
#define SAM3U_SSC_CMR_OFFSET         0x004 /* Clock Mode Register */
                                           /* 0x008: Reserved */
                                           /* 0x00c: Reserved */
#define SAM3U_SSC_RCMR_OFFSET        0x010 /* Receive Clock Mode Register */
#define SAM3U_SSC_RFMR_OFFSET        0x014 /* Receive Frame Mode Register */
#define SAM3U_SSC_TCMR_OFFSET        0x018 /* Transmit Clock Mode Register */
#define SAM3U_SSC_TFMR_OFFSET        0x01c /* Transmit Frame Mode Register */
#define SAM3U_SSC_RHR_OFFSET         0x020 /* Receive Holding Register */
#define SAM3U_SSC_THR_OFFSET         0x024 /* Transmit Holding Register */
                                           /* 0x028: Reserved */
                                           /* 0x02c: Reserved */
#define SAM3U_SSC_RSHR_OFFSET        0x030 /* Receive Sync. Holding Register */
#define SAM3U_SSC_TSHR_OFFSET        0x034 /* Transmit Sync. Holding Register */
#define SAM3U_SSC_RC0R_OFFSET        0x038 /* Receive Compare 0 Register */
#define SAM3U_SSC_RC1R_OFFSET        0x03c /* Receive Compare 1 Register */
#define SAM3U_SSC_SR_OFFSET          0x040 /* Status Register */
#define SAM3U_SSC_IER_OFFSET         0x044 /* Interrupt Enable Register */
#define SAM3U_SSC_IDR_OFFSET         0x048 /* Interrupt Disable Register */
#define SAM3U_SSC_IMR_OFFSET         0x04c /* Interrupt Mask Register */
#define SAM3U_SSC_WPMR_OFFSET        0x0e4 /* Write Protect Mode Register */
#define SAM3U_SSC_WPSR_OFFSET        0x0e8 /* Write Protect Status Register */
                                           /* 0x050-0x0fc: Reserved */
                                           /* 0x100-0x124: Reserved */

/* SSC register adresses ****************************************************************/

#define SAM3U_SSC_CR                 (SAM3U_SSC_BASE+SAM3U_SSC_CR_OFFSET)
#define SAM3U_SSC_CMR                (SAM3U_SSC_BASE+SAM3U_SSC_CMR_OFFSET)
#define SAM3U_SSC_RCMR               (SAM3U_SSC_BASE+SAM3U_SSC_RCMR_OFFSET)
#define SAM3U_SSC_RFMR               (SAM3U_SSC_BASE+SAM3U_SSC_RFMR_OFFSET)
#define SAM3U_SSC_TCMR               (SAM3U_SSC_BASE+SAM3U_SSC_TCMR_OFFSET)
#define SAM3U_SSC_TFMR               (SAM3U_SSC_BASE+SAM3U_SSC_TFMR_OFFSET)
#define SAM3U_SSC_RHR                (SAM3U_SSC_BASE+SAM3U_SSC_RHR_OFFSET)
#define SAM3U_SSC_THR                (SAM3U_SSC_BASE+SAM3U_SSC_THR_OFFSET)
#define SAM3U_SSC_RSHR               (SAM3U_SSC_BASE+SAM3U_SSC_RSHR_OFFSET)
#define SAM3U_SSC_TSHR               (SAM3U_SSC_BASE+SAM3U_SSC_TSHR_OFFSET)
#define SAM3U_SSC_RC0R               (SAM3U_SSC_BASE+SAM3U_SSC_RC0R_OFFSET)
#define SAM3U_SSC_RC1R               (SAM3U_SSC_BASE+SAM3U_SSC_RC1R_OFFSET)
#define SAM3U_SSC_SR                 (SAM3U_SSC_BASE+SAM3U_SSC_SR_OFFSET)
#define SAM3U_SSC_IER                (SAM3U_SSC_BASE+SAM3U_SSC_IER_OFFSET)
#define SAM3U_SSC_IDR                (SAM3U_SSC_BASE+SAM3U_SSC_IDR_OFFSET)
#define SAM3U_SSC_IMR                (SAM3U_SSC_BASE+SAM3U_SSC_IMR_OFFSET)
#define SAM3U_SSC_WPMR               (SAM3U_SSC_BASE+SAM3U_SSC_WPMR_OFFSET)
#define SAM3U_SSC_WPSR               (SAM3U_SSC_BASE+SAM3U_SSC_WPSR_OFFSET)

/* SSC register bit definitions *********************************************************/

/* SSC Control Register */

#define SSC_CR_RXEN                  (1 << 0)  /* Bit 0:  Receive Enable */
#define SSC_CR_RXDIS                 (1 << 1)  /* Bit 1:  Receive Disable */
#define SSC_CR_TXEN                  (1 << 8)  /* Bit 8:  Transmit Enable */
#define SSC_CR_TXDIS                 (1 << 9)  /* Bit 9:  Transmit Disable */
#define SSC_CR_SWRST                 (1 << 15) /* Bit 15: Software Reset */

/* SSC Clock Mode Register */

#define SSC_CMR_DIV_SHIFT            (0)       /* Bits 0-11:  Clock Divider */
#define SSC_CMR_DIV_MASK             (0xfff << SSC_CMR_DIV_SHIFT)

/* SSC Receive Clock Mode Register */

#define SSC_RCMR_CKS_SHIFT           (0)       /* Bits 0-1:  Receive Clock Selection */
#define SSC_RCMR_CKS_MASK            (3 << SSC_RCMR_CKS_SHIFT)
#  define SSC_RCMR_CKS_DIVIDED       (0 << SSC_RCMR_CKS_SHIFT) /* Divided Clock */
#  define SSC_RCMR_CKS_TK            (1 << SSC_RCMR_CKS_SHIFT) /* TK Clock signal */
#  define SSC_RCMR_CKS_RK            (2 << SSC_RCMR_CKS_SHIFT) /* RK pin */
#define SSC_RCMR_CKO_SHIFT           (2)       /* Bits 2-4:  Receive Clock Output Mode Selection */
#define SSC_RCMR_CKO_MASK            (7 << SSC_RCMR_CKO_SHIFT)
#  define SSC_RCMR_CKO_ NONE         (0 << SSC_RCMR_CKO_SHIFT) /* None */
#  define SSC_RCMR_CKO_CONTINUOUS    (1 << SSC_RCMR_CKO_SHIFT) /* Continuous Receive Clock */
#  define SSC_RCMR_CKO_XFERS         (2 << SSC_RCMR_CKO_SHIFT) /* Receive Clock only during data transfers */
#define SSC_RCMR_CKI                 (1 << 5)  /* Bit 5:  Receive Clock Inversion */
#define SSC_RCMR_CKG_SHIFT           (6)       /* Bits 6-7:  Receive Clock Gating Selection */
#define SSC_RCMR_CKG_MASK            (3 << SSC_RCMR_CKG_SHIFT)
#  define SSC_RCMR_CKG_NONE          (0 << SSC_RCMR_CKG_SHIFT) /* None, continuous clock */
#  define SSC_RCMR_CKG_RFLOW         (1 << SSC_RCMR_CKG_SHIFT) /* Receive Clock enabled only if RF Low */
#  define SSC_RCMR_CKG_RFHIGH        (2 << SSC_RCMR_CKG_SHIFT) /* Receive Clock enabled only if RF High */
#define SSC_RCMR_START_SHIFT         (8)      /* Bits 8-11:  Receive Start Selection */
#define SSC_RCMR_START_MASK          (15 << SSC_RCMR_START_SHIFT)
#  define SSC_RCMR_START_CONTINOUS   (0 << SSC_RCMR_START_SHIFT) /* Continuous */
#  define SSC_RCMR_START_START       (1 << SSC_RCMR_START_SHIFT) /* Transmit start */
#  define SSC_RCMR_START_RFLOW       (2 << SSC_RCMR_START_SHIFT) /* Low level on RF signal */
#  define SSC_RCMR_START_RFHIGH      (3 << SSC_RCMR_START_SHIFT) /* High level on RF signal */
#  define SSC_RCMR_START_RFFALL      (4 << SSC_RCMR_START_SHIFT) /* Falling edge on RF signal */
#  define SSC_RCMR_START_RFRISE      (5 << SSC_RCMR_START_SHIFT) /* Rising edge on RF signal */
#  define SSC_RCMR_START_ANYLEVEL    (6 << SSC_RCMR_START_SHIFT) /* Any level change on RF signal */
#  define SSC_RCMR_START_ANYEDGE     (7 << SSC_RCMR_START_SHIFT) /* Any edge on RF signal */
#  define SSC_RCMR_START_CMP0        (8 << SSC_RCMR_START_SHIFT) /* Compare 0 */
#define SSC_RCMR_STOP                (1 << 12) /* Bit 12: Receive Stop Select */
#define SSC_RCMR_STTDLY_SHIFT        (15)      /* Bits 16-23:  Receive Start Delay */
#define SSC_RCMR_STTDLY_MASK         (0xff << SSC_RCMR_STTDLY_SHIFT)
#define SSC_RCMR_PERIOD_SHIFT        (24)      /* Bits 24-31:  Receive Period Divider Selection */
#define SSC_RCMR_PERIOD_MASK         (0xff << SSC_RCMR_PERIOD_SHIFT)


/* SSC Receive Frame Mode Register */

#define SSC_RFMR_DATLEN_SHIFT        (0)       /* Bits 0-4:  Data Length */
#define SSC_RFMR_DATLEN_MASK         (31 << SSC_RFMR_DATLEN_SHIFT)
#define SSC_RFMR_LOOP                (1 << 5)  /* Bit 5:  Loop Mode */
#define SSC_RFMR_MSBF                (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_RFMR_DATNB_SHIFT         (8)       /* Bits 8-11:  Data Number per Frame */
#define SSC_RFMR_DATNB_MASK          (15 << SSC_RFMR_DATNB_SHIFT)
#define SSC_RFMR_FSLEN_SHIFT         (16)      /* Bits 16-19:  Receive Frame Sync Length */
#define SSC_RFMR_FSLEN_MASK          (15 << SSC_RFMR_FSLEN_SHIFT)
#define SSC_RFMR_FSOS_SHIFT          (20)      /* Bits 20-22:  Receive Frame Sync Output Selection */
#define SSC_RFMR_FSOS_MASK           (7 << SSC_RFMR_FSOS_SHIFT)
#  define SSC_RFMR_FSOS_NONE         (0 << SSC_RFMR_FSOS_SHIFT) /* None */
#  define SSC_RFMR_FSOS_NEG          (1 << SSC_RFMR_FSOS_SHIFT) /* 0x1 Negative Pulse */
#  define SSC_RFMR_FSOS_POW          (2 << SSC_RFMR_FSOS_SHIFT) /* 0x2 Positive Pulse */
#  define SSC_RFMR_FSOS_LOW          (3 << SSC_RFMR_FSOS_SHIFT) /* 0x3 Driven Low during data transfer */
#  define SSC_RFMR_FSOS_HIGH         (4 << SSC_RFMR_FSOS_SHIFT) /* 0x4 Driven High during data transfer */
#  define SSC_RFMR_FSOS_TOGGLE       (5 << SSC_RFMR_FSOS_SHIFT) /* 0x5 Toggling at each start of data transfer */
#define SSC_RFMR_FSEDGE              (1 << 24) /* Bit 24: Frame Sync Edge Detect */
#define SSC_RFMR_FSLENEXT_SHIFT      (28)      /* Bits 28-31:  FSLEN Field Extension */
#define SSC_RFMR_FSLENEXT_MASK       (15 << SSC_RFMR_FSLENEXT_SHIFT)

/* SSC Transmit Clock Mode Register */

#define SSC_TCMR_CKS_SHIFT           (0)       /* Bits 0-1:  Transmit Clock Selection */
#define SSC_TCMR_CKS_MASK            (3 << SSC_TCMR_CKS_SHIFT)
#  define SSC_TCMR_CKS_DIVIDED       (0 << SSC_TCMR_CKS_SHIFT) /* Divided Clock */
#  define SSC_TCMR_CKS_TK            (1 << SSC_TCMR_CKS_SHIFT) /* TK Clock signal */
#  define SSC_TCMR_CKS_RK            (2 << SSC_TCMR_CKS_SHIFT) /* RK pin */
#define SSC_TCMR_CKO_SHIFT           (2)       /* Bits 2-4:  Transmit Clock Output Mode Selection */
#define SSC_TCMR_CKO_MASK            (7 << SSC_TCMR_CKO_SHIFT)
#  define SSC_TCMR_CKO_ NONE         (0 << SSC_TCMR_CKO_SHIFT) /* None */
#  define SSC_TCMR_CKO_CONTINUOUS    (1 << SSC_TCMR_CKO_SHIFT) /* Continuous Transmit Clock */
#  define SSC_TCMR_CKO_XFERS         (2 << SSC_TCMR_CKO_SHIFT) /* Transmit Clock only during data transfers */
#define SSC_TCMR_CKI                 (1 << 5)  /* Bit 5:  Transmit Clock Inversion */
#define SSC_TCMR_CKG_SHIFT           (6)       /* Bits 6-7:  Transmit Clock Gating Selection */
#define SSC_TCMR_CKG_MASK            (3 << SSC_TCMR_CKG_SHIFT)
#  define SSC_TCMR_CKG_NONE          (0 << SSC_TCMR_CKG_SHIFT) /* None, continuous clock */
#  define SSC_tCMR_CKG_TFLOW         (1 << SSC_TCMR_CKG_SHIFT) /* Receive Clock enabled only if TF Low */
#  define SSC_TCMR_CKG_TFHIGH        (2 << SSC_TCMR_CKG_SHIFT) /* Receive Clock enabled only if TF High */
#define SSC_TCMR_START_SHIFT         (8)      /* Bits 8-11:  Transmit Start Selection */
#define SSC_TCMR_START_MASK          (15 << SSC_TCMR_START_SHIFT)
#  define SSC_TCMR_START_CONTINOUS   (0 << SSC_TCMR_START_SHIFT) /* Continuous */
#  define SSC_TCMR_START_START       (1 << SSC_TCMR_START_SHIFT) /* Receive start */
#  define SSC_TCMR_START_TFLOW       (2 << SSC_TCMR_START_SHIFT) /* Low level on TF signal */
#  define SSC_TCMR_START_TFHIGH      (3 << SSC_TCMR_START_SHIFT) /* High level on TF signal */
#  define SSC_TCMR_START_TFFALL      (4 << SSC_TCMR_START_SHIFT) /* Falling edge on TF signal */
#  define SSC_TCMR_START_TFRISE      (5 << SSC_TCMR_START_SHIFT) /* Rising edge on TF signal */
#  define SSC_TCMR_START_ANYLEVEL    (6 << SSC_TCMR_START_SHIFT) /* Any level change on TF signal */
#  define SSC_TCMR_START_ANYEDGE     (7 << SSC_TCMR_START_SHIFT) /* Any edge on TF signal */
#define SSC_TCMR_STTDLY_SHIFT        (16)      /* Bits 16-23:  Transmit Start Delay */
#define SSC_TCMR_STTDLY_MASK         (0xff << SSC_TCMR_STTDLY_SHIFT)
#define SSC_TCMR_PERIOD_SHIFT        (24)      /* Bits 24-31:  Transmit Period Divider Selection */
#define SSC_TCMR_PERIOD_MASK         (0xff << SSC_TCMR_PERIOD_SHIFT)

/* SSC Transmit Frame Mode Register */

#define SSC_TFMR_DATLEN_SHIFT        (0)       /* Bits 0-4:  Data Length */
#define SSC_TFMR_DATLEN_MASK         (31 << SSC_TFMR_DATLEN_SHIFT)
#define SSC_TFMR_DATDEF              (1 << 5)  /* Bit 5:  Data Default Value */
#define SSC_TFMR_MSBF                (1 << 7)  /* Bit 7:  Most Significant Bit First */
#define SSC_TFMR_DATNB_SHIFT         (8)       /* Bits 8-11:  Data Number per frame */
#define SSC_TFMR_DATNB_MASK          (15 << SSC_TFMR_DATNB_SHIFT)
#define SSC_TFMR_FSLEN_SHIFT         (16)      /* Bits 16-19:  Transmit Frame Syn Length */
#define SSC_TFMR_FSLEN_MASK          (15 << SSC_TFMR_FSLEN_SHIFT)
#define SSC_TFMR_FSOS_SHIFT          (20)      /* Bits 20-22:  Transmit Frame Sync Output Selection */
#define SSC_TFMR_FSOS_MASK           (7 << SSC_TFMR_FSOS_SHIFT)
#  define SSC_TFMR_FSOS_NONE         (0 << SSC_TFMR_FSOS_SHIFT) /* None */
#  define SSC_TFMR_FSOS_NEG          (1 << SSC_TFMR_FSOS_SHIFT) /* 0x1 Negative Pulse */
#  define SSC_TFMR_FSOS_POW          (2 << SSC_TFMR_FSOS_SHIFT) /* 0x2 Positive Pulse */
#  define SSC_TFMR_FSOS_LOW          (3 << SSC_TFMR_FSOS_SHIFT) /* 0x3 Driven Low during data transfer */
#  define SSC_TFMR_FSOS_HIGH         (4 << SSC_TFMR_FSOS_SHIFT) /* 0x4 Driven High during data transfer */
#  define SSC_TFMR_FSOS_TOGGLE       (5 << SSC_TFMR_FSOS_SHIFT) /* 0x5 Toggling at each start of data transfer */
#define SSC_TFMR_FSDEN               (1 << 23) /* Bit 23: Frame Sync Data Enable */
#define SSC_TFMR_FSEDGE              (1 << 24) /* Bit 24: Frame Sync Edge Detection */
#define SSC_TFMR_FSLENEXT_SHIFT      (28)      /* Bits 28-31:  FSLEN Field Extension */
#define SSC_TFMR_FSLENEXT_MASK       (15 << SSC_TFMR_FSLENEXT_SHIFT)

/* SSC Receive Synchronization Holding Register */

#define SSC_RSHR_RSDAT_SHIFT         (0)       /* Bits 0-15:  Receive Synchronization Data */
#define SSC_RSHR_RSDAT_MASK          (0xffff << SSC_RSHR_RSDAT_SHIFT)

/* SSC Transmit Synchronization Holding Register */

#define SSC_TSHR_TSDAT_SHIFT         (0)       /* Bits 0-15:  Transmit Synchronization Data */
#define SSC_TSHR_TSDAT_MASK          (0xffff << SSC_TSHR_TSDAT_SHIFT)

/* SSC Receive Compare 0 Register */

#define SSC_RC0R_CP0_SHIFT           (0)        /* Bits 0-15:  Receive Compare Data 0 */
#define SSC_RC0R_CP0_MASK            (0xffff << SSC_RC0R_CP0_SHIFT)

/* SSC Receive Compare 1 Register */

#define SSC_RC1R_CP1_SHIFT           (0)      /* Bits 0-15:  Receive Compare Data 1 */
#define SSC_RC1R_CP1_MASK            (0xffff << SSC_RC1R_CP1_SHIFT)

/* SSC Status Register, SSC Interrupt Enable Register, SSC Interrupt Disable
 * Register, and SSC Interrupt Mask Register commin bit-field definitions
 */

#define SSC_INT_TXRDY                (1 << 0)  /* Bit 0:  Transmit Ready */
#define SSC_INT_TXEMPTY              (1 << 1)  /* Bit 1:  Transmit Empty */
#define SSC_INT_ENDTX                (1 << 2)  /* Bit 2:  End of Transmission */
#define SSC_INT_TXBUFE               (1 << 3)  /* Bit 3:  Transmit Buffer Empty */
#define SSC_INT_RXRDY                (1 << 4)  /* Bit 4:  Receive Ready */
#define SSC_INT_OVRUN                (1 << 5)  /* Bit 5:  Receive Overrun */
#define SSC_INT_ENDRX                (1 << 6)  /* Bit 6:  End of Reception */
#define SSC_INT_RXBUFF               (1 << 7)  /* Bit 7:  Receive Buffer Full */
#define SSC_INT_CP0                  (1 << 8)  /* Bit 8:  Compare 0 */
#define SSC_INT_CP1                  (1 << 9)  /* Bit 9:  Compare 1 */
#define SSC_INT_TXSYN                (1 << 10) /* Bit 10: Transmit Sync */
#define SSC_INT_RXSYN                (1 << 11) /* Bit 11: Receive Sync */
#define SSC_SR_TXEN                  (1 << 16) /* Bit 16: Transmit Enable (SR only) */
#define SSC_SR_RXEN                  (1 << 17) /* Bit 17: Receive Enable (SR only) */

/* SSC Write Protect Mode Register */

#define SSC_WPMR_WPEN                (1 << 0)  /* Bit 0:  Write Protect Enable */
#define SSC_WPMR_WPKEY_SHIFT         (8)       /* Bits 8-31:  Write Protect KEY */
#define SSC_WPMR_WPKEY_MASK          (0x00ffffff << SSC_WPMR_WPKEY_SHIFT)

/* SSC Write Protect Status Register */

#define SSC_WPSR_WPVS                (1 << 0)  /* Bit 0:  Write Protect Violation Status */
#define SSC_WPSR_WPVSRC_SHIFT        (8)       /* Bits 8-23:  Write Protect Violation Source */
#define SSC_WPSR_WPVSRC_MASK         (0xffff << SSC_WPSR_WPVSRC_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_SSC_H */
