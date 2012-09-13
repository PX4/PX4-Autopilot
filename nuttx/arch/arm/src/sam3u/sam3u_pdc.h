/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_pdc.h
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_PDC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_PDC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* PDC register offsets *****************************************************************/

#define SAM3U_PDC_RPR_OFFSET         0x100 /* Receive Pointer Register */
#define SAM3U_PDC_RCR_OFFSET         0x104 /* Receive Counter Register */
#define SAM3U_PDC_TPR_OFFSET         0x108 /* Transmit Pointer Register */
#define SAM3U_PDC_TCR_OFFSET         0x10c /* Transmit Counter Register */
#define SAM3U_PDC_RNPR_OFFSET        0x110 /* Receive Next Pointer Register */
#define SAM3U_PDC_RNCR_OFFSET        0x114 /* Receive Next Counter Register */
#define SAM3U_PDC_TNPR_OFFSET        0x118 /* Transmit Next Pointer Register */
#define SAM3U_PDC_TNCR_OFFSET        0x11c /* Transmit Next Counter Register */
#define SAM3U_PDC_PTCR_OFFSET        0x120 /* Transfer Control Register */
#define SAM3U_PDC_PTSR_OFFSET        0x124 /* Transfer Status Register */

/* PDC register adresses ****************************************************************/

/* These 10 registers are mapped in the peripheral memory space at the same offset. */

/* PDC register bit definitions *********************************************************/

#define PDC_RCR_RXCTR_SHIFT          (0)      /* Bits 0-15:  Receive Counter Register */
#define PDC_RCR_RXCTR_MASK           (0xffff << PDC_RCR_RXCTR_SHIFT)

#define PDC_TCR_TXCTR_SHIFT          (0)      /* Bits 0-15:  Transmit Counter Register */
#define PDC_TCR_TXCTR_MASK           (0xffff << PDC_TCR_TXCTR_SHIFT)

#define PDC_RNCR_RXNCTR_SHIFT        (0)      /* Bits 0-15:  Receive Next Counter */
#define PDC_RNCR_RXNCTR_MASK         (0xffff << PDC_RNCR_RXNCTR_SHIFT)

#define PDC_TNCR_TXNCTR_SHIFT        (0)      /* Bits 0-15:   Transmit Counter Next */
#define PDC_TNCR_TXNCTR_MASK         (0xffff << PDC_TNCR_TXNCTR_SHIFT)

#define PDC_PTCR_RXTEN               (1 << 0)  /* Bit 0:  Receiver Transfer Enable */
#define PDC_PTCR_RXTDIS              (1 << 1)  /* Bit 1:  Receiver Transfer Disable */
#define PDC_PTCR_TXTEN               (1 << 8)  /* Bit 8:  Transmitter Transfer Enable */
#define PDC_PTCR_TXTDIS              (1 << 9)  /* Bit 9:  Transmitter Transfer Disable */

#define PDC_PTSR_RXTEN               (1 << 0)  /* Bit 0:  Receiver Transfer Enable */
#define PDC_PTSR_TXTEN               (1 << 8)  /* Bit 8:  Transmitter Transfer Enable */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_PDC_H */
