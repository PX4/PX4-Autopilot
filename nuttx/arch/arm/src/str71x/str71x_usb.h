/************************************************************************************
 * arch/arm/src/str71x/str71x_usb.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STR71X_STR71X_USB_H
#define __ARCH_ARM_SRC_STR71X_STR71X_USB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "str71x_map.h"

/************************************************************************************
 * Pre-procesor Definitions
 ************************************************************************************/

/* USB registers ********************************************************************/

#define STR71X_USB_NENDPNTS     (16)
#define STR71X_USB_EPR(ep)      (STR71X_USB_BASE + ((ep) << 4))
#define STR71X_USB_EP0R         (STR71X_USB_BASE + 0x0000)  /* Endpoint 0 */
#define STR71X_USB_EP1R         (STR71X_USB_BASE + 0x0004)  /* Endpoint 1 */
#define STR71X_USB_EP2R         (STR71X_USB_BASE + 0x0008)  /* Endpoint 2 */
#define STR71X_USB_EP3R         (STR71X_USB_BASE + 0x000c)  /* Endpoint 3 */
#define STR71X_USB_EP4R         (STR71X_USB_BASE + 0x0010)  /* Endpoint 4 */
#define STR71X_USB_EP5R         (STR71X_USB_BASE + 0x0014)  /* Endpoint 5 */
#define STR71X_USB_EP6R         (STR71X_USB_BASE + 0x0018)  /* Endpoint 6 */
#define STR71X_USB_EP7R         (STR71X_USB_BASE + 0x001c)  /* Endpoint 7 */
#define STR71X_USB_EP8R         (STR71X_USB_BASE + 0x0020)  /* Endpoint 8 */
#define STR71X_USB_EP9R         (STR71X_USB_BASE + 0x0024)  /* Endpoint 9 */
#define STR71X_USB_EP10R        (STR71X_USB_BASE + 0x0028)  /* Endpoint 10 */
#define STR71X_USB_EP11R        (STR71X_USB_BASE + 0x002c)  /* Endpoint 11 */
#define STR71X_USB_EP12R        (STR71X_USB_BASE + 0x0030)  /* Endpoint 12 */
#define STR71X_USB_EP13R        (STR71X_USB_BASE + 0x0034)  /* Endpoint 13 */
#define STR71X_USB_EP14R        (STR71X_USB_BASE + 0x0038)  /* Endpoint 14 */
#define STR71X_USB_EP15R        (STR71X_USB_BASE + 0x003c)  /* Endpoint 15 */
#define STR71X_USB_CNTR         (STR71X_USB_BASE + 0x0040)  /* Control register */
#define STR71X_USB_ISTR         (STR71X_USB_BASE + 0x0044)  /* Interrupt status register */
#define STR71X_USB_FNR          (STR71X_USB_BASE + 0x0048)  /* Frame number register */
#define STR71X_USB_DADDR        (STR71X_USB_BASE + 0x004C)  /* Device address register */
#define STR71X_USB_BTABLE       (STR71X_USB_BASE + 0x0050)  /* Buffer Table address register */

/* Register bit settings ***********************************************************/

/* Control Register (CNTR) */

#define USB_CNTR_FRES          (1 << 0)   /* Bit 0:  Force usb reset */
#define USB_CNTR_PDWN          (1 << 1)   /* Bit 1:  Power down */
#define USB_CNTR_LPMODE        (1 << 2)   /* Bit 2:  Low-power mode  */
#define USB_CNTR_FSUSP         (1 << 3)   /* Bit 3:  Force suspend */
#define USB_CNTR_RESUME        (1 << 4)   /* Bit 4:  Resume request */
#define USB_CNTR_ESOFM         (1 << 8)   /* Bit 8:  Expected start of frame */
#define USB_CNTR_SOFM          (1 << 9)   /* Bit 9:  Start of frame */
#define USB_CNTR_RESETM        (1 << 10)  /* Bit 10: Reset   */
#define USB_CNTR_SUSPM         (1 << 11)  /* Bit 11: Suspend  */
#define USB_CNTR_WKUPM         (1 << 12)  /* Bit 12: Wake up */
#define USB_CNTR_ERRM          (1 << 13)  /* Bit 13: Error */
#define USB_CNTR_DOVRM         (1 << 14)  /* Bit 14: DMA over/underrun */
#define USB_CNTR_CTRM          (1 << 15)  /* Bit 15: Correct transfer */

/* Interrupt status register (ISTR) */

#define USB_ISTR_EPID_SHIFT    0          /* Bits 0-3: Endpoint Identifier  */
#define USB_ISTR_EPID_MASK     (0x0f << USB_ISTR_EPID_SHIFT)
#define USB_ISTR_DIR           (1 << 4)   /* Bit 4:  DIRection of transaction  */
#define USB_ISTR_ESOF          (1 << 8)   /* Bit 8:  Expected start of frame */
#define USB_ISTR_SOF           (1 << 9)   /* Bit 9:  Start of frame */
#define USB_ISTR_RESET         (1 << 10)  /* Bit 10: Reset */
#define USB_ISTR_SUSP          (1 << 11)  /* Bit 11: Suspend */
#define USB_ISTR_WKUP          (1 << 12)  /* Bit 12: Wakeup */
#define USB_ISTR_ERR           (1 << 13)  /* Bit 13: Error */
#define USB_ISTR_DOVR          (1 << 14)  /* Bit 14: DMA Over/underrun */
#define USB_ISTR_CTR           (1 << 15)  /* Bit 15: Correct Transfer */

/* Frame number register (FNR) */

#define USB_FNR_FN_SHIFT       0          /* Bit 0-10: Frame number */
#define USB_FNR_LSOF_SHIFT     11         /* Bits 11-12 : Lost SOF */
#define USB_FNR_LSOF_MASK      (3 << USB_FNR_LSOF_SHIFT)
#define USB_FNR_FN_MASK        (0x07ff << USB_FNR_FN_SHIFT)
#define USB_FNR_LCK            (1 << 13)  /* Bit 13: Locked */
#define USB_FNR_RXDM           (1 << 14)  /* Bit 14: Status of D- data line */
#define USB_FNR_RXDP           (1 << 15)  /* Bit 15: Status of D+ data line */

/* Device address register (DADDR) */

#define USB_DADDR_ADD_SHIFT    0          /* Bits 0-7: Device address */
#define USB_DADDR_ADD_MASK     (0x7f << USB_DADDR_ADD_SHIFT)
#define USB_DADDR_EF           (1 << 7)   /* Bit 8:  Enable function */

/* Endpoint registers (EPR) */

#define USB_EPR_ADDRFIELD_SHIFT 0         /* Bits 0-3: Endpoint address */
#define USB_EPR_ADDRFIELD_MASK  (0x0f << USB_EPR_ADDRFIELD_SHIFT)
#define USB_EPR_TXSTAT_SHIFT    4         /* Bits 4-5: Endpoint TX status bit */
#define USB_EPR_TXSTAT_MASK     (3 << USB_EPR_TXSTAT_SHIFT)
#  define USB_EPR_TXDIS         (0 << USB_EPR_TXSTAT_SHIFT) /* Endpoint TX disabled */
#  define USB_EPR_TXSTALL       (1 << USB_EPR_TXSTAT_SHIFT) /* Endpoint TX stalled */
#  define USB_EPR_TXNAK         (2 << USB_EPR_TXSTAT_SHIFT) /* Endpoint TX NAKed */
#  define USB_EPR_TXVALID       (3 << USB_EPR_TXSTAT_SHIFT) /* Endpoint TX valid */
#  define USB_EPR_TXDTOG1       (1 << USB_EPR_TXSTAT_SHIFT) /* Bit : Endpoint TX data toggle bit1 */
#  define USB_EPR_TXDTOG2       (2 << USB_EPR_TXSTAT_SHIFT) /* Bit : Endpoint TX data toggle bit2 */
#define USB_EPR_DTOGTX          (1 << 6)  /* Bit 6:  Endpoint data toggle TX */
#define USB_EPR_CTRTX           (1 << 7)  /* Bit 7:  Endpoint correct transfer TX */
#define USB_EPR_KIND            (1 << 8)  /* Bit 8:  Endpoint kind */
#define USB_EPR_EPTYPE_SHIFT    9         /* Bits 9-10: Endpoint type */
#define USB_EPR_EPTYPE_MASK     (3 << USB_EPR_EPTYPE_SHIFT)
#  define USB_EPR_BULK          (0 << USB_EPR_EPTYPE_SHIFT)  /* Endpoint BULK */
#  define USB_EPR_CONTROL       (1 << USB_EPR_EPTYPE_SHIFT)  /* Endpoint CONTROL */
#  define USB_EPR_ISOC          (2 << USB_EPR_EPTYPE_SHIFT)) /* Endpoint ISOCHRONOUS */
#  define USB_EPR_INTERRUPT     (3 << USB_EPR_EPTYPE_SHIFT)  /* Endpoint INTERRUPT */
#define USB_EPR_SETUP           (1 << 11) /* Bit 11: Endpoint setup */
#define USB_EPR_RXSTAT_SHIFT    12        /* Bits 12-13: Endpoint RX status bit */
#define USB_EPR_RXSTAT_MASK     (3 << USB_EPR_RXSTAT_SHIFT)
#  define USB_EPR_RXDIS         (0 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX disabled */
#  define USB_EPR_RXSTALL       (1 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX stalled */
#  define USB_EPR_RXNAK         (2 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX NAKed */
#  define USB_EPR_RXVALID       (3 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX valid */
#  define USB_EPR_RXDTOG1       (1 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX data toggle bit1 */
#  define USB_EPR_RXDTOG2       (2 << USB_EPR_RXSTAT_SHIFT) /* Endpoint RX data toggle bit2 */
#define USB_EPR_DTOGRX          (1 << 14) /* Bit 14: Endpoint data toggle RX */
#define USB_EPR_CTRRX           (1 << 15) /* Bit 15: Endpoint correct transfer RX   */

/* Endpoint register mask (no toggle fields) */

#define USB_EPR_NOTOGGLE_MASK   (USB_EPR_CTRRX|USB_EPR_SETUP|USB_EPR_TFIELD|\
                                 USB_EPR_KIND|USB_EPR_CTRTX|USB_EPR_ADDRFIELD)

/* Toggles only */

#define USB_EPR_TXDTOG_MASK     (USB_EPR_TXSTAT_MASK|USB_EPR_NOTOGGLE_MASK)
#define USB_EPR_RXDTOG_MASK     (USB_EPR_RXSTAT_MASK|USB_EPR_NOTOGGLE_MASK)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_STR71X_STR71X_USB_H */
