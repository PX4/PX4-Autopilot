/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_usbdev.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_USBDEV_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_USBDEV_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip.h>

#ifdef CONFIG_STM32_STM32F10XX

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

/* Endpoint Registers */

#define STM32_USB_EPR_OFFSET(n)      ((n) << 2) /* USB endpoint n register (16-bits) */
#define STM32_USB_EP0R_OFFSET        0x0000  /* USB endpoint 0 register (16-bits) */
#define STM32_USB_EP1R_OFFSET        0x0004  /* USB endpoint 1 register (16-bits) */
#define STM32_USB_EP2R_OFFSET        0x0008  /* USB endpoint 2 register (16-bits) */
#define STM32_USB_EP3R_OFFSET        0x000c  /* USB endpoint 3 register (16-bits) */
#define STM32_USB_EP4R_OFFSET        0x0010  /* USB endpoint 4 register (16-bits) */
#define STM32_USB_EP5R_OFFSET        0x0014  /* USB endpoint 5 register (16-bits) */
#define STM32_USB_EP6R_OFFSET        0x0018  /* USB endpoint 6 register (16-bits) */
#define STM32_USB_EP7R_OFFSET        0x001c  /* USB endpoint 7 register (16-bits) */

/* Common Registers */

#define STM32_USB_CNTR_OFFSET        0x0040  /* USB control register (16-bits) */
#define STM32_USB_ISTR_OFFSET        0x0044  /* USB interrupt status register (16-bits) */
#define STM32_USB_FNR_OFFSET         0x0048  /* USB frame number register (16-bits) */
#define STM32_USB_DADDR_OFFSET       0x004c  /* USB device address (16-bits) */
#define STM32_USB_BTABLE_OFFSET      0x0050  /* Buffer table address (16-bits) */

/* Buffer Descriptor Table (Relatative to BTABLE address) */

#define STM32_USB_ADDR_TX_WOFFSET   (0)     /* Transmission buffer address n (16-bits) */
#define STM32_USB_COUNT_TX_WOFFSET  (2)     /* Transmission byte count n (16-bits) */
#define STM32_USB_ADDR_RX_WOFFSET   (4)     /* Reception buffer address n (16-bits) */
#define STM32_USB_COUNT_RX_WOFFSET  (6)     /* Reception byte count n (16-bits) */

#define STM32_USB_BTABLE_RADDR(ep,o) ((((uint32_t)getreg16(STM32_USB_BTABLE) + ((ep) << 3)) + (o))  << 1)
#define STM32_USB_ADDR_TX_OFFSET(ep)  STM32_USB_BTABLE_RADDR(ep,STM32_USB_ADDR_TX_WOFFSET)
#define STM32_USB_COUNT_TX_OFFSET(ep) STM32_USB_BTABLE_RADDR(ep,STM32_USB_COUNT_TX_WOFFSET)
#define STM32_USB_ADDR_RX_OFFSET(ep)  STM32_USB_BTABLE_RADDR(ep,STM32_USB_ADDR_RX_WOFFSET)
#define STM32_USB_COUNT_RX_OFFSET(ep) STM32_USB_BTABLE_RADDR(ep,STM32_USB_COUNT_RX_WOFFSET)

/* Register Addresses ***************************************************************/

/* Endpoint Registers */

#define STM32_USB_EPR(n)             (STM32_USB_BASE+STM32_USB_EPR_OFFSET(n))
#define STM32_USB_EP0R               (STM32_USB_BASE+STM32_USB_EP0R_OFFSET)
#define STM32_USB_EP1R               (STM32_USB_BASE+STM32_USB_EP1R_OFFSET)
#define STM32_USB_EP2R               (STM32_USB_BASE+STM32_USB_EP2R_OFFSET)
#define STM32_USB_EP3R               (STM32_USB_BASE+STM32_USB_EP3R_OFFSET)
#define STM32_USB_EP4R               (STM32_USB_BASE+STM32_USB_EP4R_OFFSET)
#define STM32_USB_EP5R               (STM32_USB_BASE+STM32_USB_EP5R_OFFSET)
#define STM32_USB_EP6R               (STM32_USB_BASE+STM32_USB_EP6R_OFFSET)
#define STM32_USB_EP7R               (STM32_USB_BASE+STM32_USB_EP7R_OFFSET)

/* Common Registers */

#define STM32_USB_CNTR               (STM32_USB_BASE+STM32_USB_CNTR_OFFSET)
#define STM32_USB_ISTR               (STM32_USB_BASE+STM32_USB_ISTR_OFFSET)
#define STM32_USB_FNR                (STM32_USB_BASE+STM32_USB_FNR_OFFSET)
#define STM32_USB_DADDR              (STM32_USB_BASE+STM32_USB_DADDR_OFFSET)
#define STM32_USB_BTABLE             (STM32_USB_BASE+STM32_USB_BTABLE_OFFSET)

/* Buffer Descriptor Table (Relatative to BTABLE address) */

#define STM32_USB_BTABLE_ADDR(ep,o)  (STM32_USBCANRAM_BASE+STM32_USB_BTABLE_RADDR(ep,o))
#define STM32_USB_ADDR_TX(ep)        STM32_USB_BTABLE_ADDR(ep,STM32_USB_ADDR_TX_WOFFSET)
#define STM32_USB_COUNT_TX(ep)       STM32_USB_BTABLE_ADDR(ep,STM32_USB_COUNT_TX_WOFFSET)
#define STM32_USB_ADDR_RX(ep)        STM32_USB_BTABLE_ADDR(ep,STM32_USB_ADDR_RX_WOFFSET)
#define STM32_USB_COUNT_RX(ep)       STM32_USB_BTABLE_ADDR(ep,STM32_USB_COUNT_RX_WOFFSET)

/* Register Bitfield Definitions ****************************************************/

/* USB endpoint register */

#define USB_EPR_EA_SHIFT             (0)       /* Bits 3:0 [3:0]: Endpoint Address */
#define USB_EPR_EA_MASK              (0X0f << USB_EPR_EA_SHIFT)
#define USB_EPR_STATTX_SHIFT         (4)       /* Bits 5-4: Status bits, for transmission transfers */
#define USB_EPR_STATTX_MASK          (3 << USB_EPR_STATTX_SHIFT)
#  define USB_EPR_STATTX_DIS         (0 << USB_EPR_STATTX_SHIFT) /* EndPoint TX DISabled */
#  define USB_EPR_STATTX_STALL       (1 << USB_EPR_STATTX_SHIFT) /* EndPoint TX STALLed */
#  define USB_EPR_STATTX_NAK         (2 << USB_EPR_STATTX_SHIFT) /* EndPoint TX NAKed */
#  define USB_EPR_STATTX_VALID       (3 << USB_EPR_STATTX_SHIFT) /* EndPoint TX VALID */
#  define USB_EPR_STATTX_DTOG1       (1 << USB_EPR_STATTX_SHIFT) /* EndPoint TX Data Toggle bit1 */
#  define USB_EPR_STATTX_DTOG2       (2 << USB_EPR_STATTX_SHIFT) /* EndPoint TX Data Toggle bit2 */
#define USB_EPR_DTOG_TX              (1 << 6)  /* Bit 6: Data Toggle, for transmission transfers */
#define USB_EPR_CTR_TX               (1 << 7)  /* Bit 7: Correct Transfer for transmission */
#define USB_EPR_EP_KIND              (1 << 8)  /* Bit 8: Endpoint Kind */
#define USB_EPR_EPTYPE_SHIFT         (9)       /* Bits 10-9: Endpoint type */
#define USB_EPR_EPTYPE_MASK          (3 << USB_EPR_EPTYPE_SHIFT)
#  define USB_EPR_EPTYPE_BULK        (0 << USB_EPR_EPTYPE_SHIFT) /* EndPoint BULK */
#  define USB_EPR_EPTYPE_CONTROL     (1 << USB_EPR_EPTYPE_SHIFT) /* EndPoint CONTROL */
#  define USB_EPR_EPTYPE_ISOC        (2 << USB_EPR_EPTYPE_SHIFT) /* EndPoint ISOCHRONOUS */
#  define USB_EPR_EPTYPE_INTERRUPT   (3 << USB_EPR_EPTYPE_SHIFT) /* EndPoint INTERRUPT */
#define USB_EPR_SETUP                (1 << 11) /* Bit 11: Setup transaction completed */
#define USB_EPR_STATRX_SHIFT         (12)      /* Bits 13-12: Status bits, for reception transfers */
#define USB_EPR_STATRX_MASK          (3 << USB_EPR_STATRX_SHIFT)
#  define USB_EPR_STATRX_DIS         (0 << USB_EPR_STATRX_SHIFT) /* EndPoint RX DISabled */
#  define USB_EPR_STATRX_STALL       (1 << USB_EPR_STATRX_SHIFT) /* EndPoint RX STALLed */
#  define USB_EPR_STATRX_NAK         (2 << USB_EPR_STATRX_SHIFT) /* EndPoint RX NAKed */
#  define USB_EPR_STATRX_VALID       (3 << USB_EPR_STATRX_SHIFT) /* EndPoint RX VALID */
#  define USB_EPR_STATRX_DTOG1       (1 << USB_EPR_STATRX_SHIFT) /* EndPoint RX Data TOGgle bit1 */
#  define USB_EPR_STATRX_DTOG2       (2 << USB_EPR_STATRX_SHIFT) /* EndPoint RX Data TOGgle bit1 */
#define USB_EPR_DTOG_RX              (1 << 14) /* Bit 14: Data Toggle, for reception transfers */
#define USB_EPR_CTR_RX               (1 << 15) /* Bit 15: Correct Transfer for reception */

/* USB control register */

#define USB_CNTR_FRES                (1 << 0)  /* Bit 0: Force USB Reset */
#define USB_CNTR_PDWN                (1 << 1)  /* Bit 1: Power down */
#define USB_CNTR_LPMODE              (1 << 2)  /* Bit 2: Low-power mode */
#define USB_CNTR_FSUSP               (1 << 3)  /* Bit 3: Force suspend */
#define USB_CNTR_RESUME              (1 << 4)  /* Bit 4: Resume request */
#define USB_CNTR_ESOFM               (1 << 8)  /* Bit 8: Expected Start Of Frame Interrupt Mask */
#define USB_CNTR_SOFM                (1 << 9)  /* Bit 9: Start Of Frame Interrupt Mask */
#define USB_CNTR_RESETM              (1 << 10) /* Bit 10: USB Reset Interrupt Mask */
#define USB_CNTR_SUSPM               (1 << 11) /* Bit 11: Suspend mode Interrupt Mask */
#define USB_CNTR_WKUPM               (1 << 12) /* Bit 12: Wakeup Interrupt Mask */
#define USB_CNTR_ERRM                (1 << 13) /* Bit 13: Error Interrupt Mask */
#define USB_CNTR_DMAOVRNM            (1 << 14) /* Bit 14: Packet Memory Area Over / Underrun Interrupt Mask */
#define USB_CNTR_CTRM                (1 << 15) /* Bit 15: Correct Transfer Interrupt Mask */

#define USB_CNTR_ALLINTS             (USB_CNTR_ESOFM|USB_CNTR_SOFM|USB_CNTR_RESETM|USB_CNTR_SUSPM|\
                                      USB_CNTR_WKUPM|USB_CNTR_ERRM|USB_CNTR_DMAOVRNM|USB_CNTR_CTRM)

/* USB interrupt status register */

#define USB_ISTR_EPID_SHIFT          (0)       /* Bits 3-0: Endpoint Identifier */
#define USB_ISTR_EPID_MASK           (0x0f << USB_ISTR_EPID_SHIFT)
#define USB_ISTR_DIR                 (1 << 4)  /* Bit 4: Direction of transaction */
#define USB_ISTR_ESOF                (1 << 8)  /* Bit 8: Expected Start Of Frame */
#define USB_ISTR_SOF                 (1 << 9)  /* Bit 9: Start Of Frame */
#define USB_ISTR_RESET               (1 << 10) /* Bit 10: USB RESET request */
#define USB_ISTR_SUSP                (1 << 11) /* Bit 11: Suspend mode request */
#define USB_ISTR_WKUP                (1 << 12) /* Bit 12: Wake up */
#define USB_ISTR_ERR                 (1 << 13) /* Bit 13: Error */
#define USB_ISTR_DMAOVRN             (1 << 14) /* Bit 14: Packet Memory Area Over / Underrun */
#define USB_ISTR_CTR                 (1 << 15) /* Bit 15: Correct Transfer */

#define USB_ISTR_ALLINTS             (USB_ISTR_ESOF|USB_ISTR_SOF|USB_ISTR_RESET|USB_ISTR_SUSP|\
                                      USB_ISTR_WKUP|USB_ISTR_ERR|USB_ISTR_DMAOVRN|USB_ISTR_CTR)

/* USB frame number register */

#define USB_FNR_FN_SHIFT             (0)       /* Bits 10-0: Frame Number */
#define USB_FNR_FN_MASK              (0x07ff << USB_FNR_FN_SHIFT)
#define USB_FNR_LSOF_SHIFT           (11)      /* Bits 12-11: Lost SOF */
#define USB_FNR_LSOF_MASK            (3 << USB_FNR_LSOF_SHIFT)
#define USB_FNR_LCK                  (1 << 13) /* Bit 13: Locked */
#define USB_FNR_RXDM                 (1 << 14) /* Bit 14: Receive Data - Line Status */
#define USB_FNR_RXDP                 (1 << 15) /* Bit 15: Receive Data + Line Status */

/* USB device address */

#define USB_DADDR_ADD_SHIFT          (0)       /* Bits 6-0: Device Address */
#define USB_DADDR_ADD_MASK           (0x7f << USB_DADDR_ADD_SHIFT)
#define USB_DADDR_EF                 (1 << 7)  /* Bit 7: Enable Function */

/* Buffer table address */

#define USB_BTABLE_SHIFT             (3)       /* Bits 15:3: Buffer Table */
#define USB_BTABLE_MASK              (0x1fff << USB_BTABLE_SHIFT)

/* Transmission buffer address */

#define USB_ADDR_TX_ZERO             (1 << 0)  /* Bit 0 Must always be written as ‘0’ */
#define USB_ADDR_TX_SHIFT            (1)       /* Bits 15-1: Transmission Buffer Address */
#define USB_ADDR_TX_MASK             (0x7fff << USB_ADDR_ADDR_TX_SHIFT)

/* Transmission byte count */

#define USB_COUNT_TX_SHIFT           (0)       /* Bits 9-0: Transmission Byte Count */
#define USB_COUNT_TX_MASK            (0x03ff << USB_COUNT_COUNT_TX_SHIFT)

/* Reception buffer address */

#define USB_ADDR_RX_ZERO             (1 << 0)  /* Bit 0 This bit must always be written as ‘0’ */
#define USB_ADDR_RX_SHIFT            (1)       /* Bits 15:1 ADDRn_RX[15:1]: Reception Buffer Address */
#define USB_ADDR_RX_MASK             (0x7fff << USB_ADDR_RX_SHIFT)

/* Reception byte count */

#define USB_COUNT_RX_BL_SIZE         (1 << 15) /* Bit 15: BLock SIZE. */
#define USB_COUNT_RX_NUM_BLOCK_SHIFT (10)      /* Bits 14-10: Number of blocks */
#define USB_COUNT_RX_NUM_BLOCK_MASK  (0x1f << USB_COUNT_RX_NUM_BLOCK_SHIFT)
#define USB_COUNT_RX_SHIFT           (0)       /* Bits 9-0: Reception Byte Count */
#define USB_COUNT_RX_MASK            (0x03ff << USB_COUNT_RX_SHIFT)

#endif /* CONFIG_STM32_STM32F10XX */
#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_USBDEV_H */

