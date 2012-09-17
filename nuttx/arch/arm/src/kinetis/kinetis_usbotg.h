/********************************************************************************************
 * arch/arm/src/kinetis/kinetis_usbotg.h
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/* Register Offsets *************************************************************************/

#define KINETIS_USB_PERID_OFFSET    0x0000 /* Peripheral ID Register */
#define KINETIS_USB_IDCOMP_OFFSET   0x0004 /* Peripheral ID Complement Register */
#define KINETIS_USB_REV_OFFSET      0x0008 /* Peripheral Revision Register */
#define KINETIS_USB_ADDINFO_OFFSET  0x000c /* Peripheral Additional Info Register */
#define KINETIS_USB_OTGISTAT_OFFSET 0x0010 /* OTG Interrupt Status Register */
#define KINETIS_USB_OTGICR_OFFSET   0x0014 /* OTG Interrupt Control Register */
#define KINETIS_USB_OTGSTAT_OFFSET  0x0018 /* OTG Status Register */
#define KINETIS_USB_OTGCTL_OFFSET   0x001c /* OTG Control Register */
#define KINETIS_USB_ISTAT_OFFSET    0x0080 /* Interrupt Status Register */
#define KINETIS_USB_INTEN_OFFSET    0x0084 /* Interrupt Enable Register */
#define KINETIS_USB_ERRSTAT_OFFSET  0x0088 /* Error Interrupt Status Register */
#define KINETIS_USB_ERREN_OFFSET    0x008c /* Error Interrupt Enable Register */
#define KINETIS_USB_STAT_OFFSET     0x0090 /* Status Register */
#define KINETIS_USB_CTL_OFFSET      0x0094 /* Control Register */
#define KINETIS_USB_ADDR_OFFSET     0x0098 /* Address Register */
#define KINETIS_USB_BDTPAGE1_OFFSET 0x009c /* BDT Page Register 1 */
#define KINETIS_USB_FRMNUML_OFFSET  0x00a0 /* Frame Number Register Low */
#define KINETIS_USB_FRMNUMH_OFFSET  0x00a4 /* Frame Number Register High */
#define KINETIS_USB_TOKEN_OFFSET    0x00a8 /* Token Register */
#define KINETIS_USB_SOFTHLD_OFFSET  0x00ac /* SOF Threshold Register */
#define KINETIS_USB_BDTPAGE2_OFFSET 0x00b0 /* BDT Page Register 2 */
#define KINETIS_USB_BDTPAGE3_OFFSET 0x00b4 /* BDT Page Register 3 */

#define KINETIS_USB_ENDPT_OFFSET(n) (0x00c0+((n)<<2)) /* Endpoint n Control Register */
#define KINETIS_USB_ENDPT0_OFFSET   0x00c0 /* Endpoint 0 Control Register */
#define KINETIS_USB_ENDPT1_OFFSET   0x00c4 /* Endpoint 1 Control Register */
#define KINETIS_USB_ENDPT2_OFFSET   0x00c8 /* Endpoint 2 Control Register */
#define KINETIS_USB_ENDPT3_OFFSET   0x00cc /* Endpoint 3 Control Register */
#define KINETIS_USB_ENDPT4_OFFSET   0x00d0 /* Endpoint 4 Control Register */
#define KINETIS_USB_ENDPT5_OFFSET   0x00d4 /* Endpoint 5 Control Register */
#define KINETIS_USB_ENDPT6_OFFSET   0x00d8 /* Endpoint 6 Control Register */
#define KINETIS_USB_ENDPT7_OFFSET   0x00dc /* Endpoint 7 Control Register */
#define KINETIS_USB_ENDPT8_OFFSET   0x00e0 /* Endpoint 8 Control Register */
#define KINETIS_USB_ENDPT9_OFFSET   0x00e4 /* Endpoint 9 Control Register */
#define KINETIS_USB_ENDPT10_OFFSET  0x00e8 /* Endpoint 10 Control Register */
#define KINETIS_USB_ENDPT11_OFFSET  0x00ec /* Endpoint 11 Control Register */
#define KINETIS_USB_ENDPT12_OFFSET  0x00f0 /* Endpoint 12 Control Register */
#define KINETIS_USB_ENDPT13_OFFSET  0x00f4 /* Endpoint 13 Control Register */
#define KINETIS_USB_ENDPT14_OFFSET  0x00f8 /* Endpoint 14 Control Register */
#define KINETIS_USB_ENDPT15_OFFSET  0x00fc /* Endpoint 15 Control Register */

#define KINETIS_USB_USBCTRL_OFFSET  0x0100 /* USB Control Register */
#define KINETIS_USB_OBSERVE_OFFSET  0x0104 /* USB OTG Observe Register */
#define KINETIS_USB_CONTROL_OFFSET  0x0108 /* USB OTG Control Register */
#define KINETIS_USB_USBTRC0_OFFSET  0x010c /* USB Transceiver Control Register 0 */

/* Register Addresses ***********************************************************************/

#define KINETIS_USB0_PERID          (KINETIS_USB0_BASE+KINETIS_USB_PERID_OFFSET)
#define KINETIS_USB0_IDCOMP         (KINETIS_USB0_BASE+KINETIS_USB_IDCOMP_OFFSET)
#define KINETIS_USB0_REV            (KINETIS_USB0_BASE+KINETIS_USB_REV_OFFSET)
#define KINETIS_USB0_ADDINFO        (KINETIS_USB0_BASE+KINETIS_USB_ADDINFO_OFFSET)
#define KINETIS_USB0_OTGISTAT       (KINETIS_USB0_BASE+KINETIS_USB_OTGISTAT_OFFSET)
#define KINETIS_USB0_OTGICR         (KINETIS_USB0_BASE+KINETIS_USB_OTGICR_OFFSET)
#define KINETIS_USB0_OTGSTAT        (KINETIS_USB0_BASE+KINETIS_USB_OTGSTAT_OFFSET)
#define KINETIS_USB0_OTGCTL         (KINETIS_USB0_BASE+KINETIS_USB_OTGCTL_OFFSET)
#define KINETIS_USB0_ISTAT          (KINETIS_USB0_BASE+KINETIS_USB_ISTAT_OFFSET)
#define KINETIS_USB0_INTEN          (KINETIS_USB0_BASE+KINETIS_USB_INTEN_OFFSET)
#define KINETIS_USB0_ERRSTAT        (KINETIS_USB0_BASE+KINETIS_USB_ERRSTAT_OFFSET)
#define KINETIS_USB0_ERREN          (KINETIS_USB0_BASE+KINETIS_USB_ERREN_OFFSET)
#define KINETIS_USB0_STAT           (KINETIS_USB0_BASE+KINETIS_USB_STAT_OFFSET)
#define KINETIS_USB0_CTL            (KINETIS_USB0_BASE+KINETIS_USB_CTL_OFFSET)
#define KINETIS_USB0_ADDR           (KINETIS_USB0_BASE+KINETIS_USB_ADDR_OFFSET)
#define KINETIS_USB0_BDTPAGE1       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE1_OFFSET)
#define KINETIS_USB0_FRMNUML        (KINETIS_USB0_BASE+KINETIS_USB_FRMNUML_OFFSET)
#define KINETIS_USB0_FRMNUMH        (KINETIS_USB0_BASE+KINETIS_USB_FRMNUMH_OFFSET)
#define KINETIS_USB0_TOKEN          (KINETIS_USB0_BASE+KINETIS_USB_TOKEN_OFFSET)
#define KINETIS_USB0_SOFTHLD        (KINETIS_USB0_BASE+KINETIS_USB_SOFTHLD_OFFSET)
#define KINETIS_USB0_BDTPAGE2       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE2_OFFSET)
#define KINETIS_USB0_BDTPAGE3       (KINETIS_USB0_BASE+KINETIS_USB_BDTPAGE3_OFFSET)

#define KINETIS_USB0_ENDPT(n)       (KINETIS_USB0_BASE+KINETIS_USB_ENDPT_OFFSET(n))
#define KINETIS_USB0_ENDPT0         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT0_OFFSET)
#define KINETIS_USB0_ENDPT1         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT1_OFFSET)
#define KINETIS_USB0_ENDPT2         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT2_OFFSET)
#define KINETIS_USB0_ENDPT3         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT3_OFFSET)
#define KINETIS_USB0_ENDPT4         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT4_OFFSET)
#define KINETIS_USB0_ENDPT5         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT5_OFFSET)
#define KINETIS_USB0_ENDPT6         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT6_OFFSET)
#define KINETIS_USB0_ENDPT7         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT7_OFFSET)
#define KINETIS_USB0_ENDPT8         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT8_OFFSET)
#define KINETIS_USB0_ENDPT9         (KINETIS_USB0_BASE+KINETIS_USB_ENDPT9_OFFSET)
#define KINETIS_USB0_ENDPT10        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT10_OFFSET)
#define KINETIS_USB0_ENDPT11        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT11_OFFSET)
#define KINETIS_USB0_ENDPT12        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT12_OFFSET)
#define KINETIS_USB0_ENDPT13        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT13_OFFSET)
#define KINETIS_USB0_ENDPT14        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT14_OFFSET)
#define KINETIS_USB0_ENDPT15        (KINETIS_USB0_BASE+KINETIS_USB_ENDPT15_OFFSET)

#define KINETIS_USB0_USBCTRL        (KINETIS_USB0_BASE+KINETIS_USB_USBCTRL_OFFSET)
#define KINETIS_USB0_OBSERVE        (KINETIS_USB0_BASE+KINETIS_USB_OBSERVE_OFFSET)
#define KINETIS_USB0_CONTROL        (KINETIS_USB0_BASE+KINETIS_USB_CONTROL_OFFSET)
#define KINETIS_USB0_USBTRC0        (KINETIS_USB0_BASE+KINETIS_USB_USBTRC0_OFFSET)

/* Register Bit Definitions *****************************************************************/

/* Peripheral ID Register (8-bit) */
                                              /* Bits 6-7: Reserved */
#define USB_PERID_MASK              (0x3f)    /* Bits 0-5: Peripheral identification bits */

/* Peripheral ID Complement Register (8-bit) */
#define USB_IDCOMP_
                                              /* Bits 6-7: Reserved */
#define USB_IDCOMP_MASK             (0x3f)    /* Bits 0-5: Ones complement of peripheral identification bits */

/* Peripheral Revision Register (8-bit revision number) */

/* Peripheral Additional Info Register (8-bit) */

#define USB_ADDINFO_IEHOST          (1 << 0)  /* Bit 0:  This bit is set if host mode is enabled */
                                              /* Bits 1-2: Reserved */
#define USB_ADDINFO_IRQNUM_SHIFT    (3)       /* Bits 3-7: Assigned Interrupt Request Number */
#define USB_ADDINFO_IRQNUM_MASK     (31 << USB_ADDINFO_IRQNUM_SHIFT)

/* OTG Interrupt Status Register(8-bit)  */

#define USB_OTGISTAT_AVBUSCHG       (1 << 0)  /* Bit 0:  Change in VBUS is detected on an A device */
                                              /* Bit 1:  Reserved */
#define USB_OTGISTAT_B_SESS_CHG     (1 << 2)  /* Bit 2:  Change in VBUS is detected on a B device */
#define USB_OTGISTAT_SESSVLDCHG     (1 << 3)  /* Bit 3:  Change in VBUS is detected */
                                              /* Bit 4:  Reserved */
#define USB_OTGISTAT_LINE_STATE_CHG (1 << 5)  /* Bit 5:  Change USB line state */
#define USB_OTGISTAT_ONEMSEC        (1 << 6)  /* Bit 6:  Set when the 1 millisecond timer expires */
#define USB_OTGISTAT_IDCHG          (1 << 7)  /* Bit 7:  Change in ID Signal from the USB connector */

/* OTG Interrupt Control Register (8-bit) */

#define USB_OTGICR_AVBUSEN          (1 << 0)  /* Bit 0:  A VBUS Valid interrupt enable */
                                              /* Bit 1:  Reserved */
#define USB_OTGICR_BSESSEN          (1 << 2)  /* Bit 2:  B Session END interrupt enable */
#define USB_OTGICR_SESSVLDEN        (1 << 3)  /* Bit 3:  Session valid interrupt enable */
                                              /* Bit 4:  Reserved */
#define USB_OTGICR_LINESTATEEN      (1 << 5)  /* Bit 5:  Line State change interrupt enable */
#define USB_OTGICR_ONEMSECEN        (1 << 6)  /* Bit 6:  1 millisecond interrupt enable */
#define USB_OTGICR_IDEN             (1 << 7)  /* Bit 7:  ID interrupt enable */

/* OTG Status Register (8-bit) */

#define USB_OTGSTAT_AVBUSVLD        (1 << 0)  /* Bit 0:  A VBUS Valid */
                                              /* Bit 1:  Reserved */
#define USB_OTGSTAT_BSESSEND        (1 << 2)  /* Bit 2:  B Session END */
#define USB_OTGSTAT_SESS_VLD        (1 << 3)  /* Bit 3:  Session valid */
                                              /* Bit 4:  Reserved */
#define USB_OTGSTAT_LINESTATESTABLE (1 << 5)  /* Bit 5:  OTGISTAT LINE_STATE_CHG bit stable */
#define USB_OTGSTAT_ONEMSECEN       (1 << 6)  /* Bit 6:  Reserved for the 1msec count */
#define USB_OTGSTAT_ID              (1 << 7)  /* Bit 7:  Current state of the ID pin on the USB connector */

/* OTG Control Register (8-bit) */
                                              /* Bits 0-1:  Reserved */
#define USB_OTGCTL_OTGEN            (1 << 2)  /* Bit 2:  On-The-Go pullup/pulldown resistor enable */
                                              /* Bit 3:  Reserved */
#define USB_OTGCTL_DMLOW            (1 << 4)  /* Bit 4:  D- Data Line pull-down resistor enable */
#define USB_OTGCTL_DPLOW            (1 << 5)  /* Bit 5:  D+ Data Line pull-down resistor enable */
                                              /* Bit 6:  Reserved */
#define USB_OTGCTL_DPHIGH           (1 << 7)  /* Bit 7: D+ Data Line pullup resistor enable */

/* Interrupt Status Register Interrupt Enable Register (8-bit) */

#define USB_INT_USBRST              (1 << 0)  /* Bit 0:  USB Module has decoded a valid USB reset */
#define USB_INT_ERROR               (1 << 1)  /* Bit 1:  Any of the error conditions within the ERRSTAT register */
#define USB_INT_SOFTOK              (1 << 2)  /* Bit 2:  USB Module received a Start Of Frame (SOF) token */
#define USB_INT_TOKDNE              (1 << 3)  /* Bit 3:  Current token being processed has completed */
#define USB_INT_SLEEP               (1 << 4)  /* Bit 4:  Constant idle on the USB bus for 3 milliseconds */
#define USB_INT_RESUME              (1 << 5)  /* Bit 5:  Signal remote wake-up signaling */
#define USB_INT_ATTACH              (1 << 6)  /* Bit 6:  Attach Interrupt */
#define USB_INT_STALL               (1 << 7)  /* Bit 7:  Stall Interrupt */

/* Error Interrupt Status Register and Error Interrupt Enable Register (8-bit) */

#define USB_ERRSTAT_PIDERR          (1 << 0)  /* Bit 0:  This bit is set when the PID check field fails */
#define USB_ERRSTAT_CRC5EOF         (1 << 1)  /* Bit 1:  Host data CRC error or End of frame errors */
#define USB_ERRSTAT_CRC16           (1 << 2)  /* Bit 2:  Data packet is rejected due to a CRC16 error */
#define USB_ERRSTAT_DFN8            (1 << 3)  /* Bit 3:  Data field received was not 8 bits in length */
#define USB_ERRSTAT_BTOERR          (1 << 4)  /* Bit 4:  Bus turnaround timeout error occurred */
#define USB_ERRSTAT_DMAERR          (1 << 5)  /* Bit 5:  DMA error */
                                              /* Bit 6:  Reserved */
#define USB_ERRSTAT_BTSERR          (1 << 7)  /* Bit 7:  Bit stuff error is detected */

/* Status Register (8-bit) */

                                              /* Bits 0-1:  Reserved */
#define USB_STAT_ODD                (1 << 2)  /* Bit 2:  Last Buffer Descriptor was in the odd bank of the BDT */
#define USB_STAT_TX                 (1 << 3)  /* Bit 3:  Transmit Indicator */
#define USB_STAT_ENDP_SHIFT         (4)       /* Bits 4-7: Endpoint address that received or transmitted the token */
#define USB_STAT_ENDP_MASK          (15 << USB_STAT_ENDP_SHIFT)

/* Control Register (8-bit) */

#define USB_CTL_USBENSOFEN          (1 << 0)  /* Bit 0:  USB Enable */
#define USB_CTL_ODDRST              (1 << 1)  /* Bit 1:  Resets all the BDT ODD ping/pong bits to 0 */
#define USB_CTL_RESUME              (1 << 2)  /* Bit 2:  Enables the USB Module to execute resume signaling */
#define USB_CTL_HOSTMODEEN          (1 << 3)  /* Bit 3:  Enables the USB Module to operate in Host mode */
#define USB_CTL_RESET               (1 << 4)  /* Bit 4:  Enables the USB Module to generate USB reset signaling */
#define USB_CTL_TXSUSPENDTOKENBUSY  (1 << 5)  /* Bit 5:  USB Module is busy executing a USB token */
#define USB_CTL_SE0                 (1 << 6)  /* Bit 6:  Live USB Single Ended Zero signal */
#define USB_CTL_JSTATE              (1 << 7)  /* Bit 7:  Live USB differential receiver JSTATE signal */

/* Address Register (8-bit) */

#define USB_ADDR_LSEN               (1 << 7)  /* Bit 7:  Low Speed Enable bit */
#define USB_ADDR_SHIFT              (0)       /* Bits 0-6: USB address */
#define USB_ADDR_MASK               (0x7f << USB_ADDR_SHIFT)

/* BDT Page Register 1 (8-bit) */
                                              /* Bit 0:  Reserved */
#define USB_BDTPAGE1_SHIFT          (1)       /* Bits 1-7: Address bits 9-15 of the BDT base address */
#define USB_BDTPAGE1_MASK           (0x7f << USB_BDTPAGE1_SHIFT)

/* Frame Number Register Low (8-bit, bits 0-7 of the 11 bit frame number) */
/* Frame Number Register High (8-bit) */
                                              /* Bits 3-7:  Reserved */
#define USB_FRMNUMH_SHIFT           (0)       /* Bits 0-2: Bits 8-10 of the 11-bit frame number */
#define USB_FRMNUMH_MASK            (7 << USB_FRMNUMH_SHIFT)

/* Token Register (8-bit) */

#define USB_TOKEN_ENDPT_SHIFT       (0)       /* Bits 0-3: Endpoint address for the token command */
#define USB_TOKEN_ENDPT_MASK        (15 << USB_TOKEN_ENDPT_SHIFT)
#define USB_TOKEN_PID_SHIFT         (4)       /* Bits 4-7: Token type executed by the USB Module */
#define USB_TOKEN_PID_MASK          (15 << USB_TOKEN_PID_SHIFT)
#  define USB_TOKEN_PID_OUT         (1 << USB_TOKEN_PID_SHIFT)  /* OUT Token */
#  define USB_TOKEN_PID_IN          (9 << USB_TOKEN_PID_SHIFT)  /* IN Token */
#  define USB_TOKEN_PID_SETUP       (13 << USB_TOKEN_PID_SHIFT) /* SETUP Token */

/* SOF Threshold Register (8-bit count value) */
/* BDT Page Register 2/3 (16 bit address in two 8-bit registers) */

/* Endpoint n Control Register (8-bit) */

#define USB_ENDPT_EPHSHK            (1 << 0)  /* Bit 0:  Enable handshaking during a transaction to the endpoint */
#define USB_ENDPT_EPSTALL           (1 << 1)  /* Bit 1:  Endpoint is stalled */
#define USB_ENDPT_EPTXEN            (1 << 2)  /* Bit 2:  Enable the endpoint for TX transfers */
#define USB_ENDPT_EPRXEN            (1 << 3)  /* Bit 3:  Enable the endpoint for RX transfers */
#define USB_ENDPT_EPCTLDIS          (1 << 4)  /* Bit 4:  Disable control (SETUP) transfers */
                                              /* Bit 5:  Reserved */
#define USB_ENDPT_RETRYDIS          (1 << 6)  /* Bit 6:  Disable host retry NAK'ed transactions (host EP0) */
#define USB_ENDPT_HOSTWOHUB         (1 << 7)  /* Bit 7:  Allows the host to communicate to a low speed device (host EP0) */

/* USB Control Register (8-bit) */
                                              /* Bits 0-5:  Reserved */
#define USB_USBCTRL_PDE             (1 << 6)  /* Bit 6:  Enables the weak pulldowns on the USB transceiver */
#define USB_USBCTRL_SUSP            (1 << 7)  /* Bit 7:  Places the USB transceiver into the suspend state */

/* USB OTG Observe Register (8-bit) */
                                              /* Bits 0-3:  Reserved */
#define USB_OBSERVE_DMPD            (1 << 4)  /* Bit 4:  D- Pull Down signal output from the USB OTG module */
                                              /* Bit 5:  Reserved */
#define USB_OBSERVE_DPPD            (1 << 6)  /* Bit 6:  D+ Pull Down signal output from the USB OTG module */
#define USB_OBSERVE_DPPU            (1 << 7)  /* Bit 7:  D+ Pull Up signal output from the USB OTG module */

/* USB OTG Control Register (8-bit) */
                                              /* Bits 0-3:  Reserved */
#define USB_CONTROL_DPPULLUPNONOTG  (1 << 4)  /* Bit 4:  Controls of the DP PULLUP in the USB OTG module */
                                              /* Bits 5-7:  Reserved */
/* USB Transceiver Control Register 0 (8-bit) */

#define USB_USBTRC0_USBRESET        (1 << 7)  /* Bit 7:  USB reset */
                                              /* Bit 6:  Reserved */
#define USB_USBTRC0_USBRESMEN       (1 << 5)  /* Bit 5:  Asynchronous Resume Interrupt Enable */
                                              /* Bits 2-4:  Reserved */
#define USB_USBTRC0_SYNC_DET        (1 << 1)  /* Bit 1:  Synchronous USB Interrupt Detect */
#define USB_USBTRC0_RESUME_INT      (1 << 0)  /* Bit 0:  USB Asynchronous Interrupt */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_USBOTG_H */
