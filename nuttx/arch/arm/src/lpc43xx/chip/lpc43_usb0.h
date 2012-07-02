/************************************************************************************************
 * arch/arm/src/lpc43xx/lpc43_usb0.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_USB0_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_USB0_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Register Offsets *****************************************************************************/
                                                    /* 0x000 - 0x0ff: Reserved */
/* Device/host capability registers */

#define LPC43_USBOTG_CAPLENGTH_OFFSET        0x0100 /* Capability length register */
#define LPC43_USBHOST_HCSPARAMS_OFFSET       0x0104 /* Host controller structural parameters */
#define LPC43_USBHOST_HCCPARAMS_OFFSET       0x0108 /* Host controller capability parameters */
#define LPC43_USBDEV_DCIVERSION_OFFSET       0x0120 /* Device interface version number */
#define LPC43_USBDEV_DCCPARAMS_OFFSET        0x0124 /* Device controller capability parameters */

/* Device/host/OTG operational registers */

#define LPC43_USBOTG_USBCMD_OFFSET           0x0140 /* USB command (both) */
#define LPC43_USBOTG_USBSTS_OFFSET           0x0144 /* USB status (both) */
#define LPC43_USBOTG_USBINTR_OFFSET          0x0148 /* USB interrupt enable (both) */
#define LPC43_USBOTG_FRINDEX_OFFSET          0x014c /* USB frame index (both) */
#define LPC43_USBHOST_PERIODICLIST_OFFSET    0x0154 /* Frame list base address (host) */
#define LPC43_USBDEV_DEVICEADDR_OFFSET       0x0154 /* USB device address (device) */
#define LPC43_USBHOST_ASYNCLISTADDR_OFFSET   0x0158 /* Next asynchronous list address (host) */
#define LPC43_USBDEV_ENDPOINTLIST_OFFSET     0x0158 /* Address of endpoint list in memory (device) */
#define LPC43_USBHOST_TTCTRL_OFFSET          0x015c /* Asynchronous buffer status for embedded TT (host) */
#define LPC43_USBOTG_BURSTSIZE_OFFSET        0x0160 /* Programmable burst size (both) */
#define LPC43_USBHOST_TXFILLTUNING_OFFSET    0x0164 /* Host transmit pre-buffer packet tuning (host) */
#define LPC43_USBOTG_BINTERVAL_OFFSET        0x0174 /* Length of virtual frame (both) */
#define LPC43_USBDEV_ENDPTNAK_OFFSET         0x0178 /* Endpoint NAK (device) */
#define LPC43_USBDEV_ENDPTNAKEN_OFFSET       0x017c /* Endpoint NAK Enable (device) */
#define LPC43_USBOTG_PORTSC1_OFFSET          0x0184 /* Port status/control 1 (both) */
#define LPC43_USBOTG_OTGSC_OFFSET            0x01a4 /* OTG status and control (otg) */
#define LPC43_USBOTG_USBMODE_OFFSET          0x01a8 /* USB device mode (both) */

/* Device side naming of common register offsets */

#define LPC43_USBDEV_USBCMD_OFFSET           LPC43_USBOTG_USBCMD_OFFSET
#define LPC43_USBDEV_USBSTS_OFFSET           LPC43_USBOTG_USBSTS_OFFSET
#define LPC43_USBDEV_USBINTR_OFFSET          LPC43_USBOTG_USBINTR_OFFSET
#define LPC43_USBDEV_FRINDEX_OFFSET          LPC43_USBOTG_FRINDEX_OFFSET
#define LPC43_USBDEV_BURSTSIZE_OFFSET        LPC43_USBOTG_BURSTSIZE_OFFSET
#define LPC43_USBDEV_BINTERVAL_OFFSET        LPC43_USBOTG_BINTERVAL_OFFSET
#define LPC43_USBDEV_PORTSC1_OFFSET          LPC43_USBOTG_USBMODE_OFFSET
#define LPC43_USBDEV_USBMODE_OFFSET          LPC43_USBOTG_USBMODE_OFFSET

/* Host side naming of common registers */

#define LPC43_USBHOST_USBCMD_OFFSET          LPC43_USBOTG_USBCMD_OFFSET
#define LPC43_USBHOST_USBSTS_OFFSET          LPC43_USBOTG_USBSTS_OFFSET
#define LPC43_USBHOST_USBINTR_OFFSET         LPC43_USBOTG_USBINTR_OFFSET
#define LPC43_USBHOST_FRINDEX_OFFSET         LPC43_USBOTG_FRINDEX_OFFSET
#define LPC43_USBHOST_BURSTSIZE_OFFSET       LPC43_USBOTG_BURSTSIZE_OFFSET
#define LPC43_USBHOST_BINTERVAL_OFFSET       LPC43_USBOTG_BINTERVAL_OFFSET
#define LPC43_USBHOST_PORTSC1_OFFSET         LPC43_USBOTG_USBMODE_OFFSET
#define LPC43_USBHOST_USBMODE_OFFSET         LPC43_USBOTG_USBMODE_OFFSET

/* Device endpoint registers */

#define LPC43_USBDEV_ENDPTSETUPSTAT_OFFSET   0x01ac /* Endpoint setup status */
#define LPC43_USBDEV_ENDPTPRIME_OFFSET       0x01b0 /* Endpoint initialization */
#define LPC43_USBDEV_ENDPTFLUSH_OFFSET       0x01b4 /* Endpoint de-initialization */
#define LPC43_USBDEV_ENDPTSTATUS_OFFSET      0x01b8 /* Endpoint status */
#define LPC43_USBDEV_ENDPTCOMPLETE_OFFSET    0x01bc /* Endpoint complete */

#define LPC43_USBDEV_ENDPTCTRL_OFFSET(n)     (0x01c0 + ((n) << 2))
#define LPC43_USBDEV_ENDPTCTRL0_OFFSET       0x01c0 /* Endpoint control 0 */
#define LPC43_USBDEV_ENDPTCTRL1_OFFSET       0x01c4 /* Endpoint control 1 */
#define LPC43_USBDEV_ENDPTCTRL2_OFFSET       0x01c8 /* Endpoint control 2 */
#define LPC43_USBDEV_ENDPTCTRL3_OFFSET       0x01cc /* Endpoint control 3 */
#define LPC43_USBDEV_ENDPTCTRL4_OFFSET       0x01d0 /* Endpoint control 4 */
#define LPC43_USBDEV_ENDPTCTRL5_OFFSET       0x01d4 /* Endpoint control 5 */

/* USB0 register (virtual) addresses **********************************************************/

/* Device/host capability registers */

#define LPC43_USBOTG_CAPLENGTH               (LPC43_USBOTG_BASE+LPC43_USBOTG_CAPLENGTH_OFFSET)
#define LPC43_USBHOST_HCIVERSION             (LPC43_USBOTG_BASE+LPC43_USBHOST_HCIVERSION_OFFSET)
#define LPC43_USBHOST_HCSPARAMS              (LPC43_USBOTG_BASE+LPC43_USBHOST_HCSPARAMS_OFFSET)
#define LPC43_USBHOST_HCCPARAMS              (LPC43_USBOTG_BASE+LPC43_USBHOST_HCCPARAMS_OFFSET)
#define LPC43_USBDEV_DCIVERSION              (LPC43_USBOTG_BASE+LPC43_USBDEV_DCIVERSION_OFFSET)
#define LPC43_USBDEV_DCCPARAMS               (LPC43_USBOTG_BASE+LPC43_USBDEV_DCCPARAMS_OFFSET)

/* Device/host operational registers */

#define LPC43_USBOTG_USBCMD                  (LPC43_USBOTG_BASE+LPC43_USBOTG_USBCMD_OFFSET)
#define LPC43_USBOTG_USBSTS                  (LPC43_USBOTG_BASE+LPC43_USBOTG_USBSTS_OFFSET)
#define LPC43_USBOTG_USBINTR                 (LPC43_USBOTG_BASE+LPC43_USBOTG_USBINTR_OFFSET)
#define LPC43_USBOTG_FRINDEX                 (LPC43_USBOTG_BASE+LPC43_USBOTG_FRINDEX_OFFSET)
#define LPC43_USBHOST_PERIODICLIST           (LPC43_USBOTG_BASE+LPC43_USBHOST_PERIODICLIST_OFFSET)
#define LPC43_USBDEV_DEVICEADDR              (LPC43_USBOTG_BASE+LPC43_USBDEV_DEVICEADDR_OFFSET)
#define LPC43_USBHOST_ASYNCLISTADDR          (LPC43_USBOTG_BASE+LPC43_USBHOST_ASYNCLISTADDR_OFFSET)
#define LPC43_USBDEV_ENDPOINTLIST            (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPOINTLIST_OFFSET)
#define LPC43_USBHOST_TTCTRL                 (LPC43_USBOTG_BASE+LPC43_USBHOST_TTCTRL_OFFSET)
#define LPC43_USBOTG_BURSTSIZE               (LPC43_USBOTG_BASE+LPC43_USBOTG_BURSTSIZE_OFFSET)
#define LPC43_USBHOST_TXFILLTUNING           (LPC43_USBOTG_BASE+LPC43_USBHOST_TXFILLTUNING_OFFSET)
#define LPC43_USBOTG_BINTERVAL               (LPC43_USBOTG_BASE+LPC43_USBOTG_BINTERVAL_OFFSET)
#define LPC43_USBDEV_ENDPTNAK                (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTNAK_OFFSET)
#define LPC43_USBDEV_ENDPTNAKEN              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTNAKEN_OFFSET)
#define LPC43_USBOTG_PORTSC1                 (LPC43_USBOTG_BASE+LPC43_USBOTG_PORTSC1_OFFSET)
#define LPC43_USBOTG_OTGSC                   (LPC43_USBOTG_BASE+LPC43_USBOTG_OTGSC_OFFSET)
#define LPC43_USBOTG_USBMODE                 (LPC43_USBOTG_BASE+LPC43_USBOTG_USBMODE_OFFSET)

/* Device side naming of common register offsets */

#define LPC43_USBDEV_USBCMD                  LPC43_USBOTG_USBCMD       
#define LPC43_USBDEV_USBSTS                  LPC43_USBOTG_USBSTS       
#define LPC43_USBDEV_USBINTR                 LPC43_USBOTG_USBINTR       
#define LPC43_USBDEV_FRINDEX                 LPC43_USBOTG_FRINDEX       
#define LPC43_USBDEV_BURSTSIZE               LPC43_USBOTG_BURSTSIZE       
#define LPC43_USBDEV_BINTERVAL               LPC43_USBOTG_BINTERVAL       
#define LPC43_USBDEV_PORTSC1                 LPC43_USBOTG_USBMODE       
#define LPC43_USBDEV_USBMODE                 LPC43_USBOTG_USBMODE       

/* Host side naming of common registers */

#define LPC43_USBHOST_USBCMD                 LPC43_USBOTG_USBCMD       
#define LPC43_USBHOST_USBSTS                 LPC43_USBOTG_USBSTS       
#define LPC43_USBHOST_USBINTR                LPC43_USBOTG_USBINTR       
#define LPC43_USBHOST_FRINDEX                LPC43_USBOTG_FRINDEX       
#define LPC43_USBHOST_BURSTSIZE              LPC43_USBOTG_BURSTSIZE       
#define LPC43_USBHOST_BINTERVAL              LPC43_USBOTG_BINTERVAL       
#define LPC43_USBHOST_PORTSC1                LPC43_USBOTG_USBMODE       
#define LPC43_USBHOST_USBMODE                LPC43_USBOTG_USBMODE       

/* Device endpoint registers */

#define LPC43_USBDEV_ENDPTSETUPSTAT          (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTSETUPSTAT_OFFSET)
#define LPC43_USBDEV_ENDPTPRIME              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTPRIME_OFFSET)
#define LPC43_USBDEV_ENDPTFLUSH              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTFLUSH_OFFSET)
#define LPC43_USBDEV_ENDPTSTATUS             (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTSTATUS_OFFSET)
#define LPC43_USBDEV_ENDPTCOMPLETE           (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCOMPLETE_OFFSET)

#define LPC43_USBDEV_ENDPTCTRL(n)            (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL_OFFSET(n))
#define LPC43_USBDEV_ENDPTCTRL0              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL0_OFFSET)
#define LPC43_USBDEV_ENDPTCTRL1              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL1_OFFSET)
#define LPC43_USBDEV_ENDPTCTRL2              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL2_OFFSET)
#define LPC43_USBDEV_ENDPTCTRL3              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL3_OFFSET)
#define LPC43_USBDEV_ENDPTCTRL4              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL4_OFFSET)
#define LPC43_USBDEV_ENDPTCTRL5              (LPC43_USBOTG_BASE+LPC43_USBDEV_ENDPTCTRL5_OFFSET)

/* USB0 register bit definitions **************************************************************/

/* Device/host capability registers */
/* Capability length register */

#define USBOTG_CAPLENGTH_SHIFT               (0)       /* Bits 0-7: Offset from register base to operational regs */
#define USBOTG_CAPLENGTH_MASK                (0xff << USBOTG_CAPLENGTH_SHIFT)
#define USBHOST_HCIVERSION_SHIFT             (8)       /* Bits 8-23: BCD encoding of the EHCI revision number */
#define USBHOST_HCIVERSION_MASK              (0xffff << USBHOST_HCIVERSION_SHIFT)
                                                       /* Bits 24-31: Reserved */
/* Host controller structural parameters */

#define USBHOST_HCSPARAMS_NPORTS_SHIF        (0)       /* Bits 0-3: Number of downstream ports */
#define USBHOST_HCSPARAMS_NPORTS_MASK        (15 << USBHOST_HCSPARAMS_NPORTS_SHIFT)
#define USBHOST_HCSPARAMS_PPC                (1 >> 4)  /* Bit 4:  Port Power Control */
                                                       /* Bits 5-7: Reserved */
#define USBHOST_HCSPARAMS_NPCC_SHIFT         (8)       /* Bits 8-11: Number of Ports per Companion Controller */
#define USBHOST_HCSPARAMS_NPCC_MASK          (15 << USBHOST_HCSPARAMS_NPCC_SHIFT)
#define USBHOST_HCSPARAMS_NCC_SHIFT          (15)      /* Bits 12-15: Number of Companion Controller */
#define USBHOST_HCSPARAMS_NCC_MASK           (15 << USBHOST_HCSPARAMS_NCC_SHIFT)
#define USBHOST_HCSPARAMS_PI                 (1 >> 16) /* Bit 16: Port indicators */
                                                       /* Bits 17-19: Reserved */
#define USBHOST_HCSPARAMS_NPTT_SHIFT         (20)      /* Bits 20-23: Number of Ports per Transaction Translator */
#define USBHOST_HCSPARAMS_NPTT_MASK          (15 << USBHOST_HCSPARAMS_NPTT_SHIFT)
#define USBHOST_HCSPARAMS_NTT_SHIFT          (24)      /* Bits 24-27: Number of Transaction Translators */
#define USBHOST_HCSPARAMS_NTT_MASK           (15 << USBHOST_HCSPARAMS_NTT_SHIFT)
                                                       /* Bits 28-31: Reserved */
/* Host controller capability parameters */

#define USBHOST_HCCPARAMS_ADC                (1 >> 0)  /* Bit 0:  64-bit Addressing Capability */
#define USBHOST_HCCPARAMS_PFL                (1 >> 1)  /* Bit 1:  Programmable Frame List Flag */
#define USBHOST_HCCPARAMS_ASP                (1 >> 2)  /* Bit 2:  Asynchronous Schedule Park Capability */
#define USBHOST_HCCPARAMS_IST_SHIFT          (4)       /* Bits 4-7: Isochronous Scheduling Threshold */
#define USBHOST_HCCPARAMS_IST_MASK           (15 << USBHOST_HCCPARAMS_IST_SHIFT)
#define USBHOST_HCCPARAMS_EECP_SHIFT         (8)       /* Bits 8-15: EHCI Extended Capabilities Pointer */
#define USBHOST_HCCPARAMS_EECP_MASK          (255 << USBHOST_HCCPARAMS_EECP_SHIFT)
                                                       /* Bits 16-31: Reserved */
/* Device interface version number */

#define USBDEV_DCIVERSION_SHIFT              (0)       /* Bits 0-15: BCD encoding of the device interface */
#define USBDEV_DCIVERSION_MASK               (0xffff << USBDEV_DCIVERSION_SHIFT)
                                                       /* Bits 16-31: Reserved */

/* Device controller capability parameters */

#define USBDEV_DCCPARAMS_DEN_SHIFT           (0)       /* Bits 0-4: DEN Device Endpoint Number */
#define USBDEV_DCCPARAMS_DEN_MASK            (31 << USBDEV_DCCPARAMS_DEN_SHIFT)
                                                       /* Bits 5-6: Reserved */
#define USBDEV_DCCPARAMS_DC                  (1 >> 7)  /* Bit 7:  Device Capable */
#define USBDEV_DCCPARAMS_HC                  (1 >> 8)  /* Bit 8:  Host Capable */
                                                       /* Bits 9-31: Reserved */
/* Device/host operational registers */
/* USB Command register USBCMD -- Device Mode */

#define USBDEV_USBCMD_RS                     (1 << 0)  /* Bit 0:  0 Run/Stop */
#define USBDEV_USBCMD_RST                    (1 << 1)  /* Bit 1:  Controller reset */
                                                       /* Bits 2-12: Reserved OR not used in device mode */
#define USBDEV_USBCMD_SUTW                   (1 << 13) /* Bit 13: Setup trip wire */
#define USBDEV_USBCMD_ATDTW                  (1 << 14) /* Bit 14: Add dTD trip wire */
                                                       /* Bit 15: Reserved OR not used in device mode */
#define USBDEV_USBCMD_ITC_SHIFT              (16)      /* Bits 16-23: Interrupt threshold control */
#define USBDEV_USBCMD_ITC_MASK               (255 << USBDEV_USBCMD_ITC_SHIFT)
#  define USBDEV_USBCMD_ITCIMME              (0  << USBDEV_USBCMD_ITC_SHIFT) /* Immediate (no threshold) */
#  define USBDEV_USBCMD_ITC1UF               (1  << USBDEV_USBCMD_ITC_SHIFT) /* 1 micro frame */
#  define USBDEV_USBCMD_ITC2UF               (2  << USBDEV_USBCMD_ITC_SHIFT) /* 2 micro frames */
#  define USBDEV_USBCMD_ITC4UF               (4  << USBDEV_USBCMD_ITC_SHIFT) /* 4 micro frames */
#  define USBDEV_USBCMD_ITC8UF               (8  << USBDEV_USBCMD_ITC_SHIFT) /* 8 micro frames */
#  define USBDEV_USBCMD_ITC16UF              (16 << USBDEV_USBCMD_ITC_SHIFT) /* 16 micro frames */
#  define USBDEV_USBCMD_ITC32UF              (32 << USBDEV_USBCMD_ITC_SHIFT) /* 32 micro frames */
#  define USBDEV_USBCMD_ITC64UF              (64 << USBDEV_USBCMD_ITC_SHIFT) /* 64 micro frames */
                                                       /* Bits 24-31: Reserved */
/* USB Command register USBCMD -- Host Mode */

#define USBHOST_USBCMD_RS                    (1 << 0)  /* Bit 0:  Run/Stop */
#define USBHOST_USBCMD_RST                   (1 << 1)  /* Bit 1:  Controller reset */
#define USBHOST_USBCMD_FS0                   (1 << 2)  /* Bit 2:  Bit 0 of the Frame List Size bits */
#define USBHOST_USBCMD_FS1                   (1 << 3)  /* Bit 3:  Bit 1 of the Frame List Size bits */
#define USBHOST_USBCMD_PSE                   (1 << 4)  /* Bit 4:  Skips processing periodic schedule */
#define USBHOST_USBCMD_ASE                   (1 << 5)  /* Bit 5:  Skips processing asynchronous schedule */
#define USBHOST_USBCMD_IAA                   (1 << 6)  /* Bit 6:  Interrupt next asynchronous schedule */
                                                       /* Bit 7: Reserved OR not used in host mode */
#define USBHOST_USBCMD_ASP_SHIFT             (8)       /* Bits 8-9: Asynchronous schedule park mode */
#define USBHOST_USBCMD_ASP_MASK              (3 << USBHOST_USBCMD_ASP_SHIFT)
                                                       /* Bit 10: Reserved OR not used in host mode */
#define USBHOST_USBCMD_ASPE                  (1 << 11) /* Bit 11: Asynchronous Schedule Park Mode Enable */
                                                       /* Bits 12-14: Reserved OR not used in host mode */
#define USBHOST_USBCMD_FS2                   (1 << 15) /* Bit 15: Bit 2 of the Frame List Size bits */
#define USBHOST_USBCMD_ITC_SHIFT             (16)      /* Bits 16-13: Interrupt threshold control */
#define USBHOST_USBCMD_ITC_MASK              (255 << USBHOST_USBCMD_ITC_SHIFT)
#  define USBHOST_USBCMD_ITCIMMED            (0  << USBHOST_USBCMD_ITC_SHIFT) /* Immediate (no threshold) */
#  define USBHOST_USBCMD_ITC1UF              (1  << USBHOST_USBCMD_ITC_SHIFT) /* 1 micro frame */
#  define USBHOST_USBCMD_ITC2UF              (2  << USBHOST_USBCMD_ITC_SHIFT) /* 2 micro frames */
#  define USBHOST_USBCMD_ITC4UF              (4  << USBHOST_USBCMD_ITC_SHIFT) /* 4 micro frames */
#  define USBHOST_USBCMD_ITC8UF              (8  << USBHOST_USBCMD_ITC_SHIFT) /* 8 micro frames */
#  define USBHOST_USBCMD_ITC16UF             (16 << USBHOST_USBCMD_ITC_SHIFT) /* 16 micro frames */
#  define USBHOST_USBCMD_ITC32UF             (32 << USBHOST_USBCMD_ITC_SHIFT) /* 32 micro frames */
#  define USBHOST_USBCMD_ITC64UF             (64 << USBHOST_USBCMD_ITC_SHIFT) /* 64 micro frames */
                                                       /* Bits 24-31: Reserved */
/* USB Status register USBSTS -- Device Mode */

#define USBDEV_USBSTS_UI                     (1 << 0)  /* Bit 0:  USB interrupt */
#define USBDEV_USBSTS_UEI                    (1 << 1)  /* Bit 1:  USB error interrupt */
#define USBDEV_USBSTS_PCI                    (1 << 2)  /* Bit 2:  Port change detect */
                                                       /* Bits 3-5: Reserved OR not used in device mode */
#define USBDEV_USBSTS_URI                    (1 << 6)  /* Bit 6:  USB reset received */
#define USBDEV_USBSTS_SRI                    (1 << 7)  /* Bit 7:  SOF received */
#define USBDEV_USBSTS_SLI                    (1 << 8)  /* Bit 8:  DCSuspend */
                                                       /* Bits 9-15: Reserved OR not used in device mode */
#define USBDEV_USBSTS_NAKI                   (1 << 16) /* Bit 16: NAK interrupt bit */
                                                       /* Bits 17-31: Reserved OR not used in device mode */
/* USB Status register USBSTS -- Host Mode */

#define USBHOST_USBSTS_UI                    (1 << 0)  /* Bit 0:  USB interrupt */
#define USBHOST_USBSTS_UEI                   (1 << 1)  /* Bit 1:  USB error interrupt */
#define USBHOST_USBSTS_PCI                   (1 << 2)  /* Bit 2:  Port change detect */
#define USBHOST_USBSTS_FRI                   (1 << 3)  /* Bit 3:  Frame list roll-over */
                                                       /* Bit 4:  Reserved */
#define USBHOST_USBSTS_AAI                   (1 << 5)  /* Bit 5:  Interrupt on async advance */
                                                       /* Bit 6:  Not used in host mode */
#define USBHOST_USBSTS_SRI                   (1 << 7)  /* Bit 7:  SOF received */
                                                       /* Bit 8-11: Reserved OR Not used in host mode */
#define USBHOST_USBSTS_HCH                   (1 << 12) /* Bit 12: HCHalted */
#define USBHOST_USBSTS_RCL                   (1 << 13) /* Bit 13: Reclamation */
#define USBHOST_USBSTS_PS                    (1 << 14) /* Bit 14: Periodic schedule status */
#define USBHOST_USBSTS_AS                    (1 << 15) /* Bit 15: Asynchronous schedule status */
                                                       /* Bit 16-17: Reserved OR Not used in host mode */
#define USBHOST_USBSTS_UAI                   (1 << 18) /* Bit 18: USB host asynchronous interrupt */
#define USBHOST_USBSTS_UPI                   (1 << 19) /* Bit 19: USB host periodic interrupt */
                                                       /* Bits 20-31: Reserved */
/* USB interrupt register USBINTR -- Device Mode */

#define USBDEV_USBINTR_UE                    (1 << 0)  /* Bit 0:  USB interrupt enable */
#define USBDEV_USBINTR_UEE                   (1 << 1)  /* Bit 1:  USB error interrupt enable */
#define USBDEV_USBINTR_PCE                   (1 << 2)  /* Bit 2:  Port change detect enable */
                                                       /* Bits 3-5: Reserved OR Not used in device mode */
#define USBDEV_USBINTR_URE                   (1 << 6)  /* Bit 6:  USB reset enable */
#define USBDEV_USBINTR_SRE                   (1 << 7)  /* Bit 7:  SOF received enable */
#define USBDEV_USBINTR_SLE                   (1 << 8)  /* Bit 8:  Sleep enable */
                                                       /* Bits 8-15: Reserved */
#define USBDEV_USBINTR_NAKE                  (1 << 16) /* Bit 16: NAK interrupt enable */
                                                       /* Bits 17-31: Reserved OR Not used in device mode */
/* USB interrupt register USBINTR -- Host Mode */

#define USBHOST_USBINTR_UE                   (1 << 0)  /* Bit 0:  USB interrupt enable */
#define USBHOST_USBINTR_UEE                  (1 << 1)  /* Bit 1:  USB error interrupt enable */
#define USBHOST_USBINTR_PCE                  (1 << 2)  /* Bit 2:  Port change detect enable */
#define USBHOST_USBINTR_FRE                  (1 << 3)  /* Bit 3:  Frame list rollover enable */
                                                       /* Bit 4:  Reserved */
#define USBHOST_USBINTR_AAE                  (1 << 5)  /* Bit 5:  Interrupt on asynchronous advance enable */
                                                       /* Bit 6:  Not used in host mode */
#define USBHOST_USBINTR_SRE                  (1 << 7)  /* Bit 7:  SOF timer interrupt enable */
                                                       /* Bits 8-17: Reserved OR Not used in host mode */
#define USBHOST_USBINTR_UAIE                 (1 << 18) /* Bit 18: USB host asynchronous interrupt enable */
#define USBHOST_USBINTR_UPIA                 (1 << 19) /* Bit 19: USB host periodic interrupt enable */
                                                       /* Bits 20-31:  Reserved */
/* Frame index register FRINDEX -- Device Mode */

#define USBDEV_FRINDEX_CUFN_SHIFT            (0)       /* Bits 0-2: Current micro frame number */
#define USBDEV_FRINDEX_CUFN_MASK             (7 << USBDEV_FRINDEX_CUFN_SHIFT)
#define USBDEV_FRINDEX_LFN_SHIFT             (3)       /* Bits 3-13: Frame number of last frame transmitted */
#define USBDEV_FRINDEX_LFN_MASK              (0x7ff << USBDEV_FRINDEX_LFN_SHIFT)
                                                       /* Bits 14-31:  Reserved */
/* Frame index register FRINDEX -- Host Mode */

#define USBHOST_FRINDEX_CUFN_SHIFT           (0)       /* Bits 0-2: Current micro frame number */
#define USBHOST_FRINDEX_CUFN_MASK            (7 << USBHOST_FRINDEX_CUFN_SHIFT)
#define USBHOST_FRINDEX_FLI_SHIFT            (3)       /* Bits 3-12: Frame list current index */
#define USBHOST_FRINDEX_FLI_MASK             (0x3ff << USBHOST_FRINDEX_FLI_SHIFT)
                                                       /* Bits 13-31:  Reserved */
/* USB Device Address register DEVICEADDR -- Device Mode */
                                                       /* Bits 0-23:  Reserved */
#define USBDEV_DEVICEADDR_USBADRA            (1 << 24) /* Bit 24: Device address advance */
#define USBDEV_DEVICEADDR_SHIFT              (25)      /* Bits 25-31: USBADR USB device address */
#define USBDEV_DEVICEADDR_MASK               (0x3f << USBDEV_DEVICEADDR_SHIFT)

/* USB Periodic List Base register PERIODICLIST -- Host Mode */
                                                       /* Bits 0-11:  Reserved */
#define USBHOST_PERIODICLIST_PERBASE_SHIFT   (12)      /* Bits 12-31: Base Address (Low) */
#define USBHOST_PERIODICLIST_PERBASE_MASK    (0x000fffff << USBHOST_PERIODICLIST_PERBASE_SHIFT)

/* USB Endpoint List Address register ENDPOINTLISTADDR -- Device Mode */
                                                       /* Bits 0-10:  Reserved */
#define USBDEV_ENDPOINTLIST_EPBASE_SHIFT     (11)      /* Bits 11-31: Endpoint list pointer (low) */
#define USBDEV_ENDPOINTLIST_EPBASE_MASK      (0x001fffff << USBDEV_ENDPOINTLIST_EPBASE_SHIFT)

/* USB Asynchronous List Address register ASYNCLISTADDR -- Host Mode */
                                                       /* Bits 0-4:  Reserved */
#define USBHOST_ASYNCLISTADDR_ASYBASE_SHIFT  (5)       /* Bits 5-31: Link pointer (Low) LPL */
#define USBHOST_ASYNCLISTADDR_ASYBASE_MASK   (0x07ffffff << USBHOST_ASYNCLISTADDR_ASYBASE_SHIFT)

/* USB TT Control register TTCTRL -- Host Mode */
                                                       /* Bits 0-23:  Reserved */
#define USBHOST_TTCTRL_TTHA_SHIFT            (24)      /* Bits 24-30: Hub address */
#define USBHOST_TTCTRL_TTHA_MASK             (0x7f << USBHOST_TTCTRL_TTHA_SHIFT)

/* USB burst size register BURSTSIZE -- Device/Host Mode */

#define USBHOST_BURSTSIZE_RXPBURST_SHIFT     (0)       /* Bits 0-7: RXPBURST Programmable RX burst length */
#define USBHOST_BURSTSIZE_RXPBURST_MASK      (255 << USBHOST_BURSTSIZE_RXPBURST_SHIFT) 
#define USBHOST_BURSTSIZE_TXPBURST_SHIFT     (8)       /* Bits 8-15: Programmable TX burst length */
#define USBHOST_BURSTSIZE_TXPBURST_MASK      (255 << USBHOST_BURSTSIZE_TXPBURST_SHIFT)
                                                       /* Bits 16-31: Reserved */

#define USBDEV_BURSTSIZE_RXPBURST_SHIFT      (0)       /* Bits 0-7: RXPBURST Programmable RX burst length */
#define USBDEV_BURSTSIZE_RXPBURST_MASK       (255 << USBDEV_BURSTSIZE_RXPBURST_SHIFT) 
#define USBDEV_BURSTSIZE_TXPBURST_SHIFT      (8)       /* Bits 8-15: Programmable TX burst length */
#define USBDEV_BURSTSIZE_TXPBURST_MASK       (255 << USBDEV_BURSTSIZE_TXPBURST_SHIFT)
                                                       /* Bits 16-31: Reserved */
/* USB Transfer buffer Fill Tuning register TXFIFOFILLTUNING -- Host Mode */

#define USBHOST_TXFILLTUNING_SCHOH_SHIFT     (0)       /* Bits 0-7: FIFO burst threshold */
#define USBHOST_TXFILLTUNING_SCHOH_MASK      (0xff << USBHOST_TXFILLTUNING_SCHOH_SHIFT)
#define USBHOST_TXFILLTUNING_SCHEATLTH_SHIFT (8)       /* Bits 8-12: Scheduler health counter */
#define USBHOST_TXFILLTUNING_SCHEATLTH_MASK  (0x1f << USBHOST_TXFILLTUNING_SCHEATLTH_SHIFT)
#define USBHOST_TXFILLTUNING_FIFOTHRES_SHIFT (16)      /* Bits 16-21: Scheduler overhead */
#define USBHOST_TXFILLTUNING_FIFOTHRES_MASK  (0x3f << USBHOST_TXFILLTUNING_FIFOTHRES_SHIFT)
                                                       /* Bits 22-31: Reserved */
/* USB BINTERVAL register BINTERVAL -- Device/Host Mode */

#define USBDEV_BINTERVAL_SHIFT               (0)       /* Bits 0-3: bInterval value */
#define USBDEV_BINTERVAL_MASK                (15 << USBDEV_BINTERVAL_SHIFT)
                                                       /* Bits 4-31: Reserved */

#define USBHOST_BINTERVAL_SHIFT              (0)       /* Bits 0-3: bInterval value */
#define USBHOST_BINTERVAL_MASK               (15 << USBHOST_BINTERVAL_SHIFT)
                                                       /* Bits 4-31: Reserved */
/* USB endpoint NAK register ENDPTNAK -- Device Mode */

#define USBDEV_ENDPTNAK_EPRN_SHIFT           (0)       /* Bits 0-5: Rx endpoint NAK */
#define USBDEV_ENDPTNAK_EPRN_MASK            (0x3f << USBDEV_ENDPTNAK_EPRN_SHIFT)
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTNAK_EPTN_SHIFT           (16)      /* Bits 16-21: Tx endpoint NAK */
#define USBDEV_ENDPTNAK_EPTN_MASK            (0x3f << USBDEV_ENDPTNAK_EPTN_SHIFT)
                                                       /* Bits 22-31: Reserved */
/* USB Endpoint NAK Enable register ENDPTNAKEN -- Device Mode */

#define USBDEV_ENDPTNAK_EPRNE_SHIFT          (0)       /* Bits 0-5: Rx endpoint NAK enable */
#define USBDEV_ENDPTNAK_EPRNE_MASK           (0x3f << USBDEV_ENDPTNAK_EPRNE_SHIFT)
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTNAK_EPTNE_SHIFT          (16)      /* Bits 16-21: Tx endpoint NAK enable */
#define USBDEV_ENDPTNAK_EPTNE_MASK           (0x3f << USBDEV_ENDPTNAK_EPTNE_SHIFT)
                                                       /* Bits 22-31: Reserved */
/* Port Status and Control register PRTSC1 -- Device Mode */

#define USBDEV_PRTSC1_CCS                    (1 << 0)  /* Bit 0:  Current connect status */
                                                       /* Bit 1:  Not used in device mode */
#define USBDEV_PRTSC1_PE                     (1 << 2)  /* Bit 2:  Port enable */
#define USBDEV_PRTSC1_PEC                    (1 << 3)  /* Bit 3:  Port enable/disable change */
#define USBDEV_PRTSC1_FPR                    (1 << 6)  /* Bit 6:  Force port resume */
#define USBDEV_PRTSC1_SUSP                   (1 << 7)  /* Bit 7:  Suspend */
#define USBDEV_PRTSC1_PR                     (1 << 8)  /* Bit 8:  Port reset */
#define USBDEV_PRTSC1_HSP                    (1 << 9)  /* Bit 9:  High-speed status */
                                                       /* Bits 10-13: Reserved OR not used in device mode */
#define USBDEV_PRTSC1_PIC_SHIFT              (14)      /* Bits 14-15: Port indicator control */
#define USBDEV_PRTSC1_PIC_MASK               (3 << USBDEV_PRTSC1_PIC_SHIFT)
#  define USBDEV_PRTSC1_PIC_OFF              (0 << USBDEV_PRTSC1_PIC_SHIFT) /* 00 Port indicators are off */
#  define USBDEV_PRTSC1_PIC_AMBER            (1 << USBDEV_PRTSC1_PIC_SHIFT) /* 01 amber */
#  define USBDEV_PRTSC1_PIC_GREEN            (2 << USBDEV_PRTSC1_PIC_SHIFT) /* 10 green */
#define USBDEV_PRTSC1_PTC_SHIFT              (16)      /* Bits 16-19: 19: Port test control */
#define USBDEV_PRTSC1_PTC_MASK               (15 << USBDEV_PRTSC1_PTC_SHIFT)
#  define USBDEV_PRTSC1_PTC_DISABLE          (0 << USBDEV_PRTSC1_PTC_SHIFT) /* TEST_MODE_DISABLE */
#  define USBDEV_PRTSC1_PTC_JSTATE           (1 << USBDEV_PRTSC1_PTC_SHIFT) /* J_STATE */
#  define USBDEV_PRTSC1_PTC_KSTATE           (2 << USBDEV_PRTSC1_PTC_SHIFT) /* K_STATE */
#  define USBDEV_PRTSC1_PTC_SE0              (3 << USBDEV_PRTSC1_PTC_SHIFT) /* SE0 (host)/NAK (device) */
#  define USBDEV_PRTSC1_PTC_PACKET           (4 << USBDEV_PRTSC1_PTC_SHIFT) /* Packet */
#  define USBDEV_PRTSC1_PTC_HS               (5 << USBDEV_PRTSC1_PTC_SHIFT) /* FORCE_ENABLE_HS */
#  define USBDEV_PRTSC1_PTC_FS               (6 << USBDEV_PRTSC1_PTC_SHIFT) /* FORCE_ENABLE_FS */
                                                       /* Bits 20-22:  Not used in device mode */
#define USBDEV_PRTSC1_PHCD                   (1 << 23) /* Bit 23: PHY low power suspend - clock disable (PLPSCD) */
#  define USBDEV_PRTSC1_PFSC                 (1 << 24) /* Bit 24: Port force full speed connect */
                                                       /* Bit 25: Reserved */
#define USBDEV_PRTSC1_PSPD_SHIFT             (26)      /* Bits 26-27: Port speed */
#define USBDEV_PRTSC1_PSPD_MASK              (3 << USBDEV_PRTSC1_PSPD_SHIFT)
#  define USBDEV_PRTSC1_PSPD_FS              (0 << USBDEV_PRTSC1_PSPD_SHIFT) /* Full-speed */
#  define USBDEV_PRTSC1_PSPD_HS              (2 << USBDEV_PRTSC1_PSPD_SHIFT) /* High-speed */
                                                       /* Bits 28-31: Reserved */
/* Port Status and Control register PRTSC1 -- Host Mode */

#define USBHOST_PRTSC1_CCS                   (1 << 0)  /* Bit 0:  Current connect status */
#define USBHOST_PRTSC1_CSC                   (1 << 1)  /* Bit 1:  Connect status change */
#define USBHOST_PRTSC1_PE                    (1 << 2)  /* Bit 2:  Port enable */
#define USBHOST_PRTSC1_PEC                   (1 << 3)  /* Bit 3:  Port disable/enable change */
#define USBHOST_PRTSC1_OCA                   (1 << 4)  /* Bit 4:  Over-current active */
#define USBHOST_PRTSC1_OCC                   (1 << 5)  /* Bit 5:  Over-current change */
#define USBHOST_PRTSC1_FPR                   (1 << 6)  /* Bit 6:  Force port resume */
#define USBHOST_PRTSC1_SUSP                  (1 << 7)  /* Bit 7:  Suspend */
#define USBHOST_PRTSC1_PR                    (1 << 8)  /* Bit 8:  Port reset */
#define USBHOST_PRTSC1_HSP                   (1 << 9)  /* Bit 9:  High-speed status */
#define USBHOST_PRTSC1_LS_SHIFT              (10)      /* Bits 10-11: Line status */
#define USBHOST_PRTSC1_LS_MASK               (3 << USBHOST_PRTSC1_LS_SHIFT)
#  define USBHOST_PRTSC1_LS_SE0              (0 << USBHOST_PRTSC1_LS_SHIFT) /* SE0 (USB_DP and USB_DM LOW) */
#  define USBHOST_PRTSC1_LS_JSTATE           (2 << USBHOST_PRTSC1_LS_SHIFT) /* J-state (USB_DP HIGH and USB_DM LOW) */
#  define USBHOST_PRTSC1_LS_KSTATE           (1 << USBHOST_PRTSC1_LS_SHIFT) /* K-state (USB_DP LOW and USB_DM HIGH) */
#define USBHOST_PRTSC1_PP                    (1 << 12) /* Bit 12: Port power control */
                                                       /* Bit 13: Reserved */
#define USBHOST_PRTSC1_PIC_SHIFT             (14)      /* Bits 14-15: Port indicator control */
#define USBHOST_PRTSC1_PIC_MASK              (3 << USBHOST_PRTSC1_PIC_SHIFT)
#  define USBHOST_PRTSC1_PIC_OFF             (0 << USBHOST_PRTSC1_PIC_SHIFT) /* 00 Port indicators are off */
#  define USBHOST_PRTSC1_PIC_AMBER           (1 << USBHOST_PRTSC1_PIC_SHIFT) /* 01 Amber */
#  define USBHOST_PRTSC1_PIC_GREEN           (2 << USBHOST_PRTSC1_PIC_SHIFT) /* 10 Green */
#define USBHOST_PRTSC1_PTC_SHIFT             (16)      /* Bits 16-19: Port test control */
#define USBHOST_PRTSC1_PTC_MASK              (15 << USBHOST_PRTSC1_PTC_SHIFT)
#  define USBHOST_PRTSC1_PTC_DISABLE         (0 << USBHOST_PRTSC1_PTC_SHIFT) /* 0000 TEST_MODE_DISABLE */
#  define USBHOST_PRTSC1_PTC_JSTATE          (1 << USBHOST_PRTSC1_PTC_SHIFT) /* 0001 J_STATE */
#  define USBHOST_PRTSC1_PTC_KSTATE          (2 << USBHOST_PRTSC1_PTC_SHIFT) /* 0010 K_STATE */
#  define USBHOST_PRTSC1_PTC_SE0             (3 << USBHOST_PRTSC1_PTC_SHIFT) /* 0011 SE0 (host)/NAK (device) */
#  define USBHOST_PRTSC1_PTC_PACKET          (4 << USBHOST_PRTSC1_PTC_SHIFT) /* 0100 Packet */
#  define USBHOST_PRTSC1_PTC_HS              (5 << USBHOST_PRTSC1_PTC_SHIFT) /* 0101 FORCE_ENABLE_HS */
#  define USBHOST_PRTSC1_PTC_FS              (6 << USBHOST_PRTSC1_PTC_SHIFT) /* 0110 FORCE_ENABLE_FS */
#  define USBHOST_PRTSC1_PTC_LS              (7 << USBHOST_PRTSC1_PTC_SHIFT) /* 0111 FORCE_ENABLE_LS */
#define USBHOST_PRTSC1_WKCN                  (1 << 20) /* Bit 20: Wake on connect enable (WKCNNT_E) */
#define USBHOST_PRTSC1_WKDC                  (1 << 21) /* Bit 21: Wake on disconnect enable (WKDSCNNT_E) */
#define USBHOST_PRTSC1_WKOC                  (1 << 22) /* Bit 22: Wake on over-current enable (WKOC_E) */
#define USBHOST_PRTSC1_PHCD                  (1 << 23) /* Bit 23: PHY low power suspend - clock disable (PLPSCD) */
#define USBHOST_PRTSC1_PFSC                  (1 << 24) /* Bit 24: Port force full speed connect */
                                                       /* Bit 25: Reserved */
#define USBHOST_PRTSC1_PSPD_SHIFT            (26)      /* Bits 26-27: Port speed */
#define USBHOST_PRTSC1_PSPD_MASK             (3 << USBHOST_PRTSC1_PSPD_SHIFT)
#  define USBHOST_PRTSC1_PSPD_FS             (0 << USBHOST_PRTSC1_PSPD_SHIFT) /* Full-speed */
#  define USBHOST_PRTSC1_PSPD_LS             (1 << USBHOST_PRTSC1_PSPD_SHIFT) /* Low-speed */
#  define USBHOST_PRTSC1_PSPD_HS             (2 << USBHOST_PRTSC1_PSPD_SHIFT) /* High-speed */
                                                       /* Bits 28-31: Reserved */
/* OTG Status and Control register */
/* OTG controls */

#define USBOTG_OTGSC_VD                      (1 << 0)  /* Bit 0:  VBUS_Discharge */
#define USBOTG_OTGSC_VC                      (1 << 1)  /* Bit 1:  VBUS_Charge */
#define USBOTG_OTGSC_HAAR                    (1 << 2)  /* Bit 2:  Hardware assist auto_reset */
#define USBOTG_OTGSC_OT                      (1 << 3)  /* Bit 3:  OTG termination */
#define USBOTG_OTGSC_DP                      (1 << 4)  /* Bit 4:  Data pulsing */
#define USBOTG_OTGSC_IDPU                    (1 << 5)  /* Bit 5:  ID pull-up */
#define USBOTG_OTGSC_HADP                    (1 << 6)  /* Bit 6:  Hardware assist data pulse */
#define USBOTG_OTGSC_HABA                    (1 << 7)  /* Bit 7:  Hardware assist B-disconnect to A-connect */

/* OTG status inputs */

#define USBOTG_OTGSC_ID                      (1 << 8)  /* Bit 8:  USB ID */
#define USBOTG_OTGSC_AVV                     (1 << 9)  /* Bit 9:  A-VBUS valid */
#define USBOTG_OTGSC_ASV                     (1 << 10) /* Bit 10: A-session valid */
#define USBOTG_OTGSC_BSV                     (1 << 11) /* Bit 11: B-session valid */
#define USBOTG_OTGSC_BSE                     (1 << 12) /* Bit 12: B-session end */
#define USBOTG_OTGSC_1MST                    (1 << 13) /* Bit 13: 1 millisecond timer toggle */
#define USBOTG_OTGSC_DPS                     (1 << 14) /* Bit 14: Data bus pulsing status */
                                                       /* Bit 15: Reserved *.
/* OTG interrupt status */

#define USBOTG_OTGSC_IDIS                    (1 << 16) /* Bit 16: USB ID interrupt status */
#define USBOTG_OTGSC_AVVIS                   (1 << 17) /* Bit 17: A-VBUS valid interrupt status */
#define USBOTG_OTGSC_ASVIS                   (1 << 18) /* Bit 18: A-Session valid interrupt status */
#define USBOTG_OTGSC_BSVIS                   (1 << 19) /* Bit 19: B-Session valid interrupt status */
#define USBOTG_OTGSC_BSEIS                   (1 << 20) /* Bit 20: B-Session end interrupt status */
#define USBOTG_OTGSC_MS1S                    (1 << 21) /* Bit 21: 1 millisecond timer interrupt status */
#define USBOTG_OTGSC_DPIS                    (1 << 22) /* Bit 22: Data pulse interrupt status */
                                                       /* Bit 23: Reserved */
/* OTG interrupt enable */

#define USBOTG_OTGSC_IDIE                    (1 << 24) /* Bit 24: USB ID interrupt enable */
#define USBOTG_OTGSC_AVVIE                   (1 << 25) /* Bit 25: A-VBUS valid interrupt enable */
#define USBOTG_OTGSC_ASVIE                   (1 << 26) /* Bit 26: A-session valid interrupt enable */
#define USBOTG_OTGSC_BSVIE                   (1 << 27) /* Bit 27: B-session valid interrupt enable */
#define USBOTG_OTGSC_BSEIE                   (1 << 28) /* Bit 28: B-session end interrupt enable */
#define USBOTG_OTGSC_MS1E                    (1 << 29) /* Bit 29: 1 millisecond timer interrupt enable */
#define USBOTG_OTGSC_DPIE                    (1 << 30) /* Bit 30: Data pulse interrupt enable */
                                                       /* Bit 31: Reserved */
/* USB Mode register USBMODE -- Device Mode */

#define USBDEV_USBMODE_CM_SHIFT              (0)       /* Bits 0-1: Controller mode */
#define USBDEV_USBMODE_CM_MASK               (3 << USBDEV_USBMODE_CM_SHIFT)
#  define USBDEV_USBMODE_CM_IDLE             (0 << USBDEV_USBMODE_CM_SHIFT) /* Idle */
#  define USBDEV_USBMODE_CM_DEVICE           (2 << USBDEV_USBMODE_CM_SHIFT) /* Device controller */
#  define USBDEV_USBMODE_CM_HOST             (3 << USBDEV_USBMODE_CM_SHIFT) /* Host controller */
#define USBDEV_USBMODE_ES                    (1 << 2)  /* Bit 2:  Endian select */
#define USBDEV_USBMODE_SLOM                  (1 << 3)  /* Bit 3:  Setup Lockout mode */
#define USBDEV_USBMODE_SDIS                  (1 << 4)  /* Bit 4:  Stream disable mode */
                                                       /* Bits 5-31:  Reserved OR not used in device mode */

/* USB Mode register USBMODE -- Host Mode */

#define USBHOST_USBMODE_CM_SHIFT             (0)       /* Bits 0-1: Controller mode */
#define USBHOST_USBMODE_CM_MASK              (3 << USBHOST_USBMODE_CM_SHIFT)
#  define USBHOST_USBMODE_CMIDLE             (0 << USBHOST_USBMODE_CM_SHIFT) /* Idle */
#  define USBHOST_USBMODE_CMDEVICE           (2 << USBHOST_USBMODE_CM_SHIFT) /* Device controller */
#  define USBHOST_USBMODE_CMHOST             (3 << USBHOST_USBMODE_CM_SHIFT) /* Host controller */
#define USBHOST_USBMODE_ES                   (1 << 2)  /* Bit 2:  Endian select */
                                                       /* Bit 3:  Not used in host mode */
#define USBHOST_USBMODE_SDIS                 (1 << 4)  /* Bit 4:  Stream disable mode */
#define USBHOST_USBMODE_VBPS                 (1 << 5)  /* Bit 5:  VBUS power select */
                                                       /* Bits 6-31: Reserved */
/* Device endpoint registers */

/* USB Endpoint Setup Status register ENDPTSETUPSTAT */

#define USBDEV_ENDPTSETSTAT_STAT(n)          (1 << (n))
#define USBDEV_ENDPTSETSTAT_STAT0            (1 << 0)  /* Bit 0:  Setup EP status for logical EP 0 */
#define USBDEV_ENDPTSETSTAT_STAT1            (1 << 1)  /* Bit 1:  Setup EP status for logical EP 1 */
#define USBDEV_ENDPTSETSTAT_STAT2            (1 << 2)  /* Bit 2:  Setup EP status for logical EP 2 */
#define USBDEV_ENDPTSETSTAT_STAT3            (1 << 3)  /* Bit 3:  Setup EP status for logical EP 3 */
#define USBDEV_ENDPTSETSTAT_STAT4            (1 << 4)  /* Bit 4:  Setup EP status for logical EP 4 */
#define USBDEV_ENDPTSETSTAT_STAT5            (1 << 5)  /* Bit 5:  Setup EP status for logical EP 5 */
                                                       /* Bits 6-31: Reserved */
/* USB Endpoint Prime register ENDPTPRIME */

#define USBDEV_ENDPTPRIM_PERB(n)             (1 << (n))
#define USBDEV_ENDPTPRIM_PERB0               (1 << 0)  /* Bit 0:  Prime EP recv buffer for physical OUT EP 0 */
#define USBDEV_ENDPTPRIM_PERB1               (1 << 1)  /* Bit 1:  Prime EP recv buffer for physical OUT EP 1 */
#define USBDEV_ENDPTPRIM_PERB2               (1 << 2)  /* Bit 2:  Prime EP recv buffer for physical OUT EP 2 */
#define USBDEV_ENDPTPRIM_PERB3               (1 << 3)  /* Bit 3:  Prime EP recv buffer for physical OUT EP 3 */
#define USBDEV_ENDPTPRIM_PERB4               (1 << 4)  /* Bit 4:  Prime EP recv buffer for physical OUT EP 4 */
#define USBDEV_ENDPTPRIM_PERB5               (1 << 5)  /* Bit 5:  Prime EP recv buffer for physical OUT EP 5 */
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTPRIM_PETB(n)             (1 << ((n) + 16))
#define USBDEV_ENDPTPRIM_PETB0               (1 << 16) /* Bit 16: Prime EP xmt buffer for physical IN EP 0 */
#define USBDEV_ENDPTPRIM_PETB1               (1 << 17) /* Bit 17: Prime EP xmt buffer for physical IN EP 1 */
#define USBDEV_ENDPTPRIM_PETB2               (1 << 18) /* Bit 18: Prime EP xmt buffer for physical IN EP 2 */
#define USBDEV_ENDPTPRIM_PETB3               (1 << 19) /* Bit 19: Prime EP xmt buffer for physical IN EP 3 */
#define USBDEV_ENDPTPRIM_PETB4               (1 << 20) /* Bit 20: Prime EP xmt buffer for physical IN EP 4 */
#define USBDEV_ENDPTPRIM_PETB5               (1 << 21) /* Bit 21: Prime EP xmt buffer for physical IN EP 5 */
                                                       /* Bits 22-31: Reserved */
/* USB Endpoint Flush register ENDPTFLUSH */

#define USBDEV_ENDPTFLUSH_FERB(n)            (1 << (n))
#define USBDEV_ENDPTFLUSH_FERB0              (1 << 0)  /* Bit 0:  Flush EP recv buffer for physical OUT EP 0 */
#define USBDEV_ENDPTFLUSH_FERB1              (1 << 1)  /* Bit 1:  Flush EP recv buffer for physical OUT EP 1 */
#define USBDEV_ENDPTFLUSH_FERB2              (1 << 2)  /* Bit 2:  Flush EP recv buffer for physical OUT EP 2 */
#define USBDEV_ENDPTFLUSH_FERB3              (1 << 3)  /* Bit 3:  Flush EP recv buffer for physical OUT EP 3 */
#define USBDEV_ENDPTFLUSH_FERB4              (1 << 4)  /* Bit 4:  Flush EP recv buffer for physical OUT EP 4 */
#define USBDEV_ENDPTFLUSH_FERB5              (1 << 5)  /* Bit 5:  Flush EP recv buffer for physical OUT EP 5 */
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTFLUSH_FETB(n)            (1 << ((n) + 16))
#define USBDEV_ENDPTFLUSH_FETB0              (1 << 16) /* Bit 16: Flush EP xmt buffer for physical IN EP 0 */
#define USBDEV_ENDPTFLUSH_FETB1              (1 << 17) /* Bit 17: Flush EP xmt buffer for physical IN EP 1 */
#define USBDEV_ENDPTFLUSH_FETB2              (1 << 18) /* Bit 18: Flush EP xmt buffer for physical IN EP 2 */
#define USBDEV_ENDPTFLUSH_FETB3              (1 << 19) /* Bit 19: Flush EP xmt buffer for physical IN EP 3 */
#define USBDEV_ENDPTFLUSH_FETB4              (1 << 20) /* Bit 20: Flush EP xmt buffer for physical IN EP 4 */
#define USBDEV_ENDPTFLUSH_FETB5              (1 << 21) /* Bit 21: Flush EP xmt buffer for physical IN EP 5 */
                                                       /* Bits 22-31: Reserved */
/* USB Endpoint Status register ENDPTSTATUS */

#define USBDEV_ENDPTSTATUS_ERBR(n)           (1 << (n))
#define USBDEV_ENDPTSTATUS_ERBR0             (1 << 0)  /* Bit 0:  EP recv buffer ready for physical OUT EP 0 */
#define USBDEV_ENDPTSTATUS_ERBR1             (1 << 1)  /* Bit 1:  EP recv buffer ready for physical OUT EP 1 */
#define USBDEV_ENDPTSTATUS_ERBR2             (1 << 2)  /* Bit 2:  EP recv buffer ready for physical OUT EP 2 */
#define USBDEV_ENDPTSTATUS_ERBR3             (1 << 3)  /* Bit 3:  EP recv buffer ready for physical OUT EP 3 */
#define USBDEV_ENDPTSTATUS_ERBR4             (1 << 4)  /* Bit 4:  EP recv buffer ready for physical OUT EP 4 */
#define USBDEV_ENDPTSTATUS_ERBR5             (1 << 5)  /* Bit 5:  EP recv buffer ready for physical OUT EP 5 */
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTSTATUS_ETBR(n)           (1 << ((n) + 16))
#define USBDEV_ENDPTSTATUS_ETBR0             (1 << 16) /* Bit 16: EP xmt buffer ready for physical IN EP 0 */
#define USBDEV_ENDPTSTATUS_ETBR1             (1 << 17) /* Bit 17: EP xmt buffer ready for physical IN EP 1 */
#define USBDEV_ENDPTSTATUS_ETBR2             (1 << 18) /* Bit 18: EP xmt buffer ready for physical IN EP 2 */
#define USBDEV_ENDPTSTATUS_ETBR3             (1 << 19) /* Bit 19: EP xmt buffer ready for physical IN EP 3 */
#define USBDEV_ENDPTSTATUS_ETBR4             (1 << 20) /* Bit 20: EP xmt buffer ready for physical IN EP 4 */
#define USBDEV_ENDPTSTATUS_ETBR5             (1 << 21) /* Bit 21: EP xmt buffer ready for physical IN EP 5 */
                                                       /* Bits 22-31: Reserved */
/* USB Endpoint Complete register ENDPTCOMPLETE */

#define USBDEV_ENDPTCOMPLETE_ERCE(n)         (1 << (n))
#define USBDEV_ENDPTCOMPLETE_ERCE0           (1 << 0)  /* Bit 0:  EP recv complete event for physical OUT EP 0 */
#define USBDEV_ENDPTCOMPLETE_ERCE1           (1 << 1)  /* Bit 1:  EP recv complete event for physical OUT EP 1 */
#define USBDEV_ENDPTCOMPLETE_ERCE2           (1 << 2)  /* Bit 2:  EP recv complete event for physical OUT EP 2 */
#define USBDEV_ENDPTCOMPLETE_ERCE3           (1 << 3)  /* Bit 3:  EP recv complete event for physical OUT EP 3 */
#define USBDEV_ENDPTCOMPLETE_ERCE4           (1 << 4)  /* Bit 4:  EP recv complete event for physical OUT EP 4 */
#define USBDEV_ENDPTCOMPLETE_ERCE5           (1 << 5)  /* Bit 4:  EP recv complete event for physical OUT EP 5 */
                                                       /* Bits 6-15: Reserved */
#define USBDEV_ENDPTCOMPLETE_ETCE(n)         (1 << ((n) + 16))
#define USBDEV_ENDPTCOMPLETE_ETCE0           (1 << 16) /* Bit 16: EP xmt complete event for physical IN EP 0 */
#define USBDEV_ENDPTCOMPLETE_ETCE1           (1 << 17) /* Bit 17: EP xmt complete event for physical IN EP 1 */
#define USBDEV_ENDPTCOMPLETE_ETCE2           (1 << 18) /* Bit 18: EP xmt complete event for physical IN EP 2 */
#define USBDEV_ENDPTCOMPLETE_ETCE3           (1 << 19) /* Bit 19: EP xmt complete event for physical IN EP 3 */
#define USBDEV_ENDPTCOMPLETE_ETCE4           (1 << 20) /* Bit 20: EP xmt complete event for physical IN EP 4 */
#define USBDEV_ENDPTCOMPLETE_ETCE5           (1 << 21) /* Bit 21: EP xmt complete event for physical IN EP 5 */
                                                       /* Bits 22-31: Reserved */
/* USB Endpoint 0 Control register ENDPTCTRL0 */

#define USBDEV_ENDPTCTRL0_RXS                (1 << 0)  /* Bit 0:  Rx endpoint stall */
                                                       /* Bit 1:  Reserved */
#define USBDEV_ENDPTCTRL0_RXT_SHIFT          (2)       /* Bits 2-3: Endpoint type */
#define USBDEV_ENDPTCTR0L_RXT_MASK           (3 << USBDEV_ENDPTCTRL0_RXT_SHIFT)
#  define USBDEV_ENDPTCTRL0_RXT_CTRL         (0 << USBDEV_ENDPTCTRL0_RXT_SHIFT) /* Control */
                                                       /* Bits 4-6: Reserved */
#define USBDEV_ENDPTCTRL0_RXE                (1 << 7)  /* Bit 7:  Rx endpoint enable */
                                                       /* Bits 8-15: Reserved */
#define USBDEV_ENDPTCTRL0_TXS                (1 << 16) /* Bit 16: Tx endpoint stall */
                                                       /* Bit 17: Reserved */
#define USBDEV_ENDPTCTRL0_TXT_SHIFT          (18)      /* Bits 18-19: Tx endpoint type */
#define USBDEV_ENDPTCTRL0_TXT_MASK           (3 << USBDEV_ENDPTCTRL0_TXT_SHIFT)
#  define USBDEV_ENDPTCTRL0_TXT_CTRL         (0 << USBDEV_ENDPTCTRL0_TXT_SHIFT) /* Control */
#define USBDEV_ENDPTCTRL0_TXE                (1 << 23) /* Bit 23: Tx endpoint enable */
                                                       /* Bits 24-31: Reserved */
/* USB Endpoint 1-3 control registers ENDPTCTRL1-ENDPPTCTRL5 */

#define USBDEV_ENDPTCTRL_RXS                 (1 << 0)  /* Bit 0:  Rx endpoint stall */
                                                       /* Bit 1:  Reserved */
#define USBDEV_ENDPTCTRL_RXT_SHIFT           (2)       /* Bits 2-3: Endpoint type */
#define USBDEV_ENDPTCTRL_RXT_MASK            (3 << USBDEV_ENDPTCTRL_RXT_SHIFT)
#  define USBDEV_ENDPTCTRL_RXT_CTRL          (0 << USBDEV_ENDPTCTRL_RXT_SHIFT) /* Control */
#  define USBDEV_ENDPTCTRL_RXT_ISOC          (1 << USBDEV_ENDPTCTRL_RXT_SHIFT) /* Isochronous */
#  define USBDEV_ENDPTCTRL_RXT_BULK          (2 << USBDEV_ENDPTCTRL_RXT_SHIFT) /* Bulk */
#  define USBDEV_ENDPTCTRL_RXT_INTR          (3 << USBDEV_ENDPTCTRL_RXT_SHIFT) /* Interrupt */
                                                       /* Bit 4:  Reserved */
#define USBDEV_ENDPTCTRL_RXI                 (1 << 5)  /* Bit 5:  Rx data toggle inhibit */
#define USBDEV_ENDPTCTRL_RXR                 (1 << 6)  /* Bit 6:  Rx data toggle reset */
#define USBDEV_ENDPTCTRL_RXE                 (1 << 7)  /* Bit 7:  Rx endpoint enable */
                                                       /* Bits 8-15: Reserved */
#define USBDEV_ENDPTCTRL_TXS                 (1 << 16) /* Bit 16: Tx endpoint stall */
                                                       /* Bit 17: Reserved */
#define USBDEV_ENDPTCTRL_TXT_SHIFT           (18)      /* Bits 18-19: Tx endpoint type */
#define USBDEV_ENDPTCTRL_TXT_MASK            (3 << USBDEV_ENDPTCTRL_TXT_SHIFT)
#  define USBDEV_ENDPTCTRL_TXT_CTRL          (0 << USBDEV_ENDPTCTRL_TXT_SHIFT) /* Control */
#  define USBDEV_ENDPTCTRL_TXT_ISOC          (1 << USBDEV_ENDPTCTRL_TXT_SHIFT) /* Isochronous */
#  define USBDEV_ENDPTCTRL_TXT_BULK          (2 << USBDEV_ENDPTCTRL_TXT_SHIFT) /* Bulk */
#  define USBDEV_ENDPTCTRL_TXT_INTR          (3 << USBDEV_ENDPTCTRL_TXT_SHIFT) /* Interrupt */
                                                       /* Bit 20: Reserved */
#define USBDEV_ENDPTCTRL_TXI                 (1 << 21) /* Bit 21: Tx data toggle inhibit */
#define USBDEV_ENDPTCTRL_TXR                 (1 << 22) /* Bit 22: Tx data toggle reset */
#define USBDEV_ENDPTCTRL_TXE                 (1 << 23) /* Bit 23: Tx endpoint enable */
                                                       /* Bits 24-31: Reserved */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_USB0_H */
