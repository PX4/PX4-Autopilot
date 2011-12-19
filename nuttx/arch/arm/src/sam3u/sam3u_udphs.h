/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_udphs.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_UDPHS_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_UDPHS_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* UDPHS register offsets ***************************************************************/

#define SAM3U_UDPHS_CTRL_OFFSET               0x00 /* UDPHS Control Register */
#define SAM3U_UDPHS_FNUM_OFFSET               0x04 /* UDPHS Frame Number Register */
                                                   /* 0x08-0x0C: Reserved */
#define SAM3U_UDPHS_IEN_OFFSET                0x10 /* UDPHS Interrupt Enable Register */
#define SAM3U_UDPHS_INTSTA_OFFSET             0x14 /* UDPHS Interrupt Status Register */
#define SAM3U_UDPHS_CLRINT_OFFSET             0x18 /* UDPHS Clear Interrupt Register */
#define SAM3U_UDPHS_EPTRST_OFFSET             0x1c /* UDPHS Endpoints Reset Register */
                                                   /* 0x20-0xcc: Reserved */
#define SAM3U_UDPHS_TST_OFFSET                0xe0 /* UDPHS Test Register */
                                                   /* 0xE4-0xE8: Reserved */
#define SAM3U_UDPHS_IPNAME1_OFFSET            0xf0 /* UDPHS Name1 Register */
#define SAM3U_UDPHS_IPNAME2_OFFSET            0xf4 /* UDPHS Name2 Register */
#define SAM3U_UDPHS_IPFEATURES_OFFSET         0xf8 /* UDPHS Features Register */

/* Endpoint registers:  Offsets for Endpoints 0-6: 0x100, 0x120, 0x140, 0x160, 0x180,
 * 0x1a0, and 0x1c0
 */

#define SAM3U_UDPHSEP_OFFSET(n)               (0x100+((n)<<5))
#define SAM3U_UDPHSEP_CFG_OFFSET              0x00 /* UDPHS Endpoint Configuration Register */
#define SAM3U_UDPHSEP_CTLENB_OFFSET           0x04 /* UDPHS Endpoint Control Enable Register */
#define SAM3U_UDPHSEP_CTLDIS_OFFSET           0x08 /* UDPHS Endpoint Control Disable Register */
#define SAM3U_UDPHSEP_CTL_OFFSET              0x0c /* UDPHS Endpoint Control Register */
                                                   /* 0x10: Reserved */
#define SAM3U_UDPHSEP_SETSTA_OFFSET           0x14 /* UDPHS Endpoint Set Status Register */
#define SAM3U_UDPHSEP_CLRSTA_OFFSET           0x18 /* UDPHS Endpoint Clear Status Register */
#define SAM3U_UDPHSEP_STA_OFFSET              0x1c /* UDPHS Endpoint Status Register */
                                                   /* 0x1e0-0x300: Reserved */
                                                   /* 0x300-0x30c: Reserved */
/* DMA Channel Registers:  Offsets for DMA channels 1-6 0x320, 0x330, 0x340, 0x350, and
 * 0x360.  NOTE that there is no DMA channel 0.
 */

#define SAM3U_UDPHSDMA_OFFSET(n)              (0x310+((n)<<4))
#define SAM3U_UDPHSDMA_NXTDSC_OFFSET          0x00 /* UDPHS DMA Next Descriptor Address Register */
#define SAM3U_UDPHSDMA_ADDRESS_OFFSET         0x04 /* UDPHS DMA Channel Address Register */
#define SAM3U_UDPHSDMA_CONTROL_OFFSET         0x08 /* UDPHS DMA Channel Control Register */
#define SAM3U_UDPHSDMA_STATUS_OFFSET)         0x0c /* UDPHS DMA Channel Status Register */

/* UDPHS register adresses **************************************************************/

#define SAM3U_UDPHS_CTRL                      (SAM3U_UDPHS_BASE+SAM3U_UDPHS_CTRL_OFFSET)
#define SAM3U_UDPHS_FNUM                      (SAM3U_UDPHS_BASE+SAM3U_UDPHS_FNUM_OFFSET)
#define SAM3U_UDPHS_IEN                       (SAM3U_UDPHS_BASE+SAM3U_UDPHS_IEN_OFFSET)
#define SAM3U_UDPHS_INTSTA                    (SAM3U_UDPHS_BASE+SAM3U_UDPHS_INTSTA_OFFSET)
#define SAM3U_UDPHS_CLRINT                    (SAM3U_UDPHS_BASE+ SAM3U_UDPHS_CLRINT_OFFSET)
#define SAM3U_UDPHS_EPTRST                    (SAM3U_UDPHS_BASE+SAM3U_UDPHS_EPTRST_OFFSET)
#define SAM3U_UDPHS_TST                       (SAM3U_UDPHS_BASE+SAM3U_UDPHS_TST_OFFSET)
#define SAM3U_UDPHS_IPNAME1                   (SAM3U_UDPHS_BASE+SAM3U_UDPHS_IPNAME1_OFFSET)
#define SAM3U_UDPHS_IPNAME2                   (SAM3U_UDPHS_BASE+SAM3U_UDPHS_IPNAME2_OFFSET)
#define SAM3U_UDPHS_IPFEATURES                (SAM3U_UDPHS_BASE+SAM3U_UDPHS_IPFEATURES_OFFSET)

/* Endpoint registers */

#define SAM3U_UDPHSEP_BASE(n))                (SAM3U_UDPHS_BASE+SAM3U_UDPHSEP_OFFSET(n))
#define SAM3U_UDPHSEP_CFG(n)                  (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_CFG_OFFSET)
#define SAM3U_UDPHSEP_CTLENB(n)               (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_CTLENB_OFFSET)
#define SAM3U_UDPHSEP_CTLDIS(n)               (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_CTLDIS_OFFSET)
#define SAM3U_UDPHSEP_CTL(n)                  (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_CTL_OFFSET)
#define SAM3U_UDPHSEP_SETSTA(n)               (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_SETSTA_OFFSET)
#define SAM3U_UDPHSEP_CLRSTA(n)               (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_CLRSTA_OFFSET)
#define SAM3U_UDPHSEP_STA(n)                  (SAM3U_UDPHSEP_BASE(n)+SAM3U_UDPHSEP_STA_OFFSET)

/* DMA Channel Registers*/

#define SAM3U_UDPHSDMA_BASE(n)                (SAM3U_UDPHS_BASE+SAM3U_UDPHSDMA_OFFSET(n))
#define SAM3U_UDPHSDMA_NXTDSC(n)              (SAM3U_UDPHSDMA_BASE(n)+SAM3U_UDPHSDMA_NXTDSC_OFFSET)
#define SAM3U_UDPHSDMA_ADDRESS(n)             (SAM3U_UDPHSDMA_BASE(n)+SAM3U_UDPHSDMA_ADDRESS_OFFSET)
#define SAM3U_UDPHSDMA_CONTROL(n)             (SAM3U_UDPHSDMA_BASE(n)+SAM3U_UDPHSDMA_CONTROL_OFFSET)
#define SAM3U_UDPHSDMA_STATUS(n)              (SAM3U_UDPHSDMA_BASE(n)+SAM3U_UDPHSDMA_STATUS_OFFSET)

/* UDPHS register bit definitions *******************************************************/
/* UDPHS Control Register */

#define UDPHS_CTRL_DEVADDR_SHIFT              (0)       /* Bits 0-6: UDPHS Address */
#define UDPHS_CTRL_DEVADDR_MASK               (0x7f << UDPHS_CTRL_DEVADDR_SHIFT)
#define UDPHS_CTRL_FADDREN                    (1 << 7)  /* Bit 7:  Function Address Enable */
#define UDPHS_CTRL_ENUDPHS                    (1 << 8)  /* Bit 8:  UDPHS Enable */
#define UDPHS_CTRL_DETACH                     (1 << 9)  /* Bit 9:  Detach Command */
#define UDPHS_CTRL_REWAKEUP                   (1 << 10) /* Bit 10: Send Remote Wake Up */
#define UDPHS_CTRL_PULLDDIS                   (1 << 11) /* Bit 11: Pull-Down Disable */

/* UDPHS Frame Number Register */

#define UDPHS_FNUM_MICROFRAMENUM_SHIFT        (0)      /* Bits 0-2: Microframe Num */
#define UDPHS_FNUM_MICROFRAMENUM_MASK         (7 << UDPHS_FNUM_MICROFRAMENUM_SHIFT)
#define UDPHS_FNUM_FRAMENUMBER_SHIFT          (3)      /* Bits 3-7: Frame Number in  Packet Field Formats */
#define UDPHS_FNUM_FRAMENUMBER_MASK           (31 << UDPHS_FNUM_FRAMENUMBER_SHIFT)
#define UDPHS_FNUM_FNUMERR_SHIFT              (8)      /* Bits 8-13: Frame Number CRC Error */
#define UDPHS_FNUM_FNUMERR_MASK               (63 << UDPHS_FNUM_FNUMERR_SHIFT)

/* UDPHS Interrupt Enable Register, UDPHS Interrupt Status Register, and UDPHS Clear
 * Interrupt Register common bit-field definitions
 */

#define USBPHS_INT_DETSUSPD                   (1 << 1)  /* Bit 1:  Suspend Interrupt (Common) */
#define USBPHS_INT_MICROSOF                   (1 << 2)  /* Bit 2:  Micro-SOF Interrupt (Common) */
#define USBPHS_INT_INTSOF                     (1 << 3)  /* Bit 3:  SOF Interrupt (Common) */
#define USBPHS_INT_ENDRESET                   (1 << 4)  /* Bit 4:  End Of Reset Interrupt (Common) */
#define USBPHS_INT_WAKEUP                     (1 << 5)  /* Bit 5:  Wake Up CPU Interrupt (Common) */
#define USBPHS_INT_ENDOFRSM                   (1 << 6)  /* Bit 6:  End Of Resume Interrupt (Common) */
#define USBPHS_INT_UPSTRRES                   (1 << 7)  /* Bit 7:  Upstream Resume Interrupt (Common) */
#define USBPHS_INT_EPT(n)                     (1 << ((n)+8))
#define USBPHS_INT_EPT0                       (1 << 8)  /* Bit 8:  Endpoint 0 Interrupt (not Clear) */
#define USBPHS_INT_EPT1                       (1 << 9)  /* Bit 9:  Endpoint 1 Interrupt (not Clear) */
#define USBPHS_INT_EPT2                       (1 << 10) /* Bit 10: Endpoint 2 Interrupt (not Clear) */
#define USBPHS_INT_EPT3                       (1 << 11) /* Bit 11: Endpoint 3 Interrupt (not Clear) */
#define USBPHS_INT_EPT4                       (1 << 12) /* Bit 12: Endpoint 4 Interrupt (not Clear) */
#define USBPHS_INT_EPT5                       (1 << 13) /* Bit 13: Endpoint 5 Interrupt (not Clear) */
#define USBPHS_INT_EPT6                       (1 << 13) /* Bit 14: Endpoint 6 Interrupt (not Clear) */
#define USBPHS_INT_DMA(n)                     (1<<((n)+24))
#define USBPHS_INT_DMA1                       (1 << 25) /* Bit 25: DMA Channel 1 Interrupt (not Clear) */
#define USBPHS_INT_DMA2                       (1 << 26) /* Bit 26: DMA Channel 2 Interrupt (not Clear) */
#define USBPHS_INT_DMA3                       (1 << 27) /* Bit 27: DMA Channel 3 Interrupt (not Clear) */
#define USBPHS_INT_DMA4                       (1 << 28) /* Bit 28: DMA Channel 4 Interrupt (not Clear) */
#define USBPHS_INT_DMA5                       (1 << 29) /* Bit 29: DMA Channel 5 Interrupt (not Clear) */
#define USBPHS_INT_DMA6                       (1 << 30) /* Bit 30: DMA Channel 6 Interrupt (not Clear) */

/* UDPHS Endpoints Reset Register */

#define UDPHS_EPTRST_EPT(n)                   (1<<(n))  /* Bit 0-6: Endpoint n Reset */

/* UDPHS Test Register */

#define UDPHS_TST_SPEEDCFG_SHIFT              (0)       /* Bits 0-1: Speed Configuration */
#define UDPHS_TST_SPEEDCFG_MASK               (3 << UDPHS_TST_SPEEDCFG_SHIFT)
00 Normal Mode
10 Force High Speed
11 Force Full Speed
#define UDPHS_TST_TSTJ                        (1 << 2)  /* Bit 2:  Test J Mode */
#define UDPHS_TST_TSTK                        (1 << 3)  /* Bit 3:  Test K Mode */
#define UDPHS_TST_TSTPKT                      (1 << 4)  /* Bit 4:  Test Packet Mo */
#define UDPHS_TST_OPMODE2                     (1 << 5)  /* Bit 5:  OpMode2 */

/* UDPHS Features Register */

#define UDPHS_IPFEATURES_EPTNBRMAX_SHIFT      (0)       /* Bits 0-3: Max Number of Endpoints */
#define UDPHS_IPFEATURES_EPTNBRMAX_MASK       (15 << UDPHS_IPFEATURES_EPTNBRMAX_SHIFT)
#define UDPHS_IPFEATURES_DMACHANNELNBR_SHIFT  (4)       /* Bits 4-6: Number of DMA Channels */
#define UDPHS_IPFEATURES_DMACHANNELNBR_MASK   (7 << UDPHS_IPFEATURES_DMACHANNELNBR_SHIFT)
#define UDPHS_IPFEATURES_DMABSIZ              (1 << 7)  /* Bit 7:  DMA Buffer Size */
#define UDPHS_IPFEATURES_DMAFIFOWDDEPTH_SHIFT (8)       /* Bits 8-11: DMA FIFO Depth in Words */
#define UDPHS_IPFEATURES_DMAFIFOWDDEPTH_MASK  (15 << UDPHS_IPFEATURES_DMAFIFOWDDEPTH_SHIFT)
#  define UDPHS_IPFEATURES_DMAFIFOWDDEPTH(n)  ((n)&15)
#define UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT    (12)      /* Bits 12-14: DPRAM Size */
#define UDPHS_IPFEATURES_FIFOMAXSIZE_MASK     (7 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT)
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_128b   (0 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 128 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_256b   (1 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 256 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_512b   (2 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 512 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_1Kb    (3 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 1024 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_2Kb    (4 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 2048 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_4Kb    (5 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 4096 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_8Kb    (6 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 8192 bytes */
#  define UDPHS_IPFEATURES_FIFOMAXSIZE_16Kb   (7 << UDPHS_IPFEATURES_FIFOMAXSIZE_SHIFT) /* DPRAM 16384 bytes */
#define UDPHS_IPFEATURES_BWDPRAM              (1 << 15) /* Bit 15: DPRAM Byte Write Capability */
#define UDPHS_IPFEATURES_DATAB168             (1 << 15) /* Bit 15: UTMI DataBus16_8 */
#define UDPHS_IPFEATURES_ISOEPT(n)            (1<<((n)+16)
#define UDPHS_IPFEATURES_ISOEPT1              (1 << 17) /* Bit 17: EP1 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT2              (1 << 18) /* Bit 18: EP2 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT3              (1 << 19) /* Bit 19: EP3 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT4              (1 << 20) /* Bit 20: EP4 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT5              (1 << 21) /* Bit 21: EP5 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT6              (1 << 22) /* Bit 22: EP6 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT7              (1 << 23) /* Bit 23: EP7 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT8              (1 << 24) /* Bit 24: EP8 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT9              (1 << 25) /* Bit 25: EP9 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT0              (1 << 26) /* Bit 26: EP10 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT1              (1 << 27) /* Bit 27: EP11 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT2              (1 << 28) /* Bit 28: EP12 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT3              (1 << 29) /* Bit 29: EP13 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT4              (1 << 30) /* Bit 30: EP14 High B/W Isoc Capability */
#define UDPHS_IPFEATURES_ISOEPT5              (1 << 31) /* Bit 31: EP15 High B/W Isoc Capability */

/* UDPHS Endpoint Configuration Register (0-6) */

#define UDPHSEP_CFG_SIZE_SHIFT                (0)       /* Bits 0-2: Endpoint Size */
#define UDPHSEP_CFG_SIZE_MASK                 (7 << UDPHSEP_CFG_SIZE_SHIFT)
#  define UDPHSEP_CFG_SIZE_8b                 (0 << UDPHSEP_CFG_SIZE_SHIFT) /* 8 bytes */
#  define UDPHSEP_CFG_SIZE_16b                (1 << UDPHSEP_CFG_SIZE_SHIFT) /* 16 bytes */
#  define UDPHSEP_CFG_SIZE_32b                (2 << UDPHSEP_CFG_SIZE_SHIFT) /* 32 bytes */
#  define UDPHSEP_CFG_SIZE_16b                (3 << UDPHSEP_CFG_SIZE_SHIFT) /* 64 bytes */
#  define UDPHSEP_CFG_SIZE_128b               (4 << UDPHSEP_CFG_SIZE_SHIFT) /* 128 bytes */
#  define UDPHSEP_CFG_SIZE_256b               (5 << UDPHSEP_CFG_SIZE_SHIFT) /* 256 bytes */
#  define UDPHSEP_CFG_SIZE_512b               (6 << UDPHSEP_CFG_SIZE_SHIFT) /* 512 bytes */
#  define UDPHSEP_CFG_SIZE_1Kb                (7 << UDPHSEP_CFG_SIZE_SHIFT) /* 1024 bytes */
#define UDPHSEP_CFG_DIR                       (1 << 3)  /* Bit 3:  Endpoint Direction */
#define UDPHSEP_CFG_TYPE_SHIFT                (4)       /* Bits 4-5: Endpoint Type */
#define UDPHSEP_CFG_TYPE_MASK                 (3 << UDPHSEP_CFG_TYPE_SHIFT)
#  define UDPHSEP_CFG_TYPE_CNTRL              (0 << UDPHSEP_CFG_TYPE_SHIFT) /* Control endpoint */
#  define UDPHSEP_CFG_TYPE_ISOC               (1 << UDPHSEP_CFG_TYPE_SHIFT) /* Isochronous endpoint */
#  define UDPHSEP_CFG_TYPE_BULK               (2 << UDPHSEP_CFG_TYPE_SHIFT) /* Bulk endpoint */
#  define UDPHSEP_CFG_TYPE_INTR               (3 << UDPHSEP_CFG_TYPE_SHIFT) /* Interrupt endpoint */
#define UDPHSEP_CFG_BKNUMBER_SHIFT            (6)       /* Bits 6-7:  Number of Banks */
#define UDPHSEP_CFG_BKNUMBER_MASK             (3 << UDPHSEP_CFG_BKNUMBER_SHIFT)
#  define UDPHSEP_CFG_BKNUMBER_0BANK          (0 << UDPHSEP_CFG_BKNUMBER_SHIFT) /* Zero bank (unmapped) */
#  define UDPHSEP_CFG_BKNUMBER_1BANK          (1 << UDPHSEP_CFG_BKNUMBER_SHIFT) /* One bank (bank 0) */
#  define UDPHSEP_CFG_BKNUMBER_2BANK          (2 << UDPHSEP_CFG_BKNUMBER_SHIFT) /* Double bank (bank 0-1) */
#  define UDPHSEP_CFG_BKNUMBER_3BANK          (3 << UDPHSEP_CFG_BKNUMBER_SHIFT) /* Triple bank (bank 0-2) */
#define UDPHSEP_CFG_NBTRANS_SHIFT             (8)      /* Bits 8-9:  Number Of Transaction per Microframe */
#define UDPHSEP_CFG_NBTRANS_MASK              (3 << UDPHSEP_CFG_NBTRANS_SHIFT)
#define UDPHSEP_CFG_MAPD                      (1 << 31)  /*Bit 31: Endpoint Mapped */

/* UDPHS Endpoint Control Enable Register, UDPHS Endpoint Control Disable Register,
 * and UDPHS Endpoint Control Register common bit-field definitions
 */

#define UDPHSEP_INT_EPT                       (1 << 0)  /* Bit 0:  Endpoint Enable/Disable */
#define UDPHSEP_INT_AUTOVALID                 (1 << 1)  /* Bit 1:  Packet Auto-Valid  */
#define UDPHSEP_INT_INTDISDMA                 (1 << 3)  /* Bit 3:  Interrupts Disable DMA */
#define UDPHSEP_INT_NYETDIS                   (1 << 4)  /* Bit 4:  NYET Disable (HS Bulk OUT EPs) */
#define UDPHSEP_INT_DATAXRX                   (1 << 6)  /* Bit 6:  DATAx Interrupt Enable (High B/W Isoc OUT EPs) */
#define UDPHSEP_INT_MDATARX                   (1 << 7)  /* Bit 7:  MDATA Interrupt Enable (High B/W Isoc OUT EPs) */
#define UDPHSEP_INT_ERROVFLW                  (1 << 8)  /* Bit 8:  Overflow Error Interrupt  */
#define UDPHSEP_INT_RXBKRDY                   (1 << 9)  /* Bit 9:  Received OUT Data Interrupt  */
#define UDPHSEP_INT_TXCOMPLT                  (1 << 10) /* Bit 10: Transmitted IN Data Complete Interrupt  */
#define UDPHSEP_INT_TXPKRDY                   (1 << 11) /* Bit 11: TX Packet Ready Interrupt  */
#define UDPHSEP_INT_ERRTRANS                  (1 << 11) /* Bit 11: Transaction Error Interrupt  */
#define UDPHSEP_INT_RXSETUP                   (1 << 12) /* Bit 12: Received SETUP Interrupt  */
#define UDPHSEP_INT_ERRFLISO                  (1 << 12) /* Bit 12: Error Flow Interrupt */
#define UDPHSEP_INT_STALLSNT                  (1 << 13) /* Bit 13: Stall Sent Interrupt  */
#define UDPHSEP_INT_ERRCRISO                  (1 << 13) /* Bit 13: ISO CRC Error Error Interrupt  */
#define UDPHSEP_INT_ERRNBTRA                  (1 << 13) /* Bit 13: Number of Transaction Error Interrupt  */
#define UDPHSEP_INT_NAKIN                     (1 << 14) /* Bit 14: NAKIN Interrupt  */
#define UDPHSEP_INT_ERRFLUSH                  (1 << 14) /* Bit 14: Bank Flush Error Interrupt  */
#define UDPHSEP_INT_NAKOUT                    (1 << 15) /* Bit 15: NAKOUT Interrupt  */
#define UDPHSEP_INT_BUSYBANK                  (1 << 18) /* Bit 18: Busy Bank Interrupt  */
#define UDPHSEP_INT_SHRTPCKT                  (1 << 31) /* Bit 31: Short Packet Send/Short Packet Interrupt  */

/* UDPHS Endpoint Set Status Register */

#define UDPHSEP_SETSTA_FRCESTALL              (1 << 5)  /* Bit 5:  Stall Handshake Request Set */
#define UDPHSEP_SETSTA_KILLBANK               (1 << 9)  /* Bit 9:  KILL Bank Set (for IN Endpoint) */
#define UDPHSEP_SETSTA_TXPKRDY                (1 << 11) /* Bit 11: TX Packet Ready Set */

/* UDPHS Endpoint Clear Status Register */

#define UDPHSEP_CLRSTA_FRCESTALL              (1 << 5)  /* Bit 5:  Stall Handshake Request Clear */
#define UDPHSEP_CLRSTA_TOGGLESQ               (1 << 6)  /* Bit 6:  Data Toggle Clear */
#define UDPHSEP_CLRSTA_RXBKRDY                (1 << 9)  /* Bit 9:  Received OUT Data Clear */
#define UDPHSEP_CLRSTA_TXCOMPLT               (1 << 10) /* Bit 10: Transmitted IN Data Complete Clear */
#define UDPHSEP_CLRSTA_RXSETUP                (1 << 12) /* Bit 12: Received SETUP Clear */
#define UDPHSEP_CLRSTA_ERRFLISO               (1 << 12) /* Bit 12: Error Flow Clear */
#define UDPHSEP_CLRSTA_STALL_NT               (1 << 13) /* Bit 13: Stall Sent Clear */
#define UDPHSEP_CLRSTA_ERRNBTRA               (1 << 13) /* Bit 13: Number of Transaction Error Clear */
#define UDPHSEP_CLRSTA_NAKIN                  (1 << 14) /* Bit 14: NAKIN Clear */
#define UDPHSEP_CLRSTA_ERRFLUSH               (1 << 14) /* Bit 14: Bank Flush Error Clear */
#define UDPHSEP_CLRSTA_NAKOUT                 (1 << 15) /* Bit 15: NAKOUT Clear */

/* UDPHS Endpoint Status Register */

#define UDPHSEP_STA_FRCESTALL                 (1 << 5) /* Bit 5: Stall Handshake Request */
#define UDPHSEP_STA_TOGGLESQSTA_SHIFT         (6)      /* Bits 6-7: Toggle Sequencing */
#define UDPHSEP_STA_TOGGLESQSTA_MASK          (3 << UDPHSEP_STA_TOGGLESQSTA_SHIFT)
#  define UDPHSEP_STA_TOGGLESQSTA_DATA0       (0 << UDPHSEP_STA_TOGGLESQSTA_SHIFT) /* Data0 */
#  define UDPHSEP_STA_TOGGLESQSTA_DATA1       (1 << UDPHSEP_STA_TOGGLESQSTA_SHIFT) /* Data1 */
#  define UDPHSEP_STA_TOGGLESQSTA_DATA2       (2 << UDPHSEP_STA_TOGGLESQSTA_SHIFT) /* Data2 (High B/W Isoc EP) */
#  define UDPHSEP_STA_TOGGLESQSTA_MDATA       (3 << UDPHSEP_STA_TOGGLESQSTA_SHIFT) /* MData (High B/W Isoc EP) */
#define UDPHSEP_STA_ERROVFLW                 (1 << 8)  /* Bit 8:  Overflow Error */
#define UDPHSEP_STA_RXBKRDY                  (1 << 9)  /* Bit 9:  Received OUT Data */
#define UDPHSEP_STA_KILLBANK                 (1 << 9)  /* Bit 9:  KILL Bank */
#define UDPHSEP_STA_TXCOMPLT                 (1 << 10) /* Bit 10: Transmitted IN Data Complete */
#define UDPHSEP_STA_TXPKRDY                  (1 << 11) /* Bit 11: TX Packet Ready */
#define UDPHSEP_STA_ERRTRANS                 (1 << 11) /* Bit 11: Transaction Error */
#define UDPHSEP_STA_RXSETUP                  (1 << 12) /* Bit 12: Received SETUP */
#define UDPHSEP_STA_ERRFLISO                 (1 << 12) /* Bit 12: Error Flow */
#define UDPHSEP_STA_STALLSNT                 (1 << 13) /* Bit 13: Stall Sent */
#define UDPHSEP_STA_ERRCRISO                 (1 << 13) /* Bit 13: CRC ISO Error */
#define UDPHSEP_STA_ERRNBTRA                 (1 << 13) /* Bit 13: Number of Transaction Error */
#define UDPHSEP_STA_NAKIN                    (1 << 14) /* Bit 14: NAK IN */
#define UDPHSEP_STA_ERRFLUSH                 (1 << 14) /* Bit 14: Bank Flush Error */
#define UDPHSEP_STA_NAKOUT                   (1 << 15) /* Bit 15: NAK OUT */
#define UDPHSEP_STA_CURRENTBANK_SHIFT        (16)      /* Bits 16-17: Current Bank */
#define UDPHSEP_STA_CURRENTBANK_MASK         (3 << UDPHSEP_STA_CURRENTBANK_MASK)
#define UDPHSEP_STA_CONTROLDIR_SHIFT         (16)      /* Bits 16-17: Control Direction */
#define UDPHSEP_STA_CONTROLDIR_MASK          (3 << UDPHSEP_STA_CONTROLDIR_SHIFT)
#define UDPHSEP_STA_BUSYBANKSTA_SHIFT        (18)      /* Bits 18-19: Busy Bank Number */
#define UDPHSEP_STA_BUSYBANKSTA_MASK         (3 << UDPHSEP_STA_BUSYBANKSTA_SHIFT)
#define UDPHSEP_STA_BYTECOUNT_SHIFT          (20)      /* Bits 20-23: UDPHS Byte Count */
#define UDPHSEP_STA_BYTECOUNT_MASK           (15 << UDPHSEP_STA_BYTECOUNT_SHIFT)
#define UDPHSEP_STA_SHRTPCKT                 (1 << 31) /* Bit 31: Short Packet

/* UDPHS DMA Channel Control Register */

#define UDPHSDMA_CONTROL_CHANNENB            (1 << 0)  /* Bit 0:  Channel Enable Command */
#define UDPHSDMA_CONTROL_LDNXTDSC            (1 << 1)  /* Bit 1:  Load Next Channel Xfr Desc Enable (Command) */
#define UDPHSDMA_CONTROL_ENDTREN             (1 << 2)  /* Bit 2:  End of Transfer Enable (Control) */
#define UDPHSDMA_CONTROL_ENDBEN              (1 << 3)  /* Bit 3:  End of Buffer Enable (Control) */
#define UDPHSDMA_CONTROL_ENDTRIT             (1 << 4)  /* Bit 4:  End of Transfer Interrupt Enable */
#define UDPHSDMA_CONTROL_ENDBUFFIT           (1 << 5)  /* Bit 5:  End of Buffer Interrupt Enable */
#define UDPHSDMA_CONTROL_DESCLDIT            (1 << 6)  /* Bit 6:  Descriptor Loaded Interrupt Enab */
#define UDPHSDMA_CONTROL_BURSTLCK            (1 << 7)  /* Bit 7:  Burst Lock Ena */
#define UDPHSDMA_CONTROL_BUFFLENGTH_SHIFT    (16)      /* Bits 16-31: Buffer Byte Length (Write-only) */
#define UDPHSDMA_CONTROL_BUFFLENGTH_MASK     (0xffff << UDPHSDMA_CONTROL_BUFFLENGTH_SHIFT)

/* UDPHS DMA Channel Status Register */

#define UDPHSDMA_STATUS_CHANNENB             (1 << 0)  /* Bit 0:  Channel Enable Status */
#define UDPHSDMA_STATUS_CHANNACT             (1 << 1)  /* Bit 1:  Channel Active Status */
#define UDPHSDMA_STATUS_ENDTRST              (1 << 4)  /* Bit 4:  End of Channel Transfer Status */
#define UDPHSDMA_STATUS_ENDBFST              (1 << 5)  /* Bit 5:  End of Channel Buffer Status */
#define UDPHSDMA_STATUS_DESCLDST             (1 << 6)  /* Bit 6:  Descriptor Loaded Status */
#define UDPHSDMA_STATUS_BUFFCOUNT_SHIFT      (16)      /* Bits 16-31: Buffer Byte Count */
#define UDPHSDMA_STATUS_BUFFCOUNT_MASK       (0xffff << UDPHSDMA_STATUS_BUFFCOUNT_SHIFT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_UDPHS_H */
