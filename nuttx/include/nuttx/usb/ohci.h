/****************************************************************************
 * include/nuttx/usb/ohci.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   References: "OpenHCI Open Host Controller Interface Specification
 *   for USB," Release 1.0a, Compaq, Microsoft, National Semiconductor,
 *   September 14, 1999.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_OHCI_H
#define __INCLUDE_NUTTX_USB_OHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
 
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/
/* Control and status registers (section 7.1) */

#define OHCI_HCIREV_OFFSET          0x0000 /* HcRevision: Version of HCI specification */
#define OHCI_CTRL_OFFSET            0x0004 /* HcControl: HC control */
#define OHCI_CMDST_OFFSET           0x0008 /* HcCommandStatus: HC command status */
#define OHCI_INTST_OFFSET           0x000c /* HcInterruptStatus: HC interrupt status */
#define OHCI_INTEN_OFFSET           0x0010 /* HcInterruptEnable: HC interrupt enable */
#define OHCI_INTDIS_OFFSET          0x0014 /* HcInterruptDisable: HC interrupt disable */

/* Memory pointer registers (section 7.2) */

#define OHCI_HCCA_OFFSET            0x0018 /* HcHCCA: HC communication area */
#define OHCI_PERED_OFFSET           0x001c /* HcPeriodCurrentED: Current isoc or int endpoint desc */
#define OHCI_CTRLHEADED_OFFSET      0x0020 /* HcControlHeadED: First EP desc in the control list */
#define OHCI_CTRLED_OFFSET          0x0024 /* HcControlCurrentED: Current EP desc in the control list */
#define OHCI_BULKHEADED_OFFSET      0x0028 /* HcBulkHeadED: First EP desc in the bulk list */
#define OHCI_BULKED_OFFSET          0x002c /* HcBulkCurrentED: Current EP desc in the bulk list */
#define OHCI_DONEHEAD_OFFSET        0x0030 /* HcDoneHead: Last transfer desc added to DONE queue */

/* Frame counter registers (section 7.3) */

#define OHCI_FMINT_OFFSET           0x0034 /* HcFmInterval: Bit time interval that would not cause overrun */
#define OHCI_FMREM_OFFSET           0x0038 /* HcFmRemaining: Bit time remaining in current frame */
#define OHCI_FMNO_OFFSET            0x003c /* HcFmNumber: Frame number counter */
#define OHCI_PERSTART_OFFSET        0x0040 /* HcPeriodicStart: Time to start processing periodic list */

/* Root hub registers (section 7.4) */

#define OHCI_LSTHRES_OFFSET         0x0044 /* HcLSThreshold: Commit to transfer threshold */
#define OHCI_RHDESCA_OFFSET         0x0048 /* HcRhDescriptorA: Describes root hub (part A) */
#define OHCI_RHDESCB_OFFSET         0x004c /* HcRhDescriptorB: Describes root hub (part B) */
#define OHCI_RHSTATUS_OFFSET        0x0050 /* HcRhStatus: Root hub status */

#define OHCI_MAX_RHPORT             15     /* Maximum number of OHCI root hub ports */

#define OHCI_RHPORTST_OFFSET(n)     (0x0054+(((n)-1) << 2))
#define OHCI_RHPORTST1_OFFSET       0x0054 /* HcRhPort1Status:  Root hub port status 1 */
#define OHCI_RHPORTST2_OFFSET       0x0058 /* HcRhPort2Status:  Root hub port status 2 */
#define OHCI_RHPORTST3_OFFSET       0x005c /* HcRhPort3Status:  Root hub port status 3 */
#define OHCI_RHPORTST4_OFFSET       0x0060 /* HcRhPort4Status:  Root hub port status 4 */
#define OHCI_RHPORTST5_OFFSET       0x0064 /* HcRhPort5Status:  Root hub port status 5 */
#define OHCI_RHPORTST6_OFFSET       0x0068 /* HcRhPort6Status:  Root hub port status 6 */
#define OHCI_RHPORTST7_OFFSET       0x006c /* HcRhPort7Status:  Root hub port status 7 */
#define OHCI_RHPORTST8_OFFSET       0x0070 /* HcRhPort8Status:  Root hub port status 8 */
#define OHCI_RHPORTST9_OFFSET       0x0074 /* HcRhPort9Status:  Root hub port status 9 */
#define OHCI_RHPORTST10_OFFSET      0x0078 /* HcRhPort10Status: Root hub port status 10 */
#define OHCI_RHPORTST11_OFFSET      0x007c /* HcRhPort11Status: Root hub port status 11 */
#define OHCI_RHPORTST12_OFFSET      0x0080 /* HcRhPort12Status: Root hub port status 12 */
#define OHCI_RHPORTST13_OFFSET      0x0084 /* HcRhPort13Status: Root hub port status 13 */
#define OHCI_RHPORTST14_OFFSET      0x0088 /* HcRhPort14Status: Root hub port status 14 */
#define OHCI_RHPORTST15_OFFSET      0x008c /* HcRhPort15Status: Root hub port status 15 */

/* Register bit definitions *************************************************/

/* HcRevision: Version of HCI specification (7.1.1) */

#define OHCI_HCIREV_SHIFT           (0)       /* Bits 0-7: HCI spec version (BCD) */
#define OHCI_HCIREV_MASK            (0xff << OHCI_HCIREV_SHIFT)

/* HcControl: HC control (7.1.2) */

#define OHCI_CTRL_CBSR              (3 << 0)  /* Bit 0:  Control/bulk service ratio */
#define OHCI_CTRL_PLE               (1 << 2)  /* Bit 1:  Periodic list enable */
#define OHCI_CTRL_IE                (1 << 3)  /* Bit 2:  Isochronous enable */
#define OHCI_CTRL_CLE               (1 << 4)  /* Bit 3:  Control list enable */
#define OHCI_CTRL_BLE               (1 << 5)  /* Bit 4:  Bulk list enable */
#define OHCI_CTRL_HCFS_SHIFT        (6)       /* Bits 6-7: Host controller functional state */
#define OHCI_CTRL_HCFS_MASK         (3 << OHCI_CTRL_HCFS_SHIFT)
#  define OHCI_CTRL_HCFS_RESET      (0 << OHCI_CTRL_HCFS_SHIFT)
#  define OHCI_CTRL_HCFS_RESUME     (1 << OHCI_CTRL_HCFS_SHIFT)
#  define OHCI_CTRL_HCFS_OPER       (2 << OHCI_CTRL_HCFS_SHIFT)
#  define OHCI_CTRL_HCFS_SUSPEND    (3 << OHCI_CTRL_HCFS_SHIFT)
#define OHCI_CTRL_IR                (1 << 8)  /* Bit 8:  Interrupt routing */
#define OHCI_CTRL_RWC               (1 << 9)  /* Bit 9:  Remote wakeup connected */
#define OHCI_CTRL_RWE               (1 << 10) /* Bit 10: Remote wakeup enable */
                                              /* Bits 11-31: Reserved */

/* HcCommandStatus: HC command status (7.1.3) */

#define OHCI_CMDST_HCR              (1 << 0)  /* Bit 0:  Host controller reset */
#define OHCI_CMDST_CLF              (1 << 1)  /* Bit 1:  Control list filled */
#define OHCI_CMDST_BLF              (1 << 2)  /* Bit 2:  Bulk list filled */
#define OHCI_CMDST_OCR              (1 << 3)  /* Bit 3:  Ownership change request */
                                              /* Bits 4-15: Reserved */
#define OHCI_CMDST_SOC              (3 << 16) /* Bit 16: Scheduling overrun count */
                                              /* Bits 17-31: Reserved */

/* HcInterruptStatus: HC interrupt status (7.1.4),
 * HcInterruptEnable: HC interrupt enable (7.1.5), and
 * HcInterruptDisable: HC interrupt disable (7.1.6)
 */

#define OHCI_INT_SO                 (1 << 0)  /* Bit 0:  Scheduling overrun */
#define OHCI_INT_WDH                (1 << 1)  /* Bit 1:  Writeback done head */
#define OHCI_INT_SF                 (1 << 2)  /* Bit 2:  Start of frame */
#define OHCI_INT_RD                 (1 << 3)  /* Bit 3:  Resume detected */
#define OHCI_INT_UE                 (1 << 4)  /* Bit 4:  Unrecoverable error */
#define OHCI_INT_FNO                (1 << 5)  /* Bit 5:  Frame number overflow */
#define OHCI_INT_RHSC               (1 << 6)  /* Bit 6:  Root hub status change */
                                              /* Bits 7-29: Reserved */
#define OHCI_INT_OC                 (1 << 30) /* Bit 30: Ownership change */
#define OHCI_INT_MIE                (1 << 31) /* Bit 31: Master interrupt enable
                                               *         (Enable/disable only) */
/* HcHCCA: HC communication area (7.2.1):
 *
 * 32-bits aligned to 256 byte boundary.
 */

/* HcPeriodCurrentED: Current isoc or int endpoint desc (7.2.2),
 * HcControlHeadED: First EP desc in the control list (7.2.3),
 * HcControlCurrentED: Current EP desc in the control list (7.2.4),
 * HcBulkHeadED: First EP desc in the bulk list (7.2.5),
 * HcBulkCurrentED: Current EP desc in the bulk list (7.2.6), and
 * HcDoneHead: Last transfer desc added to DONE queue (7.2.7):
 *
 * All 32-bits aligned to an 8-byte boundary
 */

/* HcFmInterval: Bit time interval that would not cause overrun (7.3.1) */

#define OHCI_FMINT_FI_SHIFT         (0)       /* Bits 0-13: Frame interval */
#define OHCI_FMINT_FI_MASK          (0x3fff << OHCI_FMINT_FI_SHIFT)
                                              /* Bits 14-15: Reserved */
#define OHCI_FMINT_FSMPS_SHIFT      (16)      /* Bits 16-30: FS largest packet data */
#define OHCI_FMINT_FSMPS_MASK       (0x7fff << OHCI_FMINT_FSMPS_SHIFT)
#define OHCI_FMINT_FIT              (1 << 31) /* Bit 31: Frame interval toggle */

/* HcFmRemaining: Bit time remaining in current frame (7.3.2) */

#define OHCI_FMREM_FR_SHIFT         (0)       /* Bits 0-13: Frame remaining */
#define OHCI_FMREM_FR_MASK          (0x3fff << OHCI_FMREM_FR_SHIFT)
                                              /* Bits 16-30: Reserved */
#define OHCI_FMINT_FRT              (1 << 31) /* Bit 31: Frame remaining toggle */

/* HcFmNumber: Frame number counter (7.3.3) */

#define OHCI_FMNO_FI_SHIFT          (0)       /* Bits 0-15: Frame number */
#define OHCI_FMNO_FI_MASK           (0xffff << OHCI_FMINT_FI_SHIFT)
                                              /* Bits 16-31: Reserved */
/* HcPeriodicStart: Time to start processing periodic list (7.3.4) */

#define OHCI_PERSTART_SHIFT         (0)       /* Bits 0-13: Periodic start */
#define OHCI_PERSTART_MASK          (0x3fff << OHCI_PERSTART_SHIFT)
                                              /* Bits 14-31: Reserved */
/* HcLSThreshold: Commit to transfer threshold (7.3.5) */

#define OHCI_LSTHRES_SHIFT          (0)       /* Bits 0-11: LS threshold */
#define OHCI_LSTHRES_MASK           (0x0fff << OHCI_PERSTART_SHIFT)
                                              /* Bits 12-31: Reserved */
/* HcRhDescriptorN: Describes root hub (part A) (7.4.1) */

#define OHCI_RHDESCA_NDP_SHIFT      (0)       /* Bits 0-7: Number downstream ports */
#define OHCI_RHDESCA_NDP_MASK       (0xff << OHCI_RHDESCA_NDP_SHIFT)
#define OHCI_RHDESCA_PSM            (1 << 8)  /* Bit 8:  Power switching mode */
#define OHCI_RHDESCA_NPS            (1 << 9)  /* Bit 9:  No power switching */
#define OHCI_RHDESCA_DT             (1 << 10) /* Bit 10: Device type */
#define OHCI_RHDESCA_OCPM           (1 << 11) /* Bit 11: Over current protection mode */
#define OHCI_RHDESCA_NOCP           (1 << 12) /* Bit 12: No over current protection */
                                              /* Bits 13-23: Reserved */
#define OHCI_RHDESCA_POTPGT_SHIFT   (24)      /* Bits 24-31: Power on to power good time */
#define OHCI_RHDESCA_POTPGT_MASK    (0xff << OHCI_RHDESCA_POTPGT_SHIFT)

/* HcRhDescriptorB: Describes root hub (part B) (7.4.2) */

#define OHCI_RHDESCB_DR_SHIFT      (0)        /* Bits 0-15: Device removable */
#define OHCI_RHDESCB_DR_MASK       (0xffff << OHCI_RHDESCB_DR_SHIFT)
#  define OHCI_RHDESCB_ATTACHED(n) (1 << (OHCI_RHDESCB_DR_SHIFT+(n)))
#define OHCI_RHDESCB_PPCM_SHIFT    (16)       /* Bits 16-31: Port power control mask */
#define OHCI_RHDESCB_PPCM_MASK     (0xffff << OHCI_RHDESCB_PPCM_SHIFT)
#  define OHCI_RHDESCB_POWERED(n)  (1 << (OHCI_RHDESCB_DR_SHIFT+(n)))

/* HcRhStatus: Root hub status (7.4.3) */

#define OHCI_RHSTATUS_LPS          (1 << 0)   /* Bit 0:  Local power status (read)*/
#define OHCI_RHSTATUS_CGP          (1 << 0)   /* Bit 0:  Clear global power (write)*/
#define OHCI_RHSTATUS_OCI          (1 << 1)   /* Bit 1:  Over current indicator */
                                              /* Bits 2-14: Reserved */
#define OHCI_RHSTATUS_DRWE         (1 << 15)  /* Bit 15: Device remote wakeup enable */
#define OHCI_RHSTATUS_LPSC         (1 << 16)  /* Bit 16: Local power status change (read) */
#define OHCI_RHSTATUS_SGP          (1 << 16)  /* Bit 16: Set global power (write) */
#define OHCI_RHSTATUS_OCIC         (1 << 17)  /* Bit 17: Overcurrent indicator change */
                                              /* Bits 18-30: Reserved */
#define OHCI_RHSTATUS_CRWE         (1 << 31)  /* Bit 31: Clear remote wakeup enable */

/* HcRhPortStatus: Root hub port status (7.4.4) */

#define OHCI_RHPORTST_CCS          (1 << 0)   /* Bit 0:  Current connect status */
#define OHCI_RHPORTST_PES          (1 << 1)   /* Bit 1:  Port enable status */
#define OHCI_RHPORTST_PSS          (1 << 2)   /* Bit 2:  Port suspend status */
#define OHCI_RHPORTST_POCI         (1 << 3)   /* Bit 3:  Port over current indicator */
#define OHCI_RHPORTST_PRS          (1 << 4)   /* Bit 4:  Port reset status */
                                              /* Bits 5-7: Reserved */
#define OHCI_RHPORTST_PPS          (1 << 8)   /* Bit 8:  Port power status */
#define OHCI_RHPORTST_LSDA         (1 << 9)   /* Bit 9:  Low speed device attached */
                                              /* Bits 10-15: Reserved */
#define OHCI_RHPORTST_CSC          (1 << 16)  /* Bit 16: Connect status change */
#define OHCI_RHPORTST_PESC         (1 << 17)  /* Bit 17: Port enable status change */
#define OHCI_RHPORTST_PSSC         (1 << 18)  /* Bit 18: Port suspend status change */
#define OHCI_RHPORTST_OCIC         (1 << 19)  /* Bit 19: Port over current indicator change */
#define OHCI_RHPORTST_PRSC         (1 << 20)  /* Bit 20: Port reset status change */
                                              /* Bits 21-31: Reserved */

/* Transfer Descriptors *****************************************************/
/* Endpoint Descriptor Offsets (4.2.1) */

#define ED_CONTROL_OFFSET          (0x00)     /* ED status/control bits */
#define ED_TAILP_OFFSET            (0x04)     /* TD Queue Tail Pointer (TailP) */
#define ED_HEADP_OFFSET            (0x08)     /* TD Queue Head Pointer (HeadP) */
#define ED_NEXTED_OFFSET           (0x0c)     /* Next Endpoint Descriptor (NextED) */

/* Endpoint Descriptor Bit Definitions (4.2.2) */

#define ED_CONTROL_FA_SHIFT        (0)        /* Bits 0-6: Function Address */
#define ED_CONTROL_FA_MASK         (0x7f << ED_CONTROL_FA_SHIFT)
#define ED_CONTROL_EN_SHIFT        (7)        /* Bits 7-10: Endpoint number */
#define ED_CONTROL_EN_MASK         (15 << ED_CONTROL_EN_SHIFT)
#define ED_CONTROL_D_SHIFT         (11)       /* Bits 11-12: Direction */
#define ED_CONTROL_D_MASK          (3 << ED_CONTROL_D_SHIFT)
#  define ED_CONTROL_D_TD1         (0 << ED_CONTROL_D_SHIFT) /* Get direction from TD */
#  define ED_CONTROL_D_OUT         (1 << ED_CONTROL_D_SHIFT) /* OUT */
#  define ED_CONTROL_D_IN          (2 << ED_CONTROL_D_SHIFT) /* IN */
#  define ED_CONTROL_D_TD2         (3 << ED_CONTROL_D_SHIFT) /* Get direction from TD */
#define ED_CONTROL_S               (1 << 13)  /* Bit 13: Speed (low) */
#define ED_CONTROL_K               (1 << 14)  /* Bit 14: Skip */
#define ED_CONTROL_F               (1 << 15)  /* Bit 15: Format (isochronous) */
#define ED_CONTROL_MPS_SHIFT       (16)       /* Bits 16-26: Maximum packet size */
#define ED_CONTROL_MPS_MASK        (0x7ff << ED_CONTROL_MPS_SHIFT)

#define ED_HEADP_ADDR_SHIFT        (0)
#define ED_HEADP_ADDR_MASK         0xfffffff0
#define ED_HEADP_H                 (1 << 0)  /* Bit 0: Halted */
#define ED_HEADP_C                 (1 << 1)  /* Bit 1: Toggle carry */

/* General Transfer Descriptor Offsets (4.3.1) */

#define GTD_STATUS_OFFSET          (0x00)     /* TD status bits */
#define GTD_CBP_OFFSET             (0x04)     /* Current Buffer Pointer (CBP) */
#define GTD_NEXTTD_OFFSET          (0x08)     /* Next TD (NextTD) */
#define GTD_BE_OFFSET              (0x0c)     /* Buffer End (BE) */

/* General Transfer Descriptor Bit Definitions */
                                              /* Bits 0-17: Reserved */
#define GTD_STATUS_R               (1 << 18)  /* Bit 18: Buffer rounding */
#define GTD_STATUS_DP_SHIFT        (19)       /* Bits 19-20: Direction/PID */
#define GTD_STATUS_DP_MASK         (3 << GTD_STATUS_DP_SHIFT)
#  define GTD_STATUS_DP_SETUP      (0 << GTD_STATUS_DP_SHIFT) /* To endpoint */
#  define GTD_STATUS_DP_OUT        (1 << GTD_STATUS_DP_SHIFT) /* To endpoint */
#  define GTD_STATUS_DP_IN         (2 << GTD_STATUS_DP_SHIFT) /* From endpoint */
#define GTD_STATUS_DI_SHIFT        (21)      /* Bits 21-23: Delay input */
#define GTD_STATUS_DI_MASK         (7 << GTD_STATUS_DI_SHIFT)
#define GTD_STATUS_T_SHIFT         (24)      /* Bits 24-25: Data Toggle */
#define GTD_STATUS_T_MASK          (3 << GTD_STATUS_T_SHIFT)
#  define GTD_STATUS_T_TOGGLE      (0 << GTD_STATUS_T_SHIFT)
#  define GTD_STATUS_T_DATA0       (2 << GTD_STATUS_T_SHIFT)
#  define GTD_STATUS_T_DATA1       (3 << GTD_STATUS_T_SHIFT)
#define GTD_STATUS_EC_SHIFT        (26)      /* Bits 26-27: Error count */
#define GTD_STATUS_EC_MASK         (3 << GTD_STATUS_EC_SHIFT)
#define GTD_STATUS_CC_SHIFT        (28)      /* Bits 28-31: Condition code */
#define GTD_STATUS_CC_MASK         (15 << GTD_STATUS_CC_SHIFT)

/* Isochronous Transfer Descriptor Offsets (4.3.2) */

#define ITD_STATUS_OFFSET          (0x00)    /* TD status bits */
#define ITD_BP0_OFFSET             (0x04)    /* Buffer page 0 (BP0) */
#define ITD_NEXTTD_OFFSET          (0x08)    /* Next TD (NextTD) */
#define ITD_BE_OFFSET              (0x0c)    /* Buffer End (BE) */

#define ITD_NPSW                   (8)
#define ITD_PSW0_OFFSET            (0x10)    /* Offset0/PSW0 */
#define ITD_PSW1_OFFSET            (0x12)    /* Offset1/PSW1 */
#define ITD_PSW2_OFFSET            (0x14)    /* Offset2/PSW2 */
#define ITD_PSW3_OFFSET            (0x16)    /* Offset3/PSW3 */
#define ITD_PSW4_OFFSET            (0x18)    /* Offset4/PSW4 */
#define ITD_PSW5_OFFSET            (0x1a)    /* Offset5/PSW5 */
#define ITD_PSW6_OFFSET            (0x1c)    /* Offset6/PSW6 */
#define ITD_PSW7_OFFSET            (0x1e)    /* Offset7/PSW7 */

/* Condition codes (Table 4-7) */

#define TD_CC_NOERROR              0x00
#define TD_CC_CRC                  0x01
#define TD_CC_BITSTUFFING          0x02
#define TD_CC_DATATOGGLEMISMATCH   0x03
#define TD_CC_STALL                0x04
#define TD_CC_DEVNOTRESPONDING     0x05
#define TD_CC_PIDCHECKFAILURE      0x06
#define TD_CC_UNEXPECTEDPID        0x07
#define TD_CC_DATAOVERRUN          0x08
#define TD_CC_DATAUNDERRUN         0x09
#define TD_CC_BUFFEROVERRUN        0x0c
#define TD_CC_BUFFERUNDERRUN       0x0d
#define TD_CC_NOTACCESSED          0x0f

/* Host Controller Communications Area Format (4.4.1) ***********************/

/* HccaInterrruptTable: 32x32-bit pointers to interrupt EDs */

#define HCCA_INTTBL_OFFSET         (0x00)
#define HCCA_INTTBL_WSIZE          (32)
#define HCCA_INTTBL_BSIZE          (HCCA_INTTBL_WSIZE*4)

/* HccaFrameNumber: Current frame number */

#define HCCA_FMNO_OFFSET           (0x80)
#define HCCA_FMNO_BSIZE            (2)

/* HccaPad1: Zero when frame no. updated */

#define HCCA_PAD1_OFFSET           (0x82)
#define HCCA_PAD1_BSIZE            (2)

/* HccaDoneHead: When the HC reaches the end of a frame and its deferred
 * interrupt register is 0, it writes the current value of its HcDoneHead to
 * this location and generates an interrupt.
 */

#define HCCA_DONEHEAD_OFFSET       (0x84)
#define HCCA_DONEHEAD_BSIZE        (4)

/* 0x88: 116 bytes reserved */

#define HCCA_RESERVED_OFFSET       (0x88)
#define HCCA_RESERVED_BSIZE        (116)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Endpoint Descriptor Offsets (4.2.1) */

struct ohci_ed_s
{
  volatile  uint32_t ctrl;         /* ED status/control bits */
  volatile  uint32_t tailp;        /* TD Queue Tail Pointer (TailP) */
  volatile  uint32_t headp;        /* TD Queue Head Pointer (HeadP) */
  volatile  uint32_t nexted;       /* Next Endpoint Descriptor (NextED) */
};

/* General Transfer Descriptor (4.3.1) */

struct ohci_gtd_s
{
  volatile uint32_t ctrl;          /* TD status/control bits */
  volatile uint32_t cbp;           /* Current Buffer Pointer (CBP) */
  volatile uint32_t nexttd;        /* Next TD (NextTD) */
  volatile uint32_t be;            /* Buffer End (BE) */
};

/* Isochronous Transfer Descriptor Offsets (4.3.2) */

struct ohci_itd_s
{
  volatile uint32_t ctrl;          /* TD status/control bits */
  volatile uint32_t bp0;           /* Buffer page 0 (BP0 */
  volatile uint32_t nexttd;        /* Next TD (NextTD) */
  volatile uint32_t be;            /* Buffer End (BE) */
  volatile uint16_t psw[ITD_NPSW]; /* Offset/PSW */
};

/* Host Controller Communications Area Format (4.4.1) */
 
struct ohci_hcca_s
{
  /* HccaInterrruptTable: 32x32-bit pointers to interrupt EDs */

  volatile uint32_t  inttbl[HCCA_INTTBL_WSIZE];

  /* HccaFrameNumber: Current frame number and 
   * HccaPad1: Zero when frame no. updated
   */

  volatile uint16_t fmno;
  volatile uint16_t pad1;

  /* HccaDoneHead: When the HC reaches the end of a frame and its deferred
   * interrupt register is 0, it writes the current value of its HcDoneHead to
   * this location and generates an interrupt.
   */

  volatile uint32_t donehead;
  volatile uint8_t  reserved[HCCA_RESERVED_BSIZE];
  volatile uint32_t extra;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_USB_OHCI_H */
