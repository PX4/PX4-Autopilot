/************************************************************************************
 * arch/arm/src/imx/imx_usbd.h
 *
 *   Copyright (c) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_IMX_USBD_H
#define __ARCH_ARM_IMX_USBD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* USBD Register Offsets ************************************************************/

#define USBD_FRAME_OFFSET          0x0000
#define USBD_SPEC_OFFSET           0x0004
#define USBD_STAT_OFFSET           0x0008
#define USBD_CTRL_OFFSET           0x000c
#define USBD_DADR_OFFSET           0x0010
#define USBD_DDAT_OFFSET           0x0014
#define USBD_INTR_OFFSET           0x0018
#define USBD_MASK_OFFSET           0x001c
#define USBD_ENAB_OFFSET           0x0024

#define USBD_EP0_OFFSET            0x0030
#define USBD_EP1_OFFSET            0x0060
#define USBD_EP2_OFFSET            0x0090
#define USBD_EP3_OFFSET            0x00c0
#define USBD_EP4_OFFSET            0x00f0
#define USBD_EP5_OFFSET            0x0120
#define USBD_EP_OFFSET(n)          (USBD_EP0_OFFSET + (n)*0x0030)

#define USBD_EP_STAT_OFFSET        0x0000
#define USBD_EP_INTR_OFFSET        0x0004
#define USBD_EP_MASK_OFFSET        0x0008
#define USBD_EP_FDAT_OFFSET        0x000c
#define USBD_EP_FSTAT_OFFSET       0x0010
#define USBD_EP_FCTRL_OFFSET       0x0014
#define USBD_EP_LRFP_OFFSET        0x0018
#define USBD_EP_LRWP_OFFSET        0x001c
#define USBD_EP_FALRM_OFFSET       0x0020
#define USBD_EP_FRDP_OFFSET        0x0024
#define USBD_EP_FRWP_OFFSET        0x0028

/* USBD Register Addresses **********************************************************/

#define IMX_USBD_FRAME             (IMX_USBD_VBASE + USBD_FRAME_OFFSET)
#define IMX_USBD_SPEC              (IMX_USBD_VBASE + USBD_SPEC_OFFSET)
#define IMX_USBD_STAT              (IMX_USBD_VBASE + USBD_STAT_OFFSET)
#define IMX_USBD_CTRL              (IMX_USBD_VBASE + USBD_CTRL_OFFSET)
#define IMX_USBD_DADR              (IMX_USBD_VBASE + USBD_DADR_OFFSET)
#define IMX_USBD_DDAT              (IMX_USBD_VBASE + USBD_DDAT_OFFSET)
#define IMX_USBD_INTR              (IMX_USBD_VBASE + USBD_INTR_OFFSET)
#define IMX_USBD_MASK              (IMX_USBD_VBASE + USBD_MASK_OFFSET)
#define IMX_USBD_ENAB              (IMX_USBD_VBASE + USBD_ENAB_OFFSET)

#define IMX_USBD_EP0_BASE          (IMX_USBD_VBASE + USBD_EP0_OFFSET)
#define IMX_USBD_EP1_BASE          (IMX_USBD_VBASE + USBD_EP1_OFFSET)
#define IMX_USBD_EP2_BASE          (IMX_USBD_VBASE + USBD_EP2_OFFSET)
#define IMX_USBD_EP3_BASE          (IMX_USBD_VBASE + USBD_EP3_OFFSET)
#define IMX_USBD_EP4_BASE          (IMX_USBD_VBASE + USBD_EP4_OFFSET)
#define IMX_USBD_EP5_BASE          (IMX_USBD_VBASE + USBD_EP5_OFFSET)
#define IMX_USBD_EP_BASE(n)        (IMX_USBD_VBASE + USBD_EP_OFFSET(n))

#define IMX_USBD_EP0_STAT          (IMX_USBD_EP0_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP0_INTR          (IMX_USBD_EP0_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP0_MASK          (IMX_USBD_EP0_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP0_FDAT          (IMX_USBD_EP0_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP0_FSTAT         (IMX_USBD_EP0_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP0_FCTRL         (IMX_USBD_EP0_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP0_LRFP          (IMX_USBD_EP0_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP0_LRWP          (IMX_USBD_EP0_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP0_FALRM         (IMX_USBD_EP0_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP0_FRDP          (IMX_USBD_EP0_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP0_FRWP          (IMX_USBD_EP0_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP1_STAT          (IMX_USBD_EP1_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP1_INTR          (IMX_USBD_EP1_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP1_MASK          (IMX_USBD_EP1_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP1_FDAT          (IMX_USBD_EP1_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP1_FSTAT         (IMX_USBD_EP1_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP1_FCTRL         (IMX_USBD_EP1_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP1_LRFP          (IMX_USBD_EP1_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP1_LRWP          (IMX_USBD_EP1_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP1_FALRM         (IMX_USBD_EP1_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP1_FRDP          (IMX_USBD_EP1_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP1_FRWP          (IMX_USBD_EP1_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP2_STAT          (IMX_USBD_EP2_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP2_INTR          (IMX_USBD_EP2_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP2_MASK          (IMX_USBD_EP2_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP2_FDAT          (IMX_USBD_EP2_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP2_FSTAT         (IMX_USBD_EP2_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP2_FCTRL         (IMX_USBD_EP2_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP2_LRFP          (IMX_USBD_EP2_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP2_LRWP          (IMX_USBD_EP2_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP2_FALRM         (IMX_USBD_EP2_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP2_FRDP          (IMX_USBD_EP2_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP2_FRWP          (IMX_USBD_EP2_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP3_STAT          (IMX_USBD_EP3_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP3_INTR          (IMX_USBD_EP3_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP3_MASK          (IMX_USBD_EP3_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP3_FDAT          (IMX_USBD_EP3_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP3_FSTAT         (IMX_USBD_EP3_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP3_FCTRL         (IMX_USBD_EP3_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP3_LRFP          (IMX_USBD_EP3_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP3_LRWP          (IMX_USBD_EP3_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP3_FALRM         (IMX_USBD_EP3_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP3_FRDP          (IMX_USBD_EP3_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP3_FRWP          (IMX_USBD_EP3_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP4_STAT          (IMX_USBD_EP4_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP4_INTR          (IMX_USBD_EP4_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP4_MASK          (IMX_USBD_EP4_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP4_FDAT          (IMX_USBD_EP4_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP4_FSTAT         (IMX_USBD_EP4_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP4_FCTRL         (IMX_USBD_EP4_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP4_LRFP          (IMX_USBD_EP4_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP4_LRWP          (IMX_USBD_EP4_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP4_FALRM         (IMX_USBD_EP4_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP4_FRDP          (IMX_USBD_EP4_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP4_FRWP          (IMX_USBD_EP4_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP5_STAT          (IMX_USBD_EP5_BASE + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP5_INTR          (IMX_USBD_EP5_BASE + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP5_MASK          (IMX_USBD_EP5_BASE + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP5_FDAT          (IMX_USBD_EP5_BASE + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP5_FSTAT         (IMX_USBD_EP5_BASE + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP5_FCTRL         (IMX_USBD_EP5_BASE + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP5_LRFP          (IMX_USBD_EP5_BASE + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP5_LRWP          (IMX_USBD_EP5_BASE + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP5_FALRM         (IMX_USBD_EP5_BASE + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP5_FRDP          (IMX_USBD_EP5_BASE + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP5_FRWP          (IMX_USBD_EP5_BASE + USBD_EP_FRWP_OFFSET)

#define IMX_USBD_EP_STAT(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_STAT_OFFSET)
#define IMX_USBD_EP_INTR(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_INTR_OFFSET)
#define IMX_USBD_EP_MASK(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_MASK_OFFSET)
#define IMX_USBD_EP_FDAT(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_FDAT_OFFSET)
#define IMX_USBD_EP_FSTAT(n)       (IMX_USBD_EP_BASE(n) + USBD_EP_FSTAT_OFFSET)
#define IMX_USBD_EP_FCTRL(n)       (IMX_USBD_EP_BASE(n) + USBD_EP_FCTRL_OFFSET)
#define IMX_USBD_EP_LRFP(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_LRFP_OFFSET)
#define IMX_USBD_EP_LRWP(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_LRWP_OFFSET)
#define IMX_USBD_EP_FALRM(n)       (IMX_USBD_EP_BASE(n) + USBD_EP_FALRM_OFFSET)
#define IMX_USBD_EP_FRDP(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_FRDP_OFFSET)
#define IMX_USBD_EP_FRWP(n)        (IMX_USBD_EP_BASE(n) + USBD_EP_FRWP_OFFSET)

/* USBD Register Bit Definitions ****************************************************/

/* USBD FRAME Register */

#define USBD_FRAME_FRAME_SHIFT     0         /* Bit 0-10: Frame Field */
#define USBD_FRAME_FRAME_MASK      (0x07ff << USBD_FRAME_FRAME_SHIFT)
#define USBD_FRAME_MATCH_SHIFT     16        /* Bit 16-26: Match Field */
#define USBD_FRAME_MATCH_MASK      (0x07ff << USBD_FRAME_MATCH_SHIFT)

/* USBD STAT Register */

#define USBD_STAT_ALTSET_SHIFT     0         /* Bit 0-2: Alternate Setting */
#define USBD_STAT_ALTSET_MASK      (0x07 << USBD_FRAME_MATCH_SHIFT)
#define USBD_STAT_INTF_SHIFT       3         /* Bit 3-4: Interface */
#define USBD_STAT_INTF_MASK        (0x03 << USBD_FRAME_MATCH_SHIFT)
#define USBD_STAT_CFG_SHIFT        5         /* Bit 5-6: Configuration */
#define USBD_STAT_CFG_MASK         (0x03 << USBD_FRAME_MATCH_SHIFT)
#define USBD_STAT_SUSP             (1 << 7)  /* Bit 7: Suspend */
#define USBD_STAT_RST              (1 << 8)  /* Bit 8: Reset Signaling */

/* USBD CTRL Register */

#define USBD_CTRL_RESUME           (1 << 0)  /* Bit 0: Resume */
#define USBD_CTRL_AFEENA           (1 << 1)  /* Bit 1: Analog Front-End Enable */
#define USBD_CTRL_UDCRST           (1 << 2)  /* Bit 2: UDC Reset */
#define USBD_CTRL_USBENA           (1 << 3)  /* Bit 3: USB Enable */
#define USBD_CTRL_USBSPD           (1 << 4)  /* Bit 4: USB Speed */
#define USBD_CTRL_CMDERROR         (1 << 5)  /* Bit 5: Command Error */
#define USBD_CTRL_CMDOVER          (1 << 6)  /* Bit 6: Command Over */

/* USBD DADR Register */

#define USBD_DADR_DADR_SHIFT        0        /* Bit 0-8: Desired RAM Address */
#define USBD_DADR_DADR_MASK        (0x1ff << USBD_DADR_DADR_SHIFT)
#define USBD_DADR_BSY              (1 << 30) /* Bit 30: Busy */
#define USBD_DADR_CFG              (1 << 31) /* Bit 31: Configuration */

/* USBD DDAT Register */

#define USBD_DDAT_DDAT_SHIFT       0         /* Bit 0-7: Descriptor Data Buffer */
#define USBD_DDAT_DDAT_MASK        (0xff << USBD_DDAT_DDAT_SHIFT)

/* USBD INTR Register */

#define USBD_INTR_CFGCHG           (1 << 0)  /* Bit 0: Configuration Change */
#define USBD_INTR_FRAMEMATCH       (1 << 1)  /* Bit 1: FRAME_MATCH */
#define USBD_INTR_SUSP             (1 << 2)  /* Bit 2: Active to Suspend */
#define USBD_INTR_RES              (1 << 3)  /* Bit 3: Suspend to Resume */
#define USBD_INTR_RESETSTART       (1 << 4)  /* Bit 4: Restart Signaling Start */
#define USBD_INTR_RESETSTOP        (1 << 5)  /* Bit 5: Restart Signaling Stop */
#define USBD_INTR_SOF              (1 << 6)  /* Bit 6: Start-of-Frame Interrupt */
#define USBD_INTR_MSOF             (1 << 7)  /* Bit 7: Missed Start-of-Frame Interrupt */
#define USBD_INTR_WAKEUP           (1 << 31) /* Bit 31: Wakeup */

/* USBD MASK Register */

#define USBD_MASK_CFGCHG           (1 << 0)  /* Bit 0: Configuration Change */
#define USBD_MASK_FRAMEMATCH       (1 << 1)  /* Bit 1: FRAME_MATCH */
#define USBD_MASK_SUSP             (1 << 2)  /* Bit 2: Active to Suspend */
#define USBD_MASK_RES              (1 << 3)  /* Bit 3: Suspend to Resume */
#define USBD_MASK_RESETSTART       (1 << 4)  /* Bit 4: Restart Signaling Start */
#define USBD_MASK_RESETSTOP        (1 << 5)  /* Bit 5: Restart Signaling Stop */
#define USBD_MASK_SOF              (1 << 6)  /* Bit 6: Start-of-Frame Interrupt */
#define USBD_MASK_MSOF             (1 << 7)  /* Bit 7: Missed Start-of-Frame Interrupt */
#define USBD_MASK_WAKEUP           (1 << 31) /* Bit 31: Wakeup */

/* USBD ENAB Register */

#define USBD_ENAB_PWDMD            (1 << 0)  /* Bit 0: Power Mode */
#define USBD_ENAB_ENDIANMODE       (1 << 28) /* Bit 28: Endian Mode Select */
#define USBD_ENAB_SUSPEND          (1 << 29) /* Bit 29: Suspend */
#define USBD_ENAB_ENAB             (1 << 30) /* Bit 30: Enable */
#define USBD_ENAB_RST              (1 << 31) /* Bit 31: Reset */

/* USBD EPSTAT Register */

#define USBD_EPSTAT_FORCESTALL     (1 << 0)  /* Bit 0: Force a Stall Condition */
#define USBD_EPSTAT_FLUSH          (1 << 1)  /* Bit 1: Flush */
#define USBD_EPSTAT_ZLPS           (1 << 2)  /* Bit 2: Zero Length Packet Send */
#define USBD_EPSTAT_TYP_SHIFT      3         /* Bit 3-4: Endpoint Type */
#define USBD_EPSTAT_TYP_MASK       (0x03 << USBD_EPSTAT_TYP_SHIFT)
#define USBD_EPSTAT_MAX_SHIFT      5         /* Bit 5-6: Maximum Packet Size */
#define USBD_EPSTAT_MAX_MASK       (0x03 << USBD_EPSTAT_MAX_SHIFT)
#define USBD_EPSTAT_DIR            (1 << 7)  /* Bit 7: Transfer Direction */
#define USBD_EPSTAT_SIP            (1 << 8)  /* Bit 8: Setup Packet in Progress */
#define USBD_EPSTAT_BYTECOUNT_SHIFT 16       /* Bit 16-22: Byte Count */
#define USBD_EPSTAT_BYTECOUNT_MASK (0x7f << USBD_EPSTAT_BYTECOUNT_SHIFT)

/* USBD EPINTR Register */

#define USBD_EPINTR_EOF            (1 << 0)  /* Bit 0: End-of-Frame */
#define USBD_EPINTR_DEVREQ         (1 << 1)  /* Bit 1: Device Request */
#define USBD_EPINTR_EOT            (1 << 2)  /* Bit 2: End of Transfer */
#define USBD_EPINTR_MDEVREQ        (1 << 3)  /* Bit 3: Multiple Device Request */
#define USBD_EPINTR_FIFOLOW        (1 << 4)  /* Bit 4: FIFO Low */
#define USBD_EPINTR_FIFOHIGH       (1 << 5)  /* Bit 5: FIFO High */
#define USBD_EPINTR_FIFOERROR      (1 << 6)  /* Bit 6: FIFO Error */
#define USBD_EPINTR_FIFOEMPTY      (1 << 7)  /* Bit 7: FIFO Empty */
#define USBD_EPINTR_FIFOFULL       (1 << 8)  /* Bit 8: FIFO Full */

/* USBD EPMASK Register */

#define USBD_EPMASK_EOF            (1 << 0)  /* Bit 0: End-of-Frame */
#define USBD_EPMASK_DEVREQ         (1 << 1)  /* Bit 1: Device Request */
#define USBD_EPMASK_EOT            (1 << 2)  /* Bit 2: End of Transfer */
#define USBD_EPMASK_MDEVREQ        (1 << 3)  /* Bit 3: Multiple Device Request */
#define USBD_EPMASK_FIFOLOW        (1 << 4)  /* Bit 4: FIFO Low */
#define USBD_EPMASK_FIFOHIGH       (1 << 5)  /* Bit 5: FIFO High */
#define USBD_EPMASK_FIFOERROR      (1 << 6)  /* Bit 6: FIFO Error */
#define USBD_EPMASK_FIFOEMPTY      (1 << 7)  /* Bit 7: FIFO Empty */
#define USBD_EPMASK_FIFOFULL       (1 << 8)  /* Bit 8: FIFO Full */

/* USBD EPFSTAT Register */

#define USBD_EPFSTAT_EMPTY         (1 << 16) /* Bit 16: FIFO Empty */
#define USBD_EPFSTAT_ALARM         (1 << 17) /* Bit 17: FIFO Alarm */
#define USBD_EPFSTAT_FULL          (1 << 18) /* Bit 18: FIFO Full */
#define USBD_EPFSTAT_FR            (1 << 19) /* Bit 19: FIFO Ready */
#define USBD_EPFSTAT_OF            (1 << 20) /* Bit 20: FIFO Overflow */
#define USBD_EPFSTAT_UF            (1 << 21) /* Bit 21: FIFO Underflow */
#define USBD_EPFSTAT_ERROR         (1 << 22) /* Bit 22: FIFO Error */
#define USBD_EPFSTAT_FRAME3        (1 << 24) /* Bit 24: Frame Status Bit 3 */
#define USBD_EPFSTAT_FRAME2        (1 << 25) /* Bit 25: Frame Status Bit 2 */
#define USBD_EPFSTAT_FRAME1        (1 << 26) /* Bit 26: Frame Status Bit 1 */
#define USBD_EPFSTAT_FRAME0        (1 << 27) /* Bit 27: Frame Status Bit 0 */

/* USBD EPFCTRL Register */

#define USBD_EPCTRL_GR_SHIFT       24        /* Bit 24-26: Granularity */
#define USBD_EPCTRL_GR_MASK        (0x07 << USBD_EPCTRL_GR_SHIFT)
#define USBD_EPCTRL_FRAME          (1 << 27) /* Bit 27: Frame Mode */
#define USBD_EPCTRL_WFR            (1 << 28) /* Bit 29: Write Frame End */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_USBD_H */
