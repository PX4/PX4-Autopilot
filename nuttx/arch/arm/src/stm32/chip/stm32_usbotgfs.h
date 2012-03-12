/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_usbotgfs.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_USBOTG_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_USBOTG_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/
/* Core global control and status registers */

#define STM32_OTGFS_GOTGCTL_OFFSET      0x0000 /* Control and status register */
#define STM32_OTGFS_GOTGINT_OFFSET      0x0004 /* Interrupt register */
#define STM32_OTGFS_GAHBCFG_OFFSET      0x0008 /* AHB configuration register */
#define STM32_OTGFS_GUSBCFG_OFFSET      0x000c /* USB configuration register */
#define STM32_OTGFS_GRSTCTL_OFFSET      0x0010 /* Reset register */
#define STM32_OTGFS_GINTSTS_OFFSET      0x0014 /* Core interrupt register */
#define STM32_OTGFS_GINTMSK_OFFSET      0x0018 /* Interrupt mask register */
#define STM32_OTGFS_GRXSTSR_OFFSET      0x001c /* Receive status debug read/OTG status read register */
#define STM32_OTGFS_GRXSTSP_OFFSET      0x0020 /* Receive status debug read/OTG status pop register */
#define STM32_OTGFS_GRXFSIZ_OFFSET      0x0024 /* Receive FIFO size register */
#define STM32_OTGFS_HNPTXFSIZ_OFFSET    0x0028 /* Host non-periodic transmit FIFO size register */
#define STM32_OTGFS_DIEPTXF0_OFFSET     0x0028 /* Endpoint 0 Transmit FIFO size */
#define STM32_OTGFS_HNPTXSTS_OFFSET     0x002c /* non-periodic transmit FIFO/queue status register */
#define STM32_OTGFS_GCCFG_OFFSET        0x0038 /* general core configuration register */
#define STM32_OTGFS_CID_OFFSET          0x003c /* Core ID register  */
#define STM32_OTGFS_HPTXFSIZ_OFFSET     0x0100 /* Host periodic transmit FIFO size register */

#define STM32_OTGFS_DIEPTXF_OFFSET(n)   (104+(((n)-1) << 2))
#define STM32_OTGFS_DIEPTXF1_OFFSET     0x0104 /* Device IN endpoint transmit FIFO1 size register */
#define STM32_OTGFS_DIEPTXF2_OFFSET     0x0108 /* Device IN endpoint transmit FIFO2 size register */
#define STM32_OTGFS_DIEPTXF3_OFFSET     0x010c /* Device IN endpoint transmit FIFO3 size register */

/* Host-mode control and status registers */

#define STM32_OTGFS_HCFG_OFFSET         0x0400 /* Host configuration register */
#define STM32_OTGFS_HFIR_OFFSET         0x0404 /* Host frame interval register */
#define STM32_OTGFS_HFNUM_OFFSET        0x0408 /* Host frame number/frame time remaining register */
#define STM32_OTGFS_HPTXSTS_OFFSET      0x0410 /* Host periodic transmit FIFO/queue status register */
#define STM32_OTGFS_HAINT_OFFSET        0x0414 /* Host all channels interrupt register */
#define STM32_OTGFS_HAINTMSK_OFFSET     0x0418 /* Host all channels interrupt mask register */
#define STM32_OTGFS_HPRT_OFFSET         0x0440 /* Host port control and status register */

#define STM32_OTGFS_CHAN_OFFSET(n)      (0x500 + ((n) << 5)
#define STM32_OTGFS_HCCHAR_OFFSET       0x0000 /* Host channel characteristics register */
#define STM32_OTGFS_HCINT_OFFSET        0x0008 /* Host channel interrupt register */
#define STM32_OTGFS_HCINTMSK_OFFSET     0x000c /* Host channel interrupt mask register */
#define STM32_OTGFS_HCTSIZ0_OFFSET      0x0010 /* Host channel interrupt register */

#define STM32_OTGFS_HCCHAR_OFFSET(n)    (0x500 + ((n) << 5))
#define STM32_OTGFS_HCCHAR0_OFFSET      0x0500 /* Host channel-0 characteristics register */
#define STM32_OTGFS_HCCHAR1_OFFSET      0x0520 /* Host channel-1 characteristics register */
#define STM32_OTGFS_HCCHAR2_OFFSET      0x0540 /* Host channel-2 characteristics register */
#define STM32_OTGFS_HCCHAR3_OFFSET      0x0560 /* Host channel-3 characteristics register */
#define STM32_OTGFS_HCCHAR4_OFFSET      0x0580 /* Host channel-4 characteristics register */
#define STM32_OTGFS_HCCHAR5_OFFSET      0x05a0 /* Host channel-5 characteristics register */
#define STM32_OTGFS_HCCHAR6_OFFSET      0x05c0 /* Host channel-6 characteristics register */
#define STM32_OTGFS_HCCHAR7_OFFSET      0x05e0 /* Host channel-7 characteristics register */

#define STM32_OTGFS_HCINT_OFFSET(n)     (0x508 + ((n) << 5))
#define STM32_OTGFS_HCINT0_OFFSET       0x0508 /* Host channel-0 interrupt register */
#define STM32_OTGFS_HCINT1_OFFSET       0x0528 /* Host channel-1 interrupt register */
#define STM32_OTGFS_HCINT2_OFFSET       0x0548 /* Host channel-2 interrupt register */
#define STM32_OTGFS_HCINT3_OFFSET       0x0568 /* Host channel-3 interrupt register */
#define STM32_OTGFS_HCINT4_OFFSET       0x0588 /* Host channel-4 interrupt register */
#define STM32_OTGFS_HCINT5_OFFSET       0x05a8 /* Host channel-5 interrupt register */
#define STM32_OTGFS_HCINT6_OFFSET       0x05c8 /* Host channel-6 interrupt register */
#define STM32_OTGFS_HCINT7_OFFSET       0x05e8 /* Host channel-7 interrupt register */

#define STM32_OTGFS_HCINTMSK_OFFSET(n)  (0x50c + ((n) << 5))
#define STM32_OTGFS_HCINTMSK0_OFFSET    0x050c /* Host channel-0 interrupt mask register */
#define STM32_OTGFS_HCINTMSK1_OFFSET    0x052c /* Host channel-1 interrupt mask register */
#define STM32_OTGFS_HCINTMSK2_OFFSET    0x054c /* Host channel-2 interrupt mask register */
#define STM32_OTGFS_HCINTMSK3_OFFSET    0x056c /* Host channel-3 interrupt mask register */
#define STM32_OTGFS_HCINTMSK4_OFFSET    0x058c /* Host channel-4 interrupt mask register */
#define STM32_OTGFS_HCINTMSK5_OFFSET    0x05ac /* Host channel-5 interrupt mask register */
#define STM32_OTGFS_HCINTMSK6_OFFSET    0x05cc /* Host channel-6 interrupt mask register */
#define STM32_OTGFS_HCINTMSK7_OFFSET    0x05ec /* Host channel-7 interrupt mask register */

#define STM32_OTGFS_HCTSIZ_OFFSET(n)    (0x510 + ((n) << 5))
#define STM32_OTGFS_HCTSIZ0_OFFSET      0x0510 /* Host channel-0 interrupt register */
#define STM32_OTGFS_HCTSIZ1_OFFSET      0x0530 /* Host channel-1 interrupt register */
#define STM32_OTGFS_HCTSIZ2_OFFSET      0x0550 /* Host channel-2 interrupt register */
#define STM32_OTGFS_HCTSIZ3_OFFSET      0x0570 /* Host channel-3 interrupt register */
#define STM32_OTGFS_HCTSIZ4_OFFSET      0x0590 /* Host channel-4 interrupt register */
#define STM32_OTGFS_HCTSIZ5_OFFSET      0x05b0 /* Host channel-5 interrupt register */
#define STM32_OTGFS_HCTSIZ6_OFFSET      0x05d9 /* Host channel-6 interrupt register */
#define STM32_OTGFS_HCTSIZ7_OFFSET      0x05f9 /* Host channel-7 interrupt register */

/* Device-mode control and status registers */

#define STM32_OTGFS_DCFG_OFFSET         0x0800 /* Device configuration register */
#define STM32_OTGFS_DCTL_OFFSET         0x0804 /* Device control register */
#define STM32_OTGFS_DSTS_OFFSET         0x0808 /* Device status register */
#define STM32_OTGFS_DIEPMSK_OFFSET      0x0810 /* Device IN endpoint common interrupt mask register */
#define STM32_OTGFS_DOEPMSK_OFFSET      0x0814 /* Device OUT endpoint common interrupt mask register */
#define STM32_OTGFS_DAINT_OFFSET        0x0818 /* Device all endpoints interrupt register */
#define STM32_OTGFS_DAINTMSK_OFFSET     0x081c /* All endpoints interrupt mask register */
#define STM32_OTGFS_DVBUSDIS_OFFSET     0x0828 /* Device VBUS discharge time register */
#define STM32_OTGFS_DVBUSPULSE_OFFSET   0x082c /* Device VBUS pulsing time register */
#define STM32_OTGFS_DIEPEMPMSK_OFFSET   0x0834 /* Device IN endpoint FIFO empty interrupt mask register */

#define STM32_OTGFS_DIEP_OFFSET(n)      (0x0900 + ((n) << 5)) 
#define STM32_OTGFS_DIEPCTL_OFFSET      0x0900 /* Device endpoint control register */
#define STM32_OTGFS_DIEPINT_OFFSET      0x0008 /* Device endpoint interrupt register */
#define STM32_OTGFS_DIEPTSIZ_OFFSET     0x0010 /* Device IN endpoint transfer size register */
#define STM32_OTGFS_DTXFSTS_OFFSET      0x0018 /* Device IN endpoint transmit FIFO status register */

#define STM32_OTGFS_DIEPCTL_OFFSET(n)   (0x0900 + ((n) << 5))
#define STM32_OTGFS_DIEPCTL0_OFFSET     0x0900 /* Device control IN endpoint 0 control register */
#define STM32_OTGFS_DIEPCTL1_OFFSET     0x0920 /* Device control IN endpoint 2 control register */
#define STM32_OTGFS_DIEPCTL2_OFFSET     0x0940 /* Device control IN endpoint 3 control register */
#define STM32_OTGFS_DIEPCTL3_OFFSET     0x0960 /* Device control IN endpoint 4 control register */

#define STM32_OTGFS_DIEPINT_OFFSET(n)   0x0908 /* Device endpoint-n interrupt register */
#define STM32_OTGFS_DIEPINT0_OFFSET     0x0908 /* Device endpoint-0 interrupt register */
#define STM32_OTGFS_DIEPINT1_OFFSET     0x0928 /* Device endpoint-1 interrupt register */
#define STM32_OTGFS_DIEPINT2_OFFSET     0x0948 /* Device endpoint-2 interrupt register */
#define STM32_OTGFS_DIEPINT3_OFFSET     0x0968 /* Device endpoint-3 interrupt register */

#define STM32_OTGFS_DIEPTSIZ_OFFSET(n)  (0x910 + ((n) << 5))
#define STM32_OTGFS_DIEPTSIZ0_OFFSET    0x0910 /* Device IN endpoint 0 transfer size register */
#define STM32_OTGFS_DIEPTSIZ1_OFFSET    0x0930 /* Device IN endpoint 1 transfer size register */
#define STM32_OTGFS_DIEPTSIZ2_OFFSET    0x0950 /* Device IN endpoint 2 transfer size register */
#define STM32_OTGFS_DIEPTSIZ3_OFFSET    0x0970 /* Device IN endpoint 3 transfer size register */

#define STM32_OTGFS_DTXFSTS_OFFSET(n)   (0x0918 + ((n) << 5))
#define STM32_OTGFS_DTXFSTS0_OFFSET     0x0918 /* Device OUT endpoint-0 transfer size register */
#define STM32_OTGFS_DTXFSTS1_OFFSET     0x0938 /* Device OUT endpoint-1 transfer size register */
#define STM32_OTGFS_DTXFSTS2_OFFSET     0x0958 /* Device OUT endpoint-2 transfer size register */
#define STM32_OTGFS_DTXFSTS3_OFFSET     0x0978 /* Device OUT endpoint-3 transfer size register */

#define STM32_OTGFS_DOEP_OFFSET(n)      (0x0b00 + ((n) << 5))
#define STM32_OTGFS_DOEPCTL_OFFSET      0x0000 /* Device control OUT endpoint 0 control register */
#define STM32_OTGFS_DOEPINT_OFFSET      0x0008 /* Device endpoint-x interrupt register */

#define STM32_OTGFS_DOEPCTL_OFFSET(n)   (0x0b00 + ((n) << 5))
#define STM32_OTGFS_DOEPCTL0_OFFSET     0x00b00 /* Device OUT endpoint 0 control register */
#define STM32_OTGFS_DOEPCTL1_OFFSET     0x00b20 /* Device OUT endpoint 1 control register */
#define STM32_OTGFS_DOEPCTL2_OFFSET     0x00b40 /* Device OUT endpoint 2 control register */
#define STM32_OTGFS_DOEPCTL3_OFFSET     0x00b60 /* Device OUT endpoint 3 control register */

#define STM32_OTGFS_DOEPINT_OFFSET(n)   (0x0b08 + ((n) << 5))
#define STM32_OTGFS_DOEPINT0_OFFSET     0x00b08 /* Device endpoint-0 interrupt register */
#define STM32_OTGFS_DOEPINT1_OFFSET     0x00b28 /* Device endpoint-1 interrupt register */
#define STM32_OTGFS_DOEPINT2_OFFSET     0x00b48 /* Device endpoint-2 interrupt register */
#define STM32_OTGFS_DOEPINT3_OFFSET     0x00b68 /* Device endpoint-3 interrupt register */

#define STM32_OTGFS_DOEPTSIZ_OFFSET(n)  (0x0b10 + ((n) << 5))
#define STM32_OTGFS_DOEPTSIZ0_OFFSET    0x00b10 /* Device OUT endpoint-0 transfer size register */
#define STM32_OTGFS_DOEPTSIZ1_OFFSET    0x00b30 /* Device OUT endpoint-1 transfer size register */
#define STM32_OTGFS_DOEPTSIZ2_OFFSET    0x00b50 /* Device OUT endpoint-2 transfer size register */
#define STM32_OTGFS_DOEPTSIZ3_OFFSET    0x00b70 /* Device OUT endpoint-3 transfer size register */

/* Data FIFO (DFIFO) access registers */

#define STM32_OTGFS_DFIFO_DEP_OFFSET(n) (0x1000 + ((n) << 12))
#define STM32_OTGFS_DFIFO_HCH_OFFSET(n) (0x1000 + ((n) << 12))

#define STM32_OTGFS_DFIFO_DEP0_OFFSET   0x1000 /* 0x1000–0x1ffc Device IN/OUT Endpoint 0 DFIFO Write/Read Access */
#define STM32_OTGFS_DFIFO_HCH0_OFFSET   0x1000 /* 0x1000–0x1ffc Host OUT/IN Channel 0 DFIFO Read/Write Access */

#define STM32_OTGFS_DFIFO_DEP1_OFFSET   0x2000 /* 0x2000–0x2ffc Device IN/OUT Endpoint 0 DFIFO Write/Read Access */
#define STM32_OTGFS_DFIFO_HCH1_OFFSET   0x2000 /* 0x2000–0x2ffc Host OUT/IN Channel 0 DFIFO Read/Write Access */

#define STM32_OTGFS_DFIFO_DEP2_OFFSET   0x3000 /* 0x3000–0x3ffc Device IN/OUT Endpoint 0 DFIFO Write/Read Access */
#define STM32_OTGFS_DFIFO_HCH2_OFFSET   0x3000 /* 0x3000–0x3ffc Host OUT/IN Channel 0 DFIFO Read/Write Access */

#define STM32_OTGFS_DFIFO_DEP3_OFFSET   0x4000 /* 0x4000–0x4ffc Device IN/OUT Endpoint 0 DFIFO Write/Read Access */
#define STM32_OTGFS_DFIFO_HCH3_OFFSET   0x4000 /* 0x4000–0x4ffc Host OUT/IN Channel 0 DFIFO Read/Write Access */

/* Power and clock gating registers */

#define STM32_OTGFS_PCGCCTL_OFFSET      0x0e00 /* Power and clock gating control register */

/* Register Addresses *******************************************************************************/

#define STM32_OTGFS_GOTGCTL             (STM32_OTGFS_BASE+STM32_OTGFS_GOTGCTL_OFFSET)
#define STM32_OTGFS_GOTGINT             (STM32_OTGFS_BASE+STM32_OTGFS_GOTGINT_OFFSET)
#define STM32_OTGFS_GAHBCFG             (STM32_OTGFS_BASE+STM32_OTGFS_GAHBCFG_OFFSET)
#define STM32_OTGFS_GUSBCFG             (STM32_OTGFS_BASE+STM32_OTGFS_GUSBCFG_OFFSET)
#define STM32_OTGFS_GRSTCTL             (STM32_OTGFS_BASE+STM32_OTGFS_GRSTCTL_OFFSET)
#define STM32_OTGFS_GINTSTS             (STM32_OTGFS_BASE+STM32_OTGFS_GINTSTS_OFFSET)
#define STM32_OTGFS_GINTMSK             (STM32_OTGFS_BASE+STM32_OTGFS_GINTMSK_OFFSET)
#define STM32_OTGFS_GRXSTSR             (STM32_OTGFS_BASE+STM32_OTGFS_GRXSTSR_OFFSET)
#define STM32_OTGFS_GRXSTSP             (STM32_OTGFS_BASE+STM32_OTGFS_GRXSTSP_OFFSET)
#define STM32_OTGFS_GRXFSIZ             (STM32_OTGFS_BASE+STM32_OTGFS_GRXFSIZ_OFFSET)
#define STM32_OTGFS_HNPTXFSIZ           (STM32_OTGFS_BASE+STM32_OTGFS_HNPTXFSIZ_OFFSET)
#define STM32_OTGFS_DIEPTXF0            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTXF0_OFFSET)
#define STM32_OTGFS_HNPTXSTS            (STM32_OTGFS_BASE+STM32_OTGFS_HNPTXSTS_OFFSET)
#define STM32_OTGFS_GCCFG               (STM32_OTGFS_BASE+STM32_OTGFS_GCCFG_OFFSET)
#define STM32_OTGFS_CID                 (STM32_OTGFS_BASE+STM32_OTGFS_CID_OFFSET)
#define STM32_OTGFS_HPTXFSIZ            (STM32_OTGFS_BASE+STM32_OTGFS_HPTXFSIZ_OFFSET)

#define STM32_OTGFS_DIEPTXF(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTXF_OFFSET(n))
#define STM32_OTGFS_DIEPTXF1            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTXF1_OFFSET)
#define STM32_OTGFS_DIEPTXF2            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTXF2_OFFSET)
#define STM32_OTGFS_DIEPTXF3            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTXF3_OFFSET)

/* Host-mode control and status registers */

#define STM32_OTGFS_HCFG                (STM32_OTGFS_BASE+STM32_OTGFS_HCFG_OFFSET)
#define STM32_OTGFS_HFIR                (STM32_OTGFS_BASE+STM32_OTGFS_HFIR_OFFSET)
#define STM32_OTGFS_HFNUM               (STM32_OTGFS_BASE+STM32_OTGFS_HFNUM_OFFSET)
#define STM32_OTGFS_HPTXSTS             (STM32_OTGFS_BASE+STM32_OTGFS_HPTXSTS_OFFSET)
#define STM32_OTGFS_HAINT               (STM32_OTGFS_BASE+STM32_OTGFS_HAINT_OFFSET)
#define STM32_OTGFS_HAINTMSK            (STM32_OTGFS_BASE+STM32_OTGFS_HAINTMSK_OFFSET)
#define STM32_OTGFS_HPRT                (STM32_OTGFS_BASE+STM32_OTGFS_HPRT_OFFSET)

#define STM32_OTGFS_CHAN(n)             (STM32_OTGFS_BASE+STM32_OTGFS_CHAN_OFFSET(n))

#define STM32_OTGFS_HCCHAR(n)           (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR_OFFSET(n))
#define STM32_OTGFS_HCCHAR0             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR0_OFFSET)
#define STM32_OTGFS_HCCHAR1             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR1_OFFSET)
#define STM32_OTGFS_HCCHAR2             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR2_OFFSET)
#define STM32_OTGFS_HCCHAR3             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR3_OFFSET)
#define STM32_OTGFS_HCCHAR4             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR4_OFFSET)
#define STM32_OTGFS_HCCHAR5             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR5_OFFSET)
#define STM32_OTGFS_HCCHAR6             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR6_OFFSET)
#define STM32_OTGFS_HCCHAR7             (STM32_OTGFS_BASE+STM32_OTGFS_HCCHAR7_OFFSET)

#define STM32_OTGFS_HCINT(n)            (STM32_OTGFS_BASE+STM32_OTGFS_HCINT_OFFSET(n))
#define STM32_OTGFS_HCINT0              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT0_OFFSET)
#define STM32_OTGFS_HCINT1              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT1_OFFSET)
#define STM32_OTGFS_HCINT2              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT2_OFFSET)
#define STM32_OTGFS_HCINT3              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT3_OFFSET)
#define STM32_OTGFS_HCINT4              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT4_OFFSET)
#define STM32_OTGFS_HCINT5              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT5_OFFSET)
#define STM32_OTGFS_HCINT6              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT6_OFFSET)
#define STM32_OTGFS_HCINT7              (STM32_OTGFS_BASE+STM32_OTGFS_HCINT7_OFFSET)

#define STM32_OTGFS_HCINTMSK(n)         (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK_OFFSET(n))
#define STM32_OTGFS_HCINTMSK0           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK0_OFFSET)
#define STM32_OTGFS_HCINTMSK1           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK1_OFFSET)
#define STM32_OTGFS_HCINTMSK2           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK2_OFFSET)
#define STM32_OTGFS_HCINTMSK3           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK3_OFFSET)
#define STM32_OTGFS_HCINTMSK4           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK4_OFFSET)
#define STM32_OTGFS_HCINTMSK5           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK5_OFFSET)
#define STM32_OTGFS_HCINTMSK6           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK6_OFFSET)
#define STM32_OTGFS_HCINTMSK7           (STM32_OTGFS_BASE+STM32_OTGFS_HCINTMSK7_OFFSET)_

#define STM32_OTGFS_HCTSIZ(n)           (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ_OFFSET(n))
#define STM32_OTGFS_HCTSIZ0             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ0_OFFSET)
#define STM32_OTGFS_HCTSIZ1             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ1_OFFSET)
#define STM32_OTGFS_HCTSIZ2             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ2_OFFSET)
#define STM32_OTGFS_HCTSIZ3             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ3_OFFSET)
#define STM32_OTGFS_HCTSIZ4             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ4_OFFSET)
#define STM32_OTGFS_HCTSIZ5             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ5_OFFSET)
#define STM32_OTGFS_HCTSIZ6             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ6_OFFSET)
#define STM32_OTGFS_HCTSIZ7             (STM32_OTGFS_BASE+STM32_OTGFS_HCTSIZ7_OFFSET)

/* Device-mode control and status registers */

#define STM32_OTGFS_DCFG                (STM32_OTGFS_BASE+STM32_OTGFS_DCFG_OFFSET)
#define STM32_OTGFS_DCTL                (STM32_OTGFS_BASE+STM32_OTGFS_DCTL_OFFSET)
#define STM32_OTGFS_DSTS                (STM32_OTGFS_BASE+STM32_OTGFS_DSTS_OFFSET)
#define STM32_OTGFS_DIEPMSK             (STM32_OTGFS_BASE+STM32_OTGFS_DIEPMSK_OFFSET)
#define STM32_OTGFS_DOEPMSK             (STM32_OTGFS_BASE+STM32_OTGFS_DOEPMSK_OFFSET)
#define STM32_OTGFS_DAINT               (STM32_OTGFS_BASE+STM32_OTGFS_DAINT_OFFSET)
#define STM32_OTGFS_DAINTMSK            (STM32_OTGFS_BASE+STM32_OTGFS_DAINTMSK_OFFSET)
#define STM32_OTGFS_DVBUSDIS            (STM32_OTGFS_BASE+STM32_OTGFS_DVBUSDIS_OFFSET)
#define STM32_OTGFS_DVBUSPULSE          (STM32_OTGFS_BASE+STM32_OTGFS_DVBUSPULSE_OFFSET)
#define STM32_OTGFS_DIEPEMPMSK          (STM32_OTGFS_BASE+STM32_OTGFS_DIEPEMPMSK_OFFSET)

#define STM32_OTGFS_DIEP(n)             (STM32_OTGFS_BASE+STM32_OTGFS_DIEP_OFFSET(n))
#define STM32_OTGFS_DIEPCTL             (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL_OFFSET)
#define STM32_OTGFS_DIEPINT             (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT_OFFSET)
#define STM32_OTGFS_DIEPTSIZ            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ_OFFSET)
#define STM32_OTGFS_DTXFSTS             (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS_OFFSET)

#define STM32_OTGFS_DIEPCTL(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL_OFFSET(n))
#define STM32_OTGFS_DIEPCTL0            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL0_OFFSET)
#define STM32_OTGFS_DIEPCTL1            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL1_OFFSET)
#define STM32_OTGFS_DIEPCTL2            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL2_OFFSET)
#define STM32_OTGFS_DIEPCTL3            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPCTL3_OFFSET)

#define STM32_OTGFS_DIEPINT(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT_OFFSET(n))
#define STM32_OTGFS_DIEPINT0            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT0_OFFSET)
#define STM32_OTGFS_DIEPINT1            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT1_OFFSET)
#define STM32_OTGFS_DIEPINT2            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT2_OFFSET)
#define STM32_OTGFS_DIEPINT3            (STM32_OTGFS_BASE+STM32_OTGFS_DIEPINT3_OFFSET)

#define STM32_OTGFS_DIEPTSIZ(n)         (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ_OFFSET(n))
#define STM32_OTGFS_DIEPTSIZ0           (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ0_OFFSET)
#define STM32_OTGFS_DIEPTSIZ1           (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ1_OFFSET)
#define STM32_OTGFS_DIEPTSIZ2           (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ2_OFFSET)
#define STM32_OTGFS_DIEPTSIZ3           (STM32_OTGFS_BASE+STM32_OTGFS_DIEPTSIZ3_OFFSET)

#define STM32_OTGFS_DTXFSTS(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS_OFFSET(n))
#define STM32_OTGFS_DTXFSTS0            (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS0_OFFSET)
#define STM32_OTGFS_DTXFSTS1            (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS1_OFFSET)
#define STM32_OTGFS_DTXFSTS2            (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS2_OFFSET)
#define STM32_OTGFS_DTXFSTS3            (STM32_OTGFS_BASE+STM32_OTGFS_DTXFSTS3_OFFSET)

#define STM32_OTGFS_DOEP(n)             (STM32_OTGFS_BASE+STM32_OTGFS_DOEP_OFFSET(n))

#define STM32_OTGFS_DOEPCTL(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DOEPCTL_OFFSET(n))
#define STM32_OTGFS_DOEPCTL0            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPCTL0_OFFSET)
#define STM32_OTGFS_DOEPCTL1            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPCTL1_OFFSET)
#define STM32_OTGFS_DOEPCTL2            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPCTL2_OFFSET)
#define STM32_OTGFS_DOEPCTL3            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPCTL3_OFFSET)

#define STM32_OTGFS_DOEPINT(n)          (STM32_OTGFS_BASE+STM32_OTGFS_DOEPINT_OFFSET(n))
#define STM32_OTGFS_DOEPINT0            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPINT0_OFFSET)
#define STM32_OTGFS_DOEPINT1            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPINT1_OFFSET)
#define STM32_OTGFS_DOEPINT2            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPINT2_OFFSET)
#define STM32_OTGFS_DOEPINT3            (STM32_OTGFS_BASE+STM32_OTGFS_DOEPINT3_OFFSET)

#define STM32_OTGFS_DOEPTSIZ(n)         (STM32_OTGFS_BASE+STM32_OTGFS_DOEPTSIZ_OFFSET(n))
#define STM32_OTGFS_DOEPTSIZ0           (STM32_OTGFS_BASE+STM32_OTGFS_DOEPTSIZ0_OFFSET)
#define STM32_OTGFS_DOEPTSIZ1           (STM32_OTGFS_BASE+STM32_OTGFS_DOEPTSIZ1_OFFSET)
#define STM32_OTGFS_DOEPTSIZ2           (STM32_OTGFS_BASE+STM32_OTGFS_DOEPTSIZ2_OFFSET)
#define STM32_OTGFS_DOEPTSIZ3           (STM32_OTGFS_BASE+STM32_OTGFS_DOEPTSIZ3_OFFSET)

/* Data FIFO (DFIFO) access registers */

#define STM32_OTGFS_DFIFO_DEP(n)        (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_DEP_OFFSET(n))
#define STM32_OTGFS_DFIFO_HCH(n)        (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_HCH_OFFSET(n))

#define STM32_OTGFS_DFIFO_DEP0          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_DEP0_OFFSET)
#define STM32_OTGFS_DFIFO_HCH0          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_HCH0_OFFSET)

#define STM32_OTGFS_DFIFO_DEP1          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_DEP1_OFFSET)
#define STM32_OTGFS_DFIFO_HCH1          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_HCH1_OFFSET)

#define STM32_OTGFS_DFIFO_DEP2          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_DEP2_OFFSET)
#define STM32_OTGFS_DFIFO_HCH2          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_HCH2_OFFSET)

#define STM32_OTGFS_DFIFO_DEP3          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_DEP3_OFFSET)
#define STM32_OTGFS_DFIFO_HCH3          (STM32_OTGFS_BASE+STM32_OTGFS_DFIFO_HCH3_OFFSET)

/* Power and clock gating registers */

#define STM32_OTGFS_PCGCCTL             (STM32_OTGFS_BASE+STM32_OTGFS_PCGCCTL_OFFSET)

/* Register Bitfield Definitions ********************************************************************/
/* Core global control and status registers */

/* Control and status register */

#define OTGFS_GOTGCTL_SRQSCS            (1 << 0)  /* Bit 0:  Session request success */
#define OTGFS_GOTGCTL_SRQ               (1 << 1)  /* Bit 1:  Session request */
                                                  /* Bits 2-72 Reserved, must be kept at reset value */
#define OTGFS_GOTGCTL_HNGSCS            (1 << 8)  /* Bit 8:  Host negotiation success */
#define OTGFS_GOTGCTL_HNPRQ             (1 << 9)  /* Bit 9:  HNP request */
#define OTGFS_GOTGCTL_HSHNPEN           (1 << 10) /* Bit 10: host set HNP enable */
#define OTGFS_GOTGCTL_DHNPEN            (1 << 11) /* Bit 11: Device HNP enabled */
                                                  /* Bits 12-15: Reserved, must be kept at reset value */
#define OTGFS_GOTGCTL_CIDSTS            (1 << 16) /* Bit 16: Connector ID status */
#define OTGFS_GOTGCTL_DBCT              (1 << 17) /* Bit 17: Long/short debounce time */
#define OTGFS_GOTGCTL_ASVLD             (1 << 18) /* Bit 18: A-session valid */
#define OTGFS_GOTGCTL_BSVLD             (1 << 19) /* Bit 19: B-session valid */
                                                  /* Bits 20-31: Reserved, must be kept at reset value */
/* Interrupt register */
                                                  /* Bits 1:0 Reserved, must be kept at reset value */
#define OTGFS_GOTGINT_SEDET             (1 << 2)  /* Bit 2: Session end detected */
                                                  /* Bits 3-7: Reserved, must be kept at reset value */
#define OTGFS_GOTGINT_SRSSCHG           (1 << 8)  /* Bit 8: Session request success status change */
#define OTGFS_GOTGINT_HNSSCHG           (1 << 9)  /* Bit 9: Host negotiation success status change */
                                                  /* Bits 16:10 Reserved, must be kept at reset value */
#define OTGFS_GOTGINT_HNGDET            (1 << 17) /* Bit 17: Host negotiation detected */
#define OTGFS_GOTGINT_ADTOCHG           (1 << 18) /* Bit 18: A-device timeout change */
#define OTGFS_GOTGINT_DBCDNE            (1 << 19) /* Bit 19: Debounce done */
                                                  /* Bits 2-31: Reserved, must be kept at reset value */

/* AHB configuration register */

#define OTGFS_GAHBCFG_GINTMSK           (1 << 0)  /* Bit 0: Global interrupt mask */
                                                  /* Bits 1-6: Reserved, must be kept at reset value */
#define OTGFS_GAHBCFG_TXFELVL           (1 << 7)  /* Bit 7: TxFIFO empty level */
#define OTGFS_GAHBCFG_PTXFELVL          (1 << 8)  /* Bit 8: Periodic TxFIFO empty level */
                                                  /* Bits 20-31: Reserved, must be kept at reset value */
/* USB configuration register */

#define OTGFS_GUSBCFG_TOCAL_SHIFT       (0)       /* Bits 0-2: FS timeout calibration */
#define OTGFS_GUSBCFG_TOCAL_MASK        (7 < OTGFS_GUSBCFG_TOCAL_SHIFT)
                                                  /* Bits 3-6: Reserved, must be kept at reset value */
#define OTGFS_GUSBCFG_SRPCAP            (1 << 8)  /* Bit 8: SRP-capable */
#define OTGFS_GUSBCFG_HNPCAP            (1 << 9)  /* Bit 9: HNP-capable */
#define OTGFS_GUSBCFG_TRDT_SHIFT        (10)      /* Bits 10-13: USB turnaround time */
#define OTGFS_GUSBCFG_TRDT_MASK         (15 < OTGFS_GUSBCFG_TRDT_SHIFT)
                                                  /* Bits 14-28: Reserved, must be kept at reset value */
#define OTGFS_GUSBCFG_FHMOD             (1 << 29) /* Bit 29: Force host mode */
#define OTGFS_GUSBCFG_FDMOD             (1 << 30) /* Bit 30: Force device mode */
#define OTGFS_GUSBCFG_CTXPKT            (1 << 31) /* Bit 31: Corrupt Tx packet */
                                                  /* Bits 20-31: Reserved, must be kept at reset value */
/* Reset register */

#define OTGFS_GRSTCTL_CSRST             (1 << 0)  /* Bit 0: Core soft reset */
#define OTGFS_GRSTCTL_HSRST             (1 << 1)  /* Bit 1: HCLK soft reset */
#define OTGFS_GRSTCTL_FCRST             (1 << 2)  /* Bit 2: Host frame counter reset */
                                                  /* Bit 3 Reserved, must be kept at reset value */
#define OTGFS_GRSTCTL_RXFFLSH           (1 << 4)  /* Bit 4: RxFIFO flush */
#define OTGFS_GRSTCTL_TXFFLSH           (1 << 5)  /* Bit 5: TxFIFO flush */
#define OTGFS_GRSTCTL_TXFNUM_SHIFT      (10)      /* Bits 6-10: TxFIFO number */
#define OTGFS_GRSTCTL_TXFNUM_MASK       (31 < OTGFS_GRSTCTL_TXFNUM_SHIFT)
#  define OTGFS_GRSTCTL_TXFNUM_HNONPER  (0 < OTGFS_GRSTCTL_TXFNUM_SHIFT)   /* Non-periodic TxFIFO flush in host mode */
#  define OTGFS_GRSTCTL_TXFNUM_HPER     (1 < OTGFS_GRSTCTL_TXFNUM_SHIFT)   /* Periodic TxFIFO flush in host mode */
#  define OTGFS_GRSTCTL_TXFNUM_HALL     (16 < OTGFS_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in host mode.*/
#  define OTGFS_GRSTCTL_TXFNUM_D(n)     ((n) < OTGFS_GRSTCTL_TXFNUM_SHIFT) /* TXFIFO n flush in device mode, n=0-15 */
#  define OTGFS_GRSTCTL_TXFNUM_DALL     (16 < OTGFS_GRSTCTL_TXFNUM_SHIFT)  /* Flush all the transmit FIFOs in device mode.*/
                                                  /* Bits 11-31: Reserved, must be kept at reset value */
#define OTGFS_GRSTCTL_AHBIDL            (1 << 31) /* Bit 31: AHB master idle */

/* Core interrupt and Interrupt mask registers */

#define OTGFS_GINTSTS_CMOD              (1 << 0)  /* Bit 0:  Current mode of operation */
#define OTGFS_GINT_MMIS                 (1 << 1)  /* Bit 1:  Mode mismatch interrupt */
#define OTGFS_GINT_OTG                  (1 << 2)  /* Bit 2:  OTG interrupt */
#define OTGFS_GINT_SOF                  (1 << 3)  /* Bit 3:  Start of frame */
#define OTGFS_GINT_RXFLVL               (1 << 4)  /* Bit 4:  RxFIFO non-empty */
#define OTGFS_GINT_NPTXFE               (1 << 5)  /* Bit 5:  Non-periodic TxFIFO empty */
#define OTGFS_GINT_GINAKEFF             (1 << 6)  /* Bit 6:  Global IN non-periodic NAK effective */
#define OTGFS_GINT_GONAKEFF             (1 << 7)  /* Bit 7:  Global OUT NAK effective */
                                                  /* Bits 8-9: Reserved, must be kept at reset value */
#define OTGFS_GINT_ESUSP                (1 << 10) /* Bit 10: Early suspend */
#define OTGFS_GINT_USBSUSP              (1 << 11) /* Bit 11: USB suspend */
#define OTGFS_GINT_USBRST               (1 << 12) /* Bit 12: USB reset */
#define OTGFS_GINT_ENUMDNE              (1 << 13) /* Bit 13: Enumeration done */
#define OTGFS_GINT_ISOODRP              (1 << 14) /* Bit 14: Isochronous OUT packet dropped interrupt */
#define OTGFS_GINT_EOPF                 (1 << 15) /* Bit 15: End of periodic frame interrupt */
                                                  /* Bits 16 Reserved, must be kept at reset value */
#define OTGFS_GINTMSK_EPMISM            (1 << 17) /* Bit 17: Endpoint mismatch interrupt mask */
#define OTGFS_GINT_IEP                  (1 << 18) /* Bit 18: IN endpoint interrupt */
#define OTGFS_GINT_OEP                  (1 << 19) /* Bit 19: OUT endpoint interrupt */
#define OTGFS_GINT_IISOIXFR             (1 << 20) /* Bit 20: Incomplete isochronous IN transfer */
#define OTGFS_GINT_IPXFR                (1 << 21) /* Bit 21: Incomplete periodic transfer */
                                                  /* Bits 22-23: Reserved, must be kept at reset value */
#define OTGFS_GINT_HPRT                 (1 << 24) /* Bit 24: Host port interrupt */
#define OTGFS_GINT_HC                   (1 << 25) /* Bit 25: Host channels interrupt */
#define OTGFS_GINT_PTXFE                (1 << 26) /* Bit 26: Periodic TxFIFO empty */
                                                  /* Bit 27 Reserved, must be kept at reset value */
#define OTGFS_GINT_CIDSCHG              (1 << 28) /* Bit 28: Connector ID status change */
#define OTGFS_GINT_DISC                 (1 << 29) /* Bit 29: Disconnect detected interrupt */
#define OTGFS_GINT_SRQ                  (1 << 30) /* Bit 30: Session request/new session detected interrupt */
#define OTGFS_GINT_WKUP                 (1 << 31) /* Bit 31: Resume/remote wakeup detected interrupt */

/* Receive status debug read/OTG status read and pop registers (host mode) */

#define OTGFS_GRXSTSH_CHNUM_SHIFT       (0)       /* Bits 0-3: Channel number */
#define OTGFS_GRXSTSH_CHNUM_MASK        (15 < OTGFS_GRXSTSH_CHNUM_SHIFT)
#define OTGFS_GRXSTSH_BCNT_SHIFT        (4)       /* Bits 4-14: Byte count */
#define OTGFS_GRXSTSH_BCNT_MASK         (0x7ff < OTGFS_GRXSTSH_BCNT_SHIFT)
#define OTGFS_GRXSTSH_DPID_SHIFT        (15)      /* Bits 15-16: Data PID */
#define OTGFS_GRXSTSH_DPID_MASK         (3 < OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA0      (0 < OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA2      (1 < OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_DATA1      (2 < OTGFS_GRXSTSH_DPID_SHIFT)
#  define OTGFS_GRXSTSH_DPID_MDATA      (3 < OTGFS_GRXSTSH_DPID_SHIFT)
#define OTGFS_GRXSTSH_PKTSTS_SHIFT      (17)      /* Bits 17-20: Packet status */
#define OTGFS_GRXSTSH_PKTSTS_MASK       (15 < OTGFS_GRXSTSH_PKTSTS_SHIFT)
#  define OTGFS_GRXSTSH_PKTSTS_INRECVD  (2 < OTGFS_GRXSTSH_PKTSTS_SHIFT) /* IN data packet received */
#  define OTGFS_GRXSTSH_PKTSTS_INDONE   (3 < OTGFS_GRXSTSH_PKTSTS_SHIFT) /* IN transfer completed */
#  define OTGFS_GRXSTSH_PKTSTS_DTOGERR  (2 < OTGFS_GRXSTSH_PKTSTS_SHIFT) /* Data toggle error */
#  define OTGFS_GRXSTSH_PKTSTS_HALTED   (7 < OTGFS_GRXSTSH_PKTSTS_SHIFT) /* Channel halted */
                                                  /* Bits 21-31: Reserved, must be kept at reset value.
/* Receive status debug read/OTG status read and pop registers (device mode) */
#define OTGFS_GRXSTSD_

Bits 31:25 Reserved, must be kept at reset value.
Bits 24:21 FRMNUM: Frame number
Bits 20:17 PKTSTS: Packet status
0001: Global OUT NAK (triggers an interrupt)
0010: OUT data packet received
0011: OUT transfer completed (triggers an interrupt)
0100: SETUP transaction completed (triggers an interrupt)
0110: SETUP data packet received
Bits 16:15 DPID: Data PID
00: DATA0
10: DATA1
01: DATA2
11: MDATA
Bits 14:4 BCNT: Byte count
Bits 3:0 EPNUM: Endpoint number

/* Receive FIFO size register */
#define OTGFS_GRXFSIZ_
/* Host non-periodic transmit FIFO size register */
#define OTGFS_HNPTXFSIZ_
/* Endpoint 0 Transmit FIFO size */
#define OTGFS_DIEPTXF0_
/* non-periodic transmit FIFO/queue status register */
#define OTGFS_HNPTXSTS_
/* general core configuration register */
#define OTGFS_GCCFG_
/* Core ID register  */
#define OTGFS_CID_
/* Host periodic transmit FIFO size register */
#define OTGFS_HPTXFSIZ_
/* Device IN endpoint transmit FIFOn size register */
#define OTGFS_DIEPTXF1_

/* Host-mode control and status registers */

/* Host configuration register */
#define OTGFS_HCFG_
/* Host frame interval register */
#define OTGFS_HFIR_
/* Host frame number/frame time remaining register */
#define OTGFS_HFNUM_
/* Host periodic transmit FIFO/queue status register */
#define OTGFS_HPTXSTS_
/* Host all channels interrupt register */
#define OTGFS_HAINT_
/* Host all channels interrupt mask register */
#define OTGFS_HAINTMSK_
/* Host port control and status register */
#define OTGFS_HPRT_
/* Host channel-n characteristics register */
#define OTGFS_HCCHAR0_
/* Host channel-n interrupt register */
#define OTGFS_HCINT0_
/* Host channel-0 interrupt mask register */
#define OTGFS_HCINTMSK0_
/* Host channel-n interrupt register */
#define OTGFS_HCTSIZ0_

/* Device-mode control and status registers */

/* Device configuration register */
#define OTGFS_DCFG_
/* Device control register */
#define OTGFS_DCTL_
/* Device status register */
#define OTGFS_DSTS_
/* Device IN endpoint common interrupt mask register */
#define OTGFS_DIEPMSK_
/* Device OUT endpoint common interrupt mask register */
#define OTGFS_DOEPMSK_
/* Device all endpoints interrupt register */
#define OTGFS_DAINT_
/* All endpoints interrupt mask register */
#define OTGFS_DAINTMSK_
/* Device VBUS discharge time register */
#define OTGFS_DVBUSDIS_
/* Device VBUS pulsing time register */
#define OTGFS_DVBUSPULSE_
/* Device IN endpoint FIFO empty interrupt mask register */
#define OTGFS_DIEPEMPMSK_
/* Device control IN endpoint 0 control register */
#define OTGFS_DIEPCTL0_
/* Device control IN endpoint n control register */
#define OTGFS_DIEPCTL1_
/* Device endpoint-n interrupt register */
#define OTGFS_DIEPINT_
/* Device IN endpoint 0 transfer size register */
#define OTGFS_DIEPTSIZ0_
/* Device IN endpoint n transfer size register */
#define OTGFS_DIEPTSIZ_
/* Device OUT endpoint-0 transfer size register */
#define OTGFS_DTXFSTS_
/* Device OUT endpoint 0 control register */
#define OTGFS_DOEPCTL0_
/* Device OUT endpoint n control register */
#define OTGFS_DOEPCTL_
/* Device endpoint-n interrupt register */
#define OTGFS_DOEPINT_
/* Device OUT endpoint-0 transfer size register */
#define OTGFS_DOEPTSIZ0_
/* Device OUT endpoint-n transfer size register */
#define OTGFS_DOEPTSIZ_

/* Power and clock gating registers */

/* Power and clock gating control register */
#define OTGFS_PCGCCTL_

             (1 << xx)  /* Bit nn:  
_SHIFT       (nn)       /* Bits nn-nn: 
_MASK        (xx < yy)

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_USBOTG_H */
