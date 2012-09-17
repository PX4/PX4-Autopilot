/************************************************************************************
 * arch/arm/src/imx/imx_aitc.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_IMX_AITC_H
#define __ARCH_ARM_IMX_AITC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* AITC Register Offsets ************************************************************/

#define AITC_INTCNTL_OFFSET          0x0000 /* Interrupt Control Register */
#define AITC_NIMASK_OFFSET           0x0004 /* Normal Interrupt Mask Register */
#define AITC_INTENNUM_OFFSET         0x0008 /* Interrupt Enable Number Register */
#define AITC_INTDISNUM_OFFSET        0x000c /* Interrupt Disable Number Register */
#define AITC_INTENABLEH_OFFSET       0x0010 /* Interrupt Enable Register High */
#define AITC_INTENABLEL_OFFSET       0x0014 /* Interrupt Enable Register Low */
#define AITC_INTTYPEH_OFFSET         0x0018
#define AITC_INTTYPEL_OFFSET         0x001c
#define AITC_NIPRIORITY7_OFFSET      0x0020
#define AITC_NIPRIORITY6_OFFSET      0x0024
#define AITC_NIPRIORITY5_OFFSET      0x0028
#define AITC_NIPRIORITY4_OFFSET      0x002c
#define AITC_NIPRIORITY3_OFFSET      0x0030
#define AITC_NIPRIORITY2_OFFSET      0x0034
#define AITC_NIPRIORITY1_OFFSET      0x0038
#define AITC_NIPRIORITY0_OFFSET      0x003c
#define AITC_NIPRIORITY_OFFSET(n)    (AITC_NIPRIORITY7_OFFSET + 4*(7-(n)))
#define AITC_NIVECSR_OFFSET          0x0040
#define AITC_FIVECSR_OFFSET          0x0044
#define AITC_INTSRCH_OFFSET          0x0048
#define AITC_INTSRCL_OFFSET          0x004c
#define AITC_INTFRCH_OFFSET          0x0050
#define AITC_INTFRCL_OFFSET          0x0054
#define AITC_NIPNDH_OFFSET           0x0058
#define AITC_NIPNDL_OFFSET           0x005c
#define AITC_FIPNDH_OFFSET           0x0060
#define AITC_FIPNDL_OFFSET           0x0064

/* AITC Register Addresses **********************************************************/

#define IMX_AITC_INTCNTL             (IMX_AITC_VBASE + AITC_INTCNTL_OFFSET)
#define IMX_AITC_NIMASK              (IMX_AITC_VBASE + AITC_NIMASK_OFFSET)
#define IMX_AITC_INTENNUM            (IMX_AITC_VBASE + AITC_INTENNUM_OFFSET)
#define IMX_AITC_INTDISNUM           (IMX_AITC_VBASE + AITC_INTDISNUM_OFFSET)
#define IMX_AITC_INTENABLEH          (IMX_AITC_VBASE + AITC_INTENABLEH_OFFSET)
#define IMX_AITC_INTENABLEL          (IMX_AITC_VBASE + AITC_INTENABLEL_OFFSET)
#define IMX_AITC_INTTYPEH            (IMX_AITC_VBASE + AITC_INTTYPEH_OFFSET)
#define IMX_AITC_INTTYPEL            (IMX_AITC_VBASE + AITC_INTTYPEL_OFFSET)
#define IMX_AITC_NIPRIORITY7         (IMX_AITC_VBASE + AITC_NIPRIORITY7_OFFSET)
#define IMX_AITC_NIPRIORITY6         (IMX_AITC_VBASE + AITC_NIPRIORITY6_OFFSET)
#define IMX_AITC_NIPRIORITY5         (IMX_AITC_VBASE + AITC_NIPRIORITY5_OFFSET)
#define IMX_AITC_NIPRIORITY4         (IMX_AITC_VBASE + AITC_NIPRIORITY4_OFFSET)
#define IMX_AITC_NIPRIORITY3         (IMX_AITC_VBASE + AITC_NIPRIORITY3_OFFSET)
#define IMX_AITC_NIPRIORITY2         (IMX_AITC_VBASE + AITC_NIPRIORITY2_OFFSET)
#define IMX_AITC_NIPRIORITY1         (IMX_AITC_VBASE + AITC_NIPRIORITY1_OFFSET)
#define IMX_AITC_NIPRIORITY0         (IMX_AITC_VBASE + AITC_NIPRIORITY0_OFFSET)
#define IMX_AITC_NIPRIORITY(n)       (IMX_AITC_VBASE + AITC_NIPRIORITY_OFFSET(n)))
#define IMX_AITC_NIVECSR             (IMX_AITC_VBASE + AITC_NIVECSR_OFFSET)
#define IMX_AITC_FIVECSR             (IMX_AITC_VBASE + AITC_FIVECSR_OFFSET)
#define IMX_AITC_INTSRCH             (IMX_AITC_VBASE + AITC_INTSRCH_OFFSET)
#define IMX_AITC_INTSRCL             (IMX_AITC_VBASE + AITC_INTSRCL_OFFSET)
#define IMX_AITC_INTFRCH             (IMX_AITC_VBASE + AITC_INTFRCH_OFFSET)
#define IMX_AITC_INTFRCL             (IMX_AITC_VBASE + AITC_INTFRCL_OFFSET)
#define IMX_AITC_NIPNDH              (IMX_AITC_VBASE + AITC_NIPNDH_OFFSET)
#define IMX_AITC_NIPNDL              (IMX_AITC_VBASE + AITC_NIPNDL_OFFSET)
#define IMX_AITC_FIPNDH              (IMX_AITC_VBASE + AITC_FIPNDH_OFFSET)
#define IMX_AITC_FIPNDL              (IMX_AITC_VBASE + AITC_FIPNDL_OFFSET)

/* AITC Register Bit Definitions ****************************************************/


#define AITC_NIVECSR_NIPRILVL_SHIFT  0  /* Bits 15–0: Priority of highest priority interrupt */
#define AITC_NIVECSR_NIPRILVL_MASK   (0x0000ffff << AITC_NIVECSR_NIPRILVL_SHIFT);
#define AITC_NIVECSR_NIVECTOR_SHIFT  16 /* Bits 31–16: Vector index of highest priority interrupt */
#define AITC_NIVECSR_NIVECTOR_MASK   (0x0000ffff << AITC_NIVECSR_NIVECTOR_SHIFT);

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_AITC_H */
