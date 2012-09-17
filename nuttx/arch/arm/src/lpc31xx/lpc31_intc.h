/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_intc.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_INTC_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_INTC_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* INTC register base address *******************************************************************/

#define LPC31_INTC_VBASE                (LPC31_INTC_VSECTION)
#define LPC31_INTC_PBASE                (LPC31_INTC_PSECTION)

/* INTC register offsets (with respect to the base of the INTC domain) **************************/

#define LPC31_INTC_PRIORITYMASK0_OFFSET 0x000 /* Interrupt target 0 priority threshold */
#define LPC31_INTC_PRIORITYMASK1_OFFSET 0x004 /* Interrupt target 0 priority threshold */
#define LPC31_INTC_VECTOR0_OFFSET       0x100 /* Vector register for target 0 => nIRQ */
#define LPC31_INTC_VECTOR1_OFFSET       0x104 /* Vector register for target 1 => nFIQ */
#define LPC31_INTC_PENDING_OFFSET       0x200 /* Status of interrupt request 1..29 */
#define LPC31_INTC_FEATURES_OFFSET      0x300 /* Interrupt controller configuration */
#define LPC31_INTC_REQUEST_OFFSET(n)    (0x400+((n) << 2))
#define LPC31_INTC_REQUEST1_OFFSET      0x404 /* Interrupt request 1 configuration */
#define LPC31_INTC_REQUEST2_OFFSET      0x408 /* Interrupt request 2 configuration */
#define LPC31_INTC_REQUEST3_OFFSET      0x40c /* Interrupt request 3 configuration */
#define LPC31_INTC_REQUEST4_OFFSET      0x410 /* Interrupt request 4 configuration */
#define LPC31_INTC_REQUEST5_OFFSET      0x414 /* Interrupt request 5 configuration */
#define LPC31_INTC_REQUEST6_OFFSET      0x418 /* Interrupt request 6 configuration */
#define LPC31_INTC_REQUEST7_OFFSET      0x41c /* Interrupt request 7 configuration */
#define LPC31_INTC_REQUEST8_OFFSET      0x420 /* Interrupt request 8 configuration */
#define LPC31_INTC_REQUEST9_OFFSET      0x424 /* Interrupt request 9 configuration */
#define LPC31_INTC_REQUEST10_OFFSET     0x428 /* Interrupt request 10 configuration */
#define LPC31_INTC_REQUEST11_OFFSET     0x42c /* Interrupt request 11 configuration */
#define LPC31_INTC_REQUEST12_OFFSET     0x430 /* Interrupt request 12 configuration */
#define LPC31_INTC_REQUEST13_OFFSET     0x434 /* Interrupt request 13 configuration */
#define LPC31_INTC_REQUEST14_OFFSET     0x438 /* Interrupt request 14 configuration */
#define LPC31_INTC_REQUEST15_OFFSET     0x43c /* Interrupt request 15 configuration */
#define LPC31_INTC_REQUEST16_OFFSET     0x440 /* Interrupt request 16 configuration */
#define LPC31_INTC_REQUEST17_OFFSET     0x444 /* Interrupt request 17 configuration */
#define LPC31_INTC_REQUEST18_OFFSET     0x448 /* Interrupt request 18 configuration */
#define LPC31_INTC_REQUEST19_OFFSET     0x44c /* Interrupt request 19 configuration */
#define LPC31_INTC_REQUEST20_OFFSET     0x450 /* Interrupt request 20 configuration */
#define LPC31_INTC_REQUEST21_OFFSET     0x454 /* Interrupt request 21 configuration */
#define LPC31_INTC_REQUEST22_OFFSET     0x458 /* Interrupt request 22 configuration */
#define LPC31_INTC_REQUEST23_OFFSET     0x45c /* Interrupt request 23 configuration */
#define LPC31_INTC_REQUEST24_OFFSET     0x460 /* Interrupt request 24 configuration */
#define LPC31_INTC_REQUEST25_OFFSET     0x464 /* Interrupt request 25 configuration */
#define LPC31_INTC_REQUEST26_OFFSET     0x468 /* Interrupt request 26 configuration */
#define LPC31_INTC_REQUEST27_OFFSET     0x46c /* Interrupt request 27 configuration */
#define LPC31_INTC_REQUEST28_OFFSET     0x470 /* Interrupt request 28 configuration */
#define LPC31_INTC_REQUEST29_OFFSET     0x474 /* Interrupt request 29 configuration */

/* INTC register (virtual) addresses ************************************************************/

#define LPC31_INTC_PRIORITYMASK0        (LPC31_INTC_VBASE+LPC31_INTC_PRIORITYMASK0_OFFSET)
#define LPC31_INTC_PRIORITYMASK1        (LPC31_INTC_VBASE+LPC31_INTC_PRIORITYMASK1_OFFSET)
#define LPC31_INTC_VECTOR0              (LPC31_INTC_VBASE+LPC31_INTC_VECTOR0_OFFSET)
#define LPC31_INTC_VECTOR1              (LPC31_INTC_VBASE+LPC31_INTC_VECTOR1_OFFSET)
#define LPC31_INTC_PENDING              (LPC31_INTC_VBASE+LPC31_INTC_PENDING_OFFSET)
#define LPC31_INTC_FEATURES             (LPC31_INTC_VBASE+LPC31_INTC_FEATURES_OFFSET)
#define LPC31_INTC_REQUEST(n)           (LPC31_INTC_VBASE+LPC31_INTC_REQUEST_OFFSET(n))
#define LPC31_INTC_REQUEST1             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST1_OFFSET)
#define LPC31_INTC_REQUEST2             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST2_OFFSET)
#define LPC31_INTC_REQUEST3             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST3_OFFSET)
#define LPC31_INTC_REQUEST4             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST4_OFFSET)
#define LPC31_INTC_REQUEST5             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST5_OFFSET)
#define LPC31_INTC_REQUEST6             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST6_OFFSET)
#define LPC31_INTC_REQUEST7             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST7_OFFSET)
#define LPC31_INTC_REQUEST8             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST8_OFFSET)
#define LPC31_INTC_REQUEST9             (LPC31_INTC_VBASE+LPC31_INTC_REQUEST9_OFFSET)
#define LPC31_INTC_REQUEST10            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST10_OFFSET)
#define LPC31_INTC_REQUEST11            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST11_OFFSET)
#define LPC31_INTC_REQUEST12            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST12_OFFSET)
#define LPC31_INTC_REQUEST13            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST13_OFFSET)
#define LPC31_INTC_REQUEST14            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST14_OFFSET)
#define LPC31_INTC_REQUEST15            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST15_OFFSET)
#define LPC31_INTC_REQUEST16            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST16_OFFSET)
#define LPC31_INTC_REQUEST17            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST17_OFFSET)
#define LPC31_INTC_REQUEST18            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST18_OFFSET)
#define LPC31_INTC_REQUEST19            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST19_OFFSET)
#define LPC31_INTC_REQUEST20            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST20_OFFSET)
#define LPC31_INTC_REQUEST21            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST21_OFFSET)
#define LPC31_INTC_REQUEST22            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST22_OFFSET)
#define LPC31_INTC_REQUEST23            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST23_OFFSET)
#define LPC31_INTC_REQUEST24            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST24_OFFSET)
#define LPC31_INTC_REQUEST25            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST25_OFFSET)
#define LPC31_INTC_REQUEST26            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST26_OFFSET)
#define LPC31_INTC_REQUEST27            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST27_OFFSET)
#define LPC31_INTC_REQUEST28            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST28_OFFSET)
#define LPC31_INTC_REQUEST29            (LPC31_INTC_VBASE+LPC31_INTC_REQUEST29_OFFSET)

/* INTC register bit definitions ****************************************************************/

/* Interrupt priority mask register (INT_PRIORITYMASK0 address 0x60000000 and
 * INTC_PRIORITYMASK1 address 0x60000004)
 */

#define INTC_PRIORITYMASK_PRIOLIMIT_SHIFT (0)       /* Bits 0-7: Priority threshold for interrupts */
#define INTC_PRIORITYMASK_PRIOLIMIT_MASK  (255 << INTC_PRIORITYMASK_PRIOLIMIT_MASK)

/* Interrupt vector registers (INTC_VECTOR0 address 0x60000100 and INTC_VECTOR1 address
 * 0x60000104)
 */

#define INTC_VECTOR_TABLEADDR_SHIFT       (11)      /* Bits 11-31: Table start address */
#define INTC_VECTOR_TABLEADDR_MASK        (0x001fffff << INTC_VECTOR_TABLEADDR_SHIFT)
#define INTC_VECTOR_INDEX_SHIFT           (3)       /* Bits 3-10: IRQ number of interrupt */
#define INTC_VECTOR_INDEX_MASK            (255 << INTC_VECTOR_INDEX_SHIFT)

/* Interrupt pending register (INT_PENDING1_31, address 0x60000200) */

#define INTC_PENDING_SHIFT                (1)       /* Bits 1-29: Pending interrupt request */
#define INTC_PENDING_MASK                 (0x1fffffff << INTC_PENDING_SHIFT)

/* Interrupt controller features register (INT_FEATURES, address 0x60000300) */

#define INTC_FEATURES_T_SHIFT             (16)      /* Bits 16-21: Number interrupt targets supported (+1) */
#define INTC_FEATURES_T_MASK              (63 << INTC_FEATURES_T_SHIFT)
#define INTC_FEATURES_P_SHIFT             (8)       /* Bits 8-15: Number priority levels supported */
#define INTC_FEATURES_P_MASK              (255 << INTC_FEATURES_P_SHIFT)
#define INTC_FEATURES_N_SHIFT             (0)       /* Bits 0-7: Number interrupt request inputs */
#define INTC_FEATURES_N_MASK              (255 << INTC_FEATURES_N_SHIFT)

/* Interrupt request registers (INT_REQUEST1 address 0x60000404 to INTC_REQUEST29 address
 * 0x60000474)
 */

#define INTC_REQUEST_PENDING              (1 << 31) /* Bit 31: Pending interrupt request */
#define INTC_REQUEST_SETSWINT             (1 << 30) /* Bit 30: Set software interrupt request */
#define INTC_REQUEST_CLRSWINT             (1 << 29) /* Bit 29: Clear software interrupt request */
#define INTC_REQUEST_WEPRIO               (1 << 28) /* Bit 28: Write Enable PRIORITY_LEVEL */
#define INTC_REQUEST_WETARGET             (1 << 27) /* Bit 27: Write Enable TARGET */
#define INTC_REQUEST_WEENABLE             (1 << 26) /* Bit 26: Write Enable ENABLE */
#define INTC_REQUEST_WEACTLOW             (1 << 25) /* Bit 25: Write Enable ACTIVE_LOW */
#define INTC_REQUEST_ACTLOW               (1 << 17) /* Bit 17: Active Low */
#define INTC_REQUEST_ENABLE               (1 << 16) /* Bit 16: Enable interrupt request */
#define INTC_REQUEST_TARGET_SHIFT         (8)       /* Bits 8-13: Interrupt target */
#define INTC_REQUEST_TARGET_MASK          (63 << INTC_REQUEST_TARGET_SHIFT)
#  define INTC_REQUEST_TARGET_IRQ         (INTC_REQUEST_WETARGET | (0  << INTC_REQUEST_TARGET_SHIFT)) /* Proc interrupt request 0: IRQ */
#  define INTC_REQUEST_TARGET_FIQ         (INTC_REQUEST_WETARGET | (1  << INTC_REQUEST_TARGET_SHIFT)) /* Proc interrupt request 1: FIQ */
#define INTC_REQUEST_PRIOLEVEL_SHIFT      (0)       /* Bits 0-7: Priority level */
#define INTC_REQUEST_PRIOLEVEL_MASK       (255 << INTC_REQUEST_PRIOLEVEL_SHIFT)
#  define INTC_REQUEST_PRIOLEVEL(n)       (((n)  << INTC_REQUEST_PRIOLEVEL_SHIFT) & INTC_REQUEST_PRIOLEVEL_MASK)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_INTC_H */
