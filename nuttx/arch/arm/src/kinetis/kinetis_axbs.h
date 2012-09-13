/************************************************************************************
 * arch/arm/src/kinetis/kinetis_axbs.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_AXBS_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_AXBS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_AXBS_PRS_OFFSET(n)   (0x0000 + ((n) << 8))
#define KINETIS_AXBS_CRS_OFFSET(n)   (0x0010 + ((n) << 8))
#define KINETIS_AXBS_MGPCR_OFFSET(n) (0x0800 + ((n) << 8))

#define KINETIS_AXBS_PRS0_OFFSET     0x0000 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS0_OFFSET     0x0010 /* Control Register */
#define KINETIS_AXBS_PRS1_OFFSET     0x0100 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS1_OFFSET     0x0110 /* Control Register */
#define KINETIS_AXBS_PRS2_OFFSET     0x0200 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS2_OFFSET     0x0210 /* Control Register */
#define KINETIS_AXBS_PRS3_OFFSET     0x0300 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS3_OFFSET     0x0310 /* Control Register */
#define KINETIS_AXBS_PRS4_OFFSET     0x0400 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS4_OFFSET     0x0410 /* Control Register */
#define KINETIS_AXBS_PRS5_OFFSET     0x0500 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS5_OFFSET     0x0510 /* Control Register */
#define KINETIS_AXBS_PRS6_OFFSET     0x0600 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS6_OFFSET     0x0610 /* Control Register */
#define KINETIS_AXBS_PRS7_OFFSET     0x0700 /* Priority Registers Slave */
#define KINETIS_AXBS_CRS7_OFFSET     0x0710 /* Control Register */
#define KINETIS_AXBS_MGPCR0_OFFSET   0x0800 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR1_OFFSET   0x0900 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR2_OFFSET   0x0a00 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR3_OFFSET   0x0b00 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR4_OFFSET   0x0c00 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR5_OFFSET   0x0d00 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR6_OFFSET   0x0e00 /* Master General Purpose Control Register */
#define KINETIS_AXBS_MGPCR7_OFFSET   0x0f00 /* Master General Purpose Control Register */

/* Register Addresses ***************************************************************/

#define KINETIS_AXBS_PRS(n)          (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS_OFFSET(n))
#define KINETIS_AXBS_CRS(n)          (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS_OFFSET(n))
#define KINETIS_AXBS_PRS0            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS0_OFFSET)
#define KINETIS_AXBS_CRS0            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS0_OFFSET)
#define KINETIS_AXBS_PRS1            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS1_OFFSET)
#define KINETIS_AXBS_CRS1            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS1_OFFSET)
#define KINETIS_AXBS_PRS2            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS2_OFFSET)
#define KINETIS_AXBS_CRS2            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS2_OFFSET)
#define KINETIS_AXBS_PRS3            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS3_OFFSET)
#define KINETIS_AXBS_CRS3            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS3_OFFSET)
#define KINETIS_AXBS_PRS4            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS4_OFFSET)
#define KINETIS_AXBS_CRS4            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS4_OFFSET)
#define KINETIS_AXBS_PRS5            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS5_OFFSET)
#define KINETIS_AXBS_CRS5            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS5_OFFSET)
#define KINETIS_AXBS_PRS6            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS6_OFFSET)
#define KINETIS_AXBS_CRS6            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS6_OFFSET)
#define KINETIS_AXBS_PRS7            (KINETIS_XBAR_BASE+KINETIS_AXBS_PRS7_OFFSET)
#define KINETIS_AXBS_CRS7            (KINETIS_XBAR_BASE+KINETIS_AXBS_CRS7_OFFSET)

#define KINETIS_AXBS_MGPCR(n)        (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR_OFFSET(n))
#define KINETIS_AXBS_MGPCR0          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR0_OFFSET)
#define KINETIS_AXBS_MGPCR1          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR1_OFFSET)
#define KINETIS_AXBS_MGPCR2          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR2_OFFSET)
#define KINETIS_AXBS_MGPCR3          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR3_OFFSET)
#define KINETIS_AXBS_MGPCR4          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR4_OFFSET)
#define KINETIS_AXBS_MGPCR5          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR5_OFFSET)
#define KINETIS_AXBS_MGPCR6          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR6_OFFSET)
#define KINETIS_AXBS_MGPCR7          (KINETIS_XBAR_BASE+KINETIS_AXBS_MGPCR7_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Priority Registers Slave */

#define AXBS_PRS_M0_SHIFT             (0)     /* Bits 0-2: Master 0 priority */
#define AXBS_PRS_M0_MASK              (7 << AXBS_PRS_M0_SHIFT)
#  define AXBS_PRS_M0_PRI1            (0 << AXBS_PRS_M0_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M0_PRI2            (1 << AXBS_PRS_M0_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M0_PRI3            (2 << AXBS_PRS_M0_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M0_PRI4            (3 << AXBS_PRS_M0_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M0_PRI5            (4 << AXBS_PRS_M0_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M0_PRI6            (5 << AXBS_PRS_M0_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M0_PRI7            (6 << AXBS_PRS_M0_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M0_PRI8            (7 << AXBS_PRS_M0_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 3:  Reserved */
#define AXBS_PRS_M1_SHIFT             (4)      /* Bits 4-6: Master 1 priority */
#define AXBS_PRS_M1_MASK              (7 << AXBS_PRS_M1_SHIFT)
#  define AXBS_PRS_M1_PRI1            (0 << AXBS_PRS_M1_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M1_PRI2            (1 << AXBS_PRS_M1_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M1_PRI3            (2 << AXBS_PRS_M1_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M1_PRI4            (3 << AXBS_PRS_M1_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M1_PRI5            (4 << AXBS_PRS_M1_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M1_PRI6            (5 << AXBS_PRS_M1_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M1_PRI7            (6 << AXBS_PRS_M1_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M1_PRI8            (7 << AXBS_PRS_M1_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 7:  Reserved */
#define AXBS_PRS_M2_SHIFT             (8)      /* Bits 8-10: Master 2 priority */
#define AXBS_PRS_M2_MASK              (7 << AXBS_PRS_M2_SHIFT)
#  define AXBS_PRS_M2_PRI1            (0 << AXBS_PRS_M2_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M2_PRI2            (1 << AXBS_PRS_M2_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M2_PRI3            (2 << AXBS_PRS_M2_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M2_PRI4            (3 << AXBS_PRS_M2_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M2_PRI5            (4 << AXBS_PRS_M2_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M2_PRI6            (5 << AXBS_PRS_M2_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M2_PRI7            (6 << AXBS_PRS_M2_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M2_PRI8            (7 << AXBS_PRS_M2_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 11:  Reserved */
#define AXBS_PRS_M3_SHIFT             (12)     /* Bits 12-14: Master 3 priority */
#define AXBS_PRS_M3_MASK              (7 << AXBS_PRS_M3_SHIFT)
#  define AXBS_PRS_M3_PRI1            (0 << AXBS_PRS_M3_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M3_PRI2            (1 << AXBS_PRS_M3_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M3_PRI3            (2 << AXBS_PRS_M3_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M3_PRI4            (3 << AXBS_PRS_M3_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M3_PRI5            (4 << AXBS_PRS_M3_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M3_PRI6            (5 << AXBS_PRS_M3_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M3_PRI7            (6 << AXBS_PRS_M3_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M3_PRI8            (7 << AXBS_PRS_M3_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 15:  Reserved */
#define AXBS_PRS_M4_SHIFT             (16)     /* Bits 16-18: Master 4 priority */
#define AXBS_PRS_M4_MASK              (7 << AXBS_PRS_M4_SHIFT)
#  define AXBS_PRS_M4_PRI1            (0 << AXBS_PRS_M4_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M4_PRI2            (1 << AXBS_PRS_M4_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M4_PRI3            (2 << AXBS_PRS_M4_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M4_PRI4            (3 << AXBS_PRS_M4_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M4_PRI5            (4 << AXBS_PRS_M4_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M4_PRI6            (5 << AXBS_PRS_M4_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M4_PRI7            (6 << AXBS_PRS_M4_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M4_PRI8            (7 << AXBS_PRS_M4_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 19:  Reserved */
#define AXBS_PRS_M5_SHIFT             (20)     /* Bits 20-22: Master 5 priority */
#define AXBS_PRS_M5_MASK              (7 << AXBS_PRS_M5_SHIFT)
#  define AXBS_PRS_M5_PRI1            (0 << AXBS_PRS_M5_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M5_PRI2            (1 << AXBS_PRS_M5_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M5_PRI3            (2 << AXBS_PRS_M5_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M5_PRI4            (3 << AXBS_PRS_M5_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M5_PRI5            (4 << AXBS_PRS_M5_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M5_PRI6            (5 << AXBS_PRS_M5_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M5_PRI7            (6 << AXBS_PRS_M5_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M5_PRI8            (7 << AXBS_PRS_M5_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 23:  Reserved */
#define AXBS_PRS_M6_SHIFT             (24)     /* Bits 24-26: Master 6 priority */
#define AXBS_PRS_M6_MASK              (7 << AXBS_PRS_M6_SHIFT)
#  define AXBS_PRS_M6_PRI1            (0 << AXBS_PRS_M6_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M6_PRI2            (1 << AXBS_PRS_M6_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M6_PRI3            (2 << AXBS_PRS_M6_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M6_PRI4            (3 << AXBS_PRS_M6_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M6_PRI5            (4 << AXBS_PRS_M6_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M6_PRI6            (5 << AXBS_PRS_M6_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M6_PRI7            (6 << AXBS_PRS_M6_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M6_PRI8            (7 << AXBS_PRS_M6_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 27:  Reserved */
#define AXBS_PRS_M7_SHIFT             (28)     /* Bits 28-30: Master 7 priority */
#define AXBS_PRS_M7_MASK              (7 << AXBS_PRS_M7_SHIFT)
#  define AXBS_PRS_M7_PRI1            (0 << AXBS_PRS_M7_SHIFT) /* Master has pri 1 (highest) access to slave port */
#  define AXBS_PRS_M7_PRI2            (1 << AXBS_PRS_M7_SHIFT) /* Master has pri 2 access to slave port */
#  define AXBS_PRS_M7_PRI3            (2 << AXBS_PRS_M7_SHIFT) /* Master has pri 3 access to slave port */
#  define AXBS_PRS_M7_PRI4            (3 << AXBS_PRS_M7_SHIFT) /* Master has pri 4 access to slave port */
#  define AXBS_PRS_M7_PRI5            (4 << AXBS_PRS_M7_SHIFT) /* Master has pri 5 access to slave port */
#  define AXBS_PRS_M7_PRI6            (5 << AXBS_PRS_M7_SHIFT) /* Master has pri 6 access to slave port */
#  define AXBS_PRS_M7_PRI7            (6 << AXBS_PRS_M7_SHIFT) /* Master has pri 7 access to slave port */
#  define AXBS_PRS_M7_PRI8            (7 << AXBS_PRS_M7_SHIFT) /* Master has pri 8 (lowest) access to slave port */
                                               /* Bit 31:  Reserved */
/* Control Register */

#define AXBS_CRS_PARK_SHIFT           (0)     /* Bits 0-2: Park */
#define AXBS_CRS_PARK_MASK            (7 << AXBS_CRS_PARK_SHIFT)
#  define AXBS_CRS_PARK_M0            (0 << AXBS_CRS_PARK_SHIFT) /* Park on master port M0 */
#  define AXBS_CRS_PARK_M1            (1 << AXBS_CRS_PARK_SHIFT) /* Park on master port M1 */
#  define AXBS_CRS_PARK_M2            (2 << AXBS_CRS_PARK_SHIFT) /* Park on master port M2 */
#  define AXBS_CRS_PARK_M3            (3 << AXBS_CRS_PARK_SHIFT) /* Park on master port M3 */
#  define AXBS_CRS_PARK_M4            (4 << AXBS_CRS_PARK_SHIFT) /* Park on master port M4 */
#  define AXBS_CRS_PARK_M5            (5 << AXBS_CRS_PARK_SHIFT) /* Park on master port M5 */
#define AXBS_CRS_PCTL_SHIFT           (4)      /* Bits 4-5: Parking control */
#define AXBS_CRS_PCTL_MASK            (2 << AXBS_CRS_PCTL_SHIFT)
#  define AXBS_CRS_PCTL_PARK          (0 << AXBS_CRS_PCTL_SHIFT) /* Defined by the PARK bit field */
#  define AXBS_CRS_PCTL_LAST          (1 << AXBS_CRS_PCTL_SHIFT) /* Last master in control of slave port */
#  define AXBS_CRS_PCTL_NOT           (2 << AXBS_CRS_PCTL_SHIFT) /* Not parked on a master */
#define AXBS_CRS_ARB_SHIFT            (8)      /* Bits 8-9: Arbitration mode */
#define AXBS_CRS_ARB_MASK             (3 << AXBS_CRS_ARB_SHIFT)
#  define AXBS_CRS_ARB_FIXED          (0 << AXBS_CRS_ARB_SHIFT) /* Fixed priority */
#  define AXBS_CRS_ARB_MASK           (1 << AXBS_CRS_ARB_SHIFT) /* Round-robin (rotating) priority */
                                               /* Bits 10-29:  Reserved */
#define AXBS_CRS_HLP                  (1 < 30) /* Bit 30: Halt low priority */
#define AXBS_CRS_RO                   (1 < 31) /* Bit 31: Read only */

/* Master General Purpose Control Register */

#define AXBS_MGPCR_AULB_SHIFT         (0)      /* Bits 0-2: Arbitrate on undefined length bursts */
#define AXBS_MGPCR_AULB_MASK          (7 << AXBS_MGPCR_AULB_SHIFT)
#  define AXBS_MGPCR_AULB_NONE        (0 << AXBS_MGPCR_AULB_SHIFT) /* No arbitration allowed */
#  define AXBS_MGPCR_AULB_ANY         (1 << AXBS_MGPCR_AULB_SHIFT) /* Arbitration allowed at any time */
#  define AXBS_MGPCR_AULB_4BEATS      (2 << AXBS_MGPCR_AULB_SHIFT) /* Arbitration allowed after four beats */
#  define AXBS_MGPCR_AULB_8BEATS      (3 << AXBS_MGPCR_AULB_SHIFT) /* Arbitration allowed after eight beats */
#  define AXBS_MGPCR_AULB_16BEATS     (4 << AXBS_MGPCR_AULB_SHIFT) /* Arbitration allowed after 16 beats */
                                               /* Bits 3-31:  Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_AXBS_H */
