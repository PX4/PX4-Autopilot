/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-cvr.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CVR_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CVR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_CVR_CON_OFFSET    0x0000 /* Comparator voltage reference control register */
#define PIC32MX_CVR_CONCLR_OFFSET 0x0004 /* Comparator voltage reference control clear register */
#define PIC32MX_CVR_CONSET_OFFSET 0x0008 /* Comparator voltage reference control set register */
#define PIC32MX_CVR_CONINV_OFFSET 0x000c /* Comparator voltage reference control invert register */

/* Register Addresses ***************************************************************/

#define PIC32MX_CVR_CON           (PIC32MX_CVR_K1BASE+PIC32MX_CVR_CON_OFFSET)
#define PIC32MX_CVR_CONCLR        (PIC32MX_CVR_K1BASE+PIC32MX_CVR_CONCLR_OFFSET)
#define PIC32MX_CVR_CONSET        (PIC32MX_CVR_K1BASE+PIC32MX_CVR_CONSET_OFFSET)
#define PIC32MX_CVR_CONINV        (PIC32MX_CVR_K1BASE+PIC32MX_CVR_CONINV_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Comparator voltage reference control register */

#define CVR_CON_CVR_SHIFT         (0)       /* Bits 0-3: CVREF value selection */
#define CVR_CON_CVR_MASK          (15 << CVR_CON_CVR_SHIFT)
#  define CVR_CON_CVR(n)          ((n) << CVR_CON_CVR_SHIFT)
#define CVR_CON_CVRSS             (1 << 4)  /* Bit 4:  CVREF source selection */
#define CVR_CON_CVRR              (1 << 5)  /* Bit 5:  CVREF range selection */
#define CVR_CON_CVROE             (1 << 6)  /* Bit 6:  CVREFOUT enable */
#ifdef CHIP_VRFSEL
#  define CVR_CON_BGSEL_SHIFT     (8)       /* Bits 8-9: Band gap reference source */
#  define CVR_CON_BGSEL_MASK      (3 << CVR_CON_CVR_SHIFT)
#    define CVR_CON_BGSEL_1p2V    (0 << CVR_CON_CVR_SHIFT) /* IVREF = 1.2V (nominal) */
#    define CVR_CON_BGSEL_0p6V    (1 << CVR_CON_CVR_SHIFT) /* IVREF = 0.6V (nominal) */
#    define CVR_CON_BGSEL_0p2V    (2 << CVR_CON_CVR_SHIFT) /* IVREF = 0.2V (nominal) */
#    define CVR_CON_BGSEL_VREF    (3 << CVR_CON_CVR_SHIFT) /* VREF = VREF+ */
#  define CVR_CON_VREFSEL         (1 << 10) /* Bit 10:  Voltage reference select */
#endif
#define CVR_CON_ON                (1 << 15) /* Bit 15: Comparator voltage reference on */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_CVR_H */
