/************************************************************************************
 * arch/arm/src/kinetis/kinetis_mcm.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_MCM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_MCM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_MCM_PLASC_OFFSET    0x0008 /* Crossbar switch (AXBS) slave configuration */
#define KINETIS_MCM_PLAMC_OFFSET    0x000a /* Crossbar switch (AXBS) master configuration */
#define KINETIS_MCM_SRAMAP_OFFSET   0x000c /* SRAM arbitration and protection */
#define KINETIS_MCM_ISR_OFFSET      0x0010 /* Interrupt status register */
#define KINETIS_MCM_ETBCC_OFFSET    0x0014 /* ETB counter control register */
#define KINETIS_MCM_ETBRL_OFFSET    0x0018 /* ETB reload register */
#define KINETIS_MCM_ETBCNT_OFFSET   0x001c /* ETB counter value register */

/* Register Addresses ***************************************************************/

#define KINETIS_MCM_PLASC           (KINETIS_MCM_BASE+KINETIS_MCM_PLASC_OFFSET)
#define KINETIS_MCM_PLAMC           (KINETIS_MCM_BASE+KINETIS_MCM_PLAMC_OFFSET)
#define KINETIS_MCM_SRAMAP          (KINETIS_MCM_BASE+KINETIS_MCM_SRAMAP_OFFSET)
#define KINETIS_MCM_ISR             (KINETIS_MCM_BASE+KINETIS_MCM_ISR_OFFSET)
#define KINETIS_MCM_ETBCC           (KINETIS_MCM_BASE+KINETIS_MCM_ETBCC_OFFSET)
#define KINETIS_MCM_ETBRL           (KINETIS_MCM_BASE+KINETIS_MCM_ETBRL_OFFSET)
#define KINETIS_MCM_ETBCNT          (KINETIS_MCM_BASE+KINETIS_MCM_ETBCNT_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Crossbar switch (AXBS) slave configuration */

#define MCM_PLASC_ASC_SHIFT         (0)       /* Bits 0-7: Each bit in the ASC field
                                               * indicates if there is a corresponding
                                               * connection to the crossbar switch's
                                               * slave input port. */
#define MCM_PLASC_ASC_MASK          (0xff << MCM_PLASC_ASC_SHIFT)
#define MCM_PLASC_ASC(n)            ((1 << (n)) << MCM_PLASC_ASC_SHIFT)
                                              /* Bits 8-15: Reserved */

/* Crossbar switch (AXBS) master configuration */

#define MCM_PLAMC_AMC_SHIFT         (0)       /* Bits 0-7: Each bit in the AMC field
                                               * indicates if there is a corresponding
                                               * connection to the AXBS master input port. */
#define MCM_PLAMC_AMC_MASK          (0xff << MCM_PLAMC_AMC_SHIFT)
#define MCM_PLAMC_AMC(n)            ((1 << (n)) << MCM_PLAMC_AMC_SHIFT)
                                              /* Bits 8-15: Reserved */

/* SRAM arbitration and protection */
                                              /* Bits 0-23: Reserved */
#define MCM_SRAMAP_SRAMUAP_SHIFT    (24)      /* Bits 24-25: SRAM_U arbitration priority */
#define MCM_SRAMAP_SRAMUAP_MASK     (3 << MCM_SRAMAP_SRAMUAP_SHIFT)
#  define MCM_SRAMAP_SRAMUAP_RR     (0 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Round robin */
#  define MCM_SRAMAP_SRAMUAP_SRR    (1 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Special round robin */
#  define MCM_SRAMAP_SRAMUAP_FIXED1 (2 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Fixed pri. Proc highest/backdoor lowest */
#  define MCM_SRAMAP_SRAMUAP_FIXED2 (3 << MCM_SRAMAP_SRAMUAP_SHIFT) /* Fixed pri. Backdoor highest/proc lowest */
#define MCM_SRAMAP_SRAMUWP          (1 << 26) /* Bit 26: SRAM_U write protect */
                                              /* Bit 27: Reserved */
#define MCM_SRAMAP_SRAMLAP_SHIFT    (28)      /* Bits 28-29: SRAM_L arbitration priority */
#define MCM_SRAMAP_SRAMLAP_MASK     (3 << MCM_SRAMAP_SRAMLAP_SHIFT)
#  define MCM_SRAMAP_SRAMLAP_RR     (0 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Round robin */
#  define MCM_SRAMAP_SRAMLAP_SRR    (1 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Special round robin */
#  define MCM_SRAMAP_SRAMLAP_FIXED1 (2 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Fixed pri. Proc highest/backdoor lowest */
#  define MCM_SRAMAP_SRAMLAP_FIXED2 (3 << MCM_SRAMAP_SRAMLAP_SHIFT) /* Fixed pri. Backdoor highest/proc lowest */
#define MCM_SRAMAP_SRAMLWP          (1 << 30) /* Bit 30: SRAM_L write protect */
                                              /* Bit 31: Reserved */
/* Interrupt status register */
                                              /* Bit 0: Reserved */
#define MCM_ISR_IRQ                 (1 << 1)  /* Bit 1:  Normal interrupt pending */
#define MCM_ISR_NMI                 (1 << 2)  /* Bit 2:  Non-maskable interrupt pending */
                                              /* Bits 3-31: Reserved */
/* ETB counter control register */

#define MCM_ETBCC_CNTEN             (1 << 0)  /* Bit 0:  Counter enable */
#define MCM_ETBCC_RSPT_SHIFT        (1)       /* Bits 1-2: Response type */
#define MCM_ETBCC_RSPT_MASK         (3 << MCM_ETBCC_RSPT_SHIFT)
#  define MCM_ETBCC_RSPT_NONE       (0 << MCM_ETBCC_RSPT_SHIFT) /* No response when ETB count expires */
#  define MCM_ETBCC_RSPT_INT        (1 << MCM_ETBCC_RSPT_SHIFT) /* Normal interrupt when ETB count expires */
#  define MCM_ETBCC_RSPT_NMI        (2 << MCM_ETBCC_RSPT_SHIFT) /* NMI when ETB count expires */
#  define MCM_ETBCC_RSPT_HALT       (3 << MCM_ETBCC_RSPT_SHIFT) /* Debug halt when ETB count expires */
#define MCM_ETBCC_RLRQ              (1 << 3)  /* Bit 3:  Reload request */
#define MCM_ETBCC_ETDIS             (1 << 4)  /* Bit 4:  ETM-to-TPIU disable */
#define MCM_ETBCC_ITDIS             (1 << 5)  /* Bit 5:  ITM-to-TPIU disable */
                                              /* Bits 6-31: Reserved */
/* ETB reload register */

#define MCM_ETBRL_RELOAD_SHIFT      (0)       /* Bits 0-10: Byte count reload value */
#define MCM_ETBRL_RELOAD_MASK       (0x7ff << MCM_ETBRL_RELOAD_SHIFT)
                                              /* Bits 11-31: Reserved */
/* ETB counter value register */

#define MCM_ETBCNT_COUNTER_SHIFT    (0)       /* Bits 0-10: Byte count counter value */
#define MCM_ETBCNT_COUNTER_MASK     (0x7ff << MCM_ETBCNT_COUNTER_SHIFT)
                                              /* Bits 11-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_MCM_H */
