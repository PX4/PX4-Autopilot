/************************************************************************************
 * arch/arm/src/kinetis/kinetis_smc.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SMC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SMC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_SMC_SRSH_OFFSET    0x0000 /* System Reset Status Register High */
#define KINETIS_SMC_SRSL_OFFSET    0x0001 /* System Reset Status Register Low */
#define KINETIS_SMC_PMPROT_OFFSET  0x0002 /* Power Mode Protection Register */
#define KINETIS_SMC_PMCTRL_OFFSET  0x0003 /* Power Mode Control Register */

/* Register Addresses ***************************************************************/

#define KINETIS_SMC_SRSH           (KINETIS_SMC_BASE+KINETIS_SMC_SRSH_OFFSET)
#define KINETIS_SMC_SRSL           (KINETIS_SMC_BASE+KINETIS_SMC_SRSL_OFFSET)
#define KINETIS_SMC_PMPROT         (KINETIS_SMC_BASE+KINETIS_SMC_PMPROT_OFFSET)
#define KINETIS_SMC_PMCTRL         (KINETIS_SMC_BASE+KINETIS_SMC_PMCTRL_OFFSET)

/* Register Bit Definitions *********************************************************/

/* System Reset Status Register High */

#define SMC_SRSH_JTAG              (1 << 0)  /* Bit 0:  JTAG generated reset */
#define SMC_SRSH_LOCKUP            (1 << 1)  /* Bit 1:  Core Lock-up */
#define SMC_SRSH_SW                (1 << 2)  /* Bit 2:  Software */
                                             /* Bits 3-7: Reserved */

/* System Reset Status Register Low */

#define SMC_SRSL_WAKEUP            (1 << 0)  /* Bit 0:  Low-leakage wakeup reset */
#define SMC_SRSL_LVD               (1 << 1)  /* Bit 1:  Low-voltage detect reset */
#define SMC_SRSL_LOC               (1 << 2)  /* Bit 2:  Loss-of-clock reset */
                                             /* Bits 3-4: Reserved */
#define SMC_SRSL_COP               (1 << 5)  /* Bit 5:  Computer Operating Properly (COP) Watchdog */
#define SMC_SRSL_PIN               (1 << 6)  /* Bit 6:  External reset pin */
#define SMC_SRSL_POR               (1 << 7)  /* Bit 7:  Power-on reset */

/* Power Mode Protection Register */

#define SMC_PMPROT_AVLLS1          (1 << 0)  /* Bit 0:  Allow very low leakage stop 1 mod */
#define SMC_PMPROT_AVLLS2          (1 << 1)  /* Bit 1:  Allow very low leakage stop 2 mode */
#define SMC_PMPROT_AVLLS3          (1 << 2)  /* Bit 2:  Allow Very Low Leakage Stop 3 Mode */
                                             /* Bit 3:  Reserved */
#define SMC_PMPROT_ALLS            (1 << 4)  /* Bit 4:  Allow low leakage stop mode */
#define SMC_PMPROT_AVLP            (1 << 5)  /* Bit 5:  Allow very low power modes */
                                             /* Bits 6-7: Reserved */
/* Power Mode Control Register */

#define SMC_PMCTRL_LPLLSM_SHIFT    (0)       /* Bits 0-2: Low Power, Low Leakage Stop Mode */
#define SMC_PMCTRL_LPLLSM_MASK     (7 << SMC_PMCTRL_LPLLSM_SHIFT)
#  define SMC_PMCTRL_LPLLSM_NORMAL (0 << SMC_PMCTRL_LPLLSM_SHIFT) /* Normal stop */
#  define SMC_PMCTRL_LPLLSM_VLPS   (2 << SMC_PMCTRL_LPLLSM_SHIFT) /* Very low power stop */
#  define SMC_PMCTRL_LPLLSM_LLS    (3 << SMC_PMCTRL_LPLLSM_SHIFT) /* Low leakage stop */
#  define SMC_PMCTRL_LPLLSM_VLLS3  (5 << SMC_PMCTRL_LPLLSM_SHIFT) /* Very low leakage stop 3 */
#  define SMC_PMCTRL_LPLLSM_VLLS2  (6 << SMC_PMCTRL_LPLLSM_SHIFT) /* Very low leakage stop 2 */
#  define SMC_PMCTRL_LPLLSM_VLLS1  (7 << SMC_PMCTRL_LPLLSM_SHIFT) /* Very low leakage stop 1 */
                                             /* Bits 3-4: Reserved */
#define SMC_PMCTRL_RUNM_SHIFT      (5)       /* Bits 5-6: Run Mode Enable */
#define SMC_PMCTRL_RUNM_MASK       (3 << SMC_PMCTRL_RUNM_SHIFT)
#  define SMC_PMCTRL_RUNM_NORMAL   (0 << SMC_PMCTRL_RUNM_SHIFT) /* Normal run mode */
#  define SMC_PMCTRL_RUNM_VLP      (2 << SMC_PMCTRL_RUNM_SHIFT) /* Very low power run mode */
#define SMC_PMCTRL_LPWUI           (1 << 7)  /* Bit 7:  Low Power Wake Up on Interrupt */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SMC_H */
