/************************************************************************************
 * arch/arm/src/lm/chip/lm_flash.h
 *
 *   Copyright (C) 2009, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_FLASH_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_FLASH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* FLASH register offsets ***********************************************************/

/* The FMA, FMD, FMC, FCRIS, FCIM, and FCMISC registers are relative to the Flash
 * control base address of LM_FLASHCON_BASE.
 */

#define LM_FLASH_FMA_OFFSET       0x000 /* Flash memory address */
#define LM_FLASH_FMD_OFFSET       0x004 /* Flash memory data */
#define LM_FLASH_FMC_OFFSET       0x008 /* Flash memory control */
#define LM_FLASH_FCRIS_OFFSET     0x00c /* Flash controller raw interrupt status */
#define LM_FLASH_FCIM_OFFSET      0x010 /* Flash controller interrupt mask */
#define LM_FLASH_FCMISC_OFFSET    0x014 /* Flash controller masked interrupt status and clear */

/* The FMPREn, FMPPEn, USECRL, USER_DBG, and USER_REGn registers are relative to the
 * System Control base address of LM_SYSCON_BASE
 */

#define LM_FLASH_FMPRE_OFFSET     0x130 /* Flash memory protection read enable */
#define LM_FLASH_FMPPE_OFFSET     0x134 /* Flash memory protection program enable */
#define LM_FLASH_USECRL_OFFSET    0x140 /* USec Reload */
#define LM_FLASH_USERDBG_OFFSET   0x1d0 /* User Debug */
#define LM_FLASH_USERREG0_OFFSET  0x1e0 /* User Register 0 */
#define LM_FLASH_USERREG1_OFFSET  0x1e4 /* User Register 1 */
#define LM_FLASH_FMPRE0_OFFSET    0x200 /* Flash Memory Protection Read Enable 0 */
#define LM_FLASH_FMPRE1_OFFSET    0x204 /* Flash Memory Protection Read Enable 1 */
#define LM_FLASH_FMPRE2_OFFSET    0x208 /* Flash Memory Protection Read Enable 2 */
#define LM_FLASH_FMPRE3_OFFSET    0x20c /* Flash Memory Protection Read Enable 3 */
#define LM_FLASH_FMPPE0_OFFSET    0x400 /* Flash Memory Protection Program Enable 0 */
#define LM_FLASH_FMPPE1_OFFSET    0x404 /* Flash Memory Protection Program Enable 1 */
#define LM_FLASH_FMPPE2_OFFSET    0x408 /* Flash Memory Protection Program Enable 2 */
#define LM_FLASH_FMPPE3_OFFSET    0x40c /*  Flash Memory Protection Program Enable 3 */

/* FLASH register addresses *********************************************************/

/* The FMA, FMD, FMC, FCRIS, FCIM, and FCMISC registers are relative to the Flash
 * control base address of LM_FLASHCON_BASE.
 */

#define LM_FLASH_FMA              (LM_FLASHCON_BASE + LM_FLASH_FMA_OFFSET)
#define LM_FLASH_FMD              (LM_FLASHCON_BASE + LM_FLASH_FMD_OFFSET)
#define LM_FLASH_FMC              (LM_FLASHCON_BASE + LM_FLASH_FMC_OFFSET)
#define LM_FLASH_FCRIS            (LM_FLASHCON_BASE + LM_FLASH_FCRIS_OFFSET)
#define LM_FLASH_FCIM             (LM_FLASHCON_BASE + LM_FLASH_FCIM_OFFSET)
#define LM_FLASH_FCMISC           (LM_FLASHCON_BASE + LM_FLASH_FCMISC_OFFSET)

/* The FMPREn, FMPPEn, USECRL, USER_DBG, and USER_REGn registers are relative to the
 * System Control base address of LM_SYSCON_BASE
 */

#define LM_FLASH_FMPRE            (LM_SYSCON_BASE + LM_FLASH_FMPRE_OFFSET)
#define LM_FLASH_FMPPE            (LM_SYSCON_BASE + LM_FLASH_FMPPE_OFFSET)
#define LM_FLASH_USECRL           (LM_SYSCON_BASE + LM_FLASH_USECRL_OFFSET)
#define LM_FLASH_USERDBG          (LM_SYSCON_BASE + LM_FLASH_USERDBG_OFFSET)
#define LM_FLASH_USERREG0         (LM_SYSCON_BASE + LM_FLASH_USERREG0_OFFSET)
#define LM_FLASH_USERREG1         (LM_SYSCON_BASE + LM_FLASH_USERREG1_OFFSET)
#define LM_FLASH_FMPRE0           (LM_SYSCON_BASE + LM_FLASH_FMPRE0_OFFSET)
#define LM_FLASH_FMPRE1           (LM_SYSCON_BASE + LM_FLASH_FMPRE1_OFFSET)
#define LM_FLASH_FMPRE2           (LM_SYSCON_BASE + LM_FLASH_FMPRE2_OFFSET)
#define LM_FLASH_FMPRE3           (LM_SYSCON_BASE + LM_FLASH_FMPRE3_OFFSET)
#define LM_FLASH_FMPPE0           (LM_SYSCON_BASE + LM_FLASH_FMPPE0_OFFSET)
#define LM_FLASH_FMPPE1           (LM_SYSCON_BASE + LM_FLASH_FMPPE1_OFFSET)
#define LM_FLASH_FMPPE2           (LM_SYSCON_BASE + LM_FLASH_FMPPE2_OFFSET)
#define LM_FLASH_FMPPE3           (LM_SYSCON_BASE + LM_FLASH_FMPPE3_OFFSET)

/* FLASH register bit defitiions ****************************************************/
/* To be provided */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_FLASH_H */
