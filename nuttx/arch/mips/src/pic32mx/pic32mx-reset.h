/************************************************************************************
 * arch/mips/src/pic32mx/pic32mx-reset.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "pic32mx-memorymap.h"

/************************************************************************************
 * Pre-Processor Definitions
 ************************************************************************************/
/* Register Offsets *****************************************************************/

#define PIC32MX_RESET_RCON_OFFSET      0x0000 /* Reset control register */
#define PIC32MX_RESET_RCONCLR_OFFSET   0x0004 /* RCON clear register */
#define PIC32MX_RESET_RCONSET_OFFSET   0x0008 /* RCON set register */
#define PIC32MX_RESET_RCONINV_OFFSET   0x000c /* RCON invert register */
#define PIC32MX_RESET_RSWRST_OFFSET    0x0010 /* Software reset register */
#define PIC32MX_RESET_RSWRSTCLR_OFFSET 0x0014 /* RSWRST clear register */
#define PIC32MX_RESET_RSWRSTSET_OFFSET 0x0018 /* RSWRST set register */
#define PIC32MX_RESET_RSWRSTINV_OFFSET 0x001c /* RSWRST invert register */

/* Register Addresses ***************************************************************/

#define PIC32MX_RESET_RCON             (PIC32MX_RESET_K1BASE+PIC32MX_RCON_OFFSET)
#define PIC32MX_RESET_RCONCLR          (PIC32MX_RESET_K1BASE+PIC32MX_RCONCLR_OFFSET)
#define PIC32MX_RESET_RCONSET          (PIC32MX_RESET_K1BASE+PIC32MX_RCONSET_OFFSET)
#define PIC32MX_RESET_RCONINV          (PIC32MX_RESET_K1BASE+PIC32MX_RCONINV_OFFSET)
#define PIC32MX_RESET_RSWRST           (PIC32MX_RESET_K1BASE+PIC32MX_RSWRST_OFFSET)
#define PIC32MX_RESET_RSWRSTCLR        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTCLR_OFFSET)
#define PIC32MX_RESET_RSWRSTSET        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTSET_OFFSET)
#define PIC32MX_RESET_RSWRSTINV        (PIC32MX_RESET_K1BASE+PIC32MX_RSWRSTINV_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Reset control register */

#define RESET_RCON_POR                 (1 << 0)  /* Bit 0: Power on reset */
#define RESET_RCON_BOR                 (1 << 1)  /* Bit 1: Brown out reset */
#define RESET_RCON_IDLE                (1 << 2)  /* Bit 2: Wake from idle */
#define RESET_RCON_SLEEP               (1 << 3)  /* Bit 3: Wake from sleep */
#define RESET_RCON_WDTO                (1 << 4)  /* Bit 4: Watchdog timer time-out */
#define RESET_RCON_SWR                 (1 << 6)  /* Bit 6: Software reset */
#define RESET_RCON_EXTR                (1 << 7)  /* Bit 7: External reset pin */
#define RESET_RCON_VREGS               (1 << 8)  /* Bit 8: Voltage regulator standby enable */
#define RESET_RCON_CMR                 (1 << 9)  /* Bit 9: Configuration mismatch reset */

/* Software reset register */

#define RESET_RSWRST_TRIGGER           (1 << 0)  /* Bit 0: Software reset trigger */

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
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_RESET_H */
