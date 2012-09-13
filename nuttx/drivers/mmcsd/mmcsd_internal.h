/****************************************************************************
 * drivers/mmcsd/mmcsd_internal.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __DRIVERS_MMCSD_MMCSD_INTERNAL_H
#define __DRIVERS_MMCSD_MMCSD_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Enable excessive debug options */

#undef CONFIG_MMCSD_DUMPALL /* MUST BE DEFINED MANUALLY */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined(CONFIG_DEBUG_FS)
#  undef CONFIG_MMCSD_DUMPALL
#endif

/* Card type */

#define MMCSD_CARDTYPE_UNKNOWN       0  /* Unknown card type */
#define MMCSD_CARDTYPE_MMC           1  /* Bit 0: MMC card */
#define MMCSD_CARDTYPE_SDV1          2  /* Bit 1: SD version 1.x */
#define MMCSD_CARDTYPE_SDV2          4  /* Bit 2: SD version 2.x with byte addressing */
#define MMCSD_CARDTYPE_BLOCK         8  /* Bit 3: SD version 2.x with block addressing */

#define IS_MMC(t)   (((t) & MMCSD_CARDTYPE_MMC) != 0)
#define IS_SD(t)    (((t) & (MMCSD_CARDTYPE_SDV1|MMCSD_CARDTYPE_SDV2)) != 0)
#define IS_SDV1(t)  (((t) & MMCSD_CARDTYPE_SDV1) != 0)
#define IS_SDV2(t)  (((t) & MMCSD_CARDTYPE_SDV2) != 0)
#define IS_BLOCK(t) (((t) & MMCSD_CARDTYPE_BLOCK) != 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifdef CONFIG_MMCSD_DUMPALL
#  define mmcsd_dumpbuffer(m,b,l) fvdbgdumpbuffer(m,b,l)
#else
#  define mmcsd_dumpbuffer(m,b,l)
#endif

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_FS)
EXTERN void mmcsd_dmpcsd(FAR const uint8_t *csd, uint8_t cardtype);
#else
#  define mmcsd_dmpcsd(csd,cadtype)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __DRIVERS_MMCSD_MMCSD_INTERNAL_H */
