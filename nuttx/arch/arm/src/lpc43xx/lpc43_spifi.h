/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_spifi.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H
#define __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mtd.h>

#include "chip.h"
#include "chip/lpc43_spifi.h"

#ifdef CONFIG_LPC43_SPIFI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* SPIFI Configuration ******************************************************/
/* This logic supports some special options that can be used to create an
 * MTD device on the SPIFI FLASH.
 *
 *    CONFIG_LPC43_SPIFI - Enable SPIFI support
 *
 * SPIFI device geometry:
 *
 *   CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
 *     bytes into the device address space.  This offset must be an exact
 *     multiple of the erase block size (CONFIG_SPIFI_BLKSIZE). Default 0.
 *   CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not defined
 *     then the driver will try to determine the correct erase block size by
 *     examining that data returned from spifi_initialize (which sometimes
 *     seems bad).
 *
 * Other SPIFI options
 *
 *   CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
 *     use an external library implementation of the SPIFI interface.
 *   CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
 *     FAT friendly 512 byte sector size and will manage the read-modify-write
 *     operations on the larger erase block.
 *   CONFIG_SPIFI_READONLY - Define to support only read-only operations.
 */

#ifndef CONFIG_SPIFI_OFFSET
#  define CONFIG_SPIFI_OFFSET 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lpc43_spifi_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for the SPIFI device.  MTD
 *   devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 *   SPIFI interface clocking is configured per settings in the board.h file.
 *
 * Input Parameters:
 *   None
 *
 * Returned value:
 *   One success, a reference to the initialized MTD device instance is
 *   returned;  NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *lpc43_spifi_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_LPC43_SPIFI */
#endif /* __ARCH_ARM_SRC_LPC43XX_LPC43_SPIFI_H */

