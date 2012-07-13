/****************************************************************************
 * config/lpc4330-xplorer/src/up_nsh.c
 * arch/arm/src/board/up_nsh.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

/* This should be removed someday when we are confident in SPIFI */

#ifdef CONFIG_DEBUG_FS
#  include "up_arch.h"
#  include "chip/lpc43_cgu.h"
#  include "chip/lpc43_ccu.h"
#endif

#include "chip.h"
#include <nuttx/ramdisk.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* USB Configuration ********************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_LPC4330_XPLORER
#  define CONFIG_NSH_HAVEUSBDEV 1
#else
#  error "Unrecognized board"
#  undef CONFIG_NSH_HAVEUSBDEV
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef CONFIG_NSH_HAVEUSBDEV
#endif

/* SPIFI Configuration ******************************************************/
/* CONFIG_SPIFI_BLKDRVR - Enable to create a block driver on the SPFI device.
 * CONFIG_SPIFI_DEVNO - SPIFI minor device number.  The SPFI device will be
 *   at /dev/ramN, where N is the value of CONFIG_SPIFI_DEVNO.  Default: 0.
 * CONFIG_SPIFI_RDONLY - Create a read only device on SPIFI.
 * CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
 *   bytes into the device address space.  Default 0.
 * CONFIG_SPIFI_BLKSIZE - The size of one block.  SPIFI is not block oriented,
 *   so most any size of the block used in the SPIFI block device can be
 *   used.  NOTE: FAT will support only sector sizes of 512, 1024, 2048, or
 *   4096. Default: 512
 * CONFIG_SPIFI_NBLOCKS - The number of blocks in the file system, each of
 *   size CONFIG_SPIFI_BLKSIZE.  The end of the file system will be at
 *   device offset:
 *     CONFIG_SPIFI_OFFSET + CONFIG_SPIFI_BLKSIZE*CONFIG_SPIFI_NBLOCKS
 *   The must assure that this does offset does not go beyond the end of
 *   the FLASH memory.
 */

#ifdef CONFIG_SPIFI_BLKDRVR
#  ifndef CONFIG_SPIFI_DEVNO
#    define CONFIG_SPIFI_DEVNO 0
#  endif
#  ifndef CONFIG_SPIFI_OFFSET
#    define CONFIG_SPIFI_OFFSET 0
#  endif
#  ifndef CONFIG_SPIFI_BLKSIZE
#    define CONFIG_SPIFI_BLKSIZE 512
#  endif
#  ifndef CONFIG_SPIFI_NBLOCKS
#    error "Need number of SPIFI blocks (CONFIG_SPIFI_NBLOCKS)"
#  endif
#endif

#define SPIFI_BUFFER \
  (FAR uint8_t *)(LPC43_LOCSRAM_SPIFI_BASE + CONFIG_SPIFI_OFFSET)

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lib_lowprintf(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lib_lowprintf
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_spifi_initialize
 *
 * Description:
 *   Make the SPIFI (or part of it) into a block driver that can hold a
 *   file system.
 *
 *   SPIFI AHB register clock:
 *     Base clock   = BASE_M4_CLK
 *     Branch clock = CLK_M4_SPIFI
 *   SPIFI serial clock input:
 *     Base clock   = BASE_SPIFI_CLK
 *     Branch clock = SPIFI_CLK
 *
 ****************************************************************************/

#ifdef CONFIG_SPIFI_BLKDRVR
static int nsh_spifi_initialize(void)
{
  /* This should be removed someday when we are confident in SPIFI */

#ifdef CONFIG_DEBUG_FS
  fdbg("BASE_SPIFI_CLK=%08x\n",
       getreg32(LPC43_BASE_SPIFI_CLK));
  fdbg("SPFI CFG=%08x STAT=%08x\n",
       getreg32(LPC43_CCU1_SPIFI_CFG), getreg32(LPC43_CCU1_SPIFI_STAT));
  fdbg("M4 SPFI CFG=%08x STAT=%08x\n",
       getreg32(LPC43_CCU1_M4_SPIFI_CFG), getreg32(LPC43_CCU1_M4_SPIFI_STAT));
#endif

#ifdef CONFIG_SPIFI_RDONLY
  /* Register a read-only SPIFI RAM disk at /dev/ramN, where N is the
   * value of CONFIG_SPIFI_DEVNO.
   */

  return romdisk_register(CONFIG_SPIFI_DEVNO, SPIFI_BUFFER,
                          CONFIG_SPIFI_NBLOCKS, CONFIG_SPIFI_BLKSIZE);
#else
  /* Register a write-able SPIFI RAM disk at /dev/ramN, where N is the
   * value of CONFIG_SPIFI_DEVNO.
   */

  return ramdisk_register(CONFIG_SPIFI_DEVNO, SPIFI_BUFFER,
                          CONFIG_SPIFI_NBLOCKS, CONFIG_SPIFI_BLKSIZE,
                          true);
#endif
}
#else
#  define nsh_spifi_initialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
  /* Initialize the SPIFI block device */

  return nsh_spifi_initialize();
}
