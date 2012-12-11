/****************************************************************************
 * arch/z80/src/z180/z180_mmu.h
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

#ifndef __ARCH_Z80_SRC_Z180_Z180_MMU_H
#define __ARCH_Z80_SRC_Z180_Z180_MMU_H

/* See arch/z80/src/z180/z180_mmu.txt for additional information */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "z180_iomap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Virtual addresses */

#ifndef CONFIG_Z180_BANKAREA_VIRTBASE
#  warning "Assuming Bank Area at virtual address 0x8000"
#  define CONFIG_Z180_BANKAREA_VIRTBASE 0x8000
#endif

#ifndef CONFIG_Z180_COMMON1AREA_VIRTBASE
#  warning "Assuming Common Area 1 at virtual address 0xc000"
#  define CONFIG_Z180_COMMON1AREA_VIRTBASE 0xc000
#endif

#if CONFIG_Z180_BANKAREA_VIRTBASE > CONFIG_Z180_COMMON1AREA_VIRTBASE
#  error "CONFIG_Z180_BANKAREA_VIRTBASE > CONFIG_Z180_COMMON1AREA_VIRTBASE"
#endif

/* Physical addresses */

#ifndef CONFIG_Z180_BANKAREA_PHYSBASE
#  warning "Assuming Bank Area 1 at physical address 0x080000"
#  define CONFIG_Z180_BANKAREA_PHYSBASE 0x08000
#endif

#ifndef CONFIG_Z180_PHYSHEAP_START
#  warning "Assuming physical heap starts at physical address 0x0c000"
#  define CONFIG_Z180_PHYSHEAP_START 0x0c000
#endif

#ifndef CONFIG_Z180_PHYSHEAP_END
#  warning "Assuming physical heap ends at physical address 0x100000"
#  define CONFIG_Z180_PHYSHEAP_END 0x100000
#endif

#if CONFIG_Z180_BANKAREA_PHYSBASE > CONFIG_Z180_PHYSHEAP_START
#  error "CONFIG_Z180_BANKAREA_PHYSBASE > CONFIG_Z180_PHYSHEAP_START"
#endif

#if CONFIG_Z180_PHYSHEAP_START > CONFIG_Z180_PHYSHEAP_END
#  error "CONFIG_Z180_PHYSHEAP_START > CONFIG_Z180_PHYSHEAP_END"
#endif

/* Each page is 4KB */

#define Z180_PAGESHIFT          (12)
#define Z180_PAGESIZE           (1 << Z180_PAGESHIFT)
#define Z180_PAGEMASK           (Z180_PAGESIZE - 1)
#define PHYS_ALIGN(phys)        ((phys) >> Z180_PAGESHIFT)
#define PHYS_ALIGNUP(phys)      (((phys) + Z180_PAGEMASK) >> Z180_PAGESHIFT)

/* Physical pages */

#define Z180_BANKAREA_PHYSPAGE  PHYS_ALIGN(CONFIG_Z180_BANKAREA_PHYSBASE)
#define Z180_PHYSHEAP_STARTPAGE PHYS_ALIGN(CONFIG_Z180_PHYSHEAP_START)
#define Z180_PHYSHEAP_ENDPAGE   PHYS_ALIGN(CONFIG_Z180_PHYSHEAP_END)
#define Z180_PHYSHEAP_NPAGES    (Z180_PHYSHEAP_ENDPAGE - Z180_PHYSHEAP_STARTPAGE + 1)

/* MMU register values */

#define Z180_CBAR_BA_VALUE  (((CONFIG_Z180_BANKAREA_VIRTBASE >> 12) & 0x0f) << CBAR_BA_SHIFT)
#define Z180_CBAR_CA_VALUE  (((CONFIG_Z180_COMMON1AREA_VIRTBASE >> 12) & 0x0f) << CBAR_CA_SHIFT)
#define Z180_CBAR_VALUE     (Z180_CBAR_BA_VALUE | Z180_CBAR_CA_VALUE)
#define Z180_BBR_VALUE      ((CONFIG_Z180_BANKAREA_PHYSBASE >> 12) & 0xff)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_mmu_lowinit
 *
 * Description:
 *   Low-level, power-up initialization of the z180 MMU.  this must be
 *   called very early in the boot process to get the basic operating
 *   memory configuration correct.  This function does *not* perform all
 *   necessray MMU initialization... only the basics needed at power-up.
 *   up_mmuinit() must be called later to complete the entire MMU
 *   initialization.
 *
 ****************************************************************************/

void z180_mmu_lowinit(void) __naked;

/****************************************************************************
 * Name: up_mmuinit
 *
 * Description:
 *   Perform higher level initialization of the MMU and physical memory
 *   memory management logic.  More correctly prototypes in up_internal.h.
 *
 ****************************************************************************/

int up_mmuinit(void);

#endif /* __ARCH_Z80_SRC_Z180_Z180_MMU_H */
