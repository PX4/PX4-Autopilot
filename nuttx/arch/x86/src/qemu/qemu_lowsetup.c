/****************************************************************************
 *  arch/x86/src/qemu/qemu_lowsetup.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gdt_entry_s gdt_entries[5];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_gdtentry
 *
 * Description:
 *   Set the value of one GDT entry.
 *
 ****************************************************************************/

static void up_gdtentry(struct gdt_entry_s *entry, uint32_t base,
                        uint32_t limit, uint8_t access, uint8_t gran)
{
  entry->lowbase      = (base & 0xffff);
  entry->midbase      = (base >> 16) & 0xff;
  entry->hibase       = (base >> 24) & 0xff;

  entry->lowlimit     = (limit & 0xffff);
  entry->granularity  = (limit >> 16) & 0x0f;
    
  entry->granularity |= gran & 0xf0;
  entry->access       = access;
}

/****************************************************************************
 * Name: up_gdtinit
 *
 * Description:
 *   Initialize the GDT. The Global Descriptor Table or GDT is a data
 *   structure used by Intel x86-family processors starting with the 80286
 *   in order to define the characteristics of the various memory areas used
 *   during program execution, for example the base address, the size and
 *   access privileges like executability and writability. These memory areas
 *   are called segments in Intel terminology.
 *
 ****************************************************************************/

static void up_gdtinit(void)
{
  struct gdt_ptr_s gdt_ptr;

  up_gdtentry(&gdt_entries[0], 0, 0, 0, 0);                /* Null segment */
  up_gdtentry(&gdt_entries[1], 0, 0xffffffff, 0x9a, 0xcf); /* Code segment */
  up_gdtentry(&gdt_entries[2], 0, 0xffffffff, 0x92, 0xcf); /* Data segment */
  up_gdtentry(&gdt_entries[3], 0, 0xffffffff, 0xfa, 0xcf); /* User mode code segment */
  up_gdtentry(&gdt_entries[4], 0, 0xffffffff, 0xf2, 0xcf); /* User mode data segment */

  gdt_ptr.limit = (sizeof(struct gdt_entry_s) * 5) - 1;
  gdt_ptr.base  = (uint32_t)gdt_entries;
  gdt_flush((uint32_t )&gdt_ptr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_lowsetup
 *
 * Description:
 *   Called from qemu_head BEFORE starting the operating system in order
 *   perform any necessary, early initialization.
 *
 ****************************************************************************/

void up_lowsetup(void)
{
  /* Initialize the Global descriptor table */

  up_gdtinit();

  /* Early serial driver initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Now perform board-specific initializations */

  up_boardinitialize();
}

