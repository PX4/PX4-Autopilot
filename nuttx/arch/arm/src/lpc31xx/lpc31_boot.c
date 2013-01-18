/************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_boot.c
 *
 *   Copyright (C) 2009-2010, 2012 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "up_internal.h"
#include "up_arch.h"

#include "lpc31_syscreg.h"
#include "lpc31_cgudrvr.h"
#include "lpc31_internal.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};

/************************************************************************************
 * Public Variables
 ************************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/************************************************************************************
 * Private Variables
 ************************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical address
 * space of the LPCD313x.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s section_mapping[] =
{
  { LPC31_SHADOWSPACE_PSECTION, LPC31_SHADOWSPACE_VSECTION, 
    LPC31_SHADOWSPACE_MMUFLAGS, LPC31_SHADOWSPACE_NSECTIONS},
#ifndef CONFIG_PAGING /* SRAM is already fully mapped */
  { LPC31_INTSRAM_PSECTION, LPC31_INTSRAM_VSECTION, 
    LPC31_INTSRAM_MMUFLAGS, LPC31_INTSRAM_NSECTIONS},
#endif
#ifdef CONFIG_ARCH_ROMPGTABLE
  { LPC31_INTSROM0_PSECTION, LPC31_INTSROM0_VSECTION, 
    LPC31_INTSROM_MMUFLAGS, LPC31_INTSROM0_NSECTIONS},
#endif
  { LPC31_APB01_PSECTION, LPC31_APB01_VSECTION, 
    LPC31_APB01_MMUFLAGS, LPC31_APB01_NSECTIONS},
  { LPC31_APB2_PSECTION, LPC31_APB2_VSECTION, 
    LPC31_APB2_MMUFLAGS, LPC31_APB2_NSECTIONS},
  { LPC31_APB3_PSECTION, LPC31_APB3_VSECTION, 
    LPC31_APB3_MMUFLAGS, LPC31_APB3_NSECTIONS},
  { LPC31_APB4MPMC_PSECTION, LPC31_APB4MPMC_VSECTION, 
    LPC31_APB4MPMC_MMUFLAGS, LPC31_APB4MPMC_NSECTIONS},
  { LPC31_MCI_PSECTION, LPC31_MCI_VSECTION, 
    LPC31_MCI_MMUFLAGS, LPC31_MCI_NSECTIONS},
  { LPC31_USBOTG_PSECTION, LPC31_USBOTG_VSECTION, 
    LPC31_USBOTG_MMUFLAGS, LPC31_USBOTG_NSECTIONS},
#if defined(CONFIG_ARCH_EXTSRAM0) && CONFIG_ARCH_EXTSRAM0SIZE > 0
  { LPC31_EXTSRAM_PSECTION, LPC31_EXTSRAM_VSECTION, 
    LPC31_EXTSDRAM_MMUFLAGS, LPC31_EXTSRAM_NSECTIONS},
#endif
#if defined(CONFIG_ARCH_EXTDRAM) && CONFIG_ARCH_EXTDRAMSIZE > 0
  { LPC31_EXTSDRAM0_PSECTION, LPC31_EXTSDRAM0_VSECTION, 
    LPC31_EXTSDRAM_MMUFLAGS, LPC31_EXTSDRAM0_NSECTIONS},
#endif
  { LPC31_INTC_PSECTION, LPC31_INTC_VSECTION, 
    LPC31_INTC_MMUFLAGS, LPC31_INTC_NSECTIONS},
#ifdef CONFIG_ARCH_EXTNAND
  { LPC31_NAND_PSECTION, LPC31_NAND_VSECTION 
    LPC31_NAND_MMUFLAGS, LPC31_NAND_NSECTIONS},
#endif
};
#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_setlevel1entry
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void up_setlevel1entry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t*)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
}
#endif

/************************************************************************************
 * Name: up_setlevel2coarseentry
 ************************************************************************************/

static inline void up_setlevel2coarseentry(uint32_t ctabvaddr, uint32_t paddr,
                                           uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *ctable  = (uint32_t*)ctabvaddr;
  uint32_t  index;

  /* The coarse table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The coarse page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the coarse table entry */

  ctable[index] = (paddr | mmuflags);
}

/************************************************************************************
 * Name: up_setupmappings
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static void up_setupmappings(void)
{
  int i, j;

  for (i = 0; i < NMAPPINGS; i++)
    {
      uint32_t sect_paddr = section_mapping[i].physbase;
      uint32_t sect_vaddr = section_mapping[i].virtbase;
      uint32_t mmuflags   = section_mapping[i].mmuflags;

      for (j = 0; j < section_mapping[i].nsections; j++)
        {
          up_setlevel1entry(sect_paddr, sect_vaddr, mmuflags);
          sect_paddr += SECTION_SIZE;
          sect_vaddr += SECTION_SIZE;
        }
    }
}
#endif

/************************************************************************************
 * Name: up_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ************************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
static void  up_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of ISRAM is at the base of the L2 page table */

  uint32_t *ptr = (uint32_t*)PG_L2_VECT_VADDR;
  uint32_t pte;

  /* The pte might be zero the first time this function is called. */

  pte = *ptr;
  if (pte == 0)
    {
      pte = PG_VECT_PBASE;   
    }
  else
    {
      pte &= PG_L1_PADDRMASK;
    }

  /* Update the MMU flags and save */

  *ptr = pte | mmuflags;

  /* Invalid the TLB for this address */

  tlb_invalidate_single(PG_L2_VECT_VADDR);
}
#endif

/************************************************************************************
 * Name: up_vectormapping
 *
 * Description:
 *   Setup a special mapping for the interrupt vectors when (1) the interrupt
 *   vectors are not positioned in ROM, and when (2) the interrupt vectors are
 *   located at the high address, 0xffff0000.  When the interrupt vectors are located
 *   in ROM, we just have to assume that they were set up correctly;  When vectors
 *   are located in low memory, 0x00000000, the shadow memory region will be mapped
 *   to support them.
 *
 ************************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_ARCH_LOWVECTORS)
static void up_vectormapping(void)
{
  uint32_t vector_paddr = LPC31_VECTOR_PADDR;
  uint32_t vector_vaddr = LPC31_VECTOR_VADDR;
  uint32_t end_paddr    = vector_paddr + VECTOR_TABLE_SIZE;

  /* We want to keep our interrupt vectors and interrupt-related logic in zero-wait
   * state internal RAM (IRAM).  The DM320 has 16Kb of IRAM positioned at physical
   * address 0x0000:0000; we need to map this to 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      up_setlevel2coarseentry(PGTABLE_L2_COARSE_VBASE,  vector_paddr,
                              vector_vaddr, MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 coarse page table. */

  up_setlevel1entry(PGTABLE_L2_COARSE_PBASE, LPC31_VECTOR_VCOARSE,
                    MMU_L1_VECTORFLAGS);
}
#endif

/************************************************************************************
 * Name: up_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.
 *
 ************************************************************************************/

static void up_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

  /* If we are using vectors in low memory but RAM in that area has been marked
   * read only, then temparily mark the mapping write-able (non-buffered).
   */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  up_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif

  /* Copy the vectors into ISRAM at the address that will be mapped to the vector
   * address:
   *
   *   LPC31_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
   *   LPC31_VECTOR_VSRAM - Virtual address of vector table in SRAM
   *   LPC31_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
   */

  src  = (uint32_t*)&_vector_start;
  end  = (uint32_t*)&_vector_end;
  dest = (uint32_t*)LPC31_VECTOR_VSRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

  /* Make the vectors read-only, cacheable again */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  up_vectorpermissions(MMU_L2_VECTROFLAGS);
#endif

  /* Then set the LPC313x shadow register, LPC31_SYSCREG_ARM926SHADOWPTR, so that
   * the vector table is mapped to address 0x0000:0000 - NOTE: that there is not yet
   * full support for the vector table at address 0xffff0000.
   */

  putreg32(LPC31_VECTOR_PADDR, LPC31_SYSCREG_ARM926SHADOWPTR);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_boot
 *
 * Description:
 *   Complete boot operations started in up_head.S
 *
 ************************************************************************************/

void up_boot(void)
{
  /* __start provided the basic MMU mappings for SRAM.  Now provide mappings for all
   * IO regions (Including the vector region).
   */

#ifndef CONFIG_ARCH_ROMPGTABLE
  up_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in high
   * memory.
   */

#ifndef CONFIG_ARCH_LOWVECTORS
  up_vectormapping();
#endif
#endif /* CONFIG_ARCH_ROMPGTABLE */

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * up_vector.S
   */

  up_copyvectorblock();

  /* Reset all clocks */

  lpc31_resetclks();

  /* Initialize the PLLs */

  lpc31_hp1pllconfig();
  lpc31_hp0pllconfig();
  
  /* Initialize clocking to settings provided by board-specific logic */

  lpc31_clkinit(&g_boardclks);   

  /* Map first 4KB of ARM space to ISRAM area */

  putreg32(LPC31_INTSRAM0_PADDR, LPC31_SYSCREG_ARM926SHADOWPTR);

  /* Perform common, low-level chip initialization (might do nothing) */

  lpc31_lowsetup();

  /* Perform early serial initialization if we are going to use the serial driver */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif

  /* Perform board-specific initialization */

  lpc31_boardinitialize();
}
