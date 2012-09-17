/************************************************************************************
 * arch/arm/src/dm320/dm320_boot.c
 *
 *   Copyright (C) 2007, 2009-2012 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "arm.h"
#include "up_internal.h"
#include "up_arch.h"

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

static const struct section_mapping_s section_mapping[] =
{
  { DM320_PERIPHERALS_PSECTION, DM320_PERIPHERALS_VSECTION, 
    DM320_PERIPHERALS_MMUFLAGS, DM320_PERIPHERALS_NSECTIONS},
  { DM320_FLASH_PSECTION,       DM320_FLASH_VSECTION, 
    DM320_FLASH_MMUFLAGS,       DM320_FLASH_NSECTIONS},
  { DM320_CFI_PSECTION,         DM320_CFI_VSECTION, 
    DM320_CFI_MMUFLAGS,         DM320_CFI_NSECTIONS},
  { DM320_SSFDC_PSECTION,       DM320_SSFDC_VSECTION, 
    DM320_SSFDC_MMUFLAGS,       DM320_SSFDC_NSECTIONS},
  { DM320_CE1_PSECTION,         DM320_CE1_VSECTION, 
    DM320_CE1_MMUFLAGS,         DM320_CE1_NSECTIONS},
  { DM320_CE2_PSECTION,         DM320_CE2_VSECTION, 
    DM320_CE2_MMUFLAGS,         DM320_CE2_NSECTIONS},
  { DM320_VLYNQ_PSECTION,       DM320_VLYNQ_VSECTION, 
    DM320_VLYNQ_MMUFLAGS,       DM320_VLYNQ_NSECTIONS},
  { DM320_USBOTG_PSECTION,      DM320_USBOTG_VSECTION, 
    DM320_USBOTG_MMUFLAGS,      DM320_USBOTG_NSECTIONS}
};
#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_setlevel1entry
 ************************************************************************************/

static inline void up_setlevel1entry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t*)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
}

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

/************************************************************************************
 * Name: up_vectormapping
 ************************************************************************************/

static void up_vectormapping(void)
{
  uint32_t vector_paddr = DM320_IRAM_PADDR;
  uint32_t vector_vaddr = DM320_VECTOR_VADDR;
  uint32_t end_paddr    = vector_paddr + DM320_IRAM_SIZE;

  /* We want to keep our interrupt vectors and interrupt-related logic in zero-wait
   * state internal RAM (IRAM).  The DM320 has 16Kb of IRAM positioned at physical
   * address 0x0000:0000; we need to map this to 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      up_setlevel2coarseentry(PGTABLE_L2_COARSE_VBASE, vector_paddr, vector_vaddr,
                              MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 coarse page table. */

  up_setlevel1entry(PGTABLE_L2_COARSE_PBASE, DM320_VECTOR_VCOARSE, MMU_L1_VECTORFLAGS);
}

/************************************************************************************
 * Name: up_copyvectorblock
 ************************************************************************************/

static void up_copyvectorblock(void)
{
  uint32_t *src  = (uint32_t*)&_vector_start;
  uint32_t *end  = (uint32_t*)&_vector_end;
  uint32_t *dest = (uint32_t*)VECTOR_BASE;

  while (src < end)
    {
      *dest++ = *src++;
    }
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void up_boot(void)
{
  /* __start provided the basic MMU mappings for SDRAM.  Now provide mappings for all
   * IO regions (Including the vector region).
   */

  up_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in high
   * memory.
   */

  up_vectormapping();

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * up_vector.S
   */

  up_copyvectorblock();

  /* Set up the board-specific LEDs */

#ifdef CONFIG_ARCH_LEDS
  up_ledinit();
#endif
  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif
}
