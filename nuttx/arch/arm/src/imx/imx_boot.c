/************************************************************************************
 * arch/arm/src/imx/imx_boot.c
 * arch/arm/src/chip/imx_boot.c
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
#include "up_internal.h"
#include "up_arch.h"

/************************************************************************************
 * Pre-processor Definitions
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

/* Mapping of the external memory regions will probably have to be made board
 * specific.
 */

static const struct section_mapping_s section_mapping[] =
{
  { IMX_PERIPHERALS_PSECTION, IMX_PERIPHERALS_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_PERIPHERALS_NSECTIONS},
  { IMX_FLASH_PSECTION,       IMX_FLASH_VSECTION, 
    IMX_FLASH_MMUFLAGS,       IMX_FLASH_NSECTIONS},
  { IMX_CS1_PSECTION,         IMX_CS1_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_CS1_NSECTIONS},
  { IMX_CS2_PSECTION,         IMX_CS2_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_CS2_NSECTIONS},
  { IMX_CS3_PSECTION,         IMX_CS3_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_CS3_NSECTIONS},
  { IMX_CS4_PSECTION,         IMX_CS4_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_CS4_NSECTIONS},
  { IMX_CS5_PSECTION,         IMX_CS5_VSECTION, 
    IMX_PERIPHERALS_MMUFLAGS, IMX_CS5_NSECTIONS},
};

#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/* All i.MX architectures must provide the following entry point.  This entry point
 * is called early in the intitialization -- after all memory has been configured
 * and mapped but before any devices have been initialized.
 */

extern void imx_boardinitialize(void);

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
 * Name: up_copyvectorblock
 ************************************************************************************/

static void up_copyvectorblock(void)
{
  /* There are three operational memory configurations:
   *
   * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case:
   *
   *    - Our vectors must be located at the beginning of FLASH and will
   *      also be mapped to address zero (because of the i.MX's "double map image."
   *    - There is nothing to be done here in this case.
   *
   * 2. We boot in FLASH but copy ourselves to DRAM from better performance.
   *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case:
   *
   *    - Our code image is in FLASH and we boot to FLASH initially, then copy
   *      ourself to DRAM, and
   *    - DRAM will be mapped to address zero.
   *    - There is nothing to be done here in this case.
   *
   * 3. There is bootloader that copies us to DRAM, but probably not to the beginning
   *    of DRAM (say to 0x0900:0000) (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n).
   *    In this case:
   *
   *    - DRAM will be mapped to address zero.
   *    - Interrupt vectors will be copied to address zero in this function.
   */

#if !defined(CONFIG_BOOT_RUNFROMFLASH) && !defined(CONFIG_BOOT_COPYTORAM)
  uint32_t *src  = (uint32_t*)&_vector_start;
  uint32_t *end  = (uint32_t*)&_vector_end;
  uint32_t *dest = (uint32_t*)VECTOR_BASE;

  while (src < end)
    {
      *dest++ = *src++;
    }
#endif
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

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * up_vector.S
   */

  up_copyvectorblock();

  /* Perform board-specific initialiation */

  imx_boardinitialize();

  /* Set up the board-specific LEDs */

#ifdef CONFIG_ARCH_LEDS
  up_ledinit();
#endif
  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  up_earlyserialinit();
#endif
}
