/****************************************************************************
 * arch/z80/src/z180/z180_mmu.c
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

/* See arch/z80/src/z180/z180_mmu.txt for additional information */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/gran.h>

#include <arch/irq.h>
#include <arch/io.h>

#include "up_internal.h"
#include "z180_mmu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ADDRENV
#  warning "OS address environment support is required (CONFIG_ADDRENV)"
#endif

#ifndef CONFIG_GRAN
#  warning "This file requires the granual allocator (CONFIG_GRAN)"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_GRAN_SINGLE
static GRAN_HANDLE g_physhandle;
#endif
static struct z180_cbr_s g_cbrs[CONFIG_MAX_TASKS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_mmu_alloccbr
 *
 * Description:
 *   Find an unused struture in g_cbrs (i.e., one with reference count == 0).
 *   If a structure is found, its reference count is set to one and a pointer
 *   to the structure is returned.
 *
 ****************************************************************************/

static inline FAR struct z180_cbr_s *z180_mmu_alloccbr(void)
{
  int i;

  for (i = 0; i < CONFIG_MAX_TASKS; i++)
    {
      FAR struct z180_cbr_s *cbr = &g_cbrs[i];
      if (cbr->crefs == 0)
        {
          cbr->crefs = 1;
          return cbr;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: z180_mmu_freecbr
 *
 * Description:
 *   Free a struture in g_cbrs by setting its reference count to 0;
 *
 ****************************************************************************/

#define z180_mmu_freecbr(cbr) (cbr)->crefs = 0

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

void z180_mmu_lowinit(void) __naked
{
  /* Set the CBAR register to set up the virtual address of the Bank Area and
   * Common Area 1.  Set the BBR register to set up the physical mapping for
   * the Bank Area (the physical mapping for Common Area 1 will not be done
   * until the first task is started.
   */

  __asm
	ld	c, #Z180_MMU_CBAR		; port
	ld	a, #Z180_CBAR_VALUE		; value
	out	(c), a

	ld	c, #Z180_MMU_BBR		; port
	ld	a, #Z180_BBR_VALUE		; value
	out	(c), a
  __endasm;
}

/****************************************************************************
 * Name: up_mmuinit
 *
 * Description:
 *   Perform higher level initialization of the MMU and physical memory
 *   memory management logic.
 *
 ****************************************************************************/

int up_mmuinit(void)
{
  /* Here we use the granule allocator as a page allocator.  We lie and
   * say that 1 page is 1 byte.
   */

#ifdef CONFIG_GRAN_SINGLE
return gran_initialize((FAR void *)Z180_PHYSHEAP_STARTPAGE,
                       Z180_PHYSHEAP_NPAGES, 0, 0);
#else
g_physhandle = gran_initialize((FAR void *)Z180_PHYSHEAP_STARTPAGE,
                               Z180_PHYSHEAP_NPAGES, 0, 0);
return g_physhandle ? OK : -ENOMEM;
#endif
}

/****************************************************************************
 * Address Environment Interfaces
 *
 * Low-level interfaces used in binfmt/ to instantiate tasks with address
 * environments.  These interfaces all operate on task_addrenv_t which is an
 * abstract representation of the address environment and must be provided
 * by arch/arch.h is CONFIG_ADDRENV is defined.
 *
 *   up_addrenv_create  - Create an address environment
 *   up_addrenv_vaddr   - Returns the virtual base address of the address
 *                        environment
 *   up_addrenv_select  - Instantiate an address environment
 *   up_addrenv_destroy - Destroy an address environment.
 *   up_addrenv_assign  - Assign an address environment to a TCB
 *
 * Higher-level interfaces used by the tasking logic.  These interfaces are
 * used by the functions in sched/ and all operate on the TCB which as been
 * assigned an address environment by up_addrenv_assign().
 *
 *   up_addrenv_share   - Clone the address environment assigned to one TCB
 *                        to another.  This operation is done when a pthread
 *                        is created that share's the same address
 *                        environment.
 *   up_addrenv_release - Release the TCBs reference to an address
 *                        environment when a task/thread exists.
 *
 ****************************************************************************/
/****************************************************************************
 * Name: up_addrenv_create
 *
 * Description:
 *   This function is called from the binary loader logic when a new
 *   task is created in order to instantiate an address environment for the
 *   task.  up_addrenv_create is essentially the allocator of the physical
 *   memory for the new task.
 *
 * Input Parameters:
 *   envsize - The size (in bytes) of the address environment needed by the
 *     task.
 *   addrenv - The location to return the representation of the task address
 *     environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_create(size_t envsize, FAR task_addrenv_t *addrenv)
{
  FAR struct z180_cbr_s *cbr;
  irqstate_t flags;
  uintptr_t alloc;
  unsigned int npages;
  int ret;

  /* Convert the size from bytes to numbers of pages */

  npages = PHYS_ALIGNUP(envsize);
  if (npages < 1)
    {
      /* No address environment... but I suppose that is not an error */

      sdbg("ERROR: npages is zero\n");
      return OK;
    }
  
  /* Allocate a structure in the common .bss to hold information about the
   * task's address environment.  NOTE that this is not a part of the TCB,
   * but rather a break-away structure that can be shared by the task as
   * well as other threads.  That is necessary because the life of the
   * address of environment might be longer than the life of the task.
   */

  flags = irqsave();
  cbr = z180_mmu_alloccbr();
  if (!cbr)
    {
      sdbg("ERROR: No free CBR structures\n");
      ret = -ENOMEM;
      goto errout_with_irq;
    }

  /* Now allocate the physical memory to back up the address environment */

#ifdef CONFIG_GRAN_SINGLE
  alloc = (uintptr_t)gran_alloc(npages);
#else
  alloc = (uintptr_t)gran_alloc(g_physhandle, npages);
#endif
  if (!alloc)
    {
      sdbg("ERROR: Failed to allocate %d pages\n", npages);
      ret = -ENOMEM;
      goto errout_with_cbr;
    }

  /* Save the information in the CBR structure.  Note that alloc is in
   * 4KB pages, already in the right form for the CBR.
   */

  DEBUGASSERT(alloc <= 0xff);

  cbr->cbr     = (uint8_t)alloc;
  cbr->pages   = (uint8_t)npages;
  *addrenv     = (task_addrenv_t)cbr;

  irqrestore(flags);
  return OK;

errout_with_cbr:
  z180_mmu_freecbr(cbr);
  
errout_with_irq:
  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: up_addrenv_vaddr
 *
 * Description:
 *   Return the virtual address associated with the newly create address
 *   environment.  This function is used by the binary loaders in order
 *   get an address that can be used to initialize the new task..
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *      returned by up_addrenv_create.
 *   vaddr - The location to return the virtual address.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_vaddr(FAR task_addrenv_t addrenv, FAR void **vaddr)
{
  return CONFIG_Z180_COMMON1AREA_VIRTBASE;
}

/****************************************************************************
 * Name: up_addrenv_select
 *
 * Description:
 *   After an address environment has been established for a task (via
 *   up_addrenv_create()), this function may be called to to instantiate
 *   that address environment in the virtual address space.  This might be
 *   necessary, for example, to load the code for the task from a file or
 *   to access address environment private data.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   oldenv
 *     The address environment that was in place before up_addrenv_select().
 *     This may be used with up_addrenv_restore() to restore the original
 *     address environment that was in place before up_addrenv_select() was
 *     called.  Note that this may be a task agnostic, hardware
 *     representation that is different from task_addrenv_t.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_select(task_addrenv_t addrenv, hw_addrenv_t *oldenv)
{
  FAR struct z180_cbr_s *cbr = (FAR struct z180_cbr_s *)addrenv;
  irqstate_t flags;

  DEBUGASSERT(cbr && oldenv);

  /* Return the current CBR value from the CBR register */

  flags = irqsave();
  *oldenv = (hw_addrenv_t)inp(Z180_MMU_CBR);

  /* Write the new CBR value into CBR register */

  outp(Z180_MMU_CBR, cbr->cbr);
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_restore
 *
 * Description:
 *   After an address environment has been temporarilty instantiated by
 *   up_addrenv_select, this function may be called to to restore the
 *   original address environment.
 *
 * Input Parameters:
 *   oldenv - The hardware representation of the address environment
 *     previously returned by up_addrenv_select.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_restore(hw_addrenv_t oldenv)
{
  outp(Z180_MMU_CBR, (uint8_t)oldenv);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_destroy
 *
 * Description:
 *   Called from the binary loader loader during error handling to destroy
 *   the address environment previously created by up_addrenv_create().
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_destroy(task_addrenv_t addrenv)
{
  FAR struct z180_cbr_s *cbr = (FAR struct z180_cbr_s *)addrenv;

  DEBUGASSERT(cbr);

  /* Free the physical address space backing up the mapping */

#ifdef CONFIG_GRAN_SINGLE
  gran_free((FAR void *)cbr->cbr, cbr->pages);
#else
  gran_free(g_physhandle, (FAR void *)cbr->cbr, cbr->pages);
#endif

  /* And make the CBR structure available for re-use */

  z180_mmu_freecbr(cbr);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_assign
 *
 * Description:
 *   Assign an address environment to a TCB.
 *
 * Input Parameters:
 *   addrenv - The representation of the task address environment previously
 *     returned by up_addrenv_create.
 *   tcb - The TCB of the task to receive the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_assign(task_addrenv_t addrenv, FAR _TCB *tcb)
{
  FAR struct z180_cbr_s *cbr = (FAR struct z180_cbr_s *)addrenv;
  int ret;

  /* Make sure that there is no address environment in place on this TCB */

  DEBUGASSERT(cbr && tcb->xcp.cbr == NULL);

  /* Save the CBR strucure in the TCB.  This is an atomic operation so no
   * special precautions should be needed.
   */

  tcb->xcp.cbr = cbr;
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_share
 *
 * Description:
 *   This function is called from the core scheduler logic when a thread
 *   is created that needs to share the address ennvironment of its parent
 *   task.  In this case, the parent's address environment needs to be
 *   "cloned" for the child.
 *
 * Input Parameters:
 *   ptcb - The TCB of the parent task that has the address environment.
 *   ctcb - The TCB of the child thread needing the address environment.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_share(FAR const _TCB *ptcb, FAR _TCB *ctcb)
{
  irqstate_t flags;

  /* Make sure that the child has no address environment.  It is okay if
   * if the parent does not have one.
   */

  DEBUGASSERT(ctcb->xcp.cbr == NULL);

  flags = irqsave();
  if (ptcb->xcp.cbr)
    {
      /* Clone the CBR by incrementing the reference counting and saving a
       * copy in the child thread's TCB.
       */

      ptcb->xcp.cbr->crefs++;
      ctcb->xcp.cbr = ptcb->xcp.cbr;
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: up_addrenv_release
 *
 * Description:
 *   This function is called when a task or thread exits in order to release
 *   its reference to an address environment.  When there are no further
 *   references to an address environment, that address environment should
 *   be destroyed.
 *
 * Input Parameters:
 *   tcb - The TCB of the task or thread whose the address environment will
 *     be released.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_addrenv_release(FAR _TCB *tcb)
{
  FAR struct z180_cbr_s *cbr;
  irqstate_t flags;

  /* Check if the task has an address environment. */

  flags = irqsave();
  cbr   = tcb->xcp.cbr;
  if (cbr)
    {
      /* Nullify the reference to the CBR structure and decrement the number
       * of references on the CBR.
       */

      tcb->xcp.cbr = NULL;

      /* If the reference count would decrement to zero, then free the CBR
       * structure.
       */

      if (cbr->crefs <= 1)
        {
          up_addrenv_destroy(cbr);
        }
      else
        {
          /* Otherwise, just decrement the reference count */

          cbr->crefs--;
        }
    }

  irqrestore(flags);
  return OK;
}
