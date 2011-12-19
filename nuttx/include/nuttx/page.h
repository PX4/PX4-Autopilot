/****************************************************************************
 * include/nuttx/page.h
 * This file defines interfaces used to support NuttX On-Demand Paging.
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __NUTTX_PAGE_H
#define __NUTTX_PAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <nuttx/sched.h>
#endif

#ifdef CONFIG_PAGING

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* CONFIG_PAGING_PAGESIZE - The size of one managed page.  This must be a
 *   value supported by the processor's memory management unit.  The
 *   following may need to be extended to support additional page sizes at
 *   some point.
 */

#if CONFIG_PAGING_PAGESIZE == 1024
#  define PAGESIZE                 1024
#  define PAGESHIFT                10
#  define PAGEMASK                 0x000003ff
#elif CONFIG_PAGING_PAGESIZE == 4096
#  define PAGESIZE                 4096
#  define PAGESHIFT                12
#  define PAGEMASK                 0x00000fff
#else
#  error "Need extended definitions for CONFIG_PAGING_PAGESIZE"
#endif

/* Alignment macros */

#define PG_ALIGNDOWN(addr)         ((addr) & ~PAGEMASK)
#define PG_ALIGNUP(addr)           (((addr) + PAGEMASK) & ~PAGEMASK)

/* CONFIG_PAGING_NLOCKED - This is the number of locked pages in the memory
 * map.  The size of locked address region will then be given by
 * PG_LOCKED_SIZE.  These values applies to both physical and virtual memory
 * regions.
 */

#define PG_LOCKED_SIZE             (CONFIG_PAGING_NLOCKED << PAGESHIFT)

/* CONFIG_PAGING_LOCKED_P/VBASE - May be defined to determine the base
 * address of the locked page regions (lowest in memory).  If neither
 * are defined, then this logic will be set the bases to CONFIG_DRAM_START
 * and CONFIG_DRAM_VSTART (i.e., it assumes that the base address of the
 * locked region is at the beginning of RAM).
 *
 * NOTE:  In some architectures, it may be necessary to take some memory
 * from the beginning of this region for vectors or for a page table.
 * In such cases, either (1) CONFIG_PAGING_LOCKED_P/VBASE might take that
 * into consideration to prevent overlapping the locked memory region
 * and the system data at the beginning of SRAM, (2) you extend CONFIG_PAGING_NLOCKED
 * include these pages at the beginning of memory and map let them be
 * mapped read-only.
 */

#if defined(CONFIG_PAGING_LOCKED_PBASE) && defined(CONFIG_PAGING_LOCKED_VBASE)
#  define PG_LOCKED_PBASE          CONFIG_PAGING_LOCKED_PBASE
#  define PG_LOCKED_VBASE          CONFIG_PAGING_LOCKED_VBASE
#else
#  define PG_LOCKED_PBASE          CONFIG_DRAM_START
#  define PG_LOCKED_VBASE          CONFIG_DRAM_VSTART
#endif

#define PG_LOCKED_PEND             (PG_LOCKED_PBASE + PG_LOCKED_SIZE)
#define PG_LOCKED_VEND             (PG_LOCKED_VBASE + PG_LOCKED_SIZE)

#if (PG_LOCKED_PBASE & PAGEMASK) != 0 || (PG_LOCKED_VBASE & PAGEMASK) != 0
#  error "Base address of the locked region is not page aligned"
#endif

/* CONFIG_PAGING_NPPAGED - This is the number of physical pages available to
 *   support the paged text region.
 * CONFIG_PAGING_NVPAGED - This actual size of the paged text region (in
 *   pages).  This is also the number of virtual pages required to support
 *   the entire paged region. The on-demand paging  feature is intended to
 *   support only the case where the virtual paged text area is much larger
 *   the available physical pages.  Otherwise, why would you enable on-demand
 *   paging?
 */

#if CONFIG_PAGING_NPPAGED >= CONFIG_PAGING_NVPAGED
#  error "CONFIG_PAGING_NPPAGED must be less than CONFIG_PAGING_NVPAGED"
#endif

/* The size of physical and virutal paged address regions will then be: */

#define PG_PAGED_PSIZE             (CONFIG_PAGING_NPPAGED << PAGESHIFT)
#define PG_PAGED_VSIZE             (CONFIG_PAGING_NVPAGED << PAGESHIFT)

/* This positions the paging Read-Only text region.  If the configuration
 * did not override the default, the paged region will immediately follow
 * the locked region.
 */

#if defined(CONFIG_PAGING_LOCKED_PBASE) && defined(CONFIG_PAGING_LOCKED_VBASE)
#  define PG_PAGED_PBASE           CONFIG_PAGING_LOCKED_PBASE
#  define PG_PAGED_VBASE           CONFIG_PAGING_LOCKED_VBASE
#else
#  define PG_PAGED_PBASE           PG_LOCKED_PEND
#  define PG_PAGED_VBASE           PG_LOCKED_VEND
#endif

#define PG_PAGED_PEND              (PG_PAGED_PBASE + PG_PAGED_PSIZE)
#define PG_PAGED_VEND              (PG_PAGED_VBASE + PG_PAGED_VSIZE)

/* Size and description of the overall text section.  The number of
 * pages in the text section is the sum of the number of pages in
 * both the locked and paged regions.  The base of the text section
 * is the base of the locked region.
 */

#define PG_TEXT_NPPAGES            (CONFIG_PAGING_NLOCKED + CONFIG_PAGING_NPPAGED)
#define PG_TEXT_NVPAGES            (CONFIG_PAGING_NLOCKED + CONFIG_PAGING_NVPAGED)
#define PG_TEXT_PSIZE              (PG_TEXT_NPPAGES << PAGESHIFT)
#define PG_TEXT_VSIZE              (PG_TEXT_NVPAGES << PAGESHIFT)
#define PG_TEXT_PBASE              PG_LOCKED_PBASE
#define PG_TEXT_VBASE              PG_LOCKED_VBASE

/* CONFIG_PAGING_NDATA - This is the number of data pages in the memory
 * map.  The data region will extend to the end of RAM unless overridden
 * by a setting in the configuration file.
 *
 * NOTE:  In some architectures, it may be necessary to take some memory
 * from the end of RAM for page tables or other system usage.  The
 * configuration settings and linker directives must be cognizant of that:
 * CONFIG_PAGING_NDATA should be defined to prevent the data region from
 * extending all the way to the end of memory. 
 */

#define PG_RAM_PAGES               (CONFIG_DRAM_SIZE >> PAGESHIFT)

#ifdef CONFIG_PAGING_NDATA
#  define PG_DATA_NPAGES           CONFIG_PAGING_NDATA
#elif PG_RAM_PAGES > PG_TEXT_NPPAGES
#  define PG_DATA_NPAGES           (PG_RAM_PAGES - PG_TEXT_NPAGES)
#else
#  error "Not enough memory for this page layout"
#endif

#define PG_DATA_SIZE               (PG_DATA_NPAGES << PAGESHIFT)

/* This positions the Read/Write data region.  If the configuration
 * did not override the default, the paged region will immediately follow
 * the paged region and will extend to the end of memory.
 */

#if defined(CONFIG_PAGING_DATA_PBASE) && defined(CONFIG_PAGING_DATA_VBASE)
#  define PG_DATA_PBASE            CONFIG_PAGING_DATA_PBASE
#  define PG_DATA_VBASE            CONFIG_PAGING_DATA_VBASE
#else
#  define PG_DATA_PBASE            PG_LOCKED_PEND
#  define PG_DATA_VBASE            PG_LOCKED_VEND
#endif

/* CONFIG_PAGING_DEFPRIO - The default, minimum priority of the page fill
 *   worker thread.  The priority of the page fill work thread will be boosted
 *   boosted dynmically so that it matches the priority of the task on behalf
 *   of which it peforms the fill.  This defines the minimum priority that
 *   will be used. Default: 50.
 * CONFIG_PAGING_STACKSIZE - Defines the size of the allocated stack
 *   for the page fill worker thread. Default: 1024.
 * CONFIG_PAGING_BLOCKINGFILL - The architecture specific up_fillpage()
 *   function may be blocking or non-blocking.  If defined, this setting
 *   indicates that the up_fillpage() implementation will block until the
 *   transfer is completed. Default:  Undefined (non-blocking).
 * CONFIG_PAGING_WORKPERIOD - The page fill worker thread will wake periodically
 *   even if there is no mapping to do.  This selection controls that wake-up
 *   period (in microseconds).  This wake-up a failsafe that will handle any 
 *   cases where a single is lost (that would really be a bug and shouldn't
 *   happen!) and also supports timeouts for case of non-blocking, asynchronous
 *   fills (see CONFIG_PAGING_TIMEOUT_TICKS).
 * CONFIG_PAGING_TIMEOUT_TICKS - If defined, the implementation will monitor
 *   the (asynchronous) page fill logic.  If the fill takes longer than this
 *   number if microseconds, then a fatal error will be declared.
 *   Default: No timeouts monitored.
 */

/****************************************************************************
 * Public Data
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
 * Public Functions -- Provided by common paging logic to architecture-
 *                     specific logic.
 ****************************************************************************/

/****************************************************************************
 * Name: pg_miss
 *
 * Description:
 *   This function is called from architecture-specific memory segmentation
 *   fault handling logic.  This function will perform the following
 *   operations:
 *
 *   1) Sanity checking.
 *      - ASSERT if the currently executing task is the page fill worker
 *        thread.  The page fill worker thread is how the the page fault
 *        is resolved and all logic associated with the page fill worker
 *        must be "locked" and always present in memory.
 *   2) Block the currently executing task.
 *      - Call up_block_task() to block the task at the head of the ready-
 *        to-run list.  This should cause an interrupt level context switch
 *        to the next highest priority task.
 *      - The blocked task will be marked with state TSTATE_WAIT_PAGEFILL
 *        and will be retained in the g_waitingforfill prioritized task
 *        list.
 *   3) Boost the page fill worker thread priority.
 *      - Check the priority of the task at the head of the g_waitingforfill
 *        list.  If the priority of that task is higher than the current
 *        priority of the page fill worker thread, then boost the priority
 *        of the page fill worker thread to that priority.
 *   4) Signal the page fill worker thread.
 *      - Is there a page fill pending?  If not then signal the worker
 *        thread to start working on the queued page fill requests.
 *
 * Input Parameters:
 *   None - The head of the ready-to-run list is assumed to be task that
 *   caused the exception.
 *
 * Returned Value:
 *   None - Either this function function succeeds or an assertion occurs.
 *
 * Assumptions:
 *   - It is assumed that this function is called from the level of an
 *     exception handler and that all interrupts are disabled.
 *   - It is assumed that currently executing task (the one at the head of
 *     the ready-to-run list) is the one that cause the fault.  This will
 *     always be true unless the page fault occurred in an interrupt handler.
 *     Interrupt handling logic must always be present and "locked" into
 *     memory.
 *   - The chip-specific page fault exception handler has already verified
 *     that the exception did not occur from interrupt/exception handling
 *     logic.
 *   - As mentioned above, the task causing the page fault must not be the
 *     page fill worker thread because that is the only way to complete the
 *     page fill.
 *
 * NOTES:
 *   1. One way to accomplish this would be a two pass link phase:
 *      - In the first phase, create a partially linked objected containing
 *        all interrupt/exception handling logic, the page fill worker thread
 *        plus all parts of the IDLE thread (which must always be available
 *        for execution).
 *      - All of the .text and .rodata sections of this partial link should
 *        be collected into a single section.
 *      - The second link would link the partially linked object along with
 *        the remaining object to produce the final binary.  The linker
 *        script should position the "special" section so that it lies
 *        in a reserved, "non-swappable" region.
 *
 ****************************************************************************/

EXTERN void pg_miss(void);

/****************************************************************************
 * Public Functions -- Provided by architecture-specific logic to common
 *                     paging logic.
 ****************************************************************************/
 
/****************************************************************************
 * Name: up_checkmapping()
 *
 * Description:
 *  The function up_checkmapping() returns an indication if the page fill
 *  still needs to performed or not. In certain conditions, the page fault
 *  may occur on several threads and be queued multiple times. This function
 *  will prevent the same page from be filled multiple times.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that we believe
 *         needs to have a page fill.  Architecture-specific logic can
 *         retrieve page fault information from the architecture-specific
 *         context information in this TCB and can consult processor resources
 *         (page tables or TLBs or ???) to determine if the fill still needs
 *         to be performed or not.
 *
 * Returned Value:
 *   This function will return true if the mapping is in place and false
 *   if the mapping is still needed.  Errors encountered should be
 *   interpreted as fatal.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

EXTERN bool up_checkmapping(FAR _TCB *tcb);

/****************************************************************************
 * Name: up_allocpage()
 *
 * Description:
 *  This architecture-specific function will set aside page in memory and map
 *  the page to its correct virtual address.  Architecture-specific context
 *  information saved within the TCB will provide the function with the
 *  information needed to identify the virtual miss address.
 *
 *  This function will return the allocated physical page address in vpage.
 *  The size of the underlying physical page is determined by the
 *  configuration setting CONFIG_PAGING_PAGESIZE.
 *
 *  NOTE 1: This function must always return a page allocation. If all
 *  available pages are in-use (the typical case), then this function will
 *  select a page in-use, un-map it, and make it available.
 *
 *  NOTE 2: If an in-use page is un-mapped, it may be necessary to flush the
 *  instruction cache in some architectures.
 *
 *  NOTE 3: Allocating and filling a page is a two step process.  up_allocpage()
 *  allocates the page, and up_fillpage() fills it with data from some non-
 *  volatile storage device.  This distinction is made because up_allocpage()
 *  can probably be implemented in board-independent logic whereas up_fillpage()
 *  probably must be implemented as board-specific logic.
 *
 *  NOTE 4: The initial mapping of vpage should be read-able and write-
 *  able (but not cached).  No special actions will be required of
 *  up_fillpage() in order to write into this allocated page.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that needs to
 *         have a page fill.  Architecture-specific logic can retrieve page
 *         fault information from the architecture-specific context
 *         information in this TCB to perform the mapping.
 *
 * Returned Value:
 *   This function will return zero (OK) if the allocation was successful.
 *   A negated errno value may be returned if an error occurs.  All errors,
 *   however, are fatal.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

EXTERN int up_allocpage(FAR _TCB *tcb, FAR void **vpage);

/****************************************************************************
 * Name: up_fillpage()
 *
 * Description:
 *  After a page is allocated and mapped by up_allocpage(), the actual
 *  filling of the page with data from the non-volatile, must be performed
 *  by a separate call to the architecture-specific function, up_fillpage().
 *  This function is non-blocking, it will start an asynchronous page fill.
 *  The common paging logic will provide a callback function, pg_callback,
 *  that will be called when the page fill is finished (or an error occurs).
 *  This callback is assumed to occur from an interrupt level when the
 *  device driver completes the fill operation.
 *
 *  NOTE 1: Allocating and filling a page is a two step process.  up_allocpage()
 *  allocates the page, and up_fillpage() fills it with data from some non-
 *  volatile storage device.  This distinction is made because up_allocpage()
 *  can probably be implemented in board-independent logic whereas up_fillpage()
 *  probably must be implemented as board-specific logic.
 *
 *  NOTE 2: The initial mapping of vpage will be read-able, write-able,
 *  but non-cacheable.  No special actions will be required of
 *  up_fillpage() in order to write into this allocated page.  If the
 *  virtual address maps to a text region, however, this function should
 *  remap the region so that is is read/execute only.  It should be made
 *  cache-able in any case.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that needs to
 *         have a page fill.  Architecture-specific logic can retrieve page
 *         fault information from the architecture-specific context
 *         information in this TCB to perform the fill.
 *   pg_callbck - The function to be called when the page fill is complete.
 *
 * Returned Value:
 *   This function will return zero (OK) if the page fill was successfully
 *   started (the result of the page fill is passed to the callback function
 *   as the result argument).  A negated errno value may be returned if an
 *   error occurs.  All errors, however, are fatal.
 *
 *   NOTE: -EBUSY has a special meaning. It is used internally to mean that
 *   the callback function has not executed.  Therefore, -EBUSY should
 *   never be provided in the result argument of pg_callback.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but
 *     interrupts siabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this context.
 *   - Upon return, the caller will sleep waiting for the page fill callback
 *     to occur.  The callback function will perform the wakeup.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING_BLOCKINGFILL
EXTERN int up_fillpage(FAR _TCB *tcb, FAR void *vpage);
#else
typedef void (*up_pgcallback_t)(FAR _TCB *tcb, int result);
EXTERN int up_fillpage(FAR _TCB *tcb, FAR void *vpage, up_pgcallback_t pg_callback);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_PAGING */
#endif /* __NUTTX_PAGE_H */
