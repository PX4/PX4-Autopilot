/****************************************************************************
 * sched/os_start.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#include  <sys/types.h>
#include  <stdbool.h>
#include  <debug.h>
#include  <string.h>

#include  <nuttx/arch.h>
#include  <nuttx/compiler.h>
#include  <nuttx/fs/fs.h>
#include  <nuttx/net/net.h>
#include  <nuttx/lib.h>
#include  <nuttx/kmalloc.h>
#include  <nuttx/init.h>

#include  "os_internal.h"
#include  "sig_internal.h"
#include  "wd_internal.h"
#include  "sem_internal.h"
#ifndef CONFIG_DISABLE_MQUEUE
# include "mq_internal.h"
#endif
#ifndef CONFIG_DISABLE_PTHREAD
# include "pthread_internal.h"
#endif
#include  "clock_internal.h"
#include  "timer_internal.h"
#include  "irq_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Task Lists ***************************************************************/
/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists need
 * to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  The head of this
 * list is the currently active task; the tail of this list is always the
 * IDLE task.
 */

volatile dq_queue_t g_readytorun;

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

volatile dq_queue_t g_pendingtasks;

/* This is the list of all tasks that are blocked waiting for a semaphore */

volatile dq_queue_t g_waitingforsemaphore;

/* This is the list of all tasks that are blocked waiting for a signal */

#ifndef CONFIG_DISABLE_SIGNALS
volatile dq_queue_t g_waitingforsignal;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-empty.
 */

#ifndef CONFIG_DISABLE_MQUEUE
volatile dq_queue_t g_waitingformqnotempty;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-full.
 */

#ifndef CONFIG_DISABLE_MQUEUE
volatile dq_queue_t g_waitingformqnotfull;
#endif

/* This is the list of all tasks that are blocking waiting for a page fill */

#ifdef CONFIG_PAGING
volatile dq_queue_t g_waitingforfill;
#endif

/* This the list of all tasks that have been initialized, but not yet
 * activated. NOTE:  This is the only list that is not prioritized.
 */

volatile dq_queue_t g_inactivetasks;

/* This is the list of dayed memory deallocations that need to be handled
 * within the IDLE loop.  These deallocations get queued by sched_free()
 * if the OS attempts to deallocate memory while it is within an interrupt
 * handler.
 */

volatile sq_queue_t g_delayeddeallocations;

/* This is the value of the last process ID assigned to a task */

volatile pid_t g_lastpid;

/* The following hash table is used for two things:
 *
 * 1. This hash table greatly speeds the determination of
 *    a new unique process ID for a task, and
 * 2. Is used to quickly map a process ID into a TCB.
 * It has the side effects of using more memory and limiting
 *
 * the number of tasks to CONFIG_MAX_TASKS.
 */

pidhash_t g_pidhash[CONFIG_MAX_TASKS];

/* This is a table of task lists.  This table is indexed by
 * the task state enumeration type (tstate_t) and provides
 * a pointer to the associated static task list (if there
 * is one) as well as a boolean indication as to if the list
 * is an ordered list or not.
 */

const tasklist_t g_tasklisttable[NUM_TASK_STATES] =
{
  { NULL,                    false },  /* TSTATE_TASK_INVALID */
  { &g_pendingtasks,         true  },  /* TSTATE_TASK_PENDING */
  { &g_readytorun,           true  },  /* TSTATE_TASK_READYTORUN */
  { &g_readytorun,           true  },  /* TSTATE_TASK_RUNNING */
  { &g_inactivetasks,        false },  /* TSTATE_TASK_INACTIVE */
  { &g_waitingforsemaphore,  true  }   /* TSTATE_WAIT_SEM */
#ifndef CONFIG_DISABLE_SIGNALS
  ,
  { &g_waitingforsignal,     false }  /* TSTATE_WAIT_SIG */
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  ,
  { &g_waitingformqnotempty, true  },  /* TSTATE_WAIT_MQNOTEMPTY */
  { &g_waitingformqnotfull,  true  }   /* TSTATE_WAIT_MQNOTFULL */
#endif
#ifdef CONFIG_PAGING
  ,
  { &g_waitingforfill,       true  }   /* TSTATE_WAIT_PAGEFILL */
#endif
};

/****************************************************************************
 * Private Variables
 ****************************************************************************/
/* This is the task control block for this thread of execution. This thread
 * of execution is the IDLE task.  NOTE:  the system boots into the IDLE
 * task.  The IDLE task spawns the user initialization task (user_start) and
 * that user init task is responsible for bringing up the rest of the system
 */

static FAR _TCB g_idletcb;

/* This is the name of the idle task */

static FAR const char g_idlename[] = "Idle Task";

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_start
 *
 * Description:
 *   This function is called to initialize the operating system and to spawn
 *   the user initization thread of execution
 *
 ****************************************************************************/

void os_start(void)
{
  int i;

  slldbg("Entry\n");

  /* Initialize all task lists */

  dq_init(&g_readytorun);
  dq_init(&g_pendingtasks);
  dq_init(&g_waitingforsemaphore);
#ifndef CONFIG_DISABLE_SIGNALS
  dq_init(&g_waitingforsignal);
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  dq_init(&g_waitingformqnotfull);
  dq_init(&g_waitingformqnotempty);
#endif
#ifdef CONFIG_PAGING
  dq_init(&g_waitingforfill);
#endif
  dq_init(&g_inactivetasks);
  sq_init(&g_delayeddeallocations);

  /* Initialize the logic that determine unique process IDs. */

  g_lastpid = 0;
  for (i = 0; i < CONFIG_MAX_TASKS; i++)
    {
      g_pidhash[i].tcb = NULL;
      g_pidhash[i].pid = INVALID_PROCESS_ID;
    }

  /* Assign the process ID of ZERO to the idle task */

  g_pidhash[ PIDHASH(0)].tcb = &g_idletcb;
  g_pidhash[ PIDHASH(0)].pid = 0;

  /* Initialize a TCB for this thread of execution.  NOTE:  The default
   * value for most components of the g_idletcb are zero.  The entire
   * structure is set to zero.  Then only the (potentially) non-zero
   * elements are initialized. NOTE:  The idle task is the only task in
   * that has pid == 0 and sched_priority == 0.
   */

  bzero((void*)&g_idletcb, sizeof(_TCB));
  g_idletcb.task_state = TSTATE_TASK_RUNNING;
  g_idletcb.entry.main = (main_t)os_start;

#if CONFIG_TASK_NAME_SIZE > 0
  strncpy(g_idletcb.name, g_idlename, CONFIG_TASK_NAME_SIZE-1);
  g_idletcb.argv[0] = g_idletcb.name;
#else
  g_idletcb.argv[0] = (char*)g_idlename;
#endif /* CONFIG_TASK_NAME_SIZE */

  /* Then add the idle task's TCB to the head of the ready to run list */

  dq_addfirst((FAR dq_entry_t*)&g_idletcb, (FAR dq_queue_t*)&g_readytorun);

  /* Initialize the processor-specific portion of the TCB */

  g_idletcb.flags = (TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_NOCLDWAIT);
  up_initial_state(&g_idletcb);

  /* Initialize the semaphore facility(if in link).  This has to be done
   * very early because many subsystems depend upon fully functional
   * semaphores.
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (sem_initialize != NULL)
#endif
    {
      sem_initialize();
    }

  /* Initialize the memory manager */

#ifndef CONFIG_HEAP_BASE
  {
    FAR void *heap_start;
    size_t heap_size;
    up_allocate_heap(&heap_start, &heap_size);
    kmm_initialize(heap_start, heap_size);
  }
#else
  kmm_initialize((void*)CONFIG_HEAP_BASE, CONFIG_HEAP_SIZE);
#endif

  /* Initialize tasking data structures */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (task_initialize != NULL)
#endif
    {
      task_initialize();
    }
#endif

  /* Initialize the interrupt handling subsystem (if included) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (irq_initialize != NULL)
#endif
    {
      irq_initialize();
    }

  /* Initialize the watchdog facility (if included in the link) */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (wd_initialize != NULL)
#endif
    {
      wd_initialize();
    }

  /* Initialize the POSIX timer facility (if included in the link) */

#ifndef CONFIG_DISABLE_CLOCK
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (clock_initialize != NULL)
#endif
    {
      clock_initialize();
    }
#endif

#ifndef CONFIG_DISABLE_POSIX_TIMERS
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (timer_initialize != NULL)
#endif
    {
      timer_initialize();
    }
#endif

  /* Initialize the signal facility (if in link) */

#ifndef CONFIG_DISABLE_SIGNALS
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (sig_initialize != NULL)
#endif
    {
      sig_initialize();
    }
#endif

  /* Initialize the named message queue facility (if in link) */

#ifndef CONFIG_DISABLE_MQUEUE
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (mq_initialize != NULL)
#endif
    {
      mq_initialize();
    }
#endif

  /* Initialize the thread-specific data facility (if in link) */

#ifndef CONFIG_DISABLE_PTHREAD
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (pthread_initialize != NULL)
#endif
    {
      pthread_initialize();
    }
#endif

  /* Initialize the file system (needed to support device drivers) */

#if CONFIG_NFILE_DESCRIPTORS > 0
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (fs_initialize != NULL)
#endif
    {
      fs_initialize();
    }
#endif

  /* Initialize the network system */

#ifdef CONFIG_NET
#if 0
  if (net_initialize != NULL)
#endif
    {
      net_initialize();
    }
#endif

  /* The processor specific details of running the operating system
   * will be handled here.  Such things as setting up interrupt
   * service routines and starting the clock are some of the things
   * that are different for each  processor and hardware platform.
   */

  up_initialize();

  /* Initialize the C libraries (if included in the link).  This
   * is done last because the libraries may depend on the above.
   */

#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (lib_initialize != NULL)
#endif
    {
      lib_initialize();
    }

  /* Create stdout, stderr, stdin on the IDLE task.  These will be
   * inherited by all of the threads created by the IDLE task.
   */

  (void)sched_setupidlefiles(&g_idletcb);

  /* Create initial tasks and bring-up the system */

  (void)os_bringup();

  /* When control is return to this point, the system is idle. */

  sdbg("Beginning Idle Loop\n");
  for (;;)
    {
      /* Perform garbage collection (if it is not being done by the worker
       * thread).  This cleans-up memory de-allocations that were queued
       * because they could not be freed in that execution context (for
       * example, if the memory was freed from an interrupt handler).
       */

#ifndef CONFIG_SCHED_WORKQUEUE
      /* We must have exclusive access to the memory manager to do this
       * BUT the idle task cannot wait on a semaphore.  So we only do
       * the cleanup now if we can get the semaphore -- this should be
       * possible because if the IDLE thread is running, no other task is!
       */

      if (kmm_trysemaphore() == 0)
        {
          sched_garbagecollection();
          kmm_givesemaphore();
        }
#endif

      /* Perform any processor-specific idle state operations */

      up_idle();
    }
}
