/********************************************************************************
 * include/nuttx/sched.h
 *
 *   Copyright (C) 2007-2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

#ifndef __INCLUDE_NUTTX_SCHED_H
#define __INCLUDE_NUTTX_SCHED_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <queue.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>

#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/
/* Configuration ****************************************************************/
/* Task groups currently only supported for retention of child status */

#undef HAVE_TASK_GROUP
#undef HAVE_GROUP_MEMBERS

/* We need a group an group members if we are supportint the parent/child
 * relationship.
 */

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
#  define HAVE_TASK_GROUP     1
#  define HAVE_GROUP_MEMBERS  1

/* We need a group (but not members) if any other resources are shared within
 * a task group.
 */

#else
#  if !defined(CONFIG_DISABLE_ENVIRON)
#    define HAVE_TASK_GROUP   1
#  elif CONFIG_NFILE_DESCRIPTORS > 0
#    define HAVE_TASK_GROUP   1
#  elif CONFIG_NFILE_STREAMS > 0
#    define HAVE_TASK_GROUP   1
#  endif
#endif

/* In any event, we don't need group members if support for pthreads is disabled */

#ifdef CONFIG_DISABLE_PTHREAD
#  undef HAVE_GROUP_MEMBERS
#endif

/* Task Management Definitions **************************************************/

/* This is the maximum number of times that a lock can be set */

#define MAX_LOCK_COUNT             127

/* Values for the _TCB flags bits */

#define TCB_FLAG_TTYPE_SHIFT       (0)      /* Bits 0-1: thread type */
#define TCB_FLAG_TTYPE_MASK        (3 << TCB_FLAG_TTYPE_SHIFT)
#  define TCB_FLAG_TTYPE_TASK      (0 << TCB_FLAG_TTYPE_SHIFT) /* Normal user task */
#  define TCB_FLAG_TTYPE_PTHREAD   (1 << TCB_FLAG_TTYPE_SHIFT) /* User pthread */
#  define TCB_FLAG_TTYPE_KERNEL    (2 << TCB_FLAG_TTYPE_SHIFT) /* Kernel thread */
#define TCB_FLAG_NONCANCELABLE     (1 << 2) /* Bit 2: Pthread is non-cancelable */
#define TCB_FLAG_CANCEL_PENDING    (1 << 3) /* Bit 3: Pthread cancel is pending */
#define TCB_FLAG_ROUND_ROBIN       (1 << 4) /* Bit 4: Round robin sched enabled */
#define TCB_FLAG_EXIT_PROCESSING   (1 << 5) /* Bit 5: Exitting */

/* Values for struct task_group tg_flags */

#define GROUP_FLAG_NOCLDWAIT       (1 << 0) /* Bit 0: Do not retain child exit status */

/* Values for struct child_status_s ch_flags */

#define CHILD_FLAG_TTYPE_SHIFT     (0)      /* Bits 0-1: child thread type */
#define CHILD_FLAG_TTYPE_MASK      (3 << CHILD_FLAG_TTYPE_SHIFT)
#  define CHILD_FLAG_TTYPE_TASK    (0 << CHILD_FLAG_TTYPE_SHIFT) /* Normal user task */
#  define CHILD_FLAG_TTYPE_PTHREAD (1 << CHILD_FLAG_TTYPE_SHIFT) /* User pthread */
#  define CHILD_FLAG_TTYPE_KERNEL  (2 << CHILD_FLAG_TTYPE_SHIFT) /* Kernel thread */
#define CHILD_FLAG_EXITED          (1 << 0) /* Bit 2: The child thread has exit'ed */

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

#ifndef __ASSEMBLY__

/* General Task Management Types ************************************************/

/* This is the type of the task_state field of the TCB. NOTE: the order and
 * content of this enumeration is critical since there are some OS tables indexed
 * by these values.  The range of values is assumed to fit into a uint8_t in _TCB.
 */

enum tstate_e
{
  TSTATE_TASK_INVALID    = 0, /* INVALID      - The TCB is uninitialized */
  TSTATE_TASK_PENDING,        /* READY_TO_RUN - Pending preemption unlock */
  TSTATE_TASK_READYTORUN,     /* READY-TO-RUN - But not running */
  TSTATE_TASK_RUNNING,        /* READY_TO_RUN - And running */

  TSTATE_TASK_INACTIVE,       /* BLOCKED      - Initialized but not yet activated */
  TSTATE_WAIT_SEM,            /* BLOCKED      - Waiting for a semaphore */
#ifndef CONFIG_DISABLE_SIGNALS
  TSTATE_WAIT_SIG,            /* BLOCKED      - Waiting for a signal */
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  TSTATE_WAIT_MQNOTEMPTY,     /* BLOCKED      - Waiting for a MQ to become not empty. */
  TSTATE_WAIT_MQNOTFULL,      /* BLOCKED      - Waiting for a MQ to become not full. */
#endif
#ifdef CONFIG_PAGING
  TSTATE_WAIT_PAGEFILL,       /* BLOCKED      - Waiting for page fill */
#endif
  NUM_TASK_STATES             /* Must be last */
};
typedef enum tstate_e tstate_t;

/* The following definitions are determined by tstate_t */

#define FIRST_READY_TO_RUN_STATE TSTATE_TASK_READYTORUN
#define LAST_READY_TO_RUN_STATE  TSTATE_TASK_RUNNING
#define FIRST_BLOCKED_STATE      TSTATE_TASK_INACTIVE
#define LAST_BLOCKED_STATE       (NUM_TASK_STATES-1)

/* The following is the form of a thread start-up function */

typedef void (*start_t)(void);

/* This is the entry point into the main thread of the task or into a created
 * pthread within the task.
 */

union entry_u
{
  pthread_startroutine_t pthread;
  main_t main;
};
typedef union entry_u entry_t;

/* These is the types of the functions that are executed with exit() is called
 * (if registered via atexit() on on_exit()).
 */

#ifdef CONFIG_SCHED_ATEXIT
typedef CODE void (*atexitfunc_t)(void);
#endif

#ifdef CONFIG_SCHED_ONEXIT
typedef CODE void (*onexitfunc_t)(int exitcode, FAR void *arg);
#endif

/* POSIX Message queue */

typedef struct msgq_s msgq_t;

/* struct child_status_s *********************************************************/
/* This structure is used to maintin information about child tasks.
 * pthreads work differently, they have join information.  This is 
 * only for child tasks.
 */

#ifdef CONFIG_SCHED_CHILD_STATUS
struct child_status_s
{
  FAR struct child_status_s *flink;

  uint8_t ch_flags;           /* Child status:  See CHILD_FLAG_* definitions */
  pid_t   ch_pid;             /* Child task ID */
  int     ch_status;          /* Child exit status */
};
#endif

/* struct dspace_s ***************************************************************/
/* This structure describes a reference counted D-Space region.  This must be a
 * separately allocated "break-away" structure that can be owned by a task and
 * any pthreads created by the task.
 */

#ifdef CONFIG_PIC
struct dspace_s
{
  /* The life of the structure allocation is determined by this reference
   * count.  This count is number of threads that shared the the same D-Space.
   * This includes the parent task as well as any pthreads created by the
   * parent task or any of its child threads.
   */

  uint16_t crefs;

  /* This is the allocated D-Space memory region.  This may be a physical
   * address allocated with kmalloc(), or it may be virtual address associated
   * with an address environment (if CONFIG_ADDRENV=y).
   */

  FAR uint8_t *region;
};
#endif

/* struct task_group_s ***********************************************************/
/* All threads created by pthread_create belong in the same task group (along with
 * the thread of the original task).  struct task_group_s is a shared, "breakaway"
 * structure referenced by each TCB.
 *
 * This structure should contain *all* resources shared by tasks and threads that
 * belong to the same task group:
 *
 *   Child exit status
 *   Environment varibles
 *   PIC data space and address environments
 *   File descriptors
 *   FILE streams
 *   Sockets
 *
 * Currenty, however, this implementation only applies to child exit status.
 *
 * Each instance of struct task_group_s is reference counted. Each instance is
 * created with a reference count of one.  The reference incremeneted when each
 * thread joins the group and decremented when each thread exits, leaving the
 * group.  When the refernce count decrements to zero, the struc task_group_s
 * is free.
 */

#ifdef HAVE_TASK_GROUP
struct task_group_s
{
#ifdef HAVE_GROUP_MEMBERS
  struct task_group_s *flink;       /* Supports a singly linked list            */
  gid_t      tg_gid;                /* The ID of this task group                */
  gid_t      tg_pgid;               /* The ID of the parent task group          */
#endif
  uint8_t    tg_flags;              /* See GROUP_FLAG_* definitions             */

  /* Group membership ***********************************************************/

  uint8_t    tg_nmembers;           /* Number of members in the group           */
#ifdef HAVE_GROUP_MEMBERS
  uint8_t    tg_mxmembers;          /* Number of members in allocation          */
  FAR pid_t *tg_members;            /* Members of the group                     */
#endif

  /* Child exit status **********************************************************/

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  FAR struct child_status_s *tg_children; /* Head of a list of child status     */
#endif

  /* Environment variables ******************************************************/

#ifndef CONFIG_DISABLE_ENVIRON
  size_t     tg_envsize;            /* Size of environment string allocation    */
  FAR char  *tg_envp;               /* Allocated environment strings            */
#endif

  /* PIC data space and address environments ************************************/
  /* Not yet (see struct dspace_s) */

  /* File descriptors ***********************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
  struct filelist tg_filelist;      /* Maps file descriptor to file             */
#endif

  /* FILE streams ***************************************************************/

#if CONFIG_NFILE_STREAMS > 0
  struct streamlist tg_streamlist;  /* Holds C buffered I/O info                */
#endif /* CONFIG_NFILE_STREAMS */

  /* Sockets ********************************************************************/
  /* Not yet (see struct socketlist) */
};
#endif

/* _TCB **************************************************************************/
/* This is the task control block (TCB).  Each task or thread is represented by
 * a TCB.  The TCB is the heart of the NuttX task-control logic.
 */

struct _TCB
{
  /* Fields used to support list management *************************************/

  FAR struct _TCB *flink;                /* Doubly linked list                  */
  FAR struct _TCB *blink;

  /* Task Group *****************************************************************/

#ifdef HAVE_TASK_GROUP
  FAR struct task_group_s *group;        /* Pointer to shared task group data   */
#endif

  /* Task Management Fields *****************************************************/

  pid_t    pid;                          /* This is the ID of the thread        */

#ifdef CONFIG_SCHED_HAVE_PARENT          /* Support parent-child relationship   */
#ifndef HAVE_GROUP_MEMBERS               /* Don't know pids of group members    */
  pid_t    ppid;                         /* This is the ID of the parent thread */
#ifndef CONFIG_SCHED_CHILD_STATUS        /* Retain child thread status          */
  uint16_t nchildren;                    /* This is the number active children  */
#endif
#endif
#endif /* CONFIG_SCHED_HAVE_PARENT */

  start_t  start;                        /* Thread start function               */
  entry_t  entry;                        /* Entry Point into the thread         */

#if defined(CONFIG_SCHED_ATEXIT) && !defined(CONFIG_SCHED_ONEXIT)
# if defined(CONFIG_SCHED_ATEXIT_MAX) && CONFIG_SCHED_ATEXIT_MAX > 1
  atexitfunc_t atexitfunc[CONFIG_SCHED_ATEXIT_MAX];
# else
  atexitfunc_t atexitfunc;               /* Called when exit is called.         */
# endif
#endif

#ifdef CONFIG_SCHED_ONEXIT
# if defined(CONFIG_SCHED_ONEXIT_MAX) && CONFIG_SCHED_ONEXIT_MAX > 1
  onexitfunc_t onexitfunc[CONFIG_SCHED_ONEXIT_MAX];
  FAR void *onexitarg[CONFIG_SCHED_ONEXIT_MAX];
# else
  onexitfunc_t onexitfunc;               /* Called when exit is called.         */
  FAR void *onexitarg;                   /* The argument passed to the function */
# endif
#endif

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  sem_t    exitsem;                      /* Support for waitpid                 */
  int     *stat_loc;                     /* Location to return exit status      */
#endif

  uint8_t  sched_priority;               /* Current priority of the thread      */

#ifdef CONFIG_PRIORITY_INHERITANCE
#  if CONFIG_SEM_NNESTPRIO > 0
  uint8_t  npend_reprio;                 /* Number of nested reprioritizations  */
  uint8_t  pend_reprios[CONFIG_SEM_NNESTPRIO];
#  endif
  uint8_t  base_priority;                /* "Normal" priority of the thread     */
#endif

  uint8_t  task_state;                   /* Current state of the thread         */
  uint16_t flags;                        /* Misc. general status flags          */
  int16_t  lockcount;                    /* 0=preemptable (not-locked)          */

#ifndef CONFIG_DISABLE_PTHREAD
  FAR void *joininfo;                    /* Detach-able info to support join    */
#endif

#if CONFIG_RR_INTERVAL > 0
  int      timeslice;                    /* RR timeslice interval remaining     */
#endif

  /* Values needed to restart a task ********************************************/

  uint8_t  init_priority;                /* Initial priority of the task        */
  char    *argv[CONFIG_MAX_TASK_ARGS+1]; /* Name+start-up parameters            */

  /* Stack-Related Fields *******************************************************/

#ifndef CONFIG_CUSTOM_STACK
  size_t    adj_stack_size;              /* Stack size after adjustment         */
                                         /* for hardware, processor, etc.       */
                                         /* (for debug purposes only)           */
  FAR void *stack_alloc_ptr;             /* Pointer to allocated stack          */
                                         /* Need to deallocate stack            */
  FAR void *adj_stack_ptr;               /* Adjusted stack_alloc_ptr for HW     */
                                         /* The initial stack pointer value     */
#endif

  /* External Module Support ****************************************************/

#ifdef CONFIG_PIC
  FAR struct dspace_s *dspace;           /* Allocated area for .bss and .data   */
#endif

  /* POSIX Thread Specific Data *************************************************/

#if !defined(CONFIG_DISABLE_PTHREAD) && CONFIG_NPTHREAD_KEYS > 0
  FAR void *pthread_data[CONFIG_NPTHREAD_KEYS];
#endif

  /* POSIX Semaphore Control Fields *********************************************/

  sem_t *waitsem;                        /* Semaphore ID waiting on             */

  /* POSIX Signal Control Fields ************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
  sigset_t   sigprocmask;                /* Signals that are blocked            */
  sigset_t   sigwaitmask;                /* Waiting for pending signals         */
  sq_queue_t sigactionq;                 /* List of actions for signals         */
  sq_queue_t sigpendingq;                /* List of Pending Signals             */
  sq_queue_t sigpendactionq;             /* List of pending signal actions      */
  sq_queue_t sigpostedq;                 /* List of posted signals              */
  siginfo_t  sigunbinfo;                 /* Signal info when task unblocked     */
#endif

  /* POSIX Named Message Queue Fields *******************************************/

#ifndef CONFIG_DISABLE_MQUEUE
  sq_queue_t msgdesq;                    /* List of opened message queues       */
  FAR msgq_t *msgwaitq;                  /* Waiting for this message queue      */
#endif

  /* Library related fields *****************************************************/

  int        pterrno;                    /* Current per-thread errno            */

  /* Network socket *************************************************************/

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  FAR struct socketlist *sockets;        /* Maps file descriptor to file        */
#endif

  /* State save areas ***********************************************************/
  /* The form and content of these fields are processor-specific.               */

  struct xcptcontext xcp;                /* Interrupt register save area        */

#if CONFIG_TASK_NAME_SIZE > 0
  char name[CONFIG_TASK_NAME_SIZE];      /* Task name                           */
#endif

};

/* Certain other header files may also define this time to avoid circular header
 * file inclusion issues.
 */

#ifndef __TCB_DEFINED__
typedef struct _TCB _TCB;
#define __TCB_DEFINED__
#endif

/* This is the callback type used by sched_foreach() */

typedef void (*sched_foreach_t)(FAR _TCB *tcb, FAR void *arg);

#endif /* __ASSEMBLY__ */

/********************************************************************************
 * Public Data
 ********************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

/* TCB helpers */

FAR _TCB *sched_self(void);

/* File system helpers */

#if CONFIG_NFILE_DESCRIPTORS > 0
FAR struct filelist *sched_getfiles(void);
#if CONFIG_NFILE_STREAMS > 0
FAR struct streamlist *sched_getstreams(void);
#endif /* CONFIG_NFILE_STREAMS */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#if CONFIG_NSOCKET_DESCRIPTORS > 0
FAR struct socketlist *sched_getsockets(void);
#endif /* CONFIG_NSOCKET_DESCRIPTORS */

/* Internal vfork support.The  overall sequence is:
 *
 * 1) User code calls vfork().  vfork() is provided in architecture-specific
 *    code.
 * 2) vfork()and calls task_vforksetup().
 * 3) task_vforksetup() allocates and configures the child task's TCB.  This
 *    consists of:
 *    - Allocation of the child task's TCB.
 *    - Initialization of file descriptors and streams
 *    - Configuration of environment variables
 *    - Setup the intput parameters for the task.
 *    - Initialization of the TCB (including call to up_initial_state()
 * 4) vfork() provides any additional operating context. vfork must:
 *    - Allocate and initialize the stack
 *    - Initialize special values in any CPU registers that were not
 *      already configured by up_initial_state()
 * 5) vfork() then calls task_vforkstart()
 * 6) task_vforkstart() then executes the child thread.
 *
 * task_vforkabort() may be called if an error occurs between steps 3 and 6.
 */

FAR _TCB *task_vforksetup(start_t retaddr);
pid_t task_vforkstart(FAR _TCB *child);
void task_vforkabort(FAR _TCB *child, int errcode);

/* sched_foreach will enumerate over each task and provide the
 * TCB of each task to a user callback functions.  Interrupts
 * will be disabled throughout this enumeration!
 */

void sched_foreach(sched_foreach_t handler, FAR void *arg);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_NUTTX_SCHED_H */
