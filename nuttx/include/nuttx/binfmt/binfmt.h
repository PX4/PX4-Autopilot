/****************************************************************************
 * include/nuttx/binfmt/binfmt.h
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_BINFMT_BINFMT_H
#define __INCLUDE_NUTTX_BINFMT_BINFMT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <nxflat.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BINFMT_NALLOC 3

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* EXEPATH_HANDLE is an opaque handle used to traverse the absolute paths
 * assigned to the PATH environment variable.
 */

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_BINFMT_EXEPATH)
typedef FAR void *EXEPATH_HANDLE;
#endif

/* The type of one C++ constructor or destructor */

typedef FAR void (*binfmt_ctor_t)(void);
typedef FAR void (*binfmt_dtor_t)(void);

/* This describes the file to be loaded.
 *
 * NOTE 1: The 'filename' must be the full, absolute path to the file to be
 * executed unless CONFIG_BINFMT_EXEPATH is defined.  In that case,
 * 'filename' may be a relative path; a set of candidate absolute paths
 * will be generated using the PATH environment variable and load_module()
 * will attempt to load each file that is found at those absolute paths.
 */

struct symtab_s;
struct binary_s
{
  /* If CONFIG_SCHED_HAVE_PARENT is defined then schedul_unload() will
   * manage instances of struct binary_s allocated with kmalloc.  It
   * will keep the binary data in a link list and when SIGCHLD is received
   * (meaning that the task has exit'ed, schedul_unload() will find the
   * data, unload the module, and free the structure.
   */

#ifdef CONFIG_SCHED_HAVE_PARENT
  FAR struct binary_s *flink;          /* Supports a singly linked list */
  pid_t pid;                           /* Task ID of the child task */
#endif

  /* Information provided to the loader to load and bind a module */

  FAR const char *filename;            /* Full path to the binary to be loaded (See NOTE 1 above) */
  FAR const char **argv;               /* Argument list */
  FAR const struct symtab_s *exports;  /* Table of exported symbols */
  int nexports;                        /* The number of symbols in exports[] */

  /* Information provided from the loader (if successful) describing the
   * resources used by the loaded module.
   */

  main_t entrypt;                      /* Entry point into a program module */
  FAR void *mapped;                    /* Memory-mapped, address space */
  FAR void *alloc[BINFMT_NALLOC];      /* Allocated address spaces */

  /* Constructors/destructors */

#ifdef CONFIG_BINFMT_CONSTRUCTORS
  FAR binfmt_ctor_t *ctors;            /* Pointer to a list of constructors */
  FAR binfmt_dtor_t *dtors;            /* Pointer to a list of destructors */
  uint16_t nctors;                     /* Number of constructors in the list */
  uint16_t ndtors;                     /* Number of destructors in the list */
#endif

  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   */

#ifdef CONFIG_ADDRENV
  task_addrenv_t addrenv;              /* Task address environment */
#endif

  size_t mapsize;                      /* Size of the mapped address region (needed for munmap) */

  /* Start-up information that is provided by the loader, but may be modified
   * by the caller between load_module() and exec_module() calls.
   */

  uint8_t priority;                    /* Task execution priority */
  size_t stacksize;                    /* Size of the stack in bytes (unallocated) */
};

/* This describes one binary format handler */

struct binfmt_s
{
  FAR struct binfmt_s *next;             /* Supports a singly-linked list */
  int (*load)(FAR struct binary_s *bin); /* Verify and load binary into memory */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: register_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int register_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: unregister_binfmt
 *
 * Description:
 *   Register a loader for a binary format
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unregister_binfmt(FAR struct binfmt_s *binfmt);

/****************************************************************************
 * Name: load_module
 *
 * Description:
 *   Load a module into memory, bind it to an exported symbol take, and
 *   prep the module for execution.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns 0 (OK) on success.  On failure, it returns -1 (ERROR) with
 *   errno set appropriately.
 *
 ****************************************************************************/

int load_module(FAR struct binary_s *bin);

/****************************************************************************
 * Name: unload_module
 *
 * Description:
 *   Unload a (non-executing) module from memory.  If the module has
 *   been started (via exec_module) and has not exited, calling this will
 *   be fatal.
 *
 *   However, this function must be called after the module exist.  How
 *   this is done is up to your logic.  Perhaps you register it to be
 *   called by on_exit()?
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int unload_module(FAR const struct binary_s *bin);

/****************************************************************************
 * Name: exec_module
 *
 * Description:
 *   Execute a module that has been loaded into memory by load_module().
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   Returns the PID of the exec'ed module.  On failure, it returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec_module(FAR const struct binary_s *bin);

/****************************************************************************
 * Name: schedule_unload
 *
 * Description:
 *   If CONFIG_SCHED_HAVE_PARENT is defined, this function may be called by
 *   the parent of the the newly created task to automatically unload the
 *   module when the task exits.  This assumes that (1) the caller is the
 *   parent of the created task, (2) that bin was allocated with kmalloc()
 *   or friends.  It will also automatically free the structure with kfree()
 *   after unloading the module.
 *
 * Input Parameter:
 *   pid - The task ID of the child task
 *   bin - This structure must have been allocated with kmalloc() and must
 *         persist until the task unloads
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   It returns 0 (OK) if the callback was successfully scheduled. On
 *  failure, it returns -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
int schedule_unload(pid_t pid, FAR const struct binary_s *bin);
#endif

/****************************************************************************
 * Name: exec
 *
 * Description:
 *   This is a convenience function that wraps load_ and exec_module into
 *   one call.  If CONFIG_SCHED_ONEXIT is also defined, this function will
 *   automatically call schedule_unload() to unload the module when task
 *   exits.
 *
 * Input Parameter:
 *   filename - Fulll path to the binary to be loaded
 *   argv     - Argument list
 *   exports  - Table of exported symbols
 *   nexports - The number of symbols in exports
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   It returns the PID of the exec'ed module.  On failure, it returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec(FAR const char *filename, FAR const char **argv,
         FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: exepath_init
 *
 * Description:
 *   Initialize for the traversal of each value in the PATH variable.  The
 *   usage is sequence is as follows:
 *
 *   1) Call exepath_init() to initialize for the traversal.  exepath_init()
 *      will return an opaque handle that can then be provided to
 *      exepath_next() and exepath_release().
 *   2) Call exepath_next() repeatedly to examine every file that lies
 *      in the directories of the PATH variable
 *   3) Call exepath_release() to free resources set aside by exepath_init().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, exepath_init() return a non-NULL, opaque handle that may
 *   subsequently be used in calls to exepath_next() and exepath_release().
 *   On error, a NULL handle value will be returned.  The most likely cause
 *   of an error would be that the there is no value associated with the
 *   PATH variable.
 *
 ****************************************************************************/

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_BINFMT_EXEPATH)
EXEPATH_HANDLE exepath_init(void);
#endif

/****************************************************************************
 * Name: exepath_next
 *
 * Description:
 *   Traverse all possible values in the PATH variable in attempt to find
 *   the full path to an executable file when only a relative path is
 *   provided.
 *
 * Input Parameters:
 *   handle - The handle value returned by exepath_init
 *   relpath - The relative path to the file to be found. 
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a null-terminated string is provided.
 *   This is the full path to a file that exists in the file system.  This
 *   function will verify that the file exists (but will not verify that it
 *   is marked executable).
 *
 *   NOTE: The string pointer return in the success case points to allocated
 *   memory.  This memory must be freed by the called by calling kfree().
 *
 *   NULL is returned if no path is found to any file with the provided
 *   'relpath' from any absolute path in the PATH variable.  In this case,
 *   there is no point in calling exepath_next() further; exepath_release()
 *   must be called to release resources set aside by expath_init().
 *
 ****************************************************************************/

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_BINFMT_EXEPATH)
FAR char *exepath_next(EXEPATH_HANDLE handle, FAR const char *relpath);
#endif

/****************************************************************************
 * Name: exepath_release
 *
 * Description:
 *   Release all resources set aside by exepath_init() when the handle value
 *   was created.  The handle value is invalid on return from this function.
 *   Attempts to all exepath_next() or exepath_release() with such a 'stale'
 *   handle will result in undefined (i.e., not good) behavior.
 *
 * Input Parameters:
 *   handle - The handle value returned by exepath_init
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if !defined(CONFIG_BINFMT_DISABLE) && defined(CONFIG_BINFMT_EXEPATH)
void exepath_release(EXEPATH_HANDLE handle);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_BINFMT_H */

