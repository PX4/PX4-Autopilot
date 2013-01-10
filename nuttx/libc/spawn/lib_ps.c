/****************************************************************************
 * libc/string/lib_ps.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <spawn.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn
 *
 * Description:
 *   The posix_spawn() and posix_spawnp() functions will create a new,
 *   child task, constructed a regular executable file.
 *
 * Input Parameters:
 *
 *   pid - Upon successful completion, posix_spawn() and posix_spawnp() will
 *     return the task ID of the child task to the parent task, in the
 *     variable pointed to by a non-NULL 'pid' argument.  If the 'pid'
 *     argument is a null pointer, the process ID of the child is not
 *     returned to the caller.
 *
 *   path - The 'path' argument to posix_spawn() is the absolute path that
 *     identifies the file to execute.  The 'path' argument to posix_spawnp()
 *     may also be a relative path and will be used to construct a pathname
 *     that identifies the file to execute.  In the case of a relative path,
 *     the path prefix for the file will be obtained by a search of the
 *     directories passed as the environment variable PATH.
 *
 *     NOTE: NuttX provides only one implementation:  If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
 *
 *   file_actions - If 'file_actions' is a null pointer, then file 
 *     descriptors open in the calling process will remain open in the
 *     child process (unless CONFIG_FDCLONE_STDIO is defined). If
 *     'file_actions' is not NULL, then the file descriptors open in the
 *     child process will be those open in the calling process as modified
 *     by the spawn file actions object pointed to by file_actions.
 *
 *   attr - If the value of the 'attr' parameter is NULL, the all default
 *     values for the POSIX spawn attributes will be used.  Otherwise, the
 *     attributes will be set according to the spawn flags.  The
 *     posix_spawnattr_t spawn attributes object type is defined in spawn.h.
 *     It will contains these attributes, not all of which are supported by
 *     NuttX:
 *
 *     - POSIX_SPAWN_SETPGROUP:  Setting of the new tasks process group is
 *       not supported.  NuttX does not support process groups.
 *     - POSIX_SPAWN_SETSCHEDPARAM: Set new tasks priority to the sched_param
 *       value.
 *     - POSIX_SPAWN_SETSCHEDULER: Set the new tasks scheduler priority to
 *       the sched_policy value.
 *     - POSIX_SPAWN_RESETIDS: Resetting of effective user ID of the child
 *       process is not supported.  NuttX does not support effective user
 *       IDs.
 *     - POSIX_SPAWN_SETSIGMASK: Setting the initial signal mask of the new
 *       task is not supported.  NuttX does support signal masks, but there
 *       is no mechanism in place now to do this.
 *     - POSIX_SPAWN_SETSIGDEF:  Resetting signal default actions is not
 *       supported.  NuttX does not support default signal actions.
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer. The value in argv[0] should point to a filename
 *     that is associated with the process image being started by the
 *     posix_spawn() or posix_spawnp() functions.
 *
 *   envp - The envp[] argument is not used by NuttX and may be NULL.  In
 *     standard implementations, envp[] is an array of character pointers to
 *     null-terminated strings that provide the environment for the new
 *     process image. The environment array is terminated by a null pointer.
 *     In NuttX, the envp[] argument is ignored and the new task will simply
 *     inherit the environment of the parent task.
 *
 * Returned Value:
 *   posix_spawn() and posix_spawnp() will return zero on success.
 *   Otherwise, an error number will be returned as the function return
 *   value to indicate the error:
 *
 *   - EINVAL: The value specified by 'file_actions' or 'attr' is invalid.
 *   - Any errors that might have been return if vfork() and excec[l|v]()
 *     had been called.
 *
 * Assumptions/Limitations:
 *   - NuttX provides only posix_spawn() or posix_spawnp() behavior
 *     depending upon the setting of CONFIG_BINFMT_EXEPATH: If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
 *   - The 'envp' argument is not used and the 'environ' variable is not
 *     altered (NuttX does not support the 'environ' variable.
 *   - Process groups are not supported (POSIX_SPAWN_SETPGROUP).
 *   - Effective user IDs are not supported (POSIX_SPAWN_RESETIDS).
 *   - Signal masks and signal default actions cannot be modified in the
 *     newly executed task (POSIX_SPAWN_SETSIGDEF and POSIX_SPAWN_SETSIGMASK).
 *
 * POSIX Compatibility
 *   - The value of the argv[0] received by the child task is assigned by
 *     NuttX.  For the caller of posix_spawn(), the provided argv[0] will
 *     correspond to argv[1] received by the new task.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_EXEPATH
int posix_spawnp(FAR pid_t *pid, FAR const char *path,
                 FAR const posix_spawn_file_actions_t *file_actions,
                 FAR const posix_spawnattr_t *attr,
                 FAR char *const argv[], FAR char *const envp[])
#else
int posix_spawn(FAR pid_t *pid, FAR const char *path,
                FAR const posix_spawn_file_actions_t *file_actions,
                FAR const posix_spawnattr_t *attr,
                FAR char *const argv[], FAR char *const envp[])
#endif
{
#warning "Missing Logic"
  return OK;
}
