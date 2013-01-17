/****************************************************************************
 * libc/string/lib_psa_dump.c
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
#include <debug.h>

#ifdef CONFIG_DEBUG

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawnattr_dump
 *
 * Description:
 *   Show the current attributes.
 *
 * Input Parameters:
 *   attr - The address of the spawn attributes to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void posix_spawnattr_dump(posix_spawnattr_t *attr)
{
  dbg("attr[%p]:\n", attr);
  dbg("  flags:    %04x\n", attr->flags);
  if (attr->flags == 0)
    {
      dbg("            None\n");
    }
  else
    {
      if ((attr->flags & POSIX_SPAWN_RESETIDS) != 0)
        {
          dbg("            POSIX_SPAWN_RESETIDS\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETPGROUP) != 0)
        {
          dbg("            POSIX_SPAWN_SETPGROUP\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSCHEDPARAM) != 0)
        {
          dbg("            POSIX_SPAWN_SETSCHEDPARAM\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
        {
          dbg("            POSIX_SPAWN_SETSCHEDULER\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSIGDEF) != 0)
        {
          dbg("            POSIX_SPAWN_SETSIGDEF\n");
        }

      if ((attr->flags & POSIX_SPAWN_SETSIGMASK) != 0)
        {
          dbg("            POSIX_SPAWN_SETSIGMASK\n");
        }
    }

  dbg("  priority: %d\n", attr->priority);

  dbg("  policy:   %d\n", attr->policy);
  if (attr->policy == SCHED_FIFO)
    {
      dbg("            SCHED_FIFO\n");
    }
  else if (attr->policy == SCHED_RR)
    {
      dbg("            SCHED_RR\n");
    }
  else
    {
      dbg("            Unrecognized\n");
    }

#ifndef CONFIG_DISABLE_SIGNALS
  dbg("  sigmask:  %08x\n", attr->sigmask);
#endif
};

#endif /* CONFIG_DEBUG */