/****************************************************************************
 * apps/system/free/free.c
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
#include <nuttx/progmem.h>

#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <sched.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_SYSTEM_USBMONITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USBMON_PREFIX "USB Monitor: "

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct usbmon_state_s
{
  volatile bool started;
  volatile bool stop;
  pid_t pid;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct usbmon_state_s g_usbmonitor;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usbmonitor_tracecallback(struct usbtrace_s *trace, void *arg)
{
  usbtrace_trprintf((trprintf_t)syslog, trace->event, trace->value);
  return 0;
}

static int usbmonitor_daemon(int argc, char **argv)
{
  syslog(USBMON_PREFIX "Running: %d\n", g_usbmonitor.pid);

  /* Loop until we detect that there is a request to stop. */

  while (!g_usbmonitor.stop)
    {
      sleep(2);
      (void)usbtrace_enumerate(usbmonitor_tracecallback, NULL);
    }

  /* Stopped */

  g_usbmonitor.stop    = false;
  g_usbmonitor.started = false;
  syslog(USBMON_PREFIX "Stopped: %d\n", g_usbmonitor.pid);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int usbmonitor_start(int argc, char **argv)
{
  /* Has the monitor already started? */

  sched_lock();
  if (!g_usbmonitor.started)
    {
      int ret;

      /* No.. start it now */
 
      g_usbmonitor.started = true;
      g_usbmonitor.stop    = false;

      ret = TASK_CREATE("USB Monitor", CONFIG_SYSTEM_USBMONITOR_PRIORITY,
                        CONFIG_SYSTEM_USBMONITOR_STACKSIZE,
                        (main_t)usbmonitor_daemon, (const char **)NULL);
      if (ret < 0)
        {
          int errcode = errno;
          syslog(USBMON_PREFIX
                 "ERROR: Failed to start the USB monitor: %d\n",
                 errcode);
        }
      else
        {
          g_usbmonitor.pid = ret;
          syslog(USBMON_PREFIX "Started: %d\n", g_usbmonitor.pid);
        }

      sched_unlock();
      return 0;
    }

  sched_unlock();
  syslog(USBMON_PREFIX "Running: %d\n", g_usbmonitor.pid);
  return 0;
}

int usbmonitor_stop(int argc, char **argv)
{
  /* Has the monitor already started? */

  if (g_usbmonitor.started)
    {
      syslog(USBMON_PREFIX "Stopping: %d\n", g_usbmonitor.pid);
      g_usbmonitor.stop = true;
      return 0;
    }

  syslog(USBMON_PREFIX "Stopped: %d\n", g_usbmonitor.pid);
  return 0;
}

#endif /* CONFIG_SYSTEM_USBMONITOR */
