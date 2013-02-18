/****************************************************************************
 * libc/stdio/lib_lowsyslog.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <debug.h>

#include "lib_internal.h"

/* This interface can only be used from within the kernel */

#if !defined(CONFIG_NUTTX_KERNEL) || defined(__KERNEL__)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lowvsyslog
 ****************************************************************************/

#if defined(CONFIG_ARCH_LOWPUTC) || defined(CONFIG_SYSLOG)

int lowvsyslog(const char *fmt, va_list ap)
{
  struct lib_outstream_s stream;

  /* Wrap the stdout in a stream object and let lib_vsprintf do the work. */

#ifdef CONFIG_SYSLOG
  lib_syslogstream((FAR struct lib_outstream_s *)&stream);
#else
  lib_lowoutstream((FAR struct lib_outstream_s *)&stream);
#endif
  return lib_vsprintf((FAR struct lib_outstream_s *)&stream, fmt, ap);
}

/****************************************************************************
 * Name: lowsyslog
 ****************************************************************************/

int lowsyslog(const char *fmt, ...)
{
  va_list ap;
  int     ret;

#ifdef CONFIG_SYSLOG_ENABLE
  ret = 0;
  if (g_syslogenable)
#endif
    {
      va_start(ap, fmt);
      ret = lowvsyslog(fmt, ap);
      va_end(ap);
    }

  return ret;
}

#endif /* CONFIG_ARCH_LOWPUTC || CONFIG_SYSLOG */
#endif /* __KERNEL__ */
