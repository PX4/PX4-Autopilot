/****************************************************************************
 * libc/misc/lib_dbg.c
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

#include <stdarg.h>
#include <debug.h>

#include "lib_internal.h"

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Debug output is initially disabled */

#ifdef CONFIG_SYSLOG_ENABLE
bool g_syslogenable;
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_enable
 *
 * Description:
 *  Enable or disable debug output.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_ENABLE
void syslog_enable(bool enable)
{
  g_syslogenable = enable;
}
#endif

/****************************************************************************
 * Name: dbg, lldbg, vdbg
 *
 * Description:
 *  If the cross-compiler's pre-processor does not support variable
 * length arguments, then these additional APIs will be built.
 *
 ****************************************************************************/

#ifndef CONFIG_CPP_HAVE_VARARGS
#ifdef CONFIG_DEBUG
int dbg(const char *format, ...)
{
  va_list ap;
  int     ret;

#ifdef CONFIG_SYSLOG_ENABLE
  ret = 0;
  if (g_syslogenable)
#endif
    {
      va_start(ap, format);
      ret = vsyslog(format, ap);
      va_end(ap);
    }

  return ret;
}

#ifdef CONFIG_ARCH_LOWPUTC
int lldbg(const char *format, ...)
{
  va_list ap;
  int     ret;

#ifdef CONFIG_SYSLOG_ENABLE
  ret = 0;
  if (g_syslogenable)
#endif
    {
      va_start(ap, format);
      ret = lowvsyslog(format, ap);
      va_end(ap);
    }

  return ret;
}
#endif

#ifdef CONFIG_DEBUG_VERBOSE
int vdbg(const char *format, ...)
{
  va_list ap;
  int     ret;

#ifdef CONFIG_SYSLOG_ENABLE
  ret = 0;
  if (g_syslogenable)
#endif
    {
      va_start(ap, format);
      ret = vsyslog(format, ap);
      va_end(ap);
    }

  return ret;
}

#ifdef CONFIG_ARCH_LOWPUTC
int llvdbg(const char *format, ...)
{
  va_list ap;
  int     ret;

#ifdef CONFIG_SYSLOG_ENABLE
  ret = 0;
  if (g_syslogenable)
#endif
    {
      va_start(ap, format);
      ret = lowvsyslog(format, ap);
      va_end(ap);
    }

  return ret;
}
#endif /* CONFIG_ARCH_LOWPUTC */
#endif /* CONFIG_DEBUG_VERBOSE */
#endif /* CONFIG_DEBUG */
#endif /* CONFIG_CPP_HAVE_VARARGS */
