/****************************************************************************
 * include/debug.h
 *
 *   Copyright (C) 2007-2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_DEBUG_H
#define __INCLUDE_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug macros to runtime filter the debug messages sent to the console.  In
 * general, there are four forms of the debug macros:
 *
 * [a-z]dbg() -- Outputs messages to the console similar to printf() except
 *    that the output is not buffered.  The first character indicates the
 *    system system (e.g., n=network, f=filesystm, etc.).  If the first
 *    character is missing (i.e., dbg()), then it is common.  The common
 *    dbg() macro is enabled by CONFIG_DEBUG.  Subystem debug requires an
 *    additional configuration setting to enable it (e.g., CONFIG_DEBUG_NET
 *    for the network, CONFIG_DEBUG_FS for the file syste, etc).
 *
 *    In general, error messages and output of importance use [a-z]dbg().
 *    [a-z]dbg() is implementation dependent but usually uses file descriptors.
 *    (that is a problem only because the interrupt task may have re-
 *    directed stdout).  Therefore [a-z]dbg() should not be used in interrupt
 *    handlers.
 *
 * [a-z]vdbg() -- Identifcal to [a-z]dbg() except that it also requires that
 *    CONFIG_DEBUG_VERBOSE be defined.  This is intended for general debug
 *    output that you would normally want to suppress.
 *
 * [a-z]lldbg() -- Identical to [a-z]dbg() except this is uses special
 *    interfaces provided by architecture-specific logic to talk directly
 *    to the underlying console hardware.  If the architecture provides such
 *    logic, it should define CONFIG_ARCH_LOWPUTC.
 *
 *    [a-z]lldbg() should not be used in normal code because the implementation
 *    probably disables interrupts and does things that are not consistent with
 *    good real-time performance.  However, [a-z]lldbg() is particularly useful
 *    in low-level code where it is inappropriate to use file descriptors.  For
 *    example, only [a-z]lldbg() should be used in interrupt handlers.
 *
 * [a-z]llvdbg() -- Identifcal to [a-z]lldbg() except that it also requires that
 *    CONFIG_DEBUG_VERBOSE be defined.  This is intended for general debug
 *    output that you would normally want to suppress.
 */

#ifdef CONFIG_HAVE_FUNCTIONNAME
# define EXTRA_FMT "%s: "
# define EXTRA_ARG ,__FUNCTION__
#else
# define EXTRA_FMT
# define EXTRA_ARG
#endif

/* Debug macros will differ depending upon if the toolchain supports
 * macros with a variable number of arguments or not.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS

/* Variable argument macros supported */

#ifdef CONFIG_DEBUG
# define dbg(format, arg...) \
  syslog(EXTRA_FMT format EXTRA_ARG, ##arg)

# ifdef CONFIG_ARCH_LOWPUTC
#  define lldbg(format, arg...) \
   lowsyslog(EXTRA_FMT format EXTRA_ARG, ##arg)
# else
#  define lldbg(x...)
# endif

# ifdef CONFIG_DEBUG_VERBOSE
#  define vdbg(format, arg...) \
   syslog(EXTRA_FMT format EXTRA_ARG, ##arg)

#  ifdef CONFIG_ARCH_LOWPUTC
#    define llvdbg(format, arg...) \
     lowsyslog(EXTRA_FMT format EXTRA_ARG, ##arg)
#  else
#    define llvdbg(x...)
#  endif

# else
#  define vdbg(x...)
#  define llvdbg(x...)
# endif

#else /* CONFIG_DEBUG */

# define dbg(x...)
# define lldbg(x...)
# define vdbg(x...)
# define llvdbg(x...)

#endif /* CONFIG_DEBUG */

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
# define mdbg(format, arg...)    dbg(format, ##arg)
# define mlldbg(format, arg...)  lldbg(format, ##arg)
# define mvdbg(format, arg...)   vdbg(format, ##arg)
# define mllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define mdbg(x...)
# define mlldbg(x...)
# define mvdbg(x...)
# define mllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_SCHED
# define sdbg(format, arg...)    dbg(format, ##arg)
# define slldbg(format, arg...)  lldbg(format, ##arg)
# define svdbg(format, arg...)   vdbg(format, ##arg)
# define sllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define sdbg(x...)
# define slldbg(x...)
# define svdbg(x...)
# define sllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_PAGING
# define pgdbg(format, arg...)    dbg(format, ##arg)
# define pglldbg(format, arg...)  lldbg(format, ##arg)
# define pgvdbg(format, arg...)   vdbg(format, ##arg)
# define pgllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define pgdbg(x...)
# define pglldbg(x...)
# define pgvdbg(x...)
# define pgllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_DMA
# define dmadbg(format, arg...)    dbg(format, ##arg)
# define dmalldbg(format, arg...)  lldbg(format, ##arg)
# define dmavdbg(format, arg...)   vdbg(format, ##arg)
# define dmallvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define dmadbg(x...)
# define dmalldbg(x...)
# define dmavdbg(x...)
# define dmallvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_NET
# define ndbg(format, arg...)    dbg(format, ##arg)
# define nlldbg(format, arg...)  lldbg(format, ##arg)
# define nvdbg(format, arg...)   vdbg(format, ##arg)
# define nllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define ndbg(x...)
# define nlldbg(x...)
# define nvdbg(x...)
# define nllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_USB
# define udbg(format, arg...)    dbg(format, ##arg)
# define ulldbg(format, arg...)  lldbg(format, ##arg)
# define uvdbg(format, arg...)   vdbg(format, ##arg)
# define ullvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define udbg(x...)
# define ulldbg(x...)
# define uvdbg(x...)
# define ullvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_FS
# define fdbg(format, arg...)    dbg(format, ##arg)
# define flldbg(format, arg...)  lldbg(format, ##arg)
# define fvdbg(format, arg...)   vdbg(format, ##arg)
# define fllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define fdbg(x...)
# define flldbg(x...)
# define fvdbg(x...)
# define fllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_INPUT
# define idbg(format, arg...)    dbg(format, ##arg)
# define illdbg(format, arg...)  lldbg(format, ##arg)
# define ivdbg(format, arg...)   vdbg(format, ##arg)
# define illvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define idbg(x...)
# define illdbg(x...)
# define ivdbg(x...)
# define illvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_ANALOG
# define adbg(format, arg...)    dbg(format, ##arg)
# define alldbg(format, arg...)  lldbg(format, ##arg)
# define avdbg(format, arg...)   vdbg(format, ##arg)
# define allvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define adbg(x...)
# define alldbg(x...)
# define avdbg(x...)
# define allvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
# define gdbg(format, arg...)    dbg(format, ##arg)
# define glldbg(format, arg...)  lldbg(format, ##arg)
# define gvdbg(format, arg...)   vdbg(format, ##arg)
# define gllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define gdbg(x...)
# define glldbg(x...)
# define gvdbg(x...)
# define gllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_BINFMT
# define bdbg(format, arg...)    dbg(format, ##arg)
# define blldbg(format, arg...)  lldbg(format, ##arg)
# define bvdbg(format, arg...)   vdbg(format, ##arg)
# define bllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define bdbg(x...)
# define blldbg(x...)
# define bvdbg(x...)
# define bllvdbg(x...)
#endif

#ifdef CONFIG_DEBUG_LIB
# define ldbg(format, arg...)    dbg(format, ##arg)
# define llldbg(format, arg...)  lldbg(format, ##arg)
# define lvdbg(format, arg...)   vdbg(format, ##arg)
# define lllvdbg(format, arg...) llvdbg(format, ##arg)
#else
# define ldbg(x...)
# define llldbg(x...)
# define lvdbg(x...)
# define lllvdbg(x...)
#endif

#else /* CONFIG_CPP_HAVE_VARARGS */

/* Variable argument macros NOT supported */

#ifdef CONFIG_DEBUG
# ifndef CONFIG_ARCH_LOWPUTC
#  define lldbg (void)
# endif
# ifndef CONFIG_DEBUG_VERBOSE
#  define vdbg (void)
#  define llvdbg (void)
# else
#  ifndef CONFIG_ARCH_LOWPUTC
#    define llvdbg (void)
#  endif
# endif
#else
# define dbg    (void)
# define lldbg  (void)
# define vdbg   (void)
# define llvdbg (void)
#endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
# define mdbg    dbg
# define mlldbg  lldbg
# define mvdbg   vdbg
# define mllvdbg llvdbg
#else
# define mdbg    (void)
# define mlldbg  (void)
# define mvdbg   (void)
# define mllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_SCHED
# define sdbg    dbg
# define slldbg  lldbg
# define svdbg   vdbg
# define sllvdbg llvdbg
#else
# define sdbg    (void)
# define slldbg  (void)
# define svdbg   (void)
# define sllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_PAGING
# define pgdbg    dbg
# define pglldbg  lldbg
# define pgvdbg   vdbg
# define pgllvdbg llvdbg
#else
# define pgdbg    (void)
# define pglldbg  (void)
# define pgvdbg   (void)
# define pgllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_DMA
# define dmadbg    dbg
# define dmalldbg  lldbg
# define dmavdbg   vdbg
# define dmallvdbg llvdbg
#else
# define dmadbg    (void)
# define dmalldbg  (void)
# define dmavdbg   (void)
# define dmallvdbg (void)
#endif

#ifdef CONFIG_DEBUG_NET
# define ndbg    dbg
# define nlldbg  lldbg
# define nvdbg   vdbg
# define nllvdbg llvdbg
#else
# define ndbg    (void)
# define nlldbg  (void)
# define nvdbg   (void)
# define nllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_USB
# define udbg    dbg
# define ulldbg  lldbg
# define uvdbg   vdbg
# define ullvdbg llvdbg
#else
# define udbg    (void)
# define ulldbg  (void)
# define uvdbg   (void)
# define ullvdbg (void)
#endif

#ifdef CONFIG_DEBUG_FS
# define fdbg    dbg
# define flldbg  lldbg
# define fvdbg   vdbg
# define fllvdbg llvdbg
#else
# define fdbg    (void)
# define flldbg  (void)
# define fvdbg   (void)
# define fllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_INPUT
# define idbg    dbg
# define illdbg  lldbg
# define ivdbg   vdbg
# define illvdbg llvdbg
#else
# define idbg    (void)
# define illdbg  (void)
# define ivdbg   (void)
# define illvdbg (void)
#endif

#ifdef CONFIG_DEBUG_ANALOG
# define adbg    dbg
# define alldbg  lldbg
# define avdbg   vdbg
# define allvdbg llvdbg
#else
# define adbg    (void)
# define alldbg  (void)
# define avdbg   (void)
# define allvdbg (void)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
# define gdbg    dbg
# define glldbg  lldbg
# define gvdbg   vdbg
# define gllvdbg llvdbg
#else
# define gdbg    (void)
# define glldbg  (void)
# define gvdbg   (void)
# define gllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_BINFMT
# define bdbg    dbg
# define blldbg  lldbg
# define bvdbg   vdbg
# define bllvdbg llvdbg
#else
# define bdbg    (void)
# define blldbg  (void)
# define bvdbg   (void)
# define bllvdbg (void)
#endif

#ifdef CONFIG_DEBUG_LIB
# define ldbg    dbg
# define llldbg  lldbg
# define lvdbg   vdbg
# define lllvdbg llvdbg
#else
# define ldbg    (void)
# define llldbg  (void)
# define lvdbg   (void)
# define lllvdbg (void)
#endif

#endif /* CONFIG_CPP_HAVE_VARARGS */

/* Buffer dumping macros do not depend on varargs */

#ifdef CONFIG_DEBUG
#  define dbgdumpbuffer(m,b,n) lib_dumpbuffer(m,b,n)
#  ifdef CONFIG_DEBUG_VERBOSE
#    define vdbgdumpbuffer(m,b,n) lib_dumpbuffer(m,b,n)
#  else
#   define vdbgdumpbuffer(m,b,n)
#  endif
#else
#  define dbgdumpbuffer(m,b,n)
#  define vdbgdumpbuffer(m,b,n)
# endif

/* Subsystem specific debug */

#ifdef CONFIG_DEBUG_MM
#  define mdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define mvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define mdbgdumpbuffer(m,b,n)
#  define mvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_SCHED
#  define sdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define svdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define sdbgdumpbuffer(m,b,n)
#  define svdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_PAGING
#  define pgdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define pgvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define pgdbgdumpbuffer(m,b,n)
#  define pgvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_DMA
#  define dmadbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define dmavdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define dmadbgdumpbuffer(m,b,n)
#  define dmavdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_NET
#  define ndbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define nvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define ndbgdumpbuffer(m,b,n)
#  define nvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_USB
#  define udbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define uvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define udbgdumpbuffer(m,b,n)
#  define uvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_FS
#  define fdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define fvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define fdbgdumpbuffer(m,b,n)
#  define fvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_INPUT
#  define idbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define ivdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define idbgdumpbuffer(m,b,n)
#  define ivdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_GRAPHICS
#  define gdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define gvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define gdbgdumpbuffer(m,b,n)
#  define gvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_BINFMT
#  define bdbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define bvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define bdbgdumpbuffer(m,b,n)
#  define bvdbgdumpbuffer(m,b,n)
#endif

#ifdef CONFIG_DEBUG_LIB
#  define ldbgdumpbuffer(m,b,n)  dbgdumpbuffer(m,b,n)
#  define lvdbgdumpbuffer(m,b,n) vdbgdumpbuffer(m,b,n)
#else
#  define ldbgdumpbuffer(m,b,n)
#  define lvdbgdumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/* Dump a buffer of data */

void lib_dumpbuffer(FAR const char *msg, FAR const uint8_t *buffer, unsigned int buflen);

/* The system logging interfaces are pnormally accessed via the macros
 * provided above.  If the cross-compiler's C pre-processor supports a
 * variable number of macro arguments, then those macros below will map all
 * debug statements to the logging interfaces declared in syslog.h.
 *
 * If the cross-compiler's pre-processor does not support variable length
 * arguments, then these additional APIs will be built.
 */

#ifndef CONFIG_CPP_HAVE_VARARGS
#ifdef CONFIG_DEBUG
int dbg(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int lldbg(const char *format, ...);
# endif

# ifdef CONFIG_DEBUG_VERBOSE
int vdbg(const char *format, ...);

# ifdef CONFIG_ARCH_LOWPUTC
int llvdbg(const char *format, ...);
# endif
#endif
#endif /* CONFIG_DEBUG */
#endif /* CONFIG_CPP_HAVE_VARARGS */

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DEBUG_H */
