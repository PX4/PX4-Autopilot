/****************************************************************************
 * libc/stdio/lib_libvsprintf.c
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

#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If you have floating point but no fieldwidth, then use a fixed (but
 * configurable) floating point precision.
 */

#if defined(CONFIG_LIBC_FLOATINGPOINT) && \
    defined(CONFIG_NOPRINTF_FIELDWIDTH) && \
   !defined(CONFIG_LIBC_FIXEDPRECISION)
#  define CONFIG_LIBC_FIXEDPRECISION 3
#endif

#define FLAG_SHOWPLUS            0x01
#define FLAG_ALTFORM             0x02
#define FLAG_HASDOT              0x04
#define FLAG_HASASTERISKWIDTH    0x08
#define FLAG_HASASTERISKTRUNC    0x10
#define FLAG_LONGPRECISION       0x20
#define FLAG_LONGLONGPRECISION   0x40
#define FLAG_NEGATE              0x80

#define SET_SHOWPLUS(f)          do (f) |= FLAG_SHOWPLUS; while (0)
#define SET_ALTFORM(f)           do (f) |= FLAG_ALTFORM; while (0)
#define SET_HASDOT(f)            do (f) |= FLAG_HASDOT; while (0)
#define SET_HASASTERISKWIDTH(f)  do (f) |= FLAG_HASASTERISKWIDTH; while (0)
#define SET_HASASTERISKTRUNC(f)  do (f) |= FLAG_HASASTERISKTRUNC; while (0)
#define SET_LONGPRECISION(f)     do (f) |= FLAG_LONGPRECISION; while (0)
#define SET_LONGLONGPRECISION(f) do (f) |= FLAG_LONGLONGPRECISION; while (0)
#define SET_NEGATE(f)            do (f) |= FLAG_NEGATE; while (0)

#define CLR_SHOWPLUS(f)          do (f) &= ~FLAG_SHOWPLUS; while (0)
#define CLR_ALTFORM(f)           do (f) &= ~FLAG_ALTFORM; while (0)
#define CLR_HASDOT(f)            do (f) &= ~FLAG_HASDOT; while (0)
#define CLR_HASASTERISKWIDTH(f)  do (f) &= ~FLAG_HASASTERISKWIDTH; while (0)
#define CLR_HASASTERISKTRUNC(f)  do (f) &= ~FLAG_HASASTERISKTRUNC; while (0)
#define CLR_LONGPRECISION(f)     do (f) &= ~FLAG_LONGPRECISION; while (0)
#define CLR_LONGLONGPRECISION(f) do (f) &= ~FLAG_LONGLONGPRECISION; while (0)
#define CLR_NEGATE(f)            do (f) &= ~FLAG_NEGATE; while (0)
#define CLR_SIGNED(f)            do (f) &= ~(FLAG_SHOWPLUS|FLAG_NEGATE); while (0)

#define IS_SHOWPLUS(f)           (((f) & FLAG_SHOWPLUS) != 0)
#define IS_ALTFORM(f)            (((f) & FLAG_ALTFORM) != 0)
#define IS_HASDOT(f)             (((f) & FLAG_HASDOT) != 0)
#define IS_HASASTERISKWIDTH(f)   (((f) & FLAG_HASASTERISKWIDTH) != 0)
#define IS_HASASTERISKTRUNC(f)   (((f) & FLAG_HASASTERISKTRUNC) != 0)
#define IS_LONGPRECISION(f)      (((f) & FLAG_LONGPRECISION) != 0)
#define IS_LONGLONGPRECISION(f)  (((f) & FLAG_LONGLONGPRECISION) != 0)
#define IS_NEGATE(f)             (((f) & FLAG_NEGATE) != 0)
#define IS_SIGNED(f)             (((f) & (FLAG_SHOWPLUS|FLAG_NEGATE)) != 0)

/* If CONFIG_ARCH_ROMGETC is defined, then it is assumed that the format
 * string data cannot be accessed by simply de-referencing the format string
 * pointer.  This might be in the case in Harvard architectures where string
 * data might be stored in instruction space or if string data were stored
 * on some media like EEPROM or external serial FLASH.  In all of these cases,
 * string data has to be accessed indirectly using the architecture-supplied
 * up_romgetc().  The following mechanisms attempt to make these different
 * access methods indistinguishable in the following code.
 *
 * NOTE: It is assumed that string arguments for %s still reside in memory
 * that can be directly accessed by de-referencing the string pointer.
 */

#ifdef CONFIG_ARCH_ROMGETC
#  define FMT_TOP      ch = up_romgetc(src)         /* Loop initialization */
#  define FMT_BOTTOM   src++, ch = up_romgetc(src)  /* Bottom of a loop */
#  define FMT_CHAR     ch                           /* Access a character */
#  define FMT_NEXT     src++; ch = up_romgetc(src)  /* Advance to the next character */
#  define FMT_PREV     src--; ch = up_romgetc(src)  /* Backup to the previous character */
#else
#  define FMT_TOP                                   /* Loop initialization */
#  define FMT_BOTTOM   src++                        /* Bottom of a loop */
#  define FMT_CHAR     *src                         /* Access a character */
#  define FMT_NEXT     src++                        /* Advance to the next character */
#  define FMT_PREV     src--                        /* Backup to the previous character */
#endif
 
/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

enum
{
  FMT_RJUST = 0, /* Default */
  FMT_LJUST,
  FMT_RJUST0,
  FMT_CENTER
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Pointer to ASCII conversion */

#ifdef CONFIG_PTR_IS_NOT_INT
static void ptohex(FAR struct lib_outstream_s *obj, uint8_t flags, FAR void *p);
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static int  getsizesize(uint8_t fmt, uint8_t flags, FAR void *p)
#endif /* CONFIG_NOPRINTF_FIELDWIDTH */
#endif /* CONFIG_PTR_IS_NOT_INT */

/* Unsigned int to ASCII conversion */

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n);
static void utohex(FAR struct lib_outstream_s *obj, unsigned int n, uint8_t a);
static void utooct(FAR struct lib_outstream_s *obj, unsigned int n);
static void utobin(FAR struct lib_outstream_s *obj, unsigned int n);
static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                     uint8_t flags, unsigned int lln);

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void fixup(uint8_t fmt, FAR uint8_t *flags, int *n);
static int  getusize(uint8_t fmt, uint8_t flags, unsigned int lln);
#endif

/* Unsigned long int to ASCII conversion */

#ifdef CONFIG_LONG_IS_NOT_INT
static void lutodec(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutohex(FAR struct lib_outstream_s *obj, unsigned long ln, uint8_t a);
static void lutooct(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutobin(FAR struct lib_outstream_s *obj, unsigned long ln);
static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                      uint8_t flags, unsigned long ln);
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void lfixup(uint8_t fmt, FAR uint8_t *flags, long *ln);
static int  getlusize(uint8_t fmt, FAR uint8_t flags, unsigned long ln);
#endif
#endif

/* Unsigned long long int to ASCII conversions */

#ifdef CONFIG_HAVE_LONG_LONG
static void llutodec(FAR struct lib_outstream_s *obj, unsigned long long lln);
static void llutohex(FAR struct lib_outstream_s *obj, unsigned long long lln, uint8_t a);
static void llutooct(FAR struct lib_outstream_s *obj, unsigned long long lln);
static void llutobin(FAR struct lib_outstream_s *obj, unsigned long long lln);
static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, unsigned long long lln);
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void llfixup(uint8_t fmt, FAR uint8_t *flags, FAR long long *lln);
static int  getllusize(uint8_t fmt, FAR uint8_t flags, FAR unsigned long long lln);
#endif
#endif

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, int fieldwidth, int valwidth);
static void postjustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                        uint8_t flags, int fieldwidth, int valwidth);
#endif

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_nullstring[] = "(null)";

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Include floating point functions */
 
#ifdef CONFIG_LIBC_FLOATINGPOINT
#  include "stdio/lib_libdtoa.c"
#endif

/****************************************************************************
 * Name: ptohex
 ****************************************************************************/

#ifdef CONFIG_PTR_IS_NOT_INT
static void ptohex(FAR struct lib_outstream_s *obj, uint8_t flags, FAR void *p)
{
  union
  {
    uint32_t  dw;
    FAR void *p;
  } u;
  uint8_t bits;

  /* Check for alternate form */

  if (IS_ALTFORM(flags))
    {
      /* Prefix the number with "0x" */

      obj->put(obj, '0');
      obj->put(obj, 'x');
    }

  u.dw = 0;
  u.p  = p;

  for (bits = 8*sizeof(void *); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((u.dw >> (bits - 4)) & 0xf);
      if (nibble < 10)
        {
          obj->put(obj, nibble + '0');
        }
      else
        {
          obj->put(obj, nibble + 'a' - 10);
        }
    }
}

/****************************************************************************
 * Name: getpsize
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static int getpsize(uint8_t flags, FAR void *p)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  ptohex(&nulloutstream, flags, p);
  return nulloutstream.nput;
}

#endif /* CONFIG_NOPRINTF_FIELDWIDTH */
#endif /* CONFIG_PTR_IS_NOT_INT */

/****************************************************************************
 * Name: utodec
 ****************************************************************************/

static void utodec(FAR struct lib_outstream_s *obj, unsigned int n)
{
  unsigned int remainder = n % 10;
  unsigned int dividend  = n / 10;

  if (dividend)
    {
      utodec(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: utohex
 ****************************************************************************/

static void utohex(FAR struct lib_outstream_s *obj, unsigned int n, uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8*sizeof(unsigned int); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, nibble + '0');
            }
          else
            {
              obj->put(obj, nibble + a - 10);
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: utooct
 ****************************************************************************/

static void utooct(FAR struct lib_outstream_s *obj, unsigned int n)
{
  unsigned int remainder = n & 0x7;
  unsigned int dividend = n >> 3;

  if (dividend)
    {
      utooct(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: utobin
 ****************************************************************************/

static void utobin(FAR struct lib_outstream_s *obj, unsigned int n)
{
  unsigned int remainder = n & 1;
  unsigned int dividend = n >> 1;

  if (dividend)
    {
      utobin(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: utoascii
 ****************************************************************************/

static void utoascii(FAR struct lib_outstream_s *obj, uint8_t fmt, uint8_t flags, unsigned int n)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if ((int)n < 0)
            {
              obj->put(obj, '-');
              n = (unsigned int)(-(int)n);
            }
          else if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          utodec(obj, n);
        }
        break;

      case 'u':
        /* Unigned base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          utodec(obj, n);
        }
        break;

#ifndef CONFIG_PTR_IS_NOT_INT
      case 'p':
#endif
      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              obj->put(obj, '0');
              obj->put(obj, 'x');
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              utohex(obj, n, 'A');
            }
          else
            {
              utohex(obj, n, 'a');
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           utooct(obj, n);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          utobin(obj, n);
        }
        break;

#ifdef CONFIG_PTR_IS_NOT_INT
      case 'p':
#endif
      default:
        break;
    }
}

/****************************************************************************
 * Name: fixup
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void fixup(uint8_t fmt, FAR uint8_t *flags, FAR int *n)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*n < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *n    = -*n;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getusize
 ****************************************************************************/

static int getusize(uint8_t fmt, uint8_t flags, unsigned int n)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  utoascii(&nulloutstream, fmt, flags, n);
  return nulloutstream.nput;
}

/****************************************************************************
 * Name: getdblsize
 ****************************************************************************/

#ifdef CONFIG_LIBC_FLOATINGPOINT
static int getdblsize(uint8_t fmt, int trunc, uint8_t flags, double n)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  lib_dtoa(&nulloutstream, fmt, trunc, flags, n);
  return nulloutstream.nput;
}
#endif
#endif /* CONFIG_NOPRINTF_FIELDWIDTH */

#ifdef CONFIG_LONG_IS_NOT_INT
/****************************************************************************
 * Name: lutodec
 ****************************************************************************/

static void lutodec(FAR struct lib_outstream_s *obj, unsigned long n)
{
  unsigned int  remainder = n % 10;
  unsigned long dividend  = n / 10;

  if (dividend)
    {
      lutodec(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: lutohex
 ****************************************************************************/

static void lutohex(FAR struct lib_outstream_s *obj, unsigned long n, uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8*sizeof(unsigned long); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, nibble + '0');
            }
          else
            {
              obj->put(obj, nibble + a - 10);
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: lutooct
 ****************************************************************************/

static void lutooct(FAR struct lib_outstream_s *obj, unsigned long n)
{
  unsigned int  remainder = n & 0x7;
  unsigned long dividend  = n >> 3;

  if (dividend)
    {
      lutooct(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: lutobin
 ****************************************************************************/

static void lutobin(FAR struct lib_outstream_s *obj, unsigned long n)
{
  unsigned int  remainder = n & 1;
  unsigned long dividend  = n >> 1;

  if (dividend)
    {
      lutobin(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: lutoascii
 ****************************************************************************/

static void lutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt, uint8_t flags, unsigned long ln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if ((long)ln < 0)
            {
              obj->put(obj, '-');
              ln    = (unsigned long)(-(long)ln);
            }
          else if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          lutodec(obj, ln);
        }
        break;

      case 'u':
        /* Unigned base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          lutodec(obj, ln);
        }
        break;

      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              obj->put(obj, '0');
              obj->put(obj, 'x');
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              lutohex(obj, ln, 'A');
            }
          else
            {
              lutohex(obj, ln, 'a');
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           lutooct(obj, ln);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          lutobin(obj, ln);
        }
        break;

      case 'p':
      default:
        break;
    }
}

/****************************************************************************
 * Name: lfixup
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void lfixup(uint8_t fmt, FAR uint8_t *flags, FAR long *ln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*ln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *ln    = -*ln;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getlusize
 ****************************************************************************/

static int getlusize(uint8_t fmt, uint8_t flags, unsigned long ln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);

  lutoascii(&nulloutstream, fmt, flags, ln);
  return nulloutstream.nput;
}

#endif /* CONFIG_NOPRINTF_FIELDWIDTH */
#endif /* CONFIG_LONG_IS_NOT_INT */

#ifdef CONFIG_HAVE_LONG_LONG
/****************************************************************************
 * Name: llutodec
 ****************************************************************************/

static void llutodec(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  unsigned int remainder = n % 10;
  unsigned long long dividend = n / 10;

  if (dividend)
    {
      llutodec(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: llutohex
 ****************************************************************************/

static void llutohex(FAR struct lib_outstream_s *obj, unsigned long long n, uint8_t a)
{
  bool    nonzero = false;
  uint8_t bits;

  for (bits = 8*sizeof(unsigned long long); bits > 0; bits -= 4)
    {
      uint8_t nibble = (uint8_t)((n >> (bits - 4)) & 0xf);
      if (nibble || nonzero)
        {
          nonzero = true;

          if (nibble < 10)
            {
              obj->put(obj, (nibble + '0'));
            }
          else
            {
              obj->put(obj, (nibble + a - 10));
            }
        }
    }

  if (!nonzero)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Name: llutooct
 ****************************************************************************/

static void llutooct(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  unsigned int remainder = n & 0x7;
  unsigned long long dividend = n >> 3;

  if (dividend)
    {
      llutooct(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: llutobin
 ****************************************************************************/

static void llutobin(FAR struct lib_outstream_s *obj, unsigned long long n)
{
  unsigned int remainder = n & 1;
  unsigned long long dividend = n >> 1;

  if (dividend)
    {
      llutobin(obj, dividend);
    }

  obj->put(obj, (remainder + (unsigned int)'0'));
}

/****************************************************************************
 * Name: llutoascii
 ****************************************************************************/

static void llutoascii(FAR struct lib_outstream_s *obj, uint8_t fmt, uint8_t flags, unsigned long long lln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if ((long long)lln < 0)
            {
              obj->put(obj, '-');
              lln    = (unsigned long long)(-(long long)lln);
            }
          else if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          llutodec(obj, (unsigned long long)lln);
        }
        break;

      case 'u':
        /* Unigned base 10 */
        {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
          if (IS_SHOWPLUS(flags))
            {
              obj->put(obj, '+');
            }
#endif
          /* Convert the unsigned value to a string. */

          llutodec(obj, (unsigned long long)lln);
        }
        break;

      case 'x':
      case 'X':
        /* Hexadecimal */
        {
          /* Check for alternate form */

          if (IS_ALTFORM(flags))
            {
              /* Prefix the number with "0x" */

              obj->put(obj, '0');
              obj->put(obj, 'x');
            }

          /* Convert the unsigned value to a string. */

          if (fmt == 'X')
            {
              llutohex(obj, (unsigned long long)lln, 'A');
            }
          else
            {
              llutohex(obj, (unsigned long long)lln, 'a');
            }
        }
        break;

      case 'o':
        /* Octal */
         {
           /* Check for alternate form */

           if (IS_ALTFORM(flags))
             {
               /* Prefix the number with '0' */

               obj->put(obj, '0');
             }

           /* Convert the unsigned value to a string. */

           llutooct(obj, (unsigned long long)lln);
         }
         break;

      case 'b':
        /* Binary */
        {
          /* Convert the unsigned value to a string. */

          llutobin(obj, (unsigned long long)lln);
        }
        break;

      case 'p':
      default:
        break;
    }
}

/****************************************************************************
 * Name: llfixup
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void llfixup(uint8_t fmt, FAR uint8_t *flags, FAR long long *lln)
{
  /* Perform the integer conversion according to the format specifier */

  switch (fmt)
    {
      case 'd':
      case 'i':
        /* Signed base 10 */

        if (*lln < 0)
          {
            SET_NEGATE(*flags);
            CLR_SHOWPLUS(*flags);
            *lln    = -*lln;
          }
        break;

      case 'u':
        /* Unsigned base 10 */
        break;

      case 'p':
      case 'x':
      case 'X':
        /* Hexadecimal */
      case 'o':
        /* Octal */
      case 'b':
        /* Binary */
        CLR_SIGNED(*flags);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: getllusize
 ****************************************************************************/

static int getllusize(uint8_t fmt, uint8_t flags, unsigned long long lln)
{
  struct lib_outstream_s nulloutstream;
  lib_nulloutstream(&nulloutstream);


  llutoascii(&nulloutstream, fmt, flags, lln);
  return nulloutstream.nput;
}

#endif /* CONFIG_NOPRINTF_FIELDWIDTH */
#endif /* CONFIG_HAVE_LONG_LONG */

/****************************************************************************
 * Name: prejustify
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void prejustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                       uint8_t flags, int fieldwidth, int valwidth)
{
  int i;

  switch (fmt)
    {
      default:
      case FMT_RJUST:
        if (IS_SIGNED(flags))
          {
            valwidth++;
          }

        for (i = fieldwidth - valwidth; i > 0; i--)
          {
            obj->put(obj, ' ');
          }

        if (IS_NEGATE(flags))
          {
            obj->put(obj, '-');
          }
        else if (IS_SHOWPLUS(flags))
          {
            obj->put(obj, '+');
          }
        break;

      case FMT_RJUST0:
         if (IS_NEGATE(flags))
          {
            obj->put(obj, '-');
            valwidth++;
          }
        else if (IS_SHOWPLUS(flags))
          {
            obj->put(obj, '+');
            valwidth++;
          }

        for (i = fieldwidth - valwidth; i > 0; i--)
          {
            obj->put(obj, '0');
          }
        break;

      case FMT_LJUST:
         if (IS_NEGATE(flags))
          {
            obj->put(obj, '-');
          }
        else if (IS_SHOWPLUS(flags))
          {
            obj->put(obj, '+');
          }
        break;
    }
}
#endif

/****************************************************************************
 * Name: postjustify
 ****************************************************************************/

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
static void postjustify(FAR struct lib_outstream_s *obj, uint8_t fmt,
                        uint8_t flags, int fieldwidth, int valwidth)
{
  int i;

  /* Apply field justification to the integer value. */

  switch (fmt)
    {
      default:
      case FMT_RJUST:
      case FMT_RJUST0:
        break;

      case FMT_LJUST:
        if (IS_SIGNED(flags))
          {
            valwidth++;
          }

        for (i = fieldwidth - valwidth; i > 0; i--)
          {
            obj->put(obj, ' ');
          }
        break;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * libc/stdio/lib_vsprintf
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *obj, FAR const char *src, va_list ap)
{
  FAR char        *ptmp;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
  int             width;
#ifdef CONFIG_LIBC_FLOATINGPOINT
  int             trunc;
#endif
  uint8_t         fmt;
#endif
  uint8_t         flags;
#ifdef CONFIG_ARCH_ROMGETC
  char            ch;
#endif

  for (FMT_TOP; FMT_CHAR; FMT_BOTTOM)
    {
      /* Just copy regular characters */

      if (FMT_CHAR != '%')
        {
           /* Output the character */

           obj->put(obj, FMT_CHAR);

           /* Flush the buffer if a newline is encountered */

#ifdef CONFIG_STDIO_LINEBUFFER
           if (FMT_CHAR == '\n')
             {
               /* Should return an error on a failure to flush */

               (void)obj->flush(obj);
             }
#endif
           /* Process the next character in the format */

           continue;
        }

      /* We have found a format specifier. Move past it. */

      FMT_NEXT;

      /* Assume defaults */

      flags = 0;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
      fmt   = FMT_RJUST;
      width = 0;
#ifdef CONFIG_LIBC_FLOATINGPOINT
      trunc = 0;
#endif
#endif

      /* Process each format qualifier. */

      for (; FMT_CHAR; FMT_BOTTOM)
        {
          /* Break out of the loop when the format is known. */

          if (strchr("diuxXpobeEfgGlLsc%", FMT_CHAR))
            {
              break;
            }

          /* Check for left justification. */

          else if (FMT_CHAR == '-')
            {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              fmt = FMT_LJUST;
#endif
            }

          /* Check for leading zero fill right justification. */

          else if (FMT_CHAR == '0')
            {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              fmt = FMT_RJUST0;
#endif
            }
#if 0
          /* Center justification. */

          else if (FMT_CHAR == '~')
            {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              fmt = FMT_CENTER;
#endif
            }
#endif

          else if (FMT_CHAR == '*')
            {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              int value = va_arg(ap, int);
              if (IS_HASDOT(flags))
                {
#ifdef CONFIG_LIBC_FLOATINGPOINT
                  trunc = value;
                  SET_HASASTERISKTRUNC(flags);
#endif
                }
              else
                {
                  width = value;
                  SET_HASASTERISKWIDTH(flags);
                }
#endif
            }

          /* Check for field width */

          else if (FMT_CHAR >= '1' && FMT_CHAR <= '9')
            {
#ifdef CONFIG_NOPRINTF_FIELDWIDTH
              do
                {
                  FMT_NEXT;
                }
              while (FMT_CHAR >= '0' && FMT_CHAR <= '9');
#else
              /* Accumulate the field width integer. */

              int n = ((int)(FMT_CHAR)) - (int)'0';
              for (;;)
                {
                  FMT_NEXT;
                  if (FMT_CHAR >= '0' && FMT_CHAR <= '9')
                    {
                      n = 10*n + (((int)(FMT_CHAR)) - (int)'0');
                    }
                  else
                    {
                      break;
                    }
                }

              if (IS_HASDOT(flags))
                {
#ifdef CONFIG_LIBC_FLOATINGPOINT
                  trunc = n;
#endif
                }
              else
                {
                  width = n;
                }
#endif
              /* Back up to the last digit. */

              FMT_PREV;
            }

          /* Check for a decimal point. */

          else if (FMT_CHAR == '.')
            {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              SET_HASDOT(flags);
#endif
            }

          /* Check for leading plus sign. */

          else if (FMT_CHAR == '+')
            {
              SET_SHOWPLUS(flags);
            }

          /* Check for alternate form. */

          else if (FMT_CHAR == '#')
            {
              SET_ALTFORM(flags);
            }
        }

      /* "%%" means that a literal '%' was intended (instead of a format
       * specification).
       */

      if (FMT_CHAR == '%')
        {
          obj->put(obj, '%');
          continue;
        }

      /* Check for the string format. */

      if (FMT_CHAR == 's')
        {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
          int swidth;
#endif
          /* Get the string to output */

          ptmp = va_arg(ap, char *);
          if (!ptmp)
            {
              ptmp = (char*)g_nullstring;
            }

          /* Get the widith of the string and perform right-justification
           * operations.
           */

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
          swidth = strlen(ptmp);
          prejustify(obj, fmt, 0, width, swidth);
#endif
          /* Concatenate the string into the output */

          while (*ptmp)
            {
              obj->put(obj, *ptmp);
              ptmp++;
            }

          /* Perform left-justification operations. */

#ifndef CONFIG_NOPRINTF_FIELDWIDTH
          postjustify(obj, fmt, 0, width, swidth);
#endif
          continue;
        }

      /* Check for the character output */

      else if (FMT_CHAR == 'c')
        {
          /* Just copy the character into the output. */

          int n = va_arg(ap, int);
          obj->put(obj, n);
          continue;
        }

      /* Check for the long long prefix. */

      if (FMT_CHAR == 'L')
        {
           SET_LONGLONGPRECISION(flags);
           FMT_NEXT;
        }
      else if (FMT_CHAR == 'l')
        {
          SET_LONGPRECISION(flags);
          FMT_NEXT;
          if (FMT_CHAR == 'l')
            {
              SET_LONGLONGPRECISION(flags);
              FMT_NEXT;
            }
        }

      /* Handle integer conversions */

      if (strchr("diuxXpob", FMT_CHAR))
        {
#ifdef CONFIG_HAVE_LONG_LONG
          if (IS_LONGLONGPRECISION(flags) && FMT_CHAR != 'p')
            {
              long long lln;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              int lluwidth;
#endif
              /* Extract the long long value. */

              lln = va_arg(ap, long long);

#ifdef CONFIG_NOPRINTF_FIELDWIDTH
              /* Output the number */

              llutoascii(obj, FMT_CHAR, flags, (unsigned long long)lln);
#else
              /* Resolve sign-ness and format issues */

              llfixup(FMT_CHAR, &flags, &lln);

              /* Get the width of the output */

              lluwidth = getllusize(FMT_CHAR, flags, lln);

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, lluwidth);

              /* Output the number */

              llutoascii(obj, FMT_CHAR, flags, (unsigned long long)lln);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, lluwidth);
#endif
            }
          else
#endif /* CONFIG_HAVE_LONG_LONG */
#ifdef CONFIG_LONG_IS_NOT_INT
          if (IS_LONGPRECISION(flags) && FMT_CHAR != 'p')
            {
              long ln;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              int luwidth;
#endif
              /* Extract the long value. */

              ln = va_arg(ap, long);

#ifdef CONFIG_NOPRINTF_FIELDWIDTH
              /* Output the number */

              lutoascii(obj, FMT_CHAR, flags, (unsigned long)ln);
#else
              /* Resolve sign-ness and format issues */

              lfixup(FMT_CHAR, &flags, &ln);

              /* Get the width of the output */

              luwidth = getlusize(FMT_CHAR, flags, ln);

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, luwidth);

              /* Output the number */

              lutoascii(obj, FMT_CHAR, flags, (unsigned long)ln);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, luwidth);
#endif
            }
          else
#endif /* CONFIG_LONG_IS_NOT_INT */
#ifdef CONFIG_PTR_IS_NOT_INT
          if (FMT_CHAR == 'p')
            {
              void *p;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              int pwidth;
#endif
              /* Extract the integer value. */

              p = va_arg(ap, void *);

#ifdef CONFIG_NOPRINTF_FIELDWIDTH
              /* Output the pointer value */

              ptohex(obj, flags, p);
#else
              /* Resolve sign-ness and format issues */

              lfixup(FMT_CHAR, &flags, &ln);

              /* Get the width of the output */

              luwidth = getpsize(FMT_CHAR, flags, p);

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, pwidth);

              /* Output the pointer value */

              ptohex(obj, flags, p);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, pwidth);
#endif
            }
          else
#endif
            {
              int n;
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
              int uwidth;
#endif
              /* Extract the long long value. */

              n = va_arg(ap, int);

#ifdef CONFIG_NOPRINTF_FIELDWIDTH
              /* Output the number */

              utoascii(obj, FMT_CHAR, flags, (unsigned int)n);
#else
              /* Resolve sign-ness and format issues */

              fixup(FMT_CHAR, &flags, &n);

              /* Get the width of the output */

              uwidth = getusize(FMT_CHAR, flags, n);

              /* Perform left field justification actions */

              prejustify(obj, fmt, flags, width, uwidth);

              /* Output the number */

              utoascii(obj, FMT_CHAR, flags, (unsigned int)n);

              /* Perform right field justification actions */

              postjustify(obj, fmt, flags, width, uwidth);
#endif
            }
        }

      /* Handle floating point conversions */

#ifdef CONFIG_LIBC_FLOATINGPOINT
      else if (strchr("eEfgG", FMT_CHAR))
        {
#ifndef CONFIG_NOPRINTF_FIELDWIDTH
          double dblval = va_arg(ap, double);
          int dblsize;

          /* Get the width of the output */

          dblsize = getdblsize(FMT_CHAR, trunc, flags, dblval);

          /* Perform left field justification actions */

          prejustify(obj, fmt, 0, width, dblsize);

          /* Output the number */

          lib_dtoa(obj, FMT_CHAR, trunc, flags, dblval);

          /* Perform right field justification actions */

          postjustify(obj, fmt, 0, width, dblsize);
#else
          /* Output the number with a fixed precision */

          double dblval = va_arg(ap, double);
          lib_dtoa(obj, FMT_CHAR, CONFIG_LIBC_FIXEDPRECISION, flags, dblval);
#endif
        }
#endif /* CONFIG_LIBC_FLOATINGPOINT */
    }

  return obj->nput;
}


