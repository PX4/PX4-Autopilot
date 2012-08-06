/****************************************************************************
 * lib/unistd/lib_libdtoa.c
 *
 * This file was ported to NuttX by Yolande Cates.
 *
 * Copyright (c) 1990, 1993
 *      The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Chris Torek.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by the University of
 *      California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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
 * Name: zeroes
 *
 * Description:
 *   Print the specified number of zeres
 *
 ****************************************************************************/

static void zeroes(FAR struct lib_outstream_s *obj, int nzeroes)
{
  int i;

  for (i = nzeroes; i > 0; i--)
    {
      obj->put(obj, '0');
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_dtoa
 *
 * Description:
 *   This is part of lib_vsprintf().  It handles the floating point formats.
 *   This version supports only the &f (with precision).
 *
 ****************************************************************************/

static void lib_dtoa(FAR struct lib_outstream_s *obj, int fmt, int prec,
                     uint8_t flags, double value)
{
  FAR char *digits;     /* String returned by __dtoa */
  FAR char *digalloc;   /* Copy of digits to be freed after usage */
  FAR char *rve;        /* Points to the end of the return value */
  char sign;            /* Temporary negative sign for floats */
  int  expt;            /* Integer value of exponent */
  int  numlen;          /* Actual number of digits returned by cvt */
  int  nchars;          /* Number of characters to print */
  int  dsgn;            /* Unused sign indicator */
  int  i;

  /* Non-zero... positive or negative */

  if (value < 0)
    {
      value = -value;
      sign = '-';
    }
  else
    {
      sign = '\0';
    }

  /* Perform the conversion */

  digits   = __dtoa(value, 3, prec, &expt, &dsgn, &rve);
  digalloc = digits;
  numlen   = rve - digits;

  if (sign)
    {
      obj->put(obj, '-');
    }

   /* Always print at least one digit to the right of the decimal point. */

   prec = MAX(1, prec);

  /* Special case exact zero or the case where the number is smaller than
   * the print precision.
   */

  if (value == 0 || expt < -prec)
    {
      /* kludge for __dtoa irregularity */

      obj->put(obj, '0');
      obj->put(obj, '.');
    }
  else if (expt <= 0)
    {
      obj->put(obj, '0');
      obj->put(obj, '.');

      /* Print leading zeros */

      if (expt < 0)
        {
          nchars = MIN(-expt, prec);
          zeroes(obj, nchars);
          prec -= nchars;
        }
                  
      /* Print the significant digits */

      nchars = MIN(numlen, prec);
      for (i = nchars; i > 0; i--)
        {
          obj->put(obj, *digits);
          digits++;
        }

      /* Decremnt to get the number of trailing zeroes to print */

      prec -= nchars;
    }
  else
    {
      /* Print the integer part */

      for (i = expt; i > 0; i--)
        {
          obj->put(obj, *digits);
          digits++;
        }

      /* Print the decimal place */

      obj->put(obj, '.');

      /* Print the decimal */

      numlen -= expt;
      nchars = MIN(numlen, prec);

      for (i = nchars; i > 0; i--)
        {
          obj->put(obj, *digits);
          digits++;
        }

      /* Decremnt to get the number of trailing zeroes to print */

      prec -= nchars;
    }

  /* Finally, print any trailing zeroes */

  zeroes(obj, prec);

  /* Is this memory supposed to be freed or not? */

#if 0
  if (digalloc)
    {
      free(digalloc);
    }
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

