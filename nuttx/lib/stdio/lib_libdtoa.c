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

#define MAXEXP 308

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static char* cvt(double value, int ndigits, int flags, char *sign,
                 int *decpt, int ch, int *length);
static int   exponent(char *p0, int exp, int fmtch);

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cvt
 ****************************************************************************/

static char* cvt(double value, int ndigits, int flags, char *sign,
                 int *decpt, int ch, int *length)
{
  int mode, dsgn;
  char *digits, *bp, *rve;

  if (ch == 'f')
    {
      mode = 3;               /* ndigits after the decimal point */
    }
  else
    {
      /* To obtain ndigits after the decimal point for the 'e' and 'E'
       * formats, round to ndigits + 1 significant figures.
       */

      if (ch == 'e' || ch == 'E')
        {
          ndigits++;
        }
      mode = 2;               /* ndigits significant digits */
    }

  if (value < 0)
    {
      value = -value;
      *sign = '-';
    }
  else
    {
      *sign = '\000';
    }

  digits = __dtoa(value, mode, ndigits, decpt, &dsgn, &rve);
  if ((ch != 'g' && ch != 'G') || IS_ALTFORM(flags))
    {
      /* Print trailing zeros */

      bp = digits + ndigits;
      if (ch == 'f')
        {
          if (*digits == '0' && value)
            {
              *decpt = -ndigits + 1;
            }
          bp += *decpt;
        }

      if (value == 0)
        {
	  /* kludge for __dtoa irregularity */

          rve = bp;
        }

      while (rve < bp)
        {
          *rve++ = '0';
        }
    }

  *length = rve - digits;
  return digits;
}

/****************************************************************************
 * Name: exponent
 ****************************************************************************/

static int exponent(FAR char *p0, int exp, int fmtch)
{
  FAR char *p;
  FAR char *t;
  char expbuf[MAXEXP];

  p = p0;
  *p++ = fmtch;
  if (exp < 0)
    {
      exp = -exp;
      *p++ = '-';
    }
  else
    {
      *p++ = '+';
    }

  t = expbuf + MAXEXP;
  if (exp > 9)
    {
      do
        {
          *--t = (exp % 10) + '0';
        }
      while ((exp /= 10) > 9);
      *--t = exp + '0';
      for (; t < expbuf + MAXEXP; *p++ = *t++);
    }
  else
    {
      *p++ = '0';
      *p++ = exp + '0';
    }
  return (p - p0);
}

/****************************************************************************
 * Name: lib_dtoa
 *
 * Description:
 *   This is part of lib_vsprintf().  It handles the floating point formats.
 *
 ****************************************************************************/

static void lib_dtoa(FAR struct lib_outstream_s *obj, int ch, int prec,
                     uint8_t flags, double _double)
{
  FAR char *cp;              /* Handy char pointer (short term usage) */
  FAR char *cp_free = NULL;  /* BIONIC: copy of cp to be freed after usage */
  char expstr[7];            /* Buffer for exponent string */
  char sign;                 /* Temporary negative sign for floats */
  int  expt;                 /* Integer value of exponent */
  int  expsize = 0;          /* Character count for expstr */
  int  ndig;                 /* Actual number of digits returned by cvt */
  int  size;                 /* Size of converted field or string */
  int  i;

  cp = cvt(_double, prec, flags, &sign, &expt, ch, &ndig);
  cp_free = cp;

  if (ch == 'g' || ch == 'G')
    {
      /* 'g' or 'G' fmt */

      if (expt <= -4 || expt > prec)
        {
          ch = (ch == 'g') ? 'e' : 'E';
        }
      else
        {
          ch = 'g';
        }
    }

  if (ch <= 'e')
    {
      /* 'e' or 'E' fmt */

      --expt;
      expsize = exponent(expstr, expt, ch);
      size = expsize + ndig;
      if (ndig > 1 || IS_ALTFORM(flags))
        {
          ++size;
        }
    }
  else if (ch == 'f')
    {
      /* f fmt */

      if (expt > 0)
        {
          size = expt;
          if (prec || IS_ALTFORM(flags))
            {
              size += prec + 1;
            }
        }
      else  /* "0.X" */
        {
            size = prec + 2;
        }
    }
  else if (expt >= ndig)
    {
      /* fixed g fmt */

      size = expt;
      if (IS_ALTFORM(flags))
        {
          ++size;
        }
    }
  else
    {
      size = ndig + (expt > 0 ? 1 : 2 - expt);
    }

  if (sign)
    {
      obj->put(obj, '-');
    }

  if (_double == 0)
    {
      /* kludge for __dtoa irregularity */

      obj->put(obj, '0');
      if (expt < ndig || IS_ALTFORM(flags))
        {
          obj->put(obj, '.');

          i = ndig - 1;
          while (i > 0)
            {
              obj->put(obj, '0');
              i--;
            }
        }
    }
  else if (expt <= 0)
    {
      obj->put(obj, '0');
      obj->put(obj, '.');

      i = ndig;
      while (i > 0)
        {
          obj->put(obj, *cp);
          i--;
          cp++;
        }
    }
  else if (expt >= ndig)
    {
      i = ndig;
      while (i > 0)
        {
          obj->put(obj, *cp);
          i--;
          cp++;
        }

      i = expt - ndig;
      while (i > 0)
        {
          obj->put(obj, '0');
          i--;
        }

      if (IS_ALTFORM(flags))
        {
          obj->put(obj, '.');
        }
    }
  else
    {
      /* print the integer */

      i = expt;
      while (i > 0)
        {
          obj->put(obj, *cp);
          i--;
          cp++;
        }

      /* print the decimal place */

      obj->put(obj, '.');

      /* print the decimal */

      i = ndig - expt;
      while (i > 0)
        {
          obj->put(obj, *cp);
          i--;
          cp++;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

