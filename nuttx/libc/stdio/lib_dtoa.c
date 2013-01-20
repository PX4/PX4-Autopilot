/****************************************************************************
 * libc/stdio/lib_dtoa.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef Unsigned_Shifts
#  define Sign_Extend(a,b) if (b < 0) a |= 0xffff0000;
#else
#  define Sign_Extend(a,b)      /* no-op */
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define word0(x) ((uint32_t *)&x)[0]
#  define word1(x) ((uint32_t *)&x)[1]
#else
#  define word0(x) ((uint32_t *)&x)[1]
#  define word1(x) ((uint32_t *)&x)[0]
#endif

#ifdef CONFIG_ENDIAN_BIG
#  define Storeinc(a,b,c) (((unsigned short *)a)[0] = (unsigned short)b, \
                         ((unsigned short *)a)[1] = (unsigned short)c, a++)
#else
#  define Storeinc(a,b,c) (((unsigned short *)a)[1] = (unsigned short)b, \
                         ((unsigned short *)a)[0] = (unsigned short)c, a++)
#endif

#define Exp_shift  20
#define Exp_shift1 20
#define Exp_msk1    0x100000
#define Exp_msk11   0x100000
#define Exp_mask  0x7ff00000
#define P 53
#define Bias 1023
#define IEEE_Arith
#define Emin (-1022)
#define Exp_1  0x3ff00000
#define Exp_11 0x3ff00000
#define Ebits 11
#define Frac_mask  0xfffff
#define Frac_mask1 0xfffff
#define Ten_pmax 22
#define Bletch 0x10
#define Bndry_mask  0xfffff
#define Bndry_mask1 0xfffff
#define LSB 1
#define Sign_bit 0x80000000
#define Log2P 1
#define Tiny0 0
#define Tiny1 1
#define Quick_max 14
#define Int_max 14
#define Infinite(x) (word0(x) == 0x7ff00000)    /* sufficient test for here */

#define Kmax 15

#define Bcopy(x,y) memcpy((char *)&x->sign, (char *)&y->sign, \
                          y->wds*sizeof(long) + 2*sizeof(int))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct Bigint
{
  struct Bigint *next;
  int k, maxwds, sign, wds;
  unsigned long x[1];
};

typedef struct Bigint Bigint;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static Bigint *freelist[Kmax + 1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static Bigint *Balloc(int k)
{
  int x;
  Bigint *rv;

  if ((rv = freelist[k]))
    {
      freelist[k] = rv->next;
    }
  else
    {
      x = 1 << k;
      rv = (Bigint *)lib_malloc(sizeof(Bigint) + (x - 1) * sizeof(long));
      rv->k = k;
      rv->maxwds = x;
    }
  rv->sign = rv->wds = 0;
  return rv;
}

static void Bfree(Bigint * v)
{
  if (v)
    {
      v->next = freelist[v->k];
      freelist[v->k] = v;
    }
}

/* multiply by m and add a */

static Bigint *multadd(Bigint * b, int m, int a)
{
  int i, wds;
  unsigned long *x, y;
#ifdef Pack_32
  unsigned long xi, z;
#endif
  Bigint *b1;

  wds = b->wds;
  x = b->x;
  i = 0;
  do
    {
#ifdef Pack_32
      xi = *x;
      y = (xi & 0xffff) * m + a;
      z = (xi >> 16) * m + (y >> 16);
      a = (int)(z >> 16);
      *x++ = (z << 16) + (y & 0xffff);
#else
      y = *x * m + a;
      a = (int)(y >> 16);
      *x++ = y & 0xffff;
#endif
    }
  while (++i < wds);
  if (a)
    {
      if (wds >= b->maxwds)
        {
          b1 = Balloc(b->k + 1);
          Bcopy(b1, b);
          Bfree(b);
          b = b1;
        }
      b->x[wds++] = a;
      b->wds = wds;
    }
  return b;
}

static int hi0bits(unsigned long x)
{
  int k = 0;

  if (!(x & 0xffff0000))
    {
      k = 16;
      x <<= 16;
    }

  if (!(x & 0xff000000))
    {
      k += 8;
      x <<= 8;
    }

  if (!(x & 0xf0000000))
    {
      k += 4;
      x <<= 4;
    }

  if (!(x & 0xc0000000))
    {
      k += 2;
      x <<= 2;
    }

  if (!(x & 0x80000000))
    {
      k++;
      if (!(x & 0x40000000))
        {
          return 32;
        }
    }
  return k;
}

static int lo0bits(unsigned long *y)
{
  int k;
  unsigned long x = *y;

  if (x & 7)
    {
      if (x & 1)
        {
          return 0;
        }
      if (x & 2)
        {
          *y = x >> 1;
          return 1;
        }
      *y = x >> 2;
      return 2;
    }

  k = 0;
  if (!(x & 0xffff))
    {
      k = 16;
      x >>= 16;
    }

  if (!(x & 0xff))
    {
      k += 8;
      x >>= 8;
    }

  if (!(x & 0xf))
    {
      k += 4;
      x >>= 4;
    }

  if (!(x & 0x3))
    {
      k += 2;
      x >>= 2;
    }

  if (!(x & 1))
    {
      k++;
      x >>= 1;
      if (!x & 1)
        {
          return 32;
        }
    }
  *y = x;
  return k;
}

static Bigint *i2b(int i)
{
  Bigint *b;

  b = Balloc(1);
  b->x[0] = i;
  b->wds = 1;
  return b;
}

static Bigint *mult(Bigint * a, Bigint * b)
{
  Bigint *c;
  int k, wa, wb, wc;
  unsigned long carry, y, z;
  unsigned long *x, *xa, *xae, *xb, *xbe, *xc, *xc0;
#ifdef Pack_32
  uint32_t z2;
#endif

  if (a->wds < b->wds)
    {
      c = a;
      a = b;
      b = c;
    }

  k = a->k;
  wa = a->wds;
  wb = b->wds;
  wc = wa + wb;
  if (wc > a->maxwds)
    {
      k++;
    }
  c = Balloc(k);
  for (x = c->x, xa = x + wc; x < xa; x++)
    {
      *x = 0;
    }
  xa = a->x;
  xae = xa + wa;
  xb = b->x;
  xbe = xb + wb;
  xc0 = c->x;
#ifdef Pack_32
  for (; xb < xbe; xb++, xc0++)
    {
      if ((y = *xb & 0xffff))
        {
          x = xa;
          xc = xc0;
          carry = 0;
          do
            {
              z = (*x & 0xffff) * y + (*xc & 0xffff) + carry;
              carry = z >> 16;
              z2 = (*x++ >> 16) * y + (*xc >> 16) + carry;
              carry = z2 >> 16;
              Storeinc(xc, z2, z);
            }
          while (x < xae);
          *xc = carry;
        }
      if ((y = *xb >> 16))
        {
          x = xa;
          xc = xc0;
          carry = 0;
          z2 = *xc;
          do
            {
              z = (*x & 0xffff) * y + (*xc >> 16) + carry;
              carry = z >> 16;
              Storeinc(xc, z, z2);
              z2 = (*x++ >> 16) * y + (*xc & 0xffff) + carry;
              carry = z2 >> 16;
            }
          while (x < xae);
          *xc = z2;
        }
    }
#else
  for (; xb < xbe; xc0++)
    {
      if ((y = *xb++))
        {
          x = xa;
          xc = xc0;
          carry = 0;
          do
            {
              z = *x++ * y + *xc + carry;
              carry = z >> 16;
              *xc++ = z & 0xffff;
            }
          while (x < xae);
          *xc = carry;
        }
    }
#endif
  for (xc0 = c->x, xc = xc0 + wc; wc > 0 && !*--xc; --wc);
  c->wds = wc;
  return c;
}

static Bigint *p5s;

static Bigint *pow5mult(Bigint * b, int k)
{
  Bigint *b1, *p5, *p51;
  int i;
  static int p05[3] = { 5, 25, 125 };

  if ((i = k & 3))
    b = multadd(b, p05[i - 1], 0);

  if (!(k >>= 2))
    {
      return b;
    }

  if (!(p5 = p5s))
    {
      /* first time */
      p5 = p5s = i2b(625);
      p5->next = 0;
    }

  for (;;)
    {
      if (k & 1)
        {
          b1 = mult(b, p5);
          Bfree(b);
          b = b1;
        }
      if (!(k >>= 1))
        {
          break;
        }

      if (!(p51 = p5->next))
        {
          p51 = p5->next = mult(p5, p5);
          p51->next = 0;
        }
      p5 = p51;
    }
  return b;
}

static Bigint *lshift(Bigint * b, int k)
{
  int i, k1, n, n1;
  Bigint *b1;
  unsigned long *x, *x1, *xe, z;

#ifdef Pack_32
  n = k >> 5;
#else
  n = k >> 4;
#endif
  k1 = b->k;
  n1 = n + b->wds + 1;
  for (i = b->maxwds; n1 > i; i <<= 1)
    {
      k1++;
    }
  b1 = Balloc(k1);
  x1 = b1->x;
  for (i = 0; i < n; i++)
    {
      *x1++ = 0;
    }
  x = b->x;
  xe = x + b->wds;
#ifdef Pack_32
  if (k &= 0x1f)
    {
      k1 = 32 - k;
      z = 0;
      do
        {
          *x1++ = *x << k | z;
          z = *x++ >> k1;
        }
      while (x < xe);
      if ((*x1 = z))
        {
          ++n1;
        }
    }
#else
  if (k &= 0xf)
    {
      k1 = 16 - k;
      z = 0;
      do
        {
          *x1++ = ((*x << k) & 0xffff) | z;
          z = *x++ >> k1;
        }
      while (x < xe);
      if ((*x1 = z))
        {
          ++n1;
        }
    }
#endif
  else
    do
      {
        *x1++ = *x++;
      }
    while (x < xe);
  b1->wds = n1 - 1;
  Bfree(b);
  return b1;
}

static int cmp(Bigint * a, Bigint * b)
{
  unsigned long *xa, *xa0, *xb, *xb0;
  int i, j;

  i = a->wds;
  j = b->wds;
#ifdef CONFIG_DEBUG_LIB
  if (i > 1 && !a->x[i - 1])
   {
    ldbg("cmp called with a->x[a->wds-1] == 0\n");
   }
  if (j > 1 && !b->x[j - 1])
   {
    ldbg("cmp called with b->x[b->wds-1] == 0\n");
   }
#endif
  if (i -= j)
    return i;
  xa0 = a->x;
  xa = xa0 + j;
  xb0 = b->x;
  xb = xb0 + j;
  for (;;)
    {
      if (*--xa != *--xb)
        return *xa < *xb ? -1 : 1;
      if (xa <= xa0)
        break;
    }
  return 0;
}

static Bigint *diff(Bigint * a, Bigint * b)
{
  Bigint *c;
  int i, wa, wb;
  long borrow, y;               /* We need signed shifts here. */
  unsigned long *xa, *xae, *xb, *xbe, *xc;
#ifdef Pack_32
  int32_t z;
#endif

  i = cmp(a, b);
  if (!i)
    {
      c = Balloc(0);
      c->wds = 1;
      c->x[0] = 0;
      return c;
    }
  if (i < 0)
    {
      c = a;
      a = b;
      b = c;
      i = 1;
    }
  else
    i = 0;
  c = Balloc(a->k);
  c->sign = i;
  wa = a->wds;
  xa = a->x;
  xae = xa + wa;
  wb = b->wds;
  xb = b->x;
  xbe = xb + wb;
  xc = c->x;
  borrow = 0;
#ifdef Pack_32
  do
    {
      y = (*xa & 0xffff) - (*xb & 0xffff) + borrow;
      borrow = y >> 16;
      Sign_Extend(borrow, y);
      z = (*xa++ >> 16) - (*xb++ >> 16) + borrow;
      borrow = z >> 16;
      Sign_Extend(borrow, z);
      Storeinc(xc, z, y);
    }
  while (xb < xbe);
  while (xa < xae)
    {
      y = (*xa & 0xffff) + borrow;
      borrow = y >> 16;
      Sign_Extend(borrow, y);
      z = (*xa++ >> 16) + borrow;
      borrow = z >> 16;
      Sign_Extend(borrow, z);
      Storeinc(xc, z, y);
    }
#else
  do
    {
      y = *xa++ - *xb++ + borrow;
      borrow = y >> 16;
      Sign_Extend(borrow, y);
      *xc++ = y & 0xffff;
    }
  while (xb < xbe);
  while (xa < xae)
    {
      y = *xa++ + borrow;
      borrow = y >> 16;
      Sign_Extend(borrow, y);
      *xc++ = y & 0xffff;
    }
#endif
  while (!*--xc)
    wa--;
  c->wds = wa;
  return c;
}

static Bigint *d2b(double d, int *e, int *bits)
{
  Bigint *b;
  int de, i, k;
  unsigned long *x, y, z;

#ifdef Pack_32
  b = Balloc(1);
#else
  b = Balloc(2);
#endif
  x = b->x;

  z = word0(d) & Frac_mask;
  word0(d) &= 0x7fffffff;       /* clear sign bit, which we ignore */
  if ((de = (int)(word0(d) >> Exp_shift)))
    z |= Exp_msk1;
#ifdef Pack_32
  if ((y = word1(d)))
    {
      if ((k = lo0bits(&y)))
        {
          x[0] = y | z << (32 - k);
          z >>= k;
        }
      else
        x[0] = y;
      i = b->wds = (x[1] = z) ? 2 : 1;
    }
  else
    {
#ifdef CONFIG_DEBUG_LIB
      if (!z)
        {
          ldbg("Zero passed to d2b\n");
        }
#endif
      k = lo0bits(&z);
      x[0] = z;
      i = b->wds = 1;
      k += 32;
    }
#else
  if ((y = word1(d)))
    {
      if ((k = lo0bits(&y)))
        if (k >= 16)
          {
            x[0] = y | ((z << (32 - k)) & 0xffff);
            x[1] = z >> (k - 16) & 0xffff;
            x[2] = z >> k;
            i = 2;
          }
        else
          {
            x[0] = y & 0xffff;
            x[1] = (y >> 16) | ((z << (16 - k)) & 0xffff);
            x[2] = z >> k & 0xffff;
            x[3] = z >> (k + 16);
            i = 3;
          }
      else
        {
          x[0] = y & 0xffff;
          x[1] = y >> 16;
          x[2] = z & 0xffff;
          x[3] = z >> 16;
          i = 3;
        }
    }
  else
    {
#ifdef CONFIG_DEBUG_LIB
      if (!z)
        {
          ldbg("Zero passed to d2b\n");
        }
#endif
      k = lo0bits(&z);
      if (k >= 16)
        {
          x[0] = z;
          i = 0;
        }
      else
        {
          x[0] = z & 0xffff;
          x[1] = z >> 16;
          i = 1;
        }
      k += 32;
    }
  while (!x[i])
    --i;
  b->wds = i + 1;
#endif
  if (de)
    {
      *e = de - Bias - (P - 1) + k;
      *bits = P - k;
    }
  else
    {
      *e = de - Bias - (P - 1) + 1 + k;
#ifdef Pack_32
      *bits = 32 * i - hi0bits(x[i - 1]);
#else
      *bits = (i + 2) * 16 - hi0bits(x[i]);
#endif
    }
  return b;
}

static const double tens[] = {
  1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9,
  1e10, 1e11, 1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19,
  1e20, 1e21, 1e22
};

#ifdef IEEE_Arith
static const double bigtens[] = { 1e16, 1e32, 1e64, 1e128, 1e256 };
static const double tinytens[] = { 1e-16, 1e-32, 1e-64, 1e-128, 1e-256 };

#  define n_bigtens 5
#else
static const double bigtens[] = { 1e16, 1e32 };
static const double tinytens[] = { 1e-16, 1e-32 };

#  define n_bigtens 2
#endif

static int quorem(Bigint * b, Bigint * S)
{
  int n;
  long borrow, y;
  unsigned long carry, q, ys;
  unsigned long *bx, *bxe, *sx, *sxe;
#ifdef Pack_32
  int32_t z;
  uint32_t si, zs;
#endif

  n = S->wds;
#ifdef CONFIG_DEBUG_LIB
  if (b->wds > n)
    {
      ldbg("oversize b in quorem\n");
    }
#endif
  if (b->wds < n)
    {
      return 0;
    }
  sx = S->x;
  sxe = sx + --n;
  bx = b->x;
  bxe = bx + n;
  q = *bxe / (*sxe + 1);        /* ensure q <= true quotient */
#ifdef CONFIG_DEBUG_LIB
  if (q > 9)
   {
     ldbg("oversized quotient in quorem\n");
   }
#endif
  if (q)
    {
      borrow = 0;
      carry = 0;
      do
        {
#ifdef Pack_32
          si = *sx++;
          ys = (si & 0xffff) * q + carry;
          zs = (si >> 16) * q + (ys >> 16);
          carry = zs >> 16;
          y = (*bx & 0xffff) - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          Sign_Extend(borrow, y);
          z = (*bx >> 16) - (zs & 0xffff) + borrow;
          borrow = z >> 16;
          Sign_Extend(borrow, z);
          Storeinc(bx, z, y);
#else
          ys = *sx++ * q + carry;
          carry = ys >> 16;
          y = *bx - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          Sign_Extend(borrow, y);
          *bx++ = y & 0xffff;
#endif
        }
      while (sx <= sxe);
      if (!*bxe)
        {
          bx = b->x;
          while (--bxe > bx && !*bxe)
            --n;
          b->wds = n;
        }
    }
  if (cmp(b, S) >= 0)
    {
      q++;
      borrow = 0;
      carry = 0;
      bx = b->x;
      sx = S->x;
      do
        {
#ifdef Pack_32
          si = *sx++;
          ys = (si & 0xffff) + carry;
          zs = (si >> 16) + (ys >> 16);
          carry = zs >> 16;
          y = (*bx & 0xffff) - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          Sign_Extend(borrow, y);
          z = (*bx >> 16) - (zs & 0xffff) + borrow;
          borrow = z >> 16;
          Sign_Extend(borrow, z);
          Storeinc(bx, z, y);
#else
          ys = *sx++ + carry;
          carry = ys >> 16;
          y = *bx - (ys & 0xffff) + borrow;
          borrow = y >> 16;
          Sign_Extend(borrow, y);
          *bx++ = y & 0xffff;
#endif
        }
      while (sx <= sxe);
      bx = b->x;
      bxe = bx + n;
      if (!*bxe)
        {
          while (--bxe > bx && !*bxe)
            --n;
          b->wds = n;
        }
    }
  return q;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* dtoa for IEEE arithmetic (dmg): convert double to ASCII string.
 *
 * Inspired by "How to Print Floating-Point Numbers Accurately" by
 * Guy L. Steele, Jr. and Jon L. White [Proc. ACM SIGPLAN '90, pp. 92-101].
 *
 * Modifications:
 *      1. Rather than iterating, we use a simple numeric overestimate
 *         to determine k = floor(log10(d)).  We scale relevant
 *         quantities using O(log2(k)) rather than O(k) multiplications.
 *      2. For some modes > 2 (corresponding to ecvt and fcvt), we don't
 *         try to generate digits strictly left to right.  Instead, we
 *         compute with fewer bits and propagate the carry if necessary
 *         when rounding the final digit up.  This is often faster.
 *      3. Under the assumption that input will be rounded nearest,
 *         mode 0 renders 1e23 as 1e23 rather than 9.999999999999999e22.
 *         That is, we allow equality in stopping tests when the
 *         round-nearest rule will give the same floating-point value
 *         as would satisfaction of the stopping test with strict
 *         inequality.
 *      4. We remove common factors of powers of 2 from relevant
 *         quantities.
 *      5. When converting floating-point integers less than 1e16,
 *         we use floating-point arithmetic rather than resorting
 *         to multiple-precision integers.
 *      6. When asked to produce fewer than 15 digits, we first try
 *         to get by with floating-point arithmetic; we resort to
 *         multiple-precision integer arithmetic only if we cannot
 *         guarantee that the floating-point calculation has given
 *         the correctly rounded result.  For k requested digits and
 *         "uniformly" distributed input, the probability is
 *         something like 10^(k-15) that we must resort to the int32_t
 *         calculation.
 */

char *__dtoa(double d, int mode, int ndigits, int *decpt, int *sign, char **rve)
{
  /* Arguments ndigits, decpt, sign are similar to those of ecvt and fcvt;
   * trailing zeros are suppressed from the returned string.  If not null, *rve 
   * is set to point to the end of the return value.  If d is +-Infinity or
   * NaN, then *decpt is set to 9999.
   * 
   * mode: 0 ==> shortest string that yields d when read in and rounded to
   * nearest. 1 ==> like 0, but with Steele & White stopping rule; e.g. with
   * IEEE P754 arithmetic , mode 0 gives 1e23 whereas mode 1 gives
   * 9.999999999999999e22. 2 ==> max(1,ndigits) significant digits.  This gives 
   * a return value similar to that of ecvt, except that trailing zeros are
   * suppressed. 3 ==> through ndigits past the decimal point.  This gives a
   * return value similar to that from fcvt, except that trailing zeros are
   * suppressed, and ndigits can be negative. 4-9 should give the same return
   * values as 2-3, i.e., 4 <= mode <= 9 ==> same return as mode 2 + (mode &
   * 1).  These modes are mainly for debugging; often they run slower but
   * sometimes faster than modes 2-3. 4,5,8,9 ==> left-to-right digit
   * generation. 6-9 ==> don't try fast floating-point estimate (if
   * applicable).
   * 
   * Values of mode other than 0-9 are treated as mode 0.
   * 
   * Sufficient space is allocated to the return value to hold the suppressed
   * trailing zeros. */

  int bbits, b2, b5, be, dig, i, ieps, ilim = 0, ilim0, ilim1 = 0,
    j, j_1, k, k0, k_check, leftright, m2, m5, s2, s5, spec_case = 0, try_quick;
  long L;
  int denorm;
  unsigned long x;
  Bigint *b, *b1, *delta, *mlo = NULL, *mhi, *S;
  double d2, ds, eps;
  char *s, *s0;
  static Bigint *result;
  static int result_k;

  if (result)
    {
      result->k = result_k;
      result->maxwds = 1 << result_k;
      Bfree(result);
      result = 0;
    }

  if (word0(d) & Sign_bit)
    {
      /* set sign for everything, including 0's and NaNs */
      *sign = 1;
      word0(d) &= ~Sign_bit;    /* clear sign bit */
    }
  else
    {
      *sign = 0;
    }

#if defined(IEEE_Arith)
#  ifdef IEEE_Arith
  if ((word0(d) & Exp_mask) == Exp_mask)
#else
  if (word0(d) == 0x8000)
#endif
    {
      /* Infinity or NaN */
      *decpt = 9999;
      s =
#ifdef IEEE_Arith
        !word1(d) && !(word0(d) & 0xfffff) ? "Infinity" :
#endif
        "NaN";
      if (rve)
        *rve =
#ifdef IEEE_Arith
          s[3] ? s + 8 :
#endif
          s + 3;
      return s;
    }
#endif
  if (!d)
    {
      *decpt = 1;
      s = "0";
      if (rve)
        *rve = s + 1;
      return s;
    }

  b = d2b(d, &be, &bbits);
  if ((i = (int)(word0(d) >> Exp_shift1 & (Exp_mask >> Exp_shift1))))
    {
      d2 = d;
      word0(d2) &= Frac_mask1;
      word0(d2) |= Exp_11;

      /* log(x) ~=~ log(1.5) + (x-1.5)/1.5 log10(x) = log(x) / log(10) ~=~
       * log(1.5)/log(10) + (x-1.5)/(1.5*log(10)) log10(d) =
       * (i-Bias)*log(2)/log(10) + log10(d2) This suggests computing an
       * approximation k to log10(d) by k = (i - Bias)*0.301029995663981 + (
       * (d2-1.5)*0.289529654602168 + 0.176091259055681 ); We want k to be too 
       * large rather than too small. The error in the first-order Taylor
       * series approximation is in our favor, so we just round up the constant 
       * enough to compensate for any error in the multiplication of (i - Bias) 
       * by 0.301029995663981; since |i - Bias| <= 1077, and 1077 * 0.30103 *
       * 2^-52 ~=~ 7.2e-14, adding 1e-13 to the constant term more than
       * suffices. Hence we adjust the constant term to 0.1760912590558. (We
       * could get a more accurate k by invoking log10, but this is probably
       * not worthwhile.) */

      i -= Bias;
      denorm = 0;
    }
  else
    {
      /* d is denormalized */

      i = bbits + be + (Bias + (P - 1) - 1);
      x = i > 32 ? word0(d) << (64 - i) | word1(d) >> (i - 32)
        : word1(d) << (32 - i);
      d2 = x;
      word0(d2) -= 31 * Exp_msk1;       /* adjust exponent */
      i -= (Bias + (P - 1) - 1) + 1;
      denorm = 1;
    }

  ds = (d2 - 1.5) * 0.289529654602168 + 0.1760912590558 + i * 0.301029995663981;
  k = (int)ds;
  if (ds < 0. && ds != k)
    {
      k--;  /* want k = floor(ds) */
    }
  k_check = 1;

  if (k >= 0 && k <= Ten_pmax)
    {
      if (d < tens[k])
        k--;
      k_check = 0;
    }

  j = bbits - i - 1;
  if (j >= 0)
    {
      b2 = 0;
      s2 = j;
    }
  else
    {
      b2 = -j;
      s2 = 0;
    }

  if (k >= 0)
    {
      b5 = 0;
      s5 = k;
      s2 += k;
    }
  else
    {
      b2 -= k;
      b5 = -k;
      s5 = 0;
    }

  if (mode < 0 || mode > 9)
    {
      mode = 0;
    }

  try_quick = 1;
  if (mode > 5)
    {
      mode -= 4;
      try_quick = 0;
    }

  leftright = 1;
  switch (mode)
    {
    case 0:
    case 1:
      ilim = ilim1 = -1;
      i = 18;
      ndigits = 0;
      break;

    case 2:
      leftright = 0;
      /* no break */
    case 4:
      if (ndigits <= 0)
        {
          ndigits = 1;
        }

      ilim = ilim1 = i = ndigits;
      break;

    case 3:
      leftright = 0;
      /* no break */
    case 5:
      i = ndigits + k + 1;
      ilim = i;
      ilim1 = i - 1;
      if (i <= 0)
        {
          i = 1;
        }
    }

  j = sizeof(unsigned long);
  for (result_k = 0;
       (signed)(sizeof(Bigint) - sizeof(unsigned long) + j) <= i;
       j <<= 1)
    {
      result_k++;
    }

  result = Balloc(result_k);
  s = s0 = (char *)result;

  if (ilim >= 0 && ilim <= Quick_max && try_quick)
    {
      /* Try to get by with floating-point arithmetic. */

      i = 0;
      d2 = d;
      k0 = k;
      ilim0 = ilim;
      ieps = 2;                 /* conservative */

      if (k > 0)
        {
          ds = tens[k & 0xf];
          j = k >> 4;

          if (j & Bletch)
            {
              /* prevent overflows */
              j &= Bletch - 1;
              d /= bigtens[n_bigtens - 1];
              ieps++;
            }

          for (; j; j >>= 1, i++)
            {
              if (j & 1)
                {
                  ieps++;
                  ds *= bigtens[i];
                }
            }

          d /= ds;
        }
      else if ((j_1 = -k))
        {
          d *= tens[j_1 & 0xf];
          for (j = j_1 >> 4; j; j >>= 1, i++)
            {
              if (j & 1)
                {
                  ieps++;
                  d *= bigtens[i];
                }
            }
        }

      if (k_check && d < 1. && ilim > 0)
        {
          if (ilim1 <= 0)
            {
              goto fast_failed;
            }

          ilim = ilim1;
          k--;
          d *= 10.;
          ieps++;
        }

      eps = ieps * d + 7.;
      word0(eps) -= (P - 1) * Exp_msk1;
      if (ilim == 0)
        {
          S = mhi = 0;
          d -= 5.;
          if (d > eps)
            goto one_digit;
          if (d < -eps)
            goto no_digits;
          goto fast_failed;
        }

#ifndef No_leftright
      if (leftright)
        {
          /* Use Steele & White method of only generating digits needed. */

          eps = 0.5 / tens[ilim - 1] - eps;
          for (i = 0;;)
            {
              L = (int)d;
              d -= L;
              *s++ = '0' + (int)L;
              if (d < eps)
                goto ret1;
              if (1. - d < eps)
                goto bump_up;
              if (++i >= ilim)
                break;
              eps *= 10.;
              d *= 10.;
            }
        }
      else
        {
#endif
          /* Generate ilim digits, then fix them up. */
          
          eps *= tens[ilim - 1];
          for (i = 1;; i++, d *= 10.)
            {
              L = (int)d;
              d -= L;
              *s++ = '0' + (int)L;
              if (i == ilim)
                {
                  if (d > 0.5 + eps)
                    goto bump_up;
                  else if (d < 0.5 - eps)
                    {
                      while (*--s == '0');
                      s++;
                      goto ret1;
                    }
                  break;
                }
            }
#ifndef No_leftright
        }
#endif
    fast_failed:
      s = s0;
      d = d2;
      k = k0;
      ilim = ilim0;
    }

  /* Do we have a "small" integer? */

  if (be >= 0 && k <= Int_max)
    {
      /* Yes. */

      ds = tens[k];
      if (ndigits < 0 && ilim <= 0)
        {
          S = mhi = 0;
          if (ilim < 0 || d <= 5 * ds)
            goto no_digits;
          goto one_digit;
        }

      for (i = 1;; i++)
        {
          L = (int)(d / ds);
          d -= L * ds;
#ifdef Check_FLT_ROUNDS
          /* If FLT_ROUNDS == 2, L will usually be high by 1 */
          if (d < 0)
            {
              L--;
              d += ds;
            }
#endif
          *s++ = '0' + (int)L;
          if (i == ilim)
            {
              d += d;
              if (d > ds || (d == ds && (L & 1)))
                {
                bump_up:
                  while (*--s == '9')
                    if (s == s0)
                      {
                        k++;
                        *s = '0';
                        break;
                      }
                  ++*s++;
                }
              break;
            }
          if (!(d *= 10.))
            {
              break;
            }
        }

      goto ret1;
    }

  m2 = b2;
  m5 = b5;
  mhi = mlo = 0;
  if (leftright)
    {
      if (mode < 2)
        {
          i = denorm ? be + (Bias + (P - 1) - 1 + 1) : 1 + P - bbits;
        }
      else
        {
          j = ilim - 1;
          if (m5 >= j)
            m5 -= j;
          else
            {
              s5 += j -= m5;
              b5 += j;
              m5 = 0;
            }
          if ((i = ilim) < 0)
            {
              m2 -= i;
              i = 0;
            }
        }

      b2 += i;
      s2 += i;
      mhi = i2b(1);
    }

  if (m2 > 0 && s2 > 0)
    {
      i = m2 < s2 ? m2 : s2;
      b2 -= i;
      m2 -= i;
      s2 -= i;
    }

  if (b5 > 0)
    {
      if (leftright)
        {
          if (m5 > 0)
            {
              mhi = pow5mult(mhi, m5);
              b1 = mult(mhi, b);
              Bfree(b);
              b = b1;
            }
          if ((j = b5 - m5))
            b = pow5mult(b, j);
        }
      else
        {
          b = pow5mult(b, b5);
        }
    }

  S = i2b(1);
  if (s5 > 0)
    {
      S = pow5mult(S, s5);
    }

  /* Check for special case that d is a normalized power of 2. */

  if (mode < 2)
    {
      if (!word1(d) && !(word0(d) & Bndry_mask) && word0(d) & Exp_mask)
        {
          /* The special case */
          b2 += Log2P;
          s2 += Log2P;
          spec_case = 1;
        }
      else
        {
          spec_case = 0;
        }
    }

  /* Arrange for convenient computation of quotients: shift left if
   * necessary so divisor has 4 leading 0 bits.
   *
   * Perhaps we should just compute leading 28 bits of S once and for all
   * and pass them and a shift to quorem, so it can do shifts and ors
   * to compute the numerator for q.
   */

#ifdef Pack_32
  if ((i = ((s5 ? 32 - hi0bits(S->x[S->wds - 1]) : 1) + s2) & 0x1f))
    {
      i = 32 - i;
    }
#else
  if ((i = ((s5 ? 32 - hi0bits(S->x[S->wds - 1]) : 1) + s2) & 0xf))
    {
      i = 16 - i;
    }
#endif

  if (i > 4)
    {
      i -= 4;
      b2 += i;
      m2 += i;
      s2 += i;
    }
  else if (i < 4)
    {
      i += 28;
      b2 += i;
      m2 += i;
      s2 += i;
    }

  if (b2 > 0)
    {
      b = lshift(b, b2);
    }

  if (s2 > 0)
    {
      S = lshift(S, s2);
    }

  if (k_check)
    {
      if (cmp(b, S) < 0)
        {
          k--;
          b = multadd(b, 10, 0);        /* we botched the k estimate */
          if (leftright)
            {
              mhi = multadd(mhi, 10, 0);
            }

          ilim = ilim1;
        }
    }

  if (ilim <= 0 && mode > 2)
    {
      if (ilim < 0 || cmp(b, S = multadd(S, 5, 0)) <= 0)
        {
          /* no digits, fcvt style */
        no_digits:
          k = -1 - ndigits;
          goto ret;
        }
    one_digit:
      *s++ = '1';
      k++;
      goto ret;
    }

  if (leftright)
    {
      if (m2 > 0)
        {
          mhi = lshift(mhi, m2);
        }

      /* Compute mlo -- check for special case that d is a normalized power of
       * 2. */

      mlo = mhi;
      if (spec_case)
        {
          mhi = Balloc(mhi->k);
          Bcopy(mhi, mlo);
          mhi = lshift(mhi, Log2P);
        }

      for (i = 1;; i++)
        {
          dig = quorem(b, S) + '0';
          /* Do we yet have the shortest decimal string that will round to d? */
          j = cmp(b, mlo);
          delta = diff(S, mhi);
          j_1 = delta->sign ? 1 : cmp(b, delta);
          Bfree(delta);
#ifndef ROUND_BIASED
          if (j_1 == 0 && !mode && !(word1(d) & 1))
            {
              if (dig == '9')
                {
                 goto round_9_up;
                }

              if (j > 0)
                {
                  dig++;
                }

              *s++ = dig;
              goto ret;
            }
#endif
          if (j < 0 || (j == 0 && !mode
#ifndef ROUND_BIASED
                        && (!(word1(d) & 1))
#endif
            ))
            {
              if ((j_1 > 0))
                {
                  b = lshift(b, 1);
                  j_1 = cmp(b, S);
                  if ((j_1 > 0 || (j_1 == 0 && (dig & 1))) && dig++ == '9')
                    {
                      goto round_9_up;
                    }
                }

              *s++ = dig;
              goto ret;
            }

          if (j_1 > 0)
            {
              if (dig == '9')
                {               /* possible if i == 1 */
                round_9_up:
                  *s++ = '9';
                  goto roundoff;
                }

              *s++ = dig + 1;
              goto ret;
            }

          *s++ = dig;
          if (i == ilim)
            {
              break;
            }

          b = multadd(b, 10, 0);
          if (mlo == mhi)
            {
              mlo = mhi = multadd(mhi, 10, 0);
            }
          else
            {
              mlo = multadd(mlo, 10, 0);
              mhi = multadd(mhi, 10, 0);
            }
        }
    }
  else
    {
      for (i = 1;; i++)
        {
          *s++ = dig = quorem(b, S) + '0';
          if (i >= ilim)
            {
              break;
            }

          b = multadd(b, 10, 0);
        }
    }

  /* Round off last digit */

  b = lshift(b, 1);
  j = cmp(b, S);
  if (j > 0 || (j == 0 && (dig & 1)))
    {
    roundoff:
      while (*--s == '9')
        if (s == s0)
          {
            k++;
            *s++ = '1';
            goto ret;
          }
      ++*s++;
    }
  else
    {
      while (*--s == '0');
      s++;
    }

ret:
  Bfree(S);
  if (mhi)
    {
      if (mlo && mlo != mhi)
        {
          Bfree(mlo);
        }

      Bfree(mhi);
    }
ret1:
  Bfree(b);
  if (s == s0)
    {                           /* don't return empty string */
      *s++ = '0';
      k = 0;
    }

  *s = 0;
  *decpt = k + 1;
  if (rve)
    {
      *rve = s;
    }

  return s0;
}
