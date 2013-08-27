/* TomsFastMath, a fast ISO C bignum library.
 * 
 * This project is meant to fill in where LibTomMath
 * falls short.  That is speed ;-)
 *
 * This project is public domain and free for all purposes.
 * 
 * Tom St Denis, tomstdenis@gmail.com
 */
#include <tfm.h>

int fp_radix_size(fp_int *a, int radix, int *size)
{
  int     digs;
  fp_int  t;
  fp_digit d;
   
  *size = 0;

  /* check range of the radix */
  if (radix < 2 || radix > 64) {
    return FP_VAL;
  }

  /* quick out if its zero */
  if (fp_iszero(a) == 1) {
     *size = 2;
     return FP_OKAY;
  }

  fp_init_copy(&t, a);

  /* if it is negative output a - */
  if (t.sign == FP_NEG) {
    (*size)++;
    t.sign = FP_ZPOS;
  }

  digs = 0;
  while (fp_iszero (&t) == FP_NO) {
    fp_div_d (&t, (fp_digit) radix, &t, &d);
    (*size)++;
  }

  /* append a NULL so the string is properly terminated */
  (*size)++;
  return FP_OKAY;

}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
