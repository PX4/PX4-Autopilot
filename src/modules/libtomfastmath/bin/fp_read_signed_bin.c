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

void fp_read_signed_bin(fp_int *a, unsigned char *b, int c)
{
  /* read magnitude */
  fp_read_unsigned_bin (a, b + 1, c - 1);

  /* first byte is 0 for positive, non-zero for negative */
  if (b[0] == 0) {
     a->sign = FP_ZPOS;
  } else {
     a->sign = FP_NEG;
  }
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
