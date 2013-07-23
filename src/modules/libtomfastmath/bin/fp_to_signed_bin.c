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

void fp_to_signed_bin(fp_int *a, unsigned char *b)
{
  fp_to_unsigned_bin (a, b + 1);
  b[0] = (unsigned char) ((a->sign == FP_ZPOS) ? 0 : 1);
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
