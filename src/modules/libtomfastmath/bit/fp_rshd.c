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

void fp_rshd(fp_int *a, int x)
{
  int y;

  /* too many digits just zero and return */
  if (x >= a->used) {
     fp_zero(a);
     return;
  }

   /* shift */
   for (y = 0; y < a->used - x; y++) {
      a->dp[y] = a->dp[y+x];
   }

   /* zero rest */
   for (; y < a->used; y++) {
      a->dp[y] = 0;
   }
   
   /* decrement count */
   a->used -= x;
   fp_clamp(a);
}


/* $Source$ */
/* $Revision$ */
/* $Date$ */
