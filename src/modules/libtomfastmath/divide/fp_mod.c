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

/* c = a mod b, 0 <= c < b  */
int fp_mod(fp_int *a, fp_int *b, fp_int *c)
{
   fp_int t;
   int    err;

   fp_zero(&t);
   if ((err = fp_div(a, b, NULL, &t)) != FP_OKAY) {
      return err;
   }
   if (t.sign != b->sign) {
      fp_add(&t, b, c);
   } else {
      fp_copy(&t, c);
  }
  return FP_OKAY;
}



/* $Source$ */
/* $Revision$ */
/* $Date$ */
