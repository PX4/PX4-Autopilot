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

/* c = [a, b] */
void fp_lcm(fp_int *a, fp_int *b, fp_int *c)
{
   fp_int  t1, t2;

   fp_init(&t1);
   fp_init(&t2);
   fp_gcd(a, b, &t1);
   if (fp_cmp_mag(a, b) == FP_GT) {
      fp_div(a, &t1, &t2, NULL);
      fp_mul(b, &t2, c);
   } else {
      fp_div(b, &t1, &t2, NULL);
      fp_mul(a, &t2, c);
   }   
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
