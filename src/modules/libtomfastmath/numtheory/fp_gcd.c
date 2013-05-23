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

/* c = (a, b) */
void fp_gcd(fp_int *a, fp_int *b, fp_int *c)
{
   fp_int u, v, r;

   /* either zero than gcd is the largest */
   if (fp_iszero (a) == 1 && fp_iszero (b) == 0) {
     fp_abs (b, c);
     return;
   }
   if (fp_iszero (a) == 0 && fp_iszero (b) == 1) {
     fp_abs (a, c);
     return;
   }

   /* optimized.  At this point if a == 0 then
    * b must equal zero too
    */
   if (fp_iszero (a) == 1) {
     fp_zero(c);
     return;
   }

   /* sort inputs */
   if (fp_cmp_mag(a, b) != FP_LT) {
      fp_init_copy(&u, a);
      fp_init_copy(&v, b);
   } else {
      fp_init_copy(&u, b);
      fp_init_copy(&v, a);
   }
 
   fp_zero(&r);
   while (fp_iszero(&v) == FP_NO) {
      fp_mod(&u, &v, &r);
      fp_copy(&v, &u);
      fp_copy(&r, &v);
   }
   fp_copy(&u, c);
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
