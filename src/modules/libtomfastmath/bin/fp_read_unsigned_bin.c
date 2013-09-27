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

void fp_read_unsigned_bin(fp_int *a, unsigned char *b, int c)
{
  /* zero the int */
  fp_zero (a);

  /* If we know the endianness of this architecture, and we're using
     32-bit fp_digits, we can optimize this */
#if (defined(ENDIAN_LITTLE) || defined(ENDIAN_BIG)) && !defined(FP_64BIT)
  /* But not for both simultaneously */
#if defined(ENDIAN_LITTLE) && defined(ENDIAN_BIG)
#error Both ENDIAN_LITTLE and ENDIAN_BIG defined.
#endif
  {
     unsigned char *pd = (unsigned char *)a->dp;

     if ((unsigned)c > (FP_SIZE * sizeof(fp_digit))) {
        int excess = c - (FP_SIZE * sizeof(fp_digit));
        c -= excess;
        b += excess;
     }
     a->used = (c + sizeof(fp_digit) - 1)/sizeof(fp_digit);
     /* read the bytes in */
#ifdef ENDIAN_BIG
     {
       /* Use Duff's device to unroll the loop. */
       int idx = (c - 1) & ~3;
       switch (c % 4) {
       case 0:	do { pd[idx+0] = *b++;
       case 3:	     pd[idx+1] = *b++;
       case 2:	     pd[idx+2] = *b++;
       case 1:	     pd[idx+3] = *b++;
                     idx -= 4;
	 	        } while ((c -= 4) > 0);
       }
     }
#else
     for (c -= 1; c >= 0; c -= 1) {
       pd[c] = *b++;
     }
#endif
  }
#else
  /* read the bytes in */
  for (; c > 0; c--) {
     fp_mul_2d (a, 8, a);
     a->dp[0] |= *b++;
     a->used += 1;
  }
#endif
  fp_clamp (a);
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
