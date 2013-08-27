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

#ifdef TFM_TIMING_RESISTANT

/* timing resistant montgomery ladder based exptmod 

   Based on work by Marc Joye, Sung-Ming Yen, "The Montgomery Powering Ladder", Cryptographic Hardware and Embedded Systems, CHES 2002
*/
static int _fp_exptmod(fp_int * G, fp_int * X, fp_int * P, fp_int * Y)
{
  fp_int   R[2];
  fp_digit buf, mp;
  int      err, bitcnt, digidx, y;

  /* now setup montgomery  */
  if ((err = fp_montgomery_setup (P, &mp)) != FP_OKAY) {
     return err;
  }

  fp_init(&R[0]);   
  fp_init(&R[1]);   
   
  /* now we need R mod m */
  fp_montgomery_calc_normalization (&R[0], P);

  /* now set R[0][1] to G * R mod m */
  if (fp_cmp_mag(P, G) != FP_GT) {
     /* G > P so we reduce it first */
     fp_mod(G, P, &R[1]);
  } else {
     fp_copy(G, &R[1]);
  }
  fp_mulmod (&R[1], &R[0], P, &R[1]);

  /* for j = t-1 downto 0 do
        r_!k = R0*R1; r_k = r_k^2
  */
  
  /* set initial mode and bit cnt */
  bitcnt = 1;
  buf    = 0;
  digidx = X->used - 1;

  for (;;) {
    /* grab next digit as required */
    if (--bitcnt == 0) {
      /* if digidx == -1 we are out of digits so break */
      if (digidx == -1) {
        break;
      }
      /* read next digit and reset bitcnt */
      buf    = X->dp[digidx--];
      bitcnt = (int)DIGIT_BIT;
    }

    /* grab the next msb from the exponent */
    y     = (fp_digit)(buf >> (DIGIT_BIT - 1)) & 1;
    buf <<= (fp_digit)1;

    /* do ops */
    fp_mul(&R[0], &R[1], &R[y^1]); fp_montgomery_reduce(&R[y^1], P, mp);
    fp_sqr(&R[y], &R[y]);          fp_montgomery_reduce(&R[y], P, mp);
  }

   fp_montgomery_reduce(&R[0], P, mp);
   fp_copy(&R[0], Y);
   return FP_OKAY;
}   

#else

/* y = g**x (mod b) 
 * Some restrictions... x must be positive and < b
 */
static int _fp_exptmod(fp_int * G, fp_int * X, fp_int * P, fp_int * Y)
{
  fp_int   M[64], res;
  fp_digit buf, mp;
  int      err, bitbuf, bitcpy, bitcnt, mode, digidx, x, y, winsize;

  /* find window size */
  x = fp_count_bits (X);
  if (x <= 21) {
    winsize = 1;
  } else if (x <= 36) {
    winsize = 3;
  } else if (x <= 140) {
    winsize = 4;
  } else if (x <= 450) {
    winsize = 5;
  } else {
    winsize = 6;
  } 

  /* init M array */
  memset(M, 0, sizeof(M)); 

  /* now setup montgomery  */
  if ((err = fp_montgomery_setup (P, &mp)) != FP_OKAY) {
     return err;
  }

  /* setup result */
  fp_init(&res);

  /* create M table
   *
   * The M table contains powers of the input base, e.g. M[x] = G^x mod P
   *
   * The first half of the table is not computed though accept for M[0] and M[1]
   */

   /* now we need R mod m */
   fp_montgomery_calc_normalization (&res, P);

   /* now set M[1] to G * R mod m */
   if (fp_cmp_mag(P, G) != FP_GT) {
      /* G > P so we reduce it first */
      fp_mod(G, P, &M[1]);
   } else {
      fp_copy(G, &M[1]);
   }
   fp_mulmod (&M[1], &res, P, &M[1]);

  /* compute the value at M[1<<(winsize-1)] by squaring M[1] (winsize-1) times */
  fp_copy (&M[1], &M[1 << (winsize - 1)]);
  for (x = 0; x < (winsize - 1); x++) {
    fp_sqr (&M[1 << (winsize - 1)], &M[1 << (winsize - 1)]);
    fp_montgomery_reduce (&M[1 << (winsize - 1)], P, mp);
  }

  /* create upper table */
  for (x = (1 << (winsize - 1)) + 1; x < (1 << winsize); x++) {
    fp_mul(&M[x - 1], &M[1], &M[x]);
    fp_montgomery_reduce(&M[x], P, mp);
  }

  /* set initial mode and bit cnt */
  mode   = 0;
  bitcnt = 1;
  buf    = 0;
  digidx = X->used - 1;
  bitcpy = 0;
  bitbuf = 0;

  for (;;) {
    /* grab next digit as required */
    if (--bitcnt == 0) {
      /* if digidx == -1 we are out of digits so break */
      if (digidx == -1) {
        break;
      }
      /* read next digit and reset bitcnt */
      buf    = X->dp[digidx--];
      bitcnt = (int)DIGIT_BIT;
    }

    /* grab the next msb from the exponent */
    y     = (fp_digit)(buf >> (DIGIT_BIT - 1)) & 1;
    buf <<= (fp_digit)1;

    /* if the bit is zero and mode == 0 then we ignore it
     * These represent the leading zero bits before the first 1 bit
     * in the exponent.  Technically this opt is not required but it
     * does lower the # of trivial squaring/reductions used
     */
    if (mode == 0 && y == 0) {
      continue;
    }

    /* if the bit is zero and mode == 1 then we square */
    if (mode == 1 && y == 0) {
      fp_sqr(&res, &res);
      fp_montgomery_reduce(&res, P, mp);
      continue;
    }

    /* else we add it to the window */
    bitbuf |= (y << (winsize - ++bitcpy));
    mode    = 2;

    if (bitcpy == winsize) {
      /* ok window is filled so square as required and multiply  */
      /* square first */
      for (x = 0; x < winsize; x++) {
        fp_sqr(&res, &res);
        fp_montgomery_reduce(&res, P, mp);
      }

      /* then multiply */
      fp_mul(&res, &M[bitbuf], &res);
      fp_montgomery_reduce(&res, P, mp);

      /* empty window and reset */
      bitcpy = 0;
      bitbuf = 0;
      mode   = 1;
    }
  }

  /* if bits remain then square/multiply */
  if (mode == 2 && bitcpy > 0) {
    /* square then multiply if the bit is set */
    for (x = 0; x < bitcpy; x++) {
      fp_sqr(&res, &res);
      fp_montgomery_reduce(&res, P, mp);

      /* get next bit of the window */
      bitbuf <<= 1;
      if ((bitbuf & (1 << winsize)) != 0) {
        /* then multiply */
        fp_mul(&res, &M[1], &res);
        fp_montgomery_reduce(&res, P, mp);
      }
    }
  }

  /* fixup result if Montgomery reduction is used
   * recall that any value in a Montgomery system is
   * actually multiplied by R mod n.  So we have
   * to reduce one more time to cancel out the factor
   * of R.
   */
  fp_montgomery_reduce(&res, P, mp);

  /* swap res with Y */
  fp_copy (&res, Y);
  return FP_OKAY;
}

#endif


int fp_exptmod(fp_int * G, fp_int * X, fp_int * P, fp_int * Y)
{
   fp_int tmp;
   int    err;
   
#ifdef TFM_CHECK
   /* prevent overflows */
   if (P->used > (FP_SIZE/2)) {
      return FP_VAL;
   }
#endif

   /* is X negative?  */
   if (X->sign == FP_NEG) {
      /* yes, copy G and invmod it */
      fp_copy(G, &tmp);
      if ((err = fp_invmod(&tmp, P, &tmp)) != FP_OKAY) {
         return err;
      }
      X->sign = FP_ZPOS;
      err =  _fp_exptmod(&tmp, X, P, Y);
      if (X != Y) {
         X->sign = FP_NEG;
      }
      return err;
   } else {
      /* Positive exponent so just exptmod */
      return _fp_exptmod(G, X, P, Y);
   }
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
