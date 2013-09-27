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

static int fp_invmod_slow (fp_int * a, fp_int * b, fp_int * c)
{
  fp_int  x, y, u, v, A, B, C, D;
  int     res;

  /* b cannot be negative */
  if (b->sign == FP_NEG || fp_iszero(b) == 1) {
    return FP_VAL;
  }

  /* init temps */
  fp_init(&x);    fp_init(&y);
  fp_init(&u);    fp_init(&v);
  fp_init(&A);    fp_init(&B);
  fp_init(&C);    fp_init(&D);

  /* x = a, y = b */
  if ((res = fp_mod(a, b, &x)) != FP_OKAY) {
      return res;
  }
  fp_copy(b, &y);

  /* 2. [modified] if x,y are both even then return an error! */
  if (fp_iseven (&x) == 1 && fp_iseven (&y) == 1) {
    return FP_VAL;
  }

  /* 3. u=x, v=y, A=1, B=0, C=0,D=1 */
  fp_copy (&x, &u);
  fp_copy (&y, &v);
  fp_set (&A, 1);
  fp_set (&D, 1);

top:
  /* 4.  while u is even do */
  while (fp_iseven (&u) == 1) {
    /* 4.1 u = u/2 */
    fp_div_2 (&u, &u);

    /* 4.2 if A or B is odd then */
    if (fp_isodd (&A) == 1 || fp_isodd (&B) == 1) {
      /* A = (A+y)/2, B = (B-x)/2 */
      fp_add (&A, &y, &A);
      fp_sub (&B, &x, &B);
    }
    /* A = A/2, B = B/2 */
    fp_div_2 (&A, &A);
    fp_div_2 (&B, &B);
  }

  /* 5.  while v is even do */
  while (fp_iseven (&v) == 1) {
    /* 5.1 v = v/2 */
    fp_div_2 (&v, &v);

    /* 5.2 if C or D is odd then */
    if (fp_isodd (&C) == 1 || fp_isodd (&D) == 1) {
      /* C = (C+y)/2, D = (D-x)/2 */
      fp_add (&C, &y, &C);
      fp_sub (&D, &x, &D);
    }
    /* C = C/2, D = D/2 */
    fp_div_2 (&C, &C);
    fp_div_2 (&D, &D);
  }

  /* 6.  if u >= v then */
  if (fp_cmp (&u, &v) != FP_LT) {
    /* u = u - v, A = A - C, B = B - D */
    fp_sub (&u, &v, &u);
    fp_sub (&A, &C, &A);
    fp_sub (&B, &D, &B);
  } else {
    /* v - v - u, C = C - A, D = D - B */
    fp_sub (&v, &u, &v);
    fp_sub (&C, &A, &C);
    fp_sub (&D, &B, &D);
  }

  /* if not zero goto step 4 */
  if (fp_iszero (&u) == 0)
    goto top;

  /* now a = C, b = D, gcd == g*v */

  /* if v != 1 then there is no inverse */
  if (fp_cmp_d (&v, 1) != FP_EQ) {
    return FP_VAL;
  }

  /* if its too low */
  while (fp_cmp_d(&C, 0) == FP_LT) {
      fp_add(&C, b, &C);
  }
  
  /* too big */
  while (fp_cmp_mag(&C, b) != FP_LT) {
      fp_sub(&C, b, &C);
  }
  
  /* C is now the inverse */
  fp_copy(&C, c);
  return FP_OKAY;
}

/* c = 1/a (mod b) for odd b only */
int fp_invmod(fp_int *a, fp_int *b, fp_int *c)
{
  fp_int  x, y, u, v, B, D;
  int     neg;

  /* 2. [modified] b must be odd   */
  if (fp_iseven (b) == FP_YES) {
    return fp_invmod_slow(a,b,c);
  }

  /* init all our temps */
  fp_init(&x);  fp_init(&y);
  fp_init(&u);  fp_init(&v);
  fp_init(&B);  fp_init(&D);

  /* x == modulus, y == value to invert */
  fp_copy(b, &x);

  /* we need y = |a| */
  fp_abs(a, &y);

  /* 3. u=x, v=y, A=1, B=0, C=0,D=1 */
  fp_copy(&x, &u);
  fp_copy(&y, &v);
  fp_set (&D, 1);

top:
  /* 4.  while u is even do */
  while (fp_iseven (&u) == FP_YES) {
    /* 4.1 u = u/2 */
    fp_div_2 (&u, &u);

    /* 4.2 if B is odd then */
    if (fp_isodd (&B) == FP_YES) {
      fp_sub (&B, &x, &B);
    }
    /* B = B/2 */
    fp_div_2 (&B, &B);
  }

  /* 5.  while v is even do */
  while (fp_iseven (&v) == FP_YES) {
    /* 5.1 v = v/2 */
    fp_div_2 (&v, &v);

    /* 5.2 if D is odd then */
    if (fp_isodd (&D) == FP_YES) {
      /* D = (D-x)/2 */
      fp_sub (&D, &x, &D);
    }
    /* D = D/2 */
    fp_div_2 (&D, &D);
  }

  /* 6.  if u >= v then */
  if (fp_cmp (&u, &v) != FP_LT) {
    /* u = u - v, B = B - D */
    fp_sub (&u, &v, &u);
    fp_sub (&B, &D, &B);
  } else {
    /* v - v - u, D = D - B */
    fp_sub (&v, &u, &v);
    fp_sub (&D, &B, &D);
  }

  /* if not zero goto step 4 */
  if (fp_iszero (&u) == FP_NO) {
    goto top;
  }

  /* now a = C, b = D, gcd == g*v */

  /* if v != 1 then there is no inverse */
  if (fp_cmp_d (&v, 1) != FP_EQ) {
    return FP_VAL;
  }

  /* b is now the inverse */
  neg = a->sign;
  while (D.sign == FP_NEG) {
    fp_add (&D, b, &D);
  }
  fp_copy (&D, c);
  c->sign = neg;
  return FP_OKAY;
}

/* $Source$ */
/* $Revision$ */
/* $Date$ */
