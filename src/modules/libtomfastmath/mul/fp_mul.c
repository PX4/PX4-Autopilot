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

/* c = a * b */
void fp_mul(fp_int *A, fp_int *B, fp_int *C)
{
    int   y, yy;

    /* call generic if we're out of range */
    if (A->used + B->used > FP_SIZE) {
       fp_mul_comba(A, B, C);
       return ;
    }

     y  = MAX(A->used, B->used);
     yy = MIN(A->used, B->used);
    /* pick a comba (unrolled 4/8/16/32 x or rolled) based on the size
       of the largest input.  We also want to avoid doing excess mults if the 
       inputs are not close to the next power of two.  That is, for example,
       if say y=17 then we would do (32-17)^2 = 225 unneeded multiplications 
    */

#ifdef TFM_MUL3
        if (y <= 3) {
           fp_mul_comba3(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL4
        if (y == 4) {
           fp_mul_comba4(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL6
        if (y <= 6) {
           fp_mul_comba6(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL7
        if (y == 7) {
           fp_mul_comba7(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL8
        if (y == 8) {
           fp_mul_comba8(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL9
        if (y == 9) {
           fp_mul_comba9(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL12
        if (y <= 12) {
           fp_mul_comba12(A,B,C);
           return;
        }
#endif
#ifdef TFM_MUL17
        if (y <= 17) {
           fp_mul_comba17(A,B,C);
           return;
        }
#endif

#ifdef TFM_SMALL_SET
        if (y <= 16) {
           fp_mul_comba_small(A,B,C);
           return;
        }
#endif        
#if defined(TFM_MUL20)
        if (y <= 20) {
           fp_mul_comba20(A,B,C);
           return;
        }
#endif
#if defined(TFM_MUL24)
        if (yy >= 16 && y <= 24) {
           fp_mul_comba24(A,B,C);
           return;
        }
#endif
#if defined(TFM_MUL28)
        if (yy >= 20 && y <= 28) {
           fp_mul_comba28(A,B,C);
           return;
        }
#endif
#if defined(TFM_MUL32)
        if (yy >= 24 && y <= 32) {
           fp_mul_comba32(A,B,C);
           return;
        }
#endif
#if defined(TFM_MUL48)
        if (yy >= 40 && y <= 48) {
           fp_mul_comba48(A,B,C);
           return;
        }
#endif        
#if defined(TFM_MUL64)
        if (yy >= 56 && y <= 64) {
           fp_mul_comba64(A,B,C);
           return;
        }
#endif
        fp_mul_comba(A,B,C);
}


/* $Source$ */
/* $Revision$ */
/* $Date$ */
