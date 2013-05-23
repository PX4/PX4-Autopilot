#define TFM_DEFINES
#include "fp_mul_comba.c"

#ifdef TFM_MUL4
void fp_mul_comba4(fp_int *A, fp_int *B, fp_int *C)
{
   fp_digit c0, c1, c2, at[8];

   memcpy(at, A->dp, 4 * sizeof(fp_digit));
   memcpy(at+4, B->dp, 4 * sizeof(fp_digit));
   COMBA_START;

   COMBA_CLEAR;
   /* 0 */
   MULADD(at[0], at[4]); 
   COMBA_STORE(C->dp[0]);
   /* 1 */
   COMBA_FORWARD;
   MULADD(at[0], at[5]);    MULADD(at[1], at[4]); 
   COMBA_STORE(C->dp[1]);
   /* 2 */
   COMBA_FORWARD;
   MULADD(at[0], at[6]);    MULADD(at[1], at[5]);    MULADD(at[2], at[4]); 
   COMBA_STORE(C->dp[2]);
   /* 3 */
   COMBA_FORWARD;
   MULADD(at[0], at[7]);    MULADD(at[1], at[6]);    MULADD(at[2], at[5]);    MULADD(at[3], at[4]); 
   COMBA_STORE(C->dp[3]);
   /* 4 */
   COMBA_FORWARD;
   MULADD(at[1], at[7]);    MULADD(at[2], at[6]);    MULADD(at[3], at[5]); 
   COMBA_STORE(C->dp[4]);
   /* 5 */
   COMBA_FORWARD;
   MULADD(at[2], at[7]);    MULADD(at[3], at[6]); 
   COMBA_STORE(C->dp[5]);
   /* 6 */
   COMBA_FORWARD;
   MULADD(at[3], at[7]); 
   COMBA_STORE(C->dp[6]);
   COMBA_STORE2(C->dp[7]);
   C->used = 8;
   C->sign = A->sign ^ B->sign;
   fp_clamp(C);
   COMBA_FINI;
}
#endif
