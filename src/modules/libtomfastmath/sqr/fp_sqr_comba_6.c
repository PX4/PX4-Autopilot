#define TFM_DEFINES
#include "fp_sqr_comba.c"

#ifdef TFM_SQR6
void fp_sqr_comba6(fp_int *A, fp_int *B)
{
   fp_digit *a, b[12], c0, c1, c2, sc0, sc1, sc2;
#ifdef TFM_ISO
   fp_word tt;
#endif

   a = A->dp;
   COMBA_START; 

   /* clear carries */
   CLEAR_CARRY;

   /* output 0 */
   SQRADD(a[0],a[0]);
   COMBA_STORE(b[0]);

   /* output 1 */
   CARRY_FORWARD;
   SQRADD2(a[0], a[1]); 
   COMBA_STORE(b[1]);

   /* output 2 */
   CARRY_FORWARD;
   SQRADD2(a[0], a[2]); SQRADD(a[1], a[1]); 
   COMBA_STORE(b[2]);

   /* output 3 */
   CARRY_FORWARD;
   SQRADD2(a[0], a[3]); SQRADD2(a[1], a[2]); 
   COMBA_STORE(b[3]);

   /* output 4 */
   CARRY_FORWARD;
   SQRADD2(a[0], a[4]); SQRADD2(a[1], a[3]); SQRADD(a[2], a[2]); 
   COMBA_STORE(b[4]);

   /* output 5 */
   CARRY_FORWARD;
   SQRADDSC(a[0], a[5]); SQRADDAC(a[1], a[4]); SQRADDAC(a[2], a[3]); SQRADDDB; 
   COMBA_STORE(b[5]);

   /* output 6 */
   CARRY_FORWARD;
   SQRADD2(a[1], a[5]); SQRADD2(a[2], a[4]); SQRADD(a[3], a[3]); 
   COMBA_STORE(b[6]);

   /* output 7 */
   CARRY_FORWARD;
   SQRADD2(a[2], a[5]); SQRADD2(a[3], a[4]); 
   COMBA_STORE(b[7]);

   /* output 8 */
   CARRY_FORWARD;
   SQRADD2(a[3], a[5]); SQRADD(a[4], a[4]); 
   COMBA_STORE(b[8]);

   /* output 9 */
   CARRY_FORWARD;
   SQRADD2(a[4], a[5]); 
   COMBA_STORE(b[9]);

   /* output 10 */
   CARRY_FORWARD;
   SQRADD(a[5], a[5]); 
   COMBA_STORE(b[10]);
   COMBA_STORE2(b[11]);
   COMBA_FINI;

   B->used = 12;
   B->sign = FP_ZPOS;
   memcpy(B->dp, b, 12 * sizeof(fp_digit));
   memset(B->dp + 12, 0, (FP_SIZE - 12) * sizeof(fp_digit));
   fp_clamp(B);
}
#endif


