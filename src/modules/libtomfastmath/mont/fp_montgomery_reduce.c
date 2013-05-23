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

/******************************************************************/
#if defined(TFM_X86) && !defined(TFM_SSE2) 
/* x86-32 code */

#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                                          \
asm(                                                      \
   "movl %5,%%eax \n\t"                                   \
   "mull %4       \n\t"                                   \
   "addl %1,%%eax \n\t"                                   \
   "adcl $0,%%edx \n\t"                                   \
   "addl %%eax,%0 \n\t"                                   \
   "adcl $0,%%edx \n\t"                                   \
   "movl %%edx,%1 \n\t"                                   \
:"=g"(_c[LO]), "=r"(cy)                                   \
:"0"(_c[LO]), "1"(cy), "g"(mu), "g"(*tmpm++)              \
: "%eax", "%edx", "%cc")

#define PROPCARRY                           \
asm(                                        \
   "addl   %1,%0    \n\t"                   \
   "setb   %%al     \n\t"                   \
   "movzbl %%al,%1 \n\t"                    \
:"=g"(_c[LO]), "=r"(cy)                     \
:"0"(_c[LO]), "1"(cy)                       \
: "%eax", "%cc")

/******************************************************************/
#elif defined(TFM_X86_64)
/* x86-64 code */

#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                                          \
asm(                                                      \
   "movq %5,%%rax \n\t"                                   \
   "mulq %4       \n\t"                                   \
   "addq %1,%%rax \n\t"                                   \
   "adcq $0,%%rdx \n\t"                                   \
   "addq %%rax,%0 \n\t"                                   \
   "adcq $0,%%rdx \n\t"                                   \
   "movq %%rdx,%1 \n\t"                                   \
:"=g"(_c[LO]), "=r"(cy)                                   \
:"0"(_c[LO]), "1"(cy), "r"(mu), "r"(*tmpm++)              \
: "%rax", "%rdx", "%cc")

#define INNERMUL8 \
 asm(                  \
 "movq 0(%5),%%rax    \n\t"  \
 "movq 0(%2),%%r10    \n\t"  \
 "movq 0x8(%5),%%r11  \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x8(%2),%%r10  \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0(%0)    \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x10(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x10(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x8(%0)  \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x18(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x18(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x10(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x20(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x20(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x18(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x28(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x28(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x20(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x30(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x30(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x28(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "movq 0x38(%5),%%r11 \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq 0x38(%2),%%r10 \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x30(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
 "movq %%r11,%%rax    \n\t"  \
 "mulq %4             \n\t"  \
 "addq %%r10,%%rax    \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "addq %3,%%rax       \n\t"  \
 "adcq $0,%%rdx       \n\t"  \
 "movq %%rax,0x38(%0) \n\t"  \
 "movq %%rdx,%1       \n\t"  \
 \
:"=r"(_c), "=r"(cy)                    \
: "0"(_c),  "1"(cy), "g"(mu), "r"(tmpm)\
: "%rax", "%rdx", "%r10", "%r11", "%cc")


#define PROPCARRY                           \
asm(                                        \
   "addq   %1,%0    \n\t"                   \
   "setb   %%al     \n\t"                   \
   "movzbq %%al,%1 \n\t"                    \
:"=g"(_c[LO]), "=r"(cy)                     \
:"0"(_c[LO]), "1"(cy)                       \
: "%rax", "%cc")

/******************************************************************/
#elif defined(TFM_SSE2)  
/* SSE2 code (assumes 32-bit fp_digits) */
/* XMM register assignments:
 * xmm0  *tmpm++, then Mu * (*tmpm++)
 * xmm1  c[x], then Mu
 * xmm2  mp
 * xmm3  cy
 * xmm4  _c[LO]
 */

#define MONT_START \
   asm("movd %0,%%mm2"::"g"(mp))

#define MONT_FINI \
   asm("emms")

#define LOOP_START          \
asm(                        \
"movd %0,%%mm1        \n\t" \
"pxor %%mm3,%%mm3     \n\t" \
"pmuludq %%mm2,%%mm1  \n\t" \
:: "g"(c[x]))

/* pmuludq on mmx registers does a 32x32->64 multiply. */
#define INNERMUL               \
asm(                           \
   "movd %1,%%mm4        \n\t" \
   "movd %2,%%mm0        \n\t" \
   "paddq %%mm4,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm0  \n\t" \
   "paddq %%mm0,%%mm3    \n\t" \
   "movd %%mm3,%0        \n\t" \
   "psrlq $32, %%mm3     \n\t" \
:"=g"(_c[LO]) : "0"(_c[LO]), "g"(*tmpm++) );

#define INNERMUL8 \
asm(                           \
   "movd 0(%1),%%mm4     \n\t" \
   "movd 0(%2),%%mm0     \n\t" \
   "paddq %%mm4,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm0  \n\t" \
   "movd 4(%2),%%mm5     \n\t" \
   "paddq %%mm0,%%mm3    \n\t" \
   "movd 4(%1),%%mm6     \n\t" \
   "movd %%mm3,0(%0)     \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm6,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm5  \n\t" \
   "movd 8(%2),%%mm6     \n\t" \
   "paddq %%mm5,%%mm3    \n\t" \
   "movd 8(%1),%%mm7     \n\t" \
   "movd %%mm3,4(%0)     \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm7,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm6  \n\t" \
   "movd 12(%2),%%mm7    \n\t" \
   "paddq %%mm6,%%mm3    \n\t" \
   "movd 12(%1),%%mm5     \n\t" \
   "movd %%mm3,8(%0)     \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm5,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm7  \n\t" \
   "movd 16(%2),%%mm5    \n\t" \
   "paddq %%mm7,%%mm3    \n\t" \
   "movd 16(%1),%%mm6    \n\t" \
   "movd %%mm3,12(%0)    \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm6,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm5  \n\t" \
   "movd 20(%2),%%mm6    \n\t" \
   "paddq %%mm5,%%mm3    \n\t" \
   "movd 20(%1),%%mm7    \n\t" \
   "movd %%mm3,16(%0)    \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm7,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm6  \n\t" \
   "movd 24(%2),%%mm7    \n\t" \
   "paddq %%mm6,%%mm3    \n\t" \
   "movd 24(%1),%%mm5     \n\t" \
   "movd %%mm3,20(%0)    \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm5,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm7  \n\t" \
   "movd 28(%2),%%mm5    \n\t" \
   "paddq %%mm7,%%mm3    \n\t" \
   "movd 28(%1),%%mm6    \n\t" \
   "movd %%mm3,24(%0)    \n\t" \
   "psrlq $32, %%mm3     \n\t" \
\
   "paddq %%mm6,%%mm3    \n\t" \
   "pmuludq %%mm1,%%mm5  \n\t" \
   "paddq %%mm5,%%mm3    \n\t" \
   "movd %%mm3,28(%0)    \n\t" \
   "psrlq $32, %%mm3     \n\t" \
:"=r"(_c) : "0"(_c), "g"(tmpm) );

#define LOOP_END \
asm( "movd %%mm3,%0  \n" :"=r"(cy))

#define PROPCARRY                           \
asm(                                        \
   "addl   %1,%0    \n\t"                   \
   "setb   %%al     \n\t"                   \
   "movzbl %%al,%1 \n\t"                    \
:"=g"(_c[LO]), "=r"(cy)                     \
:"0"(_c[LO]), "1"(cy)                       \
: "%eax", "%cc")

/******************************************************************/
#elif defined(TFM_ARM)
   /* ARMv4 code */

#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                    \
asm(                                \
    " LDR    r0,%1            \n\t" \
    " ADDS   r0,r0,%0         \n\t" \
    " MOVCS  %0,#1            \n\t" \
    " MOVCC  %0,#0            \n\t" \
    " UMLAL  r0,%0,%3,%4      \n\t" \
    " STR    r0,%1            \n\t" \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"r"(mu),"r"(*tmpm++),"1"(_c[0]):"r0","%cc");

#define PROPCARRY                  \
asm(                               \
    " LDR   r0,%1            \n\t" \
    " ADDS  r0,r0,%0         \n\t" \
    " STR   r0,%1            \n\t" \
    " MOVCS %0,#1            \n\t" \
    " MOVCC %0,#0            \n\t" \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"1"(_c[0]):"r0","%cc");

/******************************************************************/
#elif defined(TFM_PPC32)

/* PPC32 */
#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                     \
asm(                                 \
   " mullw    16,%3,%4       \n\t"   \
   " mulhwu   17,%3,%4       \n\t"   \
   " addc     16,16,%0       \n\t"   \
   " addze    17,17          \n\t"   \
   " lwz      18,%1          \n\t"   \
   " addc     16,16,18       \n\t"   \
   " addze    %0,17          \n\t"   \
   " stw      16,%1          \n\t"   \
:"=r"(cy),"=g"(_c[0]):"0"(cy),"r"(mu),"r"(tmpm[0]),"1"(_c[0]):"16", "17", "18","%cc"); ++tmpm;

#define PROPCARRY                    \
asm(                                 \
   " lwz      16,%1         \n\t"    \
   " addc     16,16,%0      \n\t"    \
   " stw      16,%1         \n\t"    \
   " xor      %0,%0,%0      \n\t"    \
   " addze    %0,%0         \n\t"    \
:"=r"(cy),"=g"(_c[0]):"0"(cy),"1"(_c[0]):"16","%cc");

/******************************************************************/
#elif defined(TFM_PPC64)

/* PPC64 */
#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                     \
asm(                                 \
   " mulld    r16,%3,%4       \n\t"   \
   " mulhdu   r17,%3,%4       \n\t"   \
   " addc     r16,16,%0       \n\t"   \
   " addze    r17,r17          \n\t"   \
   " ldx      r18,0,%1        \n\t"   \
   " addc     r16,r16,r18       \n\t"   \
   " addze    %0,r17          \n\t"   \
   " sdx      r16,0,%1        \n\t"   \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"r"(mu),"r"(tmpm[0]),"1"(_c[0]):"r16", "r17", "r18","%cc"); ++tmpm;

#define PROPCARRY                    \
asm(                                 \
   " ldx      r16,0,%1       \n\t"    \
   " addc     r16,r16,%0      \n\t"    \
   " sdx      r16,0,%1       \n\t"    \
   " xor      %0,%0,%0      \n\t"    \
   " addze    %0,%0         \n\t"    \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"1"(_c[0]):"r16","%cc");

/******************************************************************/
#elif defined(TFM_AVR32)

/* AVR32 */
#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                    \
asm(                                \
    " ld.w   r2,%1            \n\t" \
    " add    r2,%0            \n\t" \
    " eor    r3,r3            \n\t" \
    " acr    r3               \n\t" \
    " macu.d r2,%3,%4         \n\t" \
    " st.w   %1,r2            \n\t" \
    " mov    %0,r3            \n\t" \
:"=r"(cy),"=r"(_c):"0"(cy),"r"(mu),"r"(*tmpm++),"1"(_c):"r2","r3");

#define PROPCARRY                    \
asm(                                 \
   " ld.w     r2,%1         \n\t"    \
   " add      r2,%0         \n\t"    \
   " st.w     %1,r2         \n\t"    \
   " eor      %0,%0         \n\t"    \
   " acr      %0            \n\t"    \
:"=r"(cy),"=r"(&_c[0]):"0"(cy),"1"(&_c[0]):"r2","%cc");

/******************************************************************/
#elif defined(TFM_MIPS)

/* MIPS */
#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                     \
asm(                                 \
   " multu    %3,%4          \n\t"   \
   " mflo     $12            \n\t"   \
   " mfhi     $13            \n\t"   \
   " addu     $12,$12,%0     \n\t"   \
   " sltu     $10,$12,%0     \n\t"   \
   " addu     $13,$13,$10    \n\t"   \
   " lw       $10,%1         \n\t"   \
   " addu     $12,$12,$10    \n\t"   \
   " sltu     $10,$12,$10    \n\t"   \
   " addu     %0,$13,$10     \n\t"   \
   " sw       $12,%1         \n\t"   \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"r"(mu),"r"(tmpm[0]),"1"(_c[0]):"$10","$12","$13"); ++tmpm;

#define PROPCARRY                    \
asm(                                 \
   " lw       $10,%1        \n\t"    \
   " addu     $10,$10,%0    \n\t"    \
   " sw       $10,%1        \n\t"    \
   " sltu     %0,$10,%0     \n\t"    \
:"=r"(cy),"=m"(_c[0]):"0"(cy),"1"(_c[0]):"$10");

/******************************************************************/
#else

/* ISO C code */
#define MONT_START 
#define MONT_FINI
#define LOOP_END
#define LOOP_START \
   mu = c[x] * mp

#define INNERMUL                                      \
   do { fp_word t;                                    \
   _c[0] = t  = ((fp_word)_c[0] + (fp_word)cy) +      \
                (((fp_word)mu) * ((fp_word)*tmpm++)); \
   cy = (t >> DIGIT_BIT);                             \
   } while (0)

#define PROPCARRY \
   do { fp_digit t = _c[0] += cy; cy = (t < cy); } while (0)

#endif
/******************************************************************/


#define LO  0

#ifdef TFM_SMALL_MONT_SET
#include "fp_mont_small.i"
#endif

/* computes x/R == x (mod N) via Montgomery Reduction */
void fp_montgomery_reduce(fp_int *a, fp_int *m, fp_digit mp)
{
   fp_digit c[FP_SIZE], *_c, *tmpm, mu;
   int      oldused, x, y, pa;

   /* bail if too large */
   if (m->used > (FP_SIZE/2)) {
      return;
   }

#ifdef TFM_SMALL_MONT_SET
   if (m->used <= 16) {
      fp_montgomery_reduce_small(a, m, mp);
      return;
   }
#endif

#if defined(USE_MEMSET)
   /* now zero the buff */
   memset(c, 0, sizeof c);
#endif
   pa = m->used;

   /* copy the input */
   oldused = a->used;
   for (x = 0; x < oldused; x++) {
       c[x] = a->dp[x];
   }
#if !defined(USE_MEMSET)
   for (; x < 2*pa+1; x++) {
       c[x] = 0;
   }
#endif
   MONT_START;

   for (x = 0; x < pa; x++) {
       fp_digit cy = 0;
       /* get Mu for this round */
       LOOP_START;
       _c   = c + x;
       tmpm = m->dp;
       y = 0;
       #if (defined(TFM_SSE2) || defined(TFM_X86_64))
        for (; y < (pa & ~7); y += 8) {
              INNERMUL8;
              _c   += 8;
              tmpm += 8;
           }
       #endif

       for (; y < pa; y++) {
          INNERMUL;
          ++_c;
       }
       LOOP_END;
       while (cy) {
           PROPCARRY;
           ++_c;
       }
  }         

  /* now copy out */
  _c   = c + pa;
  tmpm = a->dp;
  for (x = 0; x < pa+1; x++) {
     *tmpm++ = *_c++;
  }

  for (; x < oldused; x++)   {
     *tmpm++ = 0;
  }

  MONT_FINI;

  a->used = pa+1;
  fp_clamp(a);
  
  /* if A >= m then A = A - m */
  if (fp_cmp_mag (a, m) != FP_LT) {
    s_fp_sub (a, m, a);
  }
}


/* $Source$ */
/* $Revision$ */
/* $Date$ */
