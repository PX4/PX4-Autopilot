/*
 * 
 * This project is meant to fill in where LibTomMath
 * falls short.  That is speed ;-)
 *
 * This project is public domain and free for all purposes.
 * 
 * Tom St Denis, tomstdenis@gmail.com
 */
#include <tfm.h>

#if defined(TFM_PRESCOTT) && defined(TFM_SSE2)
   #undef TFM_SSE2
   #define TFM_X86
#endif

#if defined(TFM_X86)

/* x86-32 optimized */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

#define SQRADD(i, j)                                      \
asm(                                            \
     "movl  %6,%%eax     \n\t"                            \
     "mull  %%eax        \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "m"(i) :"%eax","%edx","%cc");

#define SQRADD2(i, j)                                     \
asm(                                            \
     "movl  %6,%%eax     \n\t"                            \
     "mull  %7           \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "m"(i), "m"(j)  :"%eax","%edx","%cc");

#define SQRADDSC(i, j)                                    \
asm(                                                     \
     "movl  %6,%%eax     \n\t"                            \
     "mull  %7           \n\t"                            \
     "movl  %%eax,%0     \n\t"                            \
     "movl  %%edx,%1     \n\t"                            \
     "xorl  %2,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "g"(i), "g"(j) :"%eax","%edx","%cc");

#define SQRADDAC(i, j)                                    \
asm(                                                     \
     "movl  %6,%%eax     \n\t"                            \
     "mull  %7           \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "g"(i), "g"(j) :"%eax","%edx","%cc");

#define SQRADDDB                                          \
asm(                                                     \
     "addl %6,%0         \n\t"                            \
     "adcl %7,%1         \n\t"                            \
     "adcl %8,%2         \n\t"                            \
     "addl %6,%0         \n\t"                            \
     "adcl %7,%1         \n\t"                            \
     "adcl %8,%2         \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2) : "0"(c0), "1"(c1), "2"(c2), "r"(sc0), "r"(sc1), "r"(sc2) : "%cc");

#elif defined(TFM_X86_64)
/* x86-64 optimized */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

#define SQRADD(i, j)                                      \
asm(                                                     \
     "movq  %6,%%rax     \n\t"                            \
     "mulq  %%rax        \n\t"                            \
     "addq  %%rax,%0     \n\t"                            \
     "adcq  %%rdx,%1     \n\t"                            \
     "adcq  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "g"(i) :"%rax","%rdx","%cc");

#define SQRADD2(i, j)                                     \
asm(                                                     \
     "movq  %6,%%rax     \n\t"                            \
     "mulq  %7           \n\t"                            \
     "addq  %%rax,%0     \n\t"                            \
     "adcq  %%rdx,%1     \n\t"                            \
     "adcq  $0,%2        \n\t"                            \
     "addq  %%rax,%0     \n\t"                            \
     "adcq  %%rdx,%1     \n\t"                            \
     "adcq  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "g"(i), "g"(j)  :"%rax","%rdx","%cc");

#define SQRADDSC(i, j)                                    \
asm(                                                     \
     "movq  %6,%%rax     \n\t"                            \
     "mulq  %7           \n\t"                            \
     "movq  %%rax,%0     \n\t"                            \
     "movq  %%rdx,%1     \n\t"                            \
     "xorq  %2,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "g"(i), "g"(j) :"%rax","%rdx","%cc");

#define SQRADDAC(i, j)                                                         \
asm(                                                     \
     "movq  %6,%%rax     \n\t"                            \
     "mulq  %7           \n\t"                            \
     "addq  %%rax,%0     \n\t"                            \
     "adcq  %%rdx,%1     \n\t"                            \
     "adcq  $0,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "g"(i), "g"(j) :"%rax","%rdx","%cc");

#define SQRADDDB                                          \
asm(                                                     \
     "addq %6,%0         \n\t"                            \
     "adcq %7,%1         \n\t"                            \
     "adcq %8,%2         \n\t"                            \
     "addq %6,%0         \n\t"                            \
     "adcq %7,%1         \n\t"                            \
     "adcq %8,%2         \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2) : "0"(c0), "1"(c1), "2"(c2), "r"(sc0), "r"(sc1), "r"(sc2) : "%cc");

#elif defined(TFM_SSE2)

/* SSE2 Optimized */
#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI \
   asm("emms");

#define SQRADD(i, j)                                      \
asm(                                            \
     "movd  %6,%%mm0     \n\t"                            \
     "pmuludq %%mm0,%%mm0\n\t"                            \
     "movd  %%mm0,%%eax  \n\t"                            \
     "psrlq $32,%%mm0    \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "movd  %%mm0,%%eax  \n\t"                            \
     "adcl  %%eax,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "m"(i) :"%eax","%cc");

#define SQRADD2(i, j)                                     \
asm(                                            \
     "movd  %6,%%mm0     \n\t"                            \
     "movd  %7,%%mm1     \n\t"                            \
     "pmuludq %%mm1,%%mm0\n\t"                            \
     "movd  %%mm0,%%eax  \n\t"                            \
     "psrlq $32,%%mm0    \n\t"                            \
     "movd  %%mm0,%%edx  \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2): "0"(c0), "1"(c1), "2"(c2), "m"(i), "m"(j)  :"%eax","%edx","%cc");

#define SQRADDSC(i, j)                                                         \
asm(                                            \
     "movd  %6,%%mm0     \n\t"                            \
     "movd  %7,%%mm1     \n\t"                            \
     "pmuludq %%mm1,%%mm0\n\t"                            \
     "movd  %%mm0,%0     \n\t"                            \
     "psrlq $32,%%mm0    \n\t"                            \
     "movd  %%mm0,%1     \n\t"                            \
     "xorl  %2,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "m"(i), "m"(j));

#define SQRADDAC(i, j)                                                         \
asm(                                            \
     "movd  %6,%%mm0     \n\t"                            \
     "movd  %7,%%mm1     \n\t"                            \
     "pmuludq %%mm1,%%mm0\n\t"                            \
     "movd  %%mm0,%%eax  \n\t"                            \
     "psrlq $32,%%mm0    \n\t"                            \
     "movd  %%mm0,%%edx  \n\t"                            \
     "addl  %%eax,%0     \n\t"                            \
     "adcl  %%edx,%1     \n\t"                            \
     "adcl  $0,%2        \n\t"                            \
     :"=r"(sc0), "=r"(sc1), "=r"(sc2): "0"(sc0), "1"(sc1), "2"(sc2), "m"(i), "m"(j)  :"%eax","%edx","%cc");

#define SQRADDDB                                          \
asm(                                                     \
     "addl %6,%0         \n\t"                            \
     "adcl %7,%1         \n\t"                            \
     "adcl %8,%2         \n\t"                            \
     "addl %6,%0         \n\t"                            \
     "adcl %7,%1         \n\t"                            \
     "adcl %8,%2         \n\t"                            \
     :"=r"(c0), "=r"(c1), "=r"(c2) : "0"(c0), "1"(c1), "2"(c2), "r"(sc0), "r"(sc1), "r"(sc2) : "%cc");

#elif defined(TFM_ARM)

/* ARM code */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)                                             \
asm(                                                             \
"  UMULL  r0,r1,%6,%6              \n\t"                         \
"  ADDS   %0,%0,r0                 \n\t"                         \
"  ADCS   %1,%1,r1                 \n\t"                         \
"  ADC    %2,%2,#0                 \n\t"                         \
:"=r"(c0), "=r"(c1), "=r"(c2) : "0"(c0), "1"(c1), "2"(c2), "r"(i) : "r0", "r1", "%cc");
	
/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)                                            \
asm(                                                             \
"  UMULL  r0,r1,%6,%7              \n\t"                         \
"  ADDS   %0,%0,r0                 \n\t"                         \
"  ADCS   %1,%1,r1                 \n\t"                         \
"  ADC    %2,%2,#0                 \n\t"                         \
"  ADDS   %0,%0,r0                 \n\t"                         \
"  ADCS   %1,%1,r1                 \n\t"                         \
"  ADC    %2,%2,#0                 \n\t"                         \
:"=r"(c0), "=r"(c1), "=r"(c2) : "0"(c0), "1"(c1), "2"(c2), "r"(i), "r"(j) : "r0", "r1", "%cc");

#define SQRADDSC(i, j)                                           \
asm(                                                             \
"  UMULL  %0,%1,%6,%7              \n\t"                         \
"  SUB    %2,%2,%2                 \n\t"                         \
:"=r"(sc0), "=r"(sc1), "=r"(sc2) : "0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j) : "%cc");

#define SQRADDAC(i, j)                                           \
asm(                                                             \
"  UMULL  r0,r1,%6,%7              \n\t"                         \
"  ADDS   %0,%0,r0                 \n\t"                         \
"  ADCS   %1,%1,r1                 \n\t"                         \
"  ADC    %2,%2,#0                 \n\t"                         \
:"=r"(sc0), "=r"(sc1), "=r"(sc2) : "0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j) : "r0", "r1", "%cc");

#define SQRADDDB                                                 \
asm(                                                             \
"  ADDS  %0,%0,%3                     \n\t"                      \
"  ADCS  %1,%1,%4                     \n\t"                      \
"  ADC   %2,%2,%5                     \n\t"                      \
"  ADDS  %0,%0,%3                     \n\t"                      \
"  ADCS  %1,%1,%4                     \n\t"                      \
"  ADC   %2,%2,%5                     \n\t"                      \
:"=r"(c0), "=r"(c1), "=r"(c2) : "r"(sc0), "r"(sc1), "r"(sc2), "0"(c0), "1"(c1), "2"(c2) : "%cc");

#elif defined(TFM_PPC32)

/* PPC32 */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)             \
asm(                             \
   " mullw  16,%6,%6       \n\t" \
   " addc   %0,%0,16       \n\t" \
   " mulhwu 16,%6,%6       \n\t" \
   " adde   %1,%1,16       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i):"16","%cc");

/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)            \
asm(                             \
   " mullw  16,%6,%7       \n\t" \
   " mulhwu 17,%6,%7       \n\t" \
   " addc   %0,%0,16       \n\t" \
   " adde   %1,%1,17       \n\t" \
   " addze  %2,%2          \n\t" \
   " addc   %0,%0,16       \n\t" \
   " adde   %1,%1,17       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i), "r"(j):"16", "17","%cc");

#define SQRADDSC(i, j)            \
asm(                              \
   " mullw  %0,%6,%7        \n\t" \
   " mulhwu %1,%6,%7        \n\t" \
   " xor    %2,%2,%2        \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i),"r"(j) : "%cc");

#define SQRADDAC(i, j)           \
asm(                             \
   " mullw  16,%6,%7       \n\t" \
   " addc   %0,%0,16       \n\t" \
   " mulhwu 16,%6,%7       \n\t" \
   " adde   %1,%1,16       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j):"16", "%cc");

#define SQRADDDB                  \
asm(                              \
   " addc   %0,%0,%3        \n\t" \
   " adde   %1,%1,%4        \n\t" \
   " adde   %2,%2,%5        \n\t" \
   " addc   %0,%0,%3        \n\t" \
   " adde   %1,%1,%4        \n\t" \
   " adde   %2,%2,%5        \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2) : "r"(sc0), "r"(sc1), "r"(sc2), "0"(c0), "1"(c1), "2"(c2) : "%cc");

#elif defined(TFM_PPC64)
/* PPC64 */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)             \
asm(                             \
   " mulld  r16,%6,%6       \n\t" \
   " addc   %0,%0,r16       \n\t" \
   " mulhdu r16,%6,%6       \n\t" \
   " adde   %1,%1,r16       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i):"r16","%cc");

/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)            \
asm(                             \
   " mulld  r16,%6,%7       \n\t" \
   " mulhdu r17,%6,%7       \n\t" \
   " addc   %0,%0,r16       \n\t" \
   " adde   %1,%1,r17       \n\t" \
   " addze  %2,%2          \n\t" \
   " addc   %0,%0,r16       \n\t" \
   " adde   %1,%1,r17       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i), "r"(j):"r16", "r17","%cc");

#define SQRADDSC(i, j)            \
asm(                              \
   " mulld  %0,%6,%7        \n\t" \
   " mulhdu %1,%6,%7        \n\t" \
   " xor    %2,%2,%2        \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i),"r"(j) : "%cc");

#define SQRADDAC(i, j)           \
asm(                             \
   " mulld  r16,%6,%7       \n\t" \
   " addc   %0,%0,r16       \n\t" \
   " mulhdu r16,%6,%7       \n\t" \
   " adde   %1,%1,r16       \n\t" \
   " addze  %2,%2          \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j):"r16", "%cc");

#define SQRADDDB                  \
asm(                              \
   " addc   %0,%0,%3        \n\t" \
   " adde   %1,%1,%4        \n\t" \
   " adde   %2,%2,%5        \n\t" \
   " addc   %0,%0,%3        \n\t" \
   " adde   %1,%1,%4        \n\t" \
   " adde   %2,%2,%5        \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2) : "r"(sc0), "r"(sc1), "r"(sc2), "0"(c0), "1"(c1), "2"(c2) : "%cc");


#elif defined(TFM_AVR32)

/* AVR32 */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)             \
asm(                             \
   " mulu.d r2,%6,%6       \n\t" \
   " add    %0,%0,r2       \n\t" \
   " adc    %1,%1,r3       \n\t" \
   " acr    %2             \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i):"r2","r3");

/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)            \
asm(                             \
   " mulu.d r2,%6,%7       \n\t" \
   " add    %0,%0,r2       \n\t" \
   " adc    %1,%1,r3       \n\t" \
   " acr    %2,            \n\t" \
   " add    %0,%0,r2       \n\t" \
   " adc    %1,%1,r3       \n\t" \
   " acr    %2,            \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i), "r"(j):"r2", "r3");

#define SQRADDSC(i, j)            \
asm(                              \
   " mulu.d r2,%6,%7        \n\t" \
   " mov    %0,r2           \n\t" \
   " mov    %1,r3           \n\t" \
   " eor    %2,%2           \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i),"r"(j) : "r2", "r3");

#define SQRADDAC(i, j)           \
asm(                             \
   " mulu.d r2,%6,%7       \n\t" \
   " add    %0,%0,r2       \n\t" \
   " adc    %1,%1,r3       \n\t" \
   " acr    %2             \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j):"r2", "r3");

#define SQRADDDB                  \
asm(                              \
   " add    %0,%0,%3        \n\t" \
   " adc    %1,%1,%4        \n\t" \
   " adc    %2,%2,%5        \n\t" \
   " add    %0,%0,%3        \n\t" \
   " adc    %1,%1,%4        \n\t" \
   " adc    %2,%2,%5        \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2) : "r"(sc0), "r"(sc1), "r"(sc2), "0"(c0), "1"(c1), "2"(c2) : "%cc");

#elif defined(TFM_MIPS)

/* MIPS */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)              \
asm(                              \
   " multu  %6,%6          \n\t"  \
   " mflo   $12            \n\t"  \
   " mfhi   $13            \n\t"  \
   " addu    %0,%0,$12     \n\t"  \
   " sltu   $12,%0,$12     \n\t"  \
   " addu    %1,%1,$13     \n\t"  \
   " sltu   $13,%1,$13     \n\t"  \
   " addu    %1,%1,$12     \n\t"  \
   " sltu   $12,%1,$12     \n\t"  \
   " addu    %2,%2,$13     \n\t"  \
   " addu    %2,%2,$12     \n\t"  \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i):"$12","$13");

/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)            \
asm(                             \
   " multu  %6,%7          \n\t" \
   " mflo   $12            \n\t" \
   " mfhi   $13            \n\t" \
                                 \
   " addu    %0,%0,$12     \n\t" \
   " sltu   $14,%0,$12     \n\t" \
   " addu    %1,%1,$13     \n\t" \
   " sltu   $15,%1,$13     \n\t" \
   " addu    %1,%1,$14     \n\t" \
   " sltu   $14,%1,$14     \n\t" \
   " addu    %2,%2,$15     \n\t" \
   " addu    %2,%2,$14     \n\t" \
                                 \
   " addu    %0,%0,$12     \n\t" \
   " sltu   $14,%0,$12     \n\t" \
   " addu    %1,%1,$13     \n\t" \
   " sltu   $15,%1,$13     \n\t" \
   " addu    %1,%1,$14     \n\t" \
   " sltu   $14,%1,$14     \n\t" \
   " addu    %2,%2,$15     \n\t" \
   " addu    %2,%2,$14     \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2):"0"(c0), "1"(c1), "2"(c2), "r"(i), "r"(j):"$12", "$13", "$14", "$15");

#define SQRADDSC(i, j)            \
asm(                              \
   " multu  %6,%7          \n\t"  \
   " mflo   %0             \n\t"  \
   " mfhi   %1             \n\t"  \
   " xor    %2,%2,%2       \n\t"  \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i),"r"(j) : "%cc");

#define SQRADDAC(i, j)           \
asm(                             \
   " multu  %6,%7          \n\t" \
   " mflo   $12            \n\t" \
   " mfhi   $13            \n\t" \
   " addu    %0,%0,$12     \n\t" \
   " sltu   $12,%0,$12     \n\t" \
   " addu    %1,%1,$13     \n\t" \
   " sltu   $13,%1,$13     \n\t" \
   " addu    %1,%1,$12     \n\t" \
   " sltu   $12,%1,$12     \n\t" \
   " addu    %2,%2,$13     \n\t" \
   " addu    %2,%2,$12     \n\t" \
:"=r"(sc0), "=r"(sc1), "=r"(sc2):"0"(sc0), "1"(sc1), "2"(sc2), "r"(i), "r"(j):"$12", "$13", "$14");

#define SQRADDDB                  \
asm(                              \
   " addu    %0,%0,%3       \n\t" \
   " sltu   $10,%0,%3       \n\t" \
   " addu    %1,%1,$10      \n\t" \
   " sltu   $10,%1,$10      \n\t" \
   " addu    %1,%1,%4       \n\t" \
   " sltu   $11,%1,%4       \n\t" \
   " addu    %2,%2,$10      \n\t" \
   " addu    %2,%2,$11      \n\t" \
   " addu    %2,%2,%5       \n\t" \
                                  \
   " addu    %0,%0,%3       \n\t" \
   " sltu   $10,%0,%3       \n\t" \
   " addu    %1,%1,$10      \n\t" \
   " sltu   $10,%1,$10      \n\t" \
   " addu    %1,%1,%4       \n\t" \
   " sltu   $11,%1,%4       \n\t" \
   " addu    %2,%2,$10      \n\t" \
   " addu    %2,%2,$11      \n\t" \
   " addu    %2,%2,%5       \n\t" \
:"=r"(c0), "=r"(c1), "=r"(c2) : "r"(sc0), "r"(sc1), "r"(sc2), "0"(c0), "1"(c1), "2"(c2) : "$10", "$11");

#else

#define TFM_ISO

/* ISO C portable code */

#define COMBA_START

#define CLEAR_CARRY \
   c0 = c1 = c2 = 0;

#define COMBA_STORE(x) \
   x = c0;

#define COMBA_STORE2(x) \
   x = c1;

#define CARRY_FORWARD \
   do { c0 = c1; c1 = c2; c2 = 0; } while (0);

#define COMBA_FINI

/* multiplies point i and j, updates carry "c1" and digit c2 */
#define SQRADD(i, j)                                 \
   do { fp_word t;                                   \
   t = c0 + ((fp_word)i) * ((fp_word)j);  c0 = t;    \
   t = c1 + (t >> DIGIT_BIT);             c1 = t; c2 += t >> DIGIT_BIT; \
   } while (0);
  

/* for squaring some of the terms are doubled... */
#define SQRADD2(i, j)                                                 \
   do { fp_word t;                                                    \
   t  = ((fp_word)i) * ((fp_word)j);                                  \
   tt = (fp_word)c0 + t;                 c0 = tt;                              \
   tt = (fp_word)c1 + (tt >> DIGIT_BIT); c1 = tt; c2 += tt >> DIGIT_BIT;       \
   tt = (fp_word)c0 + t;                 c0 = tt;                              \
   tt = (fp_word)c1 + (tt >> DIGIT_BIT); c1 = tt; c2 += tt >> DIGIT_BIT;       \
   } while (0);

#define SQRADDSC(i, j)                                                         \
   do { fp_word t;                                                             \
      t =  ((fp_word)i) * ((fp_word)j);                                        \
      sc0 = (fp_digit)t; sc1 = (t >> DIGIT_BIT); sc2 = 0;                      \
   } while (0);

#define SQRADDAC(i, j)                                                         \
   do { fp_word t;                                                             \
   t = sc0 + ((fp_word)i) * ((fp_word)j);  sc0 = t;                            \
   t = sc1 + (t >> DIGIT_BIT);             sc1 = t; sc2 += t >> DIGIT_BIT;     \
   } while (0);

#define SQRADDDB                                                               \
   do { fp_word t;                                                             \
   t = ((fp_word)sc0) + ((fp_word)sc0) + c0; c0 = t;                                                 \
   t = ((fp_word)sc1) + ((fp_word)sc1) + c1 + (t >> DIGIT_BIT); c1 = t;                              \
   c2 = c2 + ((fp_word)sc2) + ((fp_word)sc2) + (t >> DIGIT_BIT);                                     \
   } while (0);

#endif

/* $Source$ */
/* $Revision$ */
/* $Date$ */
