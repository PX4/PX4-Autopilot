#include <stdio.h>

int main(void)
{
   int x, y, z;

printf(
#if 1
"#ifdef TFM_SMALL_SET\n"
"/* computes x/R == x (mod N) via Montgomery Reduction */\n"
"void fp_montgomery_reduce_small(fp_int *a, fp_int *m, fp_digit mp)\n"
"{\n"
"   fp_digit c[FP_SIZE], *_c, *tmpm, mu, cy;\n"
"   int      oldused, x, y, pa;\n"
"\n"
"#if defined(USE_MEMSET)\n"
"   /* now zero the buff */\n"
"   memset(c, 0, sizeof c);\n"
"#endif\n"
"   pa = m->used;\n"
"\n"
"   /* copy the input */\n"
"   oldused = a->used;\n"
"   for (x = 0; x < oldused; x++) {\n"
"       c[x] = a->dp[x];\n"
"   }\n"
"#if !defined(USE_MEMSET)\n"
"   for (; x < 2*pa+3; x++) {\n"
"       c[x] = 0;\n"
"   }\n"
"#endif\n"
"   MONT_START;\n"
#endif
"\n"
"   switch (pa) {\n");

for (x = 1; x <= 16; x++) {
if (x > 16 && (x != 32 && x != 48 && x != 64)) continue;
if (x > 16) printf("#ifdef TFM_HUGE\n");



printf("      case %d:\n", x);

for (y = 0; y < x; y++) {

printf("            x = %d; cy   = 0;\n"
       "            LOOP_START;\n"
       "            _c   = c + %d;\n"
       "            tmpm = m->dp;\n", y, y);

printf("#ifdef INNERMUL8\n");
for (z = 0; z+8 <= x; z += 8) {
printf("            INNERMUL8; _c += 8; tmpm += 8;\n");
}
for (; z < x; z++) {
printf("            INNERMUL; ++_c;\n");
}
printf("#else\n");
for (z = 0; z < x; z++) {
printf("            INNERMUL; ++_c;\n");
}
printf("#endif\n");
printf("            LOOP_END;\n"
       "            while (cy) {\n"
       "               PROPCARRY;\n"
       "               ++_c;\n"
       "            }\n");
}
//printf("         }\n");
printf("         break;\n");



#define LOOP_MACRO(stride)                                 \
   for (x = 0; x < stride; x++) {                          \
       fp_digit cy = 0;                                    \
       /* get Mu for this round */                         \
       LOOP_START;                                         \
       _c   = c + x;                                       \
       tmpm = m->dp;                                       \
       for (y = 0; y < stride; y++) {                      \
          INNERMUL;                                        \
          ++_c;                                            \
       }                                                   \
       LOOP_END;                                           \
       while (cy) {                                        \
           PROPCARRY;                                      \
           ++_c;                                           \
       }                                                   \
  }         





if (x > 16) printf("#endif /* TFM_HUGE */\n");


}

#if 1

printf(
"  }\n"
"  /* now copy out */\n"
"  _c   = c + pa;\n"
"  tmpm = a->dp;\n"
"  for (x = 0; x < pa+1; x++) {\n"
"     *tmpm++ = *_c++;\n"
"  }\n"
"\n"
"  for (; x < oldused; x++)   {\n"
"     *tmpm++ = 0;\n"
"  }\n"
"\n"
"  MONT_FINI;\n"
"\n"
"  a->used = pa+1;\n"
"  fp_clamp(a);\n"
"\n"  
"  /* if A >= m then A = A - m */\n"
"  if (fp_cmp_mag (a, m) != FP_LT) {\n"
"    s_fp_sub (a, m, a);\n"
"  }\n"
"}\n\n#endif\n");

#endif


return 0;
}
