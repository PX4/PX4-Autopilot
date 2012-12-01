/****************************************************************************
 * libc/stdlib/lib_rand.c
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIB_RAND_ORDER
#  define CONFIG_LIB_RAND_ORDER 1
#endif

/* Values needed by the random number generator */

#define RND1_CONSTK  470001
#define RND1_CONSTP  999563
#define RND2_CONSTK1 366528
#define RND2_CONSTK2 508531
#define RND2_CONSTP  998917
#define RND3_CONSTK1 360137
#define RND3_CONSTK2 519815
#define RND3_CONSTK3 616087
#define RND3_CONSTP  997783

#if CONFIG_LIB_RAND_ORDER == 1
#  define RND_CONSTP RND1_CONSTP
#elif CONFIG_LIB_RAND_ORDER == 2
#  define RND_CONSTP RND2_CONSTP
#else
#  define RND_CONSTP RND3_CONSTP
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static unsigned int nrand(unsigned int nLimit);
static double_t frand1(void);
#if (CONFIG_LIB_RAND_ORDER > 1)
static double_t frand2(void);
#if (CONFIG_LIB_RAND_ORDER > 2)
static double_t frand3(void);
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static unsigned long g_randint1;
#if (CONFIG_LIB_RAND_ORDER > 1)
static unsigned long g_randint2;
#if (CONFIG_LIB_RAND_ORDER > 2)
static unsigned long g_randint3;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned int nrand(unsigned int nLimit)
{
  unsigned long result;
  double_t ratio;

  /* Loop to be sure a legal random number is generated */

  do
    {
      /* Get a random integer in the requested range */

#if (CONFIG_LIB_RAND_ORDER == 1)
      ratio = frand1();
#elif (CONFIG_LIB_RAND_ORDER == 2)
      ratio = frand2();
#else
      ratio = frand3();
#endif

      /* Then, produce the return-able value */

      result = (unsigned long)(((double_t)nLimit) * ratio);
    }
  while (result >= (unsigned long)nLimit);

  return (unsigned int)result;
}

static double_t frand1(void)
{
  unsigned long randint;

  /* First order congruential generator.  One is added to the result of the
   * generated value to avoid the value zero which breaks the logic.
   */

  randint    = (RND1_CONSTK * g_randint1) % RND1_CONSTP + 1;
  g_randint1 = randint;

  /* Construct an floating point value in the range from 0.0 up to 1.0 */

  return ((double_t)randint) / ((double_t)RND_CONSTP);
}

#if (CONFIG_LIB_RAND_ORDER > 1)
static double_t frand2(void)
{
  unsigned long randint;

  /* Second order congruential generator.  One is added to the result of the
   * generated value to avoid the value zero which breaks the logic.
   */

  randint    = (RND2_CONSTK1 * g_randint1 +
                RND2_CONSTK2 * g_randint2) % RND2_CONSTP + 1;

  g_randint2 = g_randint1;
  g_randint1 = randint;

  /* Construct an floating point value in the range from 0.0 up to 1.0 */

  return ((double_t)randint) / ((double_t)RND_CONSTP);

}

#if (CONFIG_LIB_RAND_ORDER > 2)
static double_t frand3(void)
{
  unsigned long randint;

  /* Third order congruential generator.  One is added to the result of the
   * generated value to avoid the value zero which breaks the logic.
   */

  randint    = (RND3_CONSTK1 * g_randint1 +
                RND3_CONSTK2 * g_randint2 +
                RND3_CONSTK2 * g_randint3) % RND3_CONSTP + 1;

  g_randint3 = g_randint2;
  g_randint2 = g_randint1;
  g_randint1 = randint;

  /* Construct an floating point value in the range from 0.0 up to 1.0 */

  return ((double_t)randint) / ((double_t)RND_CONSTP);
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  srand, rand
 ****************************************************************************/

void srand(unsigned int seed)
{
  g_randint1 = seed;
#if (CONFIG_LIB_RAND_ORDER > 1)
  g_randint2 = seed;
  (void)frand1();
#if (CONFIG_LIB_RAND_ORDER > 2)
  g_randint3 = seed;
  (void)frand2();
#endif
#endif
}

int rand(void)
{
  return (int)nrand(32768);
}
