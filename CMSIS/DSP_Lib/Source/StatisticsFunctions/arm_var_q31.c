/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_var_q31.c    
*    
* Description:	Variance of an array of Q31 type.    
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
*   
* Version 1.0.10 2011/7/15  
*    Big Endian support added and Merged M0 and M3/M4 Source code.   
*    
* Version 1.0.3 2010/11/29   
*    Re-organized the CMSIS folders and updated documentation.    
*     
* Version 1.0.2 2010/11/11    
*    Documentation updated.     
*    
* Version 1.0.1 2010/10/05     
*    Production release and review comments incorporated.    
*    
* Version 1.0.0 2010/09/20     
*    Production release and review comments incorporated.    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**        
 * @ingroup groupStats        
 */

/**        
 * @addtogroup variance        
 * @{        
 */

/**        
 * @brief Variance of the elements of a Q31 vector.        
 * @param[in]       *pSrc points to the input vector        
 * @param[in]       blockSize length of the input vector        
 * @param[out]      *pResult variance value returned here        
 * @return none.        
 *        
 * @details        
 * <b>Scaling and Overflow Behavior:</b>        
 *        
 *\par        
 * The function is implemented using an internal 64-bit accumulator.        
 * The input is represented in 1.31 format, and intermediate multiplication        
 * yields a 2.62 format.        
 * The accumulator maintains full precision of the intermediate multiplication results,         
 * but provides only a single guard bit.        
 * There is no saturation on intermediate additions.        
 * If the accumulator overflows it wraps around and distorts the result.        
 * In order to avoid overflows completely the input signal must be scaled down by         
 * log2(blockSize) bits, as a total of blockSize additions are performed internally.         
 * Finally, the 2.62 accumulator is right shifted by 31 bits to yield a 1.31 format value.        
 *        
 */


void arm_var_q31(
  q31_t * pSrc,
  uint32_t blockSize,
  q63_t * pResult)
{
  q63_t sum = 0, sumSquare = 0;                  /* Accumulator */
  q31_t meanOfSquares, squareOfMean;             /* square of mean and mean of square */
  q31_t mean;                                    /* mean */
  q31_t in;                                      /* input value */
  q31_t t;                                       /* Temporary variable */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  q63_t sumSquare1 = 0;                          /* Accumulator */
  q31_t in1, in2, in3, in4;                      /* Temporary input variables */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1])  */
    /* Compute Sum of squares of the input samples        
     * and then store the result in a temporary variable, sum. */
    /* read input samples from source buffer */
    in1 = pSrc[0];
    in2 = pSrc[1];

    /* calculate sum of inputs */
    sum += in1;
    /* calculate sum of squares */
    sumSquare += ((q63_t) (in1) * (in1));
    in3 = pSrc[2];
    sum += in2;
    sumSquare1 += ((q63_t) (in2) * (in2));
    in4 = pSrc[3];
    sum += in3;
    sumSquare += ((q63_t) (in3) * (in3));
    sum += in4;
    sumSquare1 += ((q63_t) (in4) * (in4));

    /* update input pointer to process next samples */
    pSrc += 4u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* add two accumulators */
  sumSquare = sumSquare + sumSquare1;

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0 */

  while(blkCnt > 0u)
  {
    /* C = (A[0] * A[0] + A[1] * A[1] + ... + A[blockSize-1] * A[blockSize-1]) */
    /* Compute Sum of squares of the input samples        
     * and then store the result in a temporary variable, sum. */
    in = *pSrc++;
    sumSquare += ((q63_t) (in) * (in));
    sum += in;

    /* Decrement the loop counter */
    blkCnt--;
  }

  t = (q31_t) ((1.0f / (float32_t) (blockSize - 1u)) * 1073741824.0f);

  /* Compute Mean of squares of the input samples        
   * and then store the result in a temporary variable, meanOfSquares. */
  sumSquare = (sumSquare >> 31);
  meanOfSquares = (q31_t) ((sumSquare * t) >> 30);

  /* Compute mean of all input values */
  t = (q31_t) ((1.0f / (blockSize * (blockSize - 1u))) * 2147483648.0f);
  mean = (q31_t) (sum);

  /* Compute square of mean */
  squareOfMean = (q31_t) (((q63_t) mean * mean) >> 31);
  squareOfMean = (q31_t) (((q63_t) squareOfMean * t) >> 31);

  /* Compute variance and then store the result to the destination */
  *pResult = (q63_t) meanOfSquares - squareOfMean;

}

/**        
 * @} end of variance group        
 */
