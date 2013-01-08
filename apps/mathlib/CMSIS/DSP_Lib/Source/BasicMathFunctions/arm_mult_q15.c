/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_mult_q15.c    
*    
* Description:	Q15 vector multiplication.    
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
*    
* Version 0.0.5  2010/04/26     
*    incorporated review comments and updated with latest CMSIS layer    
*    
* Version 0.0.3  2010/03/10     
*    Initial version    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupMath    
 */

/**    
 * @addtogroup BasicMult    
 * @{    
 */


/**    
 * @brief           Q15 vector multiplication    
 * @param[in]       *pSrcA points to the first input vector    
 * @param[in]       *pSrcB points to the second input vector    
 * @param[out]      *pDst points to the output vector    
 * @param[in]       blockSize number of samples in each vector    
 * @return none.    
 *    
 * <b>Scaling and Overflow Behavior:</b>    
 * \par    
 * The function uses saturating arithmetic.    
 * Results outside of the allowable Q15 range [0x8000 0x7FFF] will be saturated.    
 */

void arm_mult_q15(
  q15_t * pSrcA,
  q15_t * pSrcB,
  q15_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counters */

#ifndef ARM_MATH_CM0

/* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t inA1, inA2, inB1, inB2;                  /* temporary input variables */
  q15_t out1, out2, out3, out4;                  /* temporary output variables */
  q31_t mul1, mul2, mul3, mul4;                  /* temporary variables */

  /* loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.        
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* read two samples at a time from sourceA */
    inA1 = *__SIMD32(pSrcA)++;
    /* read two samples at a time from sourceB */
    inB1 = *__SIMD32(pSrcB)++;
    /* read two samples at a time from sourceA */
    inA2 = *__SIMD32(pSrcA)++;
    /* read two samples at a time from sourceB */
    inB2 = *__SIMD32(pSrcB)++;

    /* multiply mul = sourceA * sourceB */
    mul1 = (q31_t) ((q15_t) (inA1 >> 16) * (q15_t) (inB1 >> 16));
    mul2 = (q31_t) ((q15_t) inA1 * (q15_t) inB1);
    mul3 = (q31_t) ((q15_t) (inA2 >> 16) * (q15_t) (inB2 >> 16));
    mul4 = (q31_t) ((q15_t) inA2 * (q15_t) inB2);

    /* saturate result to 16 bit */
    out1 = (q15_t) __SSAT(mul1 >> 15, 16);
    out2 = (q15_t) __SSAT(mul2 >> 15, 16);
    out3 = (q15_t) __SSAT(mul3 >> 15, 16);
    out4 = (q15_t) __SSAT(mul4 >> 15, 16);

    /* store the result */
#ifndef ARM_MATH_BIG_ENDIAN

    *__SIMD32(pDst)++ = __PKHBT(out2, out1, 16);
    *__SIMD32(pDst)++ = __PKHBT(out4, out3, 16);

#else

    *__SIMD32(pDst)++ = __PKHBT(out2, out1, 16);
    *__SIMD32(pDst)++ = __PKHBT(out4, out3, 16);

#endif //      #ifndef ARM_MATH_BIG_ENDIAN

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.    
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /* #ifndef ARM_MATH_CM0 */


  while(blkCnt > 0u)
  {
    /* C = A * B */
    /* Multiply the inputs and store the result in the destination buffer */
    *pDst++ = (q15_t) __SSAT((((q31_t) (*pSrcA++) * (*pSrcB++)) >> 15), 16);

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }
}

/**    
 * @} end of BasicMult group    
 */
