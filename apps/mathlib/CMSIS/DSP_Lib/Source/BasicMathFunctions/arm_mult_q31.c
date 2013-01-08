/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_mult_q31.c    
*    
* Description:	Q31 vector multiplication.    
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
 * @brief Q31 vector multiplication.    
 * @param[in]       *pSrcA points to the first input vector    
 * @param[in]       *pSrcB points to the second input vector    
 * @param[out]      *pDst points to the output vector    
 * @param[in]       blockSize number of samples in each vector    
 * @return none.    
 *    
 * <b>Scaling and Overflow Behavior:</b>    
 * \par    
 * The function uses saturating arithmetic.    
 * Results outside of the allowable Q31 range[0x80000000 0x7FFFFFFF] will be saturated.    
 */

void arm_mult_q31(
  q31_t * pSrcA,
  q31_t * pSrcB,
  q31_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counters */

#ifndef ARM_MATH_CM0

/* Run the below code for Cortex-M4 and Cortex-M3 */
  q31_t inA1, inA2, inA3, inA4;                  /* temporary input variables */
  q31_t inB1, inB2, inB3, inB4;                  /* temporary input variables */
  q31_t out1, out2, out3, out4;                  /* temporary output variables */

  /* loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = A * B */
    /* Multiply the inputs and then store the results in the destination buffer. */
    inA1 = *pSrcA++;
    inA2 = *pSrcA++;
    inA3 = *pSrcA++;
    inA4 = *pSrcA++;
    inB1 = *pSrcB++;
    inB2 = *pSrcB++;
    inB3 = *pSrcB++;
    inB4 = *pSrcB++;

    out1 = ((q63_t) inA1 * inB1) >> 32;
    out2 = ((q63_t) inA2 * inB2) >> 32;
    out3 = ((q63_t) inA3 * inB3) >> 32;
    out4 = ((q63_t) inA4 * inB4) >> 32;

    out1 = __SSAT(out1, 31);
    out2 = __SSAT(out2, 31);
    out3 = __SSAT(out3, 31);
    out4 = __SSAT(out4, 31);

    *pDst++ = out1 << 1u;
    *pDst++ = out2 << 1u;
    *pDst++ = out3 << 1u;
    *pDst++ = out4 << 1u;

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
    /* Multiply the inputs and then store the results in the destination buffer. */
    *pDst++ =
      (q31_t) clip_q63_to_q31(((q63_t) (*pSrcA++) * (*pSrcB++)) >> 31);

    /* Decrement the blockSize loop counter */
    blkCnt--;
  }
}

/**    
 * @} end of BasicMult group    
 */
