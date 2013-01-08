/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_shift_q31.c    
*    
* Description:	Shifts the elements of a Q31 vector by a specified number of bits.    
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
* Version 0.0.7  2010/06/10     
*    Misra-C changes done    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**        
 * @ingroup groupMath        
 */
/**        
 * @defgroup shift Vector Shift        
 *        
 * Shifts the elements of a fixed-point vector by a specified number of bits.        
 * There are separate functions for Q7, Q15, and Q31 data types.        
 * The underlying algorithm used is:        
 *        
 * <pre>        
 *     pDst[n] = pSrc[n] << shift,   0 <= n < blockSize.        
 * </pre>        
 *        
 * If <code>shift</code> is positive then the elements of the vector are shifted to the left.        
 * If <code>shift</code> is negative then the elements of the vector are shifted to the right.        
 */

/**        
 * @addtogroup shift        
 * @{        
 */

/**        
 * @brief  Shifts the elements of a Q31 vector a specified number of bits.        
 * @param[in]  *pSrc points to the input vector        
 * @param[in]  shiftBits number of bits to shift.  A positive value shifts left; a negative value shifts right.        
 * @param[out]  *pDst points to the output vector        
 * @param[in]  blockSize number of samples in the vector        
 * @return none.        
 *        
 *        
 * <b>Scaling and Overflow Behavior:</b>        
 * \par        
 * The function uses saturating arithmetic.        
 * Results outside of the allowable Q31 range [0x80000000 0x7FFFFFFF] will be saturated.        
 */

void arm_shift_q31(
  q31_t * pSrc,
  int8_t shiftBits,
  q31_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */
  uint8_t sign = (shiftBits & 0x80);             /* Sign of shiftBits */

#ifndef ARM_MATH_CM0

  q31_t in1, in2, in3, in4;                      /* Temporary input variables */
  q31_t out1, out2, out3, out4;                  /* Temporary output variables */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;


  if(sign == 0u)
  {
    /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
     ** a second loop below computes the remaining 1 to 3 samples. */
    while(blkCnt > 0u)
    {
      /* C = A  << shiftBits */
      /* Shift the input and then store the results in the destination buffer. */
      in1 = *pSrc;
      in2 = *(pSrc + 1);
      out1 = in1 << shiftBits;
      in3 = *(pSrc + 2);
      out2 = in2 << shiftBits;
      in4 = *(pSrc + 3);
      if(in1 != (out1 >> shiftBits))
        out1 = 0x7FFFFFFF ^ (in1 >> 31);

      if(in2 != (out2 >> shiftBits))
        out2 = 0x7FFFFFFF ^ (in2 >> 31);

      *pDst = out1;
      out3 = in3 << shiftBits;
      *(pDst + 1) = out2;
      out4 = in4 << shiftBits;

      if(in3 != (out3 >> shiftBits))
        out3 = 0x7FFFFFFF ^ (in3 >> 31);

      if(in4 != (out4 >> shiftBits))
        out4 = 0x7FFFFFFF ^ (in4 >> 31);

      *(pDst + 2) = out3;
      *(pDst + 3) = out4;

      /* Update destination pointer to process next sampels */
      pSrc += 4u;
      pDst += 4u;

      /* Decrement the loop counter */
      blkCnt--;
    }
  }
  else
  {

    /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
     ** a second loop below computes the remaining 1 to 3 samples. */
    while(blkCnt > 0u)
    {
      /* C = A >>  shiftBits */
      /* Shift the input and then store the results in the destination buffer. */
      in1 = *pSrc;
      in2 = *(pSrc + 1);
      in3 = *(pSrc + 2);
      in4 = *(pSrc + 3);

      *pDst = (in1 >> -shiftBits);
      *(pDst + 1) = (in2 >> -shiftBits);
      *(pDst + 2) = (in3 >> -shiftBits);
      *(pDst + 3) = (in4 >> -shiftBits);


      pSrc += 4u;
      pDst += 4u;

      blkCnt--;
    }

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
    /* C = A (>> or <<) shiftBits */
    /* Shift the input and then store the result in the destination buffer. */
    *pDst++ = (sign == 0u) ? clip_q63_to_q31((q63_t) * pSrc++ << shiftBits) :
      (*pSrc++ >> -shiftBits);

    /* Decrement the loop counter */
    blkCnt--;
  }


}

/**        
 * @} end of shift group        
 */
