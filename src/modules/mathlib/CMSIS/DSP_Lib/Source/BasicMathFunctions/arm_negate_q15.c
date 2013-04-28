/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_negate_q15.c    
*    
* Description:	Negates Q15 vectors.    
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
 * @addtogroup negate        
 * @{        
 */

/**        
 * @brief  Negates the elements of a Q15 vector.        
 * @param[in]  *pSrc points to the input vector        
 * @param[out]  *pDst points to the output vector        
 * @param[in]  blockSize number of samples in the vector        
 * @return none.        
 *    
 * \par Conditions for optimum performance    
 *  Input and output buffers should be aligned by 32-bit    
 *    
 *        
 * <b>Scaling and Overflow Behavior:</b>        
 * \par        
 * The function uses saturating arithmetic.        
 * The Q15 value -1 (0x8000) will be saturated to the maximum allowable positive value 0x7FFF.        
 */

void arm_negate_q15(
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */
  q15_t in;

#ifndef ARM_MATH_CM0

/* Run the below code for Cortex-M4 and Cortex-M3 */

  q31_t in1, in2;                                /* Temporary variables */


  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.        
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = -A */
    /* Read two inputs at a time */
    in1 = _SIMD32_OFFSET(pSrc);
    in2 = _SIMD32_OFFSET(pSrc + 2);

    /* negate two samples at a time */
    in1 = __QSUB16(0, in1);

    /* negate two samples at a time */
    in2 = __QSUB16(0, in2);

    /* store the result to destination 2 samples at a time */
    _SIMD32_OFFSET(pDst) = in1;
    /* store the result to destination 2 samples at a time */
    _SIMD32_OFFSET(pDst + 2) = in2;


    /* update pointers to process next samples */
    pSrc += 4u;
    pDst += 4u;

    /* Decrement the loop counter */
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
    /* C = -A */
    /* Negate and then store the result in the destination buffer. */
    in = *pSrc++;
    *pDst++ = (in == (q15_t) 0x8000) ? 0x7fff : -in;

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**        
 * @} end of negate group        
 */
