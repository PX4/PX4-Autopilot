/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_abs_f32.c    
*    
* Description:	Vector absolute value.    
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
* ---------------------------------------------------------------------------- */

#include "arm_math.h"
#include <math.h>

/**        
 * @ingroup groupMath        
 */

/**        
 * @defgroup BasicAbs Vector Absolute Value        
 *        
 * Computes the absolute value of a vector on an element-by-element basis.        
 *        
 * <pre>        
 *     pDst[n] = abs(pSrcA[n]),   0 <= n < blockSize.        
 * </pre>        
 *        
 * The operation can be done in-place by setting the input and output pointers to the same buffer.        
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.        
 */

/**        
 * @addtogroup BasicAbs        
 * @{        
 */

/**        
 * @brief Floating-point vector absolute value.        
 * @param[in]       *pSrc points to the input buffer        
 * @param[out]      *pDst points to the output buffer        
 * @param[in]       blockSize number of samples in each vector        
 * @return none.        
 */

void arm_abs_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  float32_t in1, in2, in3, in4;                  /* temporary variables */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.    
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Calculate absolute and then store the results in the destination buffer. */
    /* read sample from source */
    in1 = *pSrc;
    in2 = *(pSrc + 1);
    in3 = *(pSrc + 2);

    /* find absolute value */
    in1 = fabsf(in1);

    /* read sample from source */
    in4 = *(pSrc + 3);

    /* find absolute value */
    in2 = fabsf(in2);

    /* read sample from source */
    *pDst = in1;

    /* find absolute value */
    in3 = fabsf(in3);

    /* find absolute value */
    in4 = fabsf(in4);

    /* store result to destination */
    *(pDst + 1) = in2;

    /* store result to destination */
    *(pDst + 2) = in3;

    /* store result to destination */
    *(pDst + 3) = in4;


    /* Update source pointer to process next sampels */
    pSrc += 4u;

    /* Update destination pointer to process next sampels */
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

#endif /*   #ifndef ARM_MATH_CM0   */

  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Calculate absolute and then store the results in the destination buffer. */
    *pDst++ = fabsf(*pSrc++);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**        
 * @} end of BasicAbs group        
 */
