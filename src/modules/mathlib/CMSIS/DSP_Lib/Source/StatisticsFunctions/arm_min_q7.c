/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_min_q7.c    
*    
* Description:	Minimum value of a Q7 vector.    
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
* ---------------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupStats    
 */

/**    
 * @addtogroup Min    
 * @{    
 */


/**    
 * @brief Minimum value of a Q7 vector.    
 * @param[in]       *pSrc points to the input vector    
 * @param[in]       blockSize length of the input vector    
 * @param[out]      *pResult minimum value returned here    
 * @param[out]      *pIndex index of minimum value returned here    
 * @return none.    
 *    
 */

void arm_min_q7(
  q7_t * pSrc,
  uint32_t blockSize,
  q7_t * pResult,
  uint32_t * pIndex)
{
#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  q7_t minVal1, minVal2, out;                    /* Temporary variables to store the output value. */
  uint32_t blkCnt, outIndex, count;              /* loop counter */

  /* Initialise the count value. */
  count = 0u;
  /* Initialise the index value to zero. */
  outIndex = 0u;
  /* Load first input value that act as reference value for comparision */
  out = *pSrc++;

  /* Loop unrolling */
  blkCnt = (blockSize - 1u) >> 2u;

  while(blkCnt > 0)
  {
    /* Initialize minVal to the next consecutive values one by one */
    minVal1 = *pSrc++;
    minVal2 = *pSrc++;

    /* compare for the minimum value */
    if(out > minVal1)
    {
      /* Update the minimum value and its index */
      out = minVal1;
      outIndex = count + 1u;
    }

    minVal1 = *pSrc++;

    /* compare for the minimum value */
    if(out > minVal2)
    {
      /* Update the minimum value and its index */
      out = minVal2;
      outIndex = count + 2u;
    }

    minVal2 = *pSrc++;

    /* compare for the minimum value */
    if(out > minVal1)
    {
      /* Update the minimum value and its index */
      out = minVal1;
      outIndex = count + 3u;
    }

    /* compare for the minimum value */
    if(out > minVal2)
    {
      /* Update the minimum value and its index */
      out = minVal2;
      outIndex = count + 4u;
    }

    count += 4u;

    blkCnt--;
  }

  /* if (blockSize - 1u ) is not multiple of 4 */
  blkCnt = (blockSize - 1u) % 4u;

#else

  /* Run the below code for Cortex-M0 */

  q7_t minVal1, out;                             /* Temporary variables to store the output value. */
  uint32_t blkCnt, outIndex;                     /* loop counter */

  /* Initialise the index value to zero. */
  outIndex = 0u;
  /* Load first input value that act as reference value for comparision */
  out = *pSrc++;

  blkCnt = (blockSize - 1u);

#endif //      #ifndef ARM_MATH_CM0

  while(blkCnt > 0)
  {
    /* Initialize minVal to the next consecutive values one by one */
    minVal1 = *pSrc++;

    /* compare for the minimum value */
    if(out > minVal1)
    {
      /* Update the minimum value and it's index */
      out = minVal1;
      outIndex = blockSize - blkCnt;
    }

    blkCnt--;

  }

  /* Store the minimum value and its index into destination pointers */
  *pResult = out;
  *pIndex = outIndex;


}

/**    
 * @} end of Min group    
 */
