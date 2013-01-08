/* ----------------------------------------------------------------------     
* Copyright (C) 2011 ARM Limited. All rights reserved.  
*     
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*     
* Project:      CMSIS DSP Library  
* Title:		arm_sqrt_q15.c     
*     
* Description:	Q15 square root function.    
*     
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
* 
* Version 1.0.0 2011/03/08 
*     Alpha release. 
* 
* Version 1.0.1 2011/09/30 
*     Beta release.  
*     
* -------------------------------------------------------------------- */
#include "arm_math.h"
#include "arm_common_tables.h"


/**     
 * @ingroup groupFastMath     
 */

/**     
 * @addtogroup SQRT     
 * @{     
 */

  /**    
   * @brief  Q15 square root function.    
   * @param[in]   in     input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF.    
   * @param[out]  *pOut  square root of input value.    
   * @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if    
   * <code>in</code> is negative value and returns zero output for negative values.    
   */

arm_status arm_sqrt_q15(
  q15_t in,
  q15_t * pOut)
{
  q15_t number, temp1, var1, signBits1, half;
  q31_t bits_val1;
  float32_t temp_float1;

  number = in;

  /* If the input is a positive number then compute the signBits. */
  if(number > 0)
  {
    signBits1 = __CLZ(number) - 17;

    /* Shift by the number of signBits1 */
    if((signBits1 % 2) == 0)
    {
      number = number << signBits1;
    }
    else
    {
      number = number << (signBits1 - 1);
    }

    /* Calculate half value of the number */
    half = number >> 1;
    /* Store the number for later use */
    temp1 = number;

    /*Convert to float */
    temp_float1 = number * 3.051757812500000e-005f;
    /*Store as integer */
    bits_val1 = *(int *) &temp_float1;
    /* Subtract the shifted value from the magic number to give intial guess */
    bits_val1 = 0x5f3759df - (bits_val1 >> 1);  // gives initial guess  
    /* Store as float */
    temp_float1 = *(float *) &bits_val1;
    /* Convert to integer format */
    var1 = (q31_t) (temp_float1 * 16384);

    /* 1st iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 2nd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;
    /* 3rd iteration */
    var1 = ((q15_t) ((q31_t) var1 * (0x3000 -
                                     ((q15_t)
                                      ((((q15_t)
                                         (((q31_t) var1 * var1) >> 15)) *
                                        (q31_t) half) >> 15))) >> 15)) << 2;

    /* Multiply the inverse square root with the original value */
    var1 = ((q15_t) (((q31_t) temp1 * var1) >> 15)) << 1;

    /* Shift the output down accordingly */
    if((signBits1 % 2) == 0)
    {
      var1 = var1 >> (signBits1 / 2);
    }
    else
    {
      var1 = var1 >> ((signBits1 - 1) / 2);
    }
    *pOut = var1;

    return (ARM_MATH_SUCCESS);
  }
  /* If the number is a negative number then store zero as its square root value */
  else
  {
    *pOut = 0;
    return (ARM_MATH_ARGUMENT_ERROR);
  }
}

/**     
 * @} end of SQRT group     
 */
