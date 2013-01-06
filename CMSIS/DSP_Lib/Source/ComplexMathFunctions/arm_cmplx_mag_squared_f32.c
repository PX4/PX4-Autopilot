/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:		arm_cmplx_mag_squared_f32.c    
*    
* Description:	Floating-point complex magnitude squared.    
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
 * @ingroup groupCmplxMath        
 */

/**        
 * @defgroup cmplx_mag_squared Complex Magnitude Squared        
 *        
 * Computes the magnitude squared of the elements of a complex data vector.        
 *       
 * The <code>pSrc</code> points to the source data and        
 * <code>pDst</code> points to the where the result should be written.        
 * <code>numSamples</code> specifies the number of complex samples        
 * in the input array and the data is stored in an interleaved fashion        
 * (real, imag, real, imag, ...).        
 * The input array has a total of <code>2*numSamples</code> values;        
 * the output array has a total of <code>numSamples</code> values.        
 *        
 * The underlying algorithm is used:        
 *        
 * <pre>        
 * for(n=0; n<numSamples; n++) {        
 *     pDst[n] = pSrc[(2*n)+0]^2 + pSrc[(2*n)+1]^2;        
 * }        
 * </pre>        
 *        
 * There are separate functions for floating-point, Q15, and Q31 data types.        
 */

/**        
 * @addtogroup cmplx_mag_squared        
 * @{        
 */


/**        
 * @brief  Floating-point complex magnitude squared        
 * @param[in]  *pSrc points to the complex input vector        
 * @param[out]  *pDst points to the real output vector        
 * @param[in]  numSamples number of complex samples in the input vector        
 * @return none.        
 */

void arm_cmplx_mag_squared_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t real, imag;                          /* Temporary variables to store real and imaginary values */
  uint32_t blkCnt;                               /* loop counter */

#ifndef ARM_MATH_CM0
  float32_t real1, real2, real3, real4;          /* Temporary variables to hold real values */
  float32_t imag1, imag2, imag3, imag4;          /* Temporary variables to hold imaginary values */
  float32_t mul1, mul2, mul3, mul4;              /* Temporary variables */
  float32_t mul5, mul6, mul7, mul8;              /* Temporary variables */
  float32_t out1, out2, out3, out4;              /* Temporary variables to hold output values */

  /*loop Unrolling */
  blkCnt = numSamples >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.        
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C[0] = (A[0] * A[0] + A[1] * A[1]) */
    /* read real input sample from source buffer */
    real1 = pSrc[0];
    /* read imaginary input sample from source buffer */
    imag1 = pSrc[1];

    /* calculate power of real value */
    mul1 = real1 * real1;

    /* read real input sample from source buffer */
    real2 = pSrc[2];

    /* calculate power of imaginary value */
    mul2 = imag1 * imag1;

    /* read imaginary input sample from source buffer */
    imag2 = pSrc[3];

    /* calculate power of real value */
    mul3 = real2 * real2;

    /* read real input sample from source buffer */
    real3 = pSrc[4];

    /* calculate power of imaginary value */
    mul4 = imag2 * imag2;

    /* read imaginary input sample from source buffer */
    imag3 = pSrc[5];

    /* calculate power of real value */
    mul5 = real3 * real3;
    /* calculate power of imaginary value */
    mul6 = imag3 * imag3;

    /* read real input sample from source buffer */
    real4 = pSrc[6];

    /* accumulate real and imaginary powers */
    out1 = mul1 + mul2;

    /* read imaginary input sample from source buffer */
    imag4 = pSrc[7];

    /* accumulate real and imaginary powers */
    out2 = mul3 + mul4;

    /* calculate power of real value */
    mul7 = real4 * real4;
    /* calculate power of imaginary value */
    mul8 = imag4 * imag4;

    /* store output to destination */
    pDst[0] = out1;

    /* accumulate real and imaginary powers */
    out3 = mul5 + mul6;

    /* store output to destination */
    pDst[1] = out2;

    /* accumulate real and imaginary powers */
    out4 = mul7 + mul8;

    /* store output to destination */
    pDst[2] = out3;

    /* increment destination pointer by 8 to process next samples */
    pSrc += 8u;

    /* store output to destination */
    pDst[3] = out4;

    /* increment destination pointer by 4 to process next samples */
    pDst += 4u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the numSamples is not a multiple of 4, compute any remaining output samples here.        
   ** No loop unrolling is used. */
  blkCnt = numSamples % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  blkCnt = numSamples;

#endif /* #ifndef ARM_MATH_CM0 */

  while(blkCnt > 0u)
  {
    /* C[0] = (A[0] * A[0] + A[1] * A[1]) */
    real = *pSrc++;
    imag = *pSrc++;

    /* out = (real * real) + (imag * imag) */
    /* store the result in the destination buffer. */
    *pDst++ = (real * real) + (imag * imag);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**        
 * @} end of cmplx_mag_squared group        
 */
