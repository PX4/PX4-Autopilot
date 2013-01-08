/* ----------------------------------------------------------------------   
* Copyright (C) 2010 ARM Limited. All rights reserved.   
*   
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*   
* Project: 	    CMSIS DSP Library   
* Title:	    arm_cfft_radix2_f32.c   
*   
* Description:	Radix-2 Decimation in Frequency CFFT & CIFFT Floating point processing function   
*   
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
*   
* Version 1.0.3 2010/11/29  
*    Initial version   
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**   
 * @ingroup groupTransforms   
 */

/**   
 * @defgroup Radix2_CFFT_CIFFT Radix-2 Complex FFT Functions   
 *   
 * \par   
 * Complex Fast Fourier Transform(CFFT) and Complex Inverse Fast Fourier Transform(CIFFT) is an efficient algorithm to compute Discrete Fourier Transform(DFT) and Inverse Discrete Fourier Transform(IDFT).   
 * Computational complexity of CFFT reduces drastically when compared to DFT.   
 * \par   
 * This set of functions implements CFFT/CIFFT   
 * for Q15, Q31, and floating-point data types.  The functions operates on in-place buffer which uses same buffer for input and output.   
 * Complex input is stored in input buffer in an interleaved fashion.   
 *   
 * \par   
 * The functions operate on blocks of input and output data and each call to the function processes   
 * <code>2*fftLen</code> samples through the transform.  <code>pSrc</code>  points to In-place arrays containing <code>2*fftLen</code> values.   
 * \par  
 * The <code>pSrc</code> points to the array of in-place buffer of size <code>2*fftLen</code> and inputs and outputs are stored in an interleaved fashion as shown below.   
 * <pre> {real[0], imag[0], real[1], imag[1],..} </pre>   
 *   
 * \par Lengths supported by the transform:  
 * \par   
 * Internally, the function utilize a radix-2 decimation in frequency(DIF) algorithm   
 * and the size of the FFT supported are of the lengths [16, 32, 64, 128, 256, 512, 1024, 2048, 4096].  
 *    
 *   
 * \par Algorithm:   
 *   
 * <b>Complex Fast Fourier Transform:</b>   
 * \par    
 * Input real and imaginary data:   
 * <pre>   
 * x(n) = xa + j * ya   
 * x(n+N/2 ) = xb + j * yb   
 * </pre>   
 * where N is length of FFT   
 * \par   
 * Output real and imaginary data:   
 * <pre>   
 * X(2r) = xa'+ j * ya'   
 * X(2r+1) = xb'+ j * yb'   
 * </pre>   
 * \par   
 * Twiddle factors for radix-2 FFT:   
 * <pre>   
 * Wn = cosVal + j * (- sinVal)   
 * </pre>   
 *   
 * \par   
 * \image html CFFT_Radix2.gif "Radix-2 Decimation-in Frequency Complex Fast Fourier Transform"   
 *   
 * \par   
 * Output from Radix-2 CFFT Results in Digit reversal order. Interchange middle two branches of every butterfly results in Bit reversed output.   
 * \par   
 * <b> Butterfly CFFT equations:</b>   
 * <pre>   
 * xa' = xa + xb  
 * ya' = ya + yb  
 * xb' = (xa-xb)* cosVal + (ya-yb) * sinVal   
 * yb' = (ya-yb)* cosVal - (xa-xb) * sinVal   
 * </pre>   
 *   
 *   
 * <b>Complex Inverse Fast Fourier Transform:</b>   
 * \par   
 * CIFFT uses same twiddle factor table as CFFT with modifications in the design equation as shown below.   
 *   
 * \par   
 * <b> Modified Butterfly CIFFT equations:</b>   
 * <pre>   
 * xa' = xa + xb  
 * ya' = ya + yb  
 * xb' = (xa-xb)* cosVal - (ya-yb) * sinVal   
 * yb' = (ya-yb)* cosVal + (xa-xb) * sinVal   
 * </pre>   
 *   
 * \par Instance Structure   
 * A separate instance structure must be defined for each Instance but the twiddle factors and bit reversal tables can be reused.   
 * There are separate instance structure declarations for each of the 3 supported data types.   
 *   
 * \par Initialization Functions   
 * There is also an associated initialization function for each data type.   
 * The initialization function performs the following operations:   
 * - Sets the values of the internal structure fields.   
 * - Initializes twiddle factor table and bit reversal table pointers   
 * \par   
 * Use of the initialization function is optional.   
 * However, if the initialization function is used, then the instance structure cannot be placed into a const data section.   
 * To place an instance structure into a const data section, the instance structure must be manually initialized.   
 * Manually initialize the instance structure as follows:   
 * <pre>   
 *arm_cfft_radix2_instance_f32 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor, onebyfftLen};   
 *arm_cfft_radix2_instance_q31 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor};   
 *arm_cfft_radix2_instance_q15 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor};   
 * </pre>   
 * \par   
 * where <code>fftLen</code> length of CFFT/CIFFT; <code>ifftFlag</code> Flag for selection of CFFT or CIFFT(Set ifftFlag to calculate CIFFT otherwise calculates CFFT);   
 * <code>bitReverseFlag</code> Flag for selection of output order(Set bitReverseFlag to output in normal order otherwise output in bit reversed order);    
 * <code>pTwiddle</code>points to array of twiddle coefficients; <code>pBitRevTable</code> points to the array of bit reversal table.   
 * <code>twidCoefModifier</code> modifier for twiddle factor table which supports all FFT lengths with same table;    
 * <code>pBitRevTable</code> modifier for bit reversal table which supports all FFT lengths with same table.   
 * <code>onebyfftLen</code> value of 1/fftLen to calculate CIFFT;   
 *  
 * \par Fixed-Point Behavior   
 * Care must be taken when using the fixed-point versions of the CFFT/CIFFT function.   
 * Refer to the function specific documentation below for usage guidelines.   
 */


/**   
 * @addtogroup Radix2_CFFT_CIFFT   
 * @{   
 */

/**   
 * @details   
 * @brief Processing function for the floating-point Radix-2 CFFT/CIFFT.  
 * @param[in]      *S    points to an instance of the floating-point Radix-2 CFFT/CIFFT structure.  
 * @param[in, out] *pSrc points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place.  
 * @return none.  
 */

void arm_cfft_radix2_f32(
  const arm_cfft_radix2_instance_f32 * S,
  float32_t * pSrc)
{

  if(S->ifftFlag == 1u)
  {
    /*  Complex IFFT radix-2  */
    arm_radix2_butterfly_inverse_f32(pSrc, S->fftLen, S->pTwiddle,
                                     S->twidCoefModifier, S->onebyfftLen);
  }
  else
  {
    /*  Complex FFT radix-2  */
    arm_radix2_butterfly_f32(pSrc, S->fftLen, S->pTwiddle,
                             S->twidCoefModifier);
  }

  if(S->bitReverseFlag == 1u)
  {
    /*  Bit Reversal */
    arm_bitreversal_f32(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
  }

}


/**    
 * @} end of Radix2_CFFT_CIFFT group    
 */



/* ----------------------------------------------------------------------    
** Internal helper function used by the FFTs    
** ------------------------------------------------------------------- */

/*    
 * @brief  Core function for the floating-point CFFT butterfly process.   
 * @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.   
 * @param[in]      fftLen           length of the FFT.   
 * @param[in]      *pCoef           points to the twiddle coefficient buffer.   
 * @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.   
 * @return none.   
 */

void arm_radix2_butterfly_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier)
{

  int i, j, k, l;
  int n1, n2, ia;
  float32_t xt, yt, cosVal, sinVal;

#ifndef ARM_MATH_CM0

  /*  Initializations for the first stage */
  n2 = fftLen;

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  // loop for groups 
  for (i = 0; i < n2; i++)
  {
    cosVal = pCoef[ia * 2];
    sinVal = pCoef[(ia * 2) + 1];

    /*  Twiddle coefficients index modifier */
    ia = ia + twidCoefModifier;

    /*  index calculation for the input as, */
    /*  pSrc[i + 0], pSrc[i + fftLen/1] */
    l = i + n2;

    /*  Butterfly implementation */
    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

    pSrc[2u * l] = xt * cosVal + yt * sinVal;

    pSrc[2u * l + 1u] = yt * cosVal - xt * sinVal;

  }                             // groups loop end 

  twidCoefModifier = twidCoefModifier << 1u;

  // loop for stage 
  for (k = fftLen / 2; k > 2; k = k >> 1)
  {
    n1 = n2;
    n2 = n2 >> 1;
    ia = 0;

    // loop for groups 
    for (j = 0; j < n2; j++)
    {
      cosVal = pCoef[ia * 2];
      sinVal = pCoef[(ia * 2) + 1];
      ia = ia + twidCoefModifier;

      // loop for butterfly 
      for (i = j; i < fftLen; i += n1)
      {
        l = i + n2;
        xt = pSrc[2 * i] - pSrc[2 * l];
        pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

        pSrc[2u * l] = xt * cosVal + yt * sinVal;

        pSrc[2u * l + 1u] = yt * cosVal - xt * sinVal;

      }                         // butterfly loop end 

    }                           // groups loop end 

    twidCoefModifier = twidCoefModifier << 1u;
  }                             // stages loop end 

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  cosVal = pCoef[ia * 2];
  sinVal = pCoef[(ia * 2) + 1];
  ia = ia + twidCoefModifier;

  // loop for butterfly 
  for (i = 0; i < fftLen; i += n1)
  {
    l = i + n2;
    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]);

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]);

    pSrc[2u * l] = xt;

    pSrc[2u * l + 1u] = yt;

  }                             // groups loop end 

#else

  //N = fftLen; 
  n2 = fftLen;

  // loop for stage 
  for (k = fftLen; k > 1; k = k >> 1)
  {
    n1 = n2;
    n2 = n2 >> 1;
    ia = 0;

    // loop for groups 
    for (j = 0; j < n2; j++)
    {
      cosVal = pCoef[ia * 2];
      sinVal = pCoef[(ia * 2) + 1];
      ia = ia + twidCoefModifier;

      // loop for butterfly 
      for (i = j; i < fftLen; i += n1)
      {
        l = i + n2;
        xt = pSrc[2 * i] - pSrc[2 * l];
        pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

        pSrc[2 * l] = (cosVal * xt + sinVal * yt);      // >> 15; 
        pSrc[2 * l + 1] = (cosVal * yt - sinVal * xt);  // >> 15; 

      }
    }
    twidCoefModifier = twidCoefModifier << 1u;
  }

#endif //    #ifndef ARM_MATH_CM0

}


void arm_radix2_butterfly_inverse_f32(
  float32_t * pSrc,
  uint32_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier,
  float32_t onebyfftLen)
{

  int i, j, k, l;
  int n1, n2, ia;
  float32_t xt, yt, cosVal, sinVal;

#ifndef ARM_MATH_CM0

  //N = fftLen; 
  n2 = fftLen;

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  // loop for groups 
  for (i = 0; i < n2; i++)
  {
    cosVal = pCoef[ia * 2];
    sinVal = pCoef[(ia * 2) + 1];
    ia = ia + twidCoefModifier;

    l = i + n2;
    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

    pSrc[2u * l] = xt * cosVal - yt * sinVal;

    pSrc[2u * l + 1u] = yt * cosVal + xt * sinVal;

  }                             // groups loop end 

  twidCoefModifier = twidCoefModifier << 1u;

  // loop for stage 
  for (k = fftLen / 2; k > 2; k = k >> 1)
  {
    n1 = n2;
    n2 = n2 >> 1;
    ia = 0;

    // loop for groups 
    for (j = 0; j < n2; j++)
    {
      cosVal = pCoef[ia * 2];
      sinVal = pCoef[(ia * 2) + 1];
      ia = ia + twidCoefModifier;

      // loop for butterfly 
      for (i = j; i < fftLen; i += n1)
      {
        l = i + n2;
        xt = pSrc[2 * i] - pSrc[2 * l];
        pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

        pSrc[2u * l] = xt * cosVal - yt * sinVal;

        pSrc[2u * l + 1u] = yt * cosVal + xt * sinVal;

      }                         // butterfly loop end 

    }                           // groups loop end 

    twidCoefModifier = twidCoefModifier << 1u;
  }                             // stages loop end 

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  cosVal = pCoef[ia * 2];
  sinVal = pCoef[(ia * 2) + 1];
  ia = ia + twidCoefModifier;

  // loop for butterfly 
  for (i = 0; i < fftLen; i += n1)
  {
    l = i + n2;
    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) * onebyfftLen;

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) * onebyfftLen;

    pSrc[2u * l] = xt * onebyfftLen;

    pSrc[2u * l + 1u] = yt * onebyfftLen;

  }                             // butterfly loop end 

#else

  //N = fftLen; 
  n2 = fftLen;

  // loop for stage 
  for (k = fftLen; k > 2; k = k >> 1)
  {
    n1 = n2;
    n2 = n2 >> 1;
    ia = 0;

    // loop for groups 
    for (j = 0; j < n2; j++)
    {
      cosVal = pCoef[ia * 2];
      sinVal = pCoef[(ia * 2) + 1];
      ia = ia + twidCoefModifier;

      // loop for butterfly 
      for (i = j; i < fftLen; i += n1)
      {
        l = i + n2;
        xt = pSrc[2 * i] - pSrc[2 * l];
        pSrc[2 * i] = pSrc[2 * i] + pSrc[2 * l];

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = pSrc[2 * l + 1] + pSrc[2 * i + 1];

        pSrc[2u * l] = xt * cosVal - yt * sinVal;

        pSrc[2u * l + 1u] = yt * cosVal + xt * sinVal;

      }                         // butterfly loop end 

    }                           // groups loop end 

    twidCoefModifier = twidCoefModifier << 1u;
  }                             // stages loop end 

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  cosVal = pCoef[ia * 2];
  sinVal = pCoef[(ia * 2) + 1];
  ia = ia + twidCoefModifier;

  // loop for butterfly 
  for (i = 0; i < fftLen; i += n1)
  {
    l = i + n2;
    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) * onebyfftLen;

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) * onebyfftLen;

    pSrc[2u * l] = xt * onebyfftLen;

    pSrc[2u * l + 1u] = yt * onebyfftLen;

  }                             // butterfly loop end 

#endif //      #ifndef ARM_MATH_CM0

}
