/* ----------------------------------------------------------------------   
* Copyright (C) 2010 ARM Limited. All rights reserved.   
*   
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*   
* Project: 	    CMSIS DSP Library   
* Title:	    arm_cfft_radix2_q31.c   
*   
* Description:	Radix-2 Decimation in Frequency CFFT & CIFFT Fixed point processing function   
*   
*   
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
*   
* Version 0.0.3  2010/03/10    
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
 */


/**   
 * @addtogroup Radix2_CFFT_CIFFT   
 * @{   
 */

/**   
 * @details   
 * @brief Processing function for the fixed-point CFFT/CIFFT.  
 * @param[in]      *S    points to an instance of the fixed-point CFFT/CIFFT structure.  
 * @param[in, out] *pSrc points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place.  
 * @return none.  
 */

void arm_cfft_radix2_q31(
  const arm_cfft_radix2_instance_q31 * S,
  q31_t * pSrc)
{

  if(S->ifftFlag == 1u)
  {
    arm_radix2_butterfly_inverse_q31(pSrc, S->fftLen,
                                     S->pTwiddle, S->twidCoefModifier);
  }
  else
  {
    arm_radix2_butterfly_q31(pSrc, S->fftLen,
                             S->pTwiddle, S->twidCoefModifier);
  }

  arm_bitreversal_q31(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
}

/**   
 * @} end of Radix2_CFFT_CIFFT group   
 */

void arm_radix2_butterfly_q31(
  q31_t * pSrc,
  uint32_t fftLen,
  q31_t * pCoef,
  uint16_t twidCoefModifier)
{

  int i, j, k, l;
  int n1, n2, ia;
  q31_t xt, yt, cosVal, sinVal;

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
    xt = (pSrc[2 * i] >> 2u) - (pSrc[2 * l] >> 2u);
    pSrc[2 * i] = ((pSrc[2 * i] >> 2u) + (pSrc[2 * l] >> 2u)) >> 1u;

    yt = (pSrc[2 * i + 1] >> 2u) - (pSrc[2 * l + 1] >> 2u);
    pSrc[2 * i + 1] =
      ((pSrc[2 * l + 1] >> 2u) + (pSrc[2 * i + 1] >> 2u)) >> 1u;

    pSrc[2u * l] = (((int32_t) (((q63_t) xt * cosVal) >> 32)) +
                    ((int32_t) (((q63_t) yt * sinVal) >> 32)));

    pSrc[2u * l + 1u] = (((int32_t) (((q63_t) yt * cosVal) >> 32)) -
                         ((int32_t) (((q63_t) xt * sinVal) >> 32)));

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
        pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) >> 1u;

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) >> 1u;

        pSrc[2u * l] = (((int32_t) (((q63_t) xt * cosVal) >> 32)) +
                        ((int32_t) (((q63_t) yt * sinVal) >> 32)));

        pSrc[2u * l + 1u] = (((int32_t) (((q63_t) yt * cosVal) >> 32)) -
                             ((int32_t) (((q63_t) xt * sinVal) >> 32)));

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

    i += n1;
    l = i + n2;

    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]);

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]);

    pSrc[2u * l] = xt;

    pSrc[2u * l + 1u] = yt;

  }                             // butterfly loop end 

}


void arm_radix2_butterfly_inverse_q31(
  q31_t * pSrc,
  uint32_t fftLen,
  q31_t * pCoef,
  uint16_t twidCoefModifier)
{

  int i, j, k, l;
  int n1, n2, ia;
  q31_t xt, yt, cosVal, sinVal;

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
    xt = (pSrc[2 * i] >> 2u) - (pSrc[2 * l] >> 2u);
    pSrc[2 * i] = ((pSrc[2 * i] >> 2u) + (pSrc[2 * l] >> 2u)) >> 1u;

    yt = (pSrc[2 * i + 1] >> 2u) - (pSrc[2 * l + 1] >> 2u);
    pSrc[2 * i + 1] =
      ((pSrc[2 * l + 1] >> 2u) + (pSrc[2 * i + 1] >> 2u)) >> 1u;

    pSrc[2u * l] = (((int32_t) (((q63_t) xt * cosVal) >> 32)) -
                    ((int32_t) (((q63_t) yt * sinVal) >> 32)));

    pSrc[2u * l + 1u] = (((int32_t) (((q63_t) yt * cosVal) >> 32)) +
                         ((int32_t) (((q63_t) xt * sinVal) >> 32)));

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
        pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) >> 1u;

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) >> 1u;

        pSrc[2u * l] = (((int32_t) (((q63_t) xt * cosVal) >> 32)) -
                        ((int32_t) (((q63_t) yt * sinVal) >> 32)));

        pSrc[2u * l + 1u] = (((int32_t) (((q63_t) yt * cosVal) >> 32)) +
                             ((int32_t) (((q63_t) xt * sinVal) >> 32)));

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

    i += n1;
    l = i + n2;

    xt = pSrc[2 * i] - pSrc[2 * l];
    pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]);

    yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
    pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]);

    pSrc[2u * l] = xt;

    pSrc[2u * l + 1u] = yt;

  }                             // butterfly loop end 

}
