/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_bitreversal.c    
*    
* Description:	This file has common tables like Bitreverse, reciprocal etc which are used across different functions    
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Version 1.1.0 2012/02/15 
*    Updated with more optimizations, bug fixes and minor API changes.  
*   
* Version 1.0.10 2011/7/15  
*    Initial Version  
* -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_common_tables.h"

/*    
 * @brief  In-place bit reversal function.   
 * @param[in, out] *pSrc        points to the in-place buffer of floating-point data type.   
 * @param[in]      fftSize      length of the FFT.   
 * @param[in]      bitRevFactor bit reversal modifier that supports different size FFTs with the same bit reversal table.   
 * @param[in]      *pBitRevTab  points to the bit reversal table.   
 * @return none.   
 */

void arm_bitreversal_f32(
  float32_t * pSrc,
  uint16_t fftSize,
  uint16_t bitRevFactor,
  uint16_t * pBitRevTab)
{
  uint16_t fftLenBy2, fftLenBy2p1;
  uint16_t i, j;
  float32_t in;

  /*  Initializations */
  j = 0u;
  fftLenBy2 = fftSize >> 1u;
  fftLenBy2p1 = (fftSize >> 1u) + 1u;

  /* Bit Reversal Implementation */
  for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u)
  {
    if(i < j)
    {
      /*  pSrc[i] <-> pSrc[j]; */
      in = pSrc[2u * i];
      pSrc[2u * i] = pSrc[2u * j];
      pSrc[2u * j] = in;

      /*  pSrc[i+1u] <-> pSrc[j+1u] */
      in = pSrc[(2u * i) + 1u];
      pSrc[(2u * i) + 1u] = pSrc[(2u * j) + 1u];
      pSrc[(2u * j) + 1u] = in;

      /*  pSrc[i+fftLenBy2p1] <-> pSrc[j+fftLenBy2p1] */
      in = pSrc[2u * (i + fftLenBy2p1)];
      pSrc[2u * (i + fftLenBy2p1)] = pSrc[2u * (j + fftLenBy2p1)];
      pSrc[2u * (j + fftLenBy2p1)] = in;

      /*  pSrc[i+fftLenBy2p1+1u] <-> pSrc[j+fftLenBy2p1+1u] */
      in = pSrc[(2u * (i + fftLenBy2p1)) + 1u];
      pSrc[(2u * (i + fftLenBy2p1)) + 1u] =
        pSrc[(2u * (j + fftLenBy2p1)) + 1u];
      pSrc[(2u * (j + fftLenBy2p1)) + 1u] = in;

    }

    /*  pSrc[i+1u] <-> pSrc[j+1u] */
    in = pSrc[2u * (i + 1u)];
    pSrc[2u * (i + 1u)] = pSrc[2u * (j + fftLenBy2)];
    pSrc[2u * (j + fftLenBy2)] = in;

    /*  pSrc[i+2u] <-> pSrc[j+2u] */
    in = pSrc[(2u * (i + 1u)) + 1u];
    pSrc[(2u * (i + 1u)) + 1u] = pSrc[(2u * (j + fftLenBy2)) + 1u];
    pSrc[(2u * (j + fftLenBy2)) + 1u] = in;

    /*  Reading the index for the bit reversal */
    j = *pBitRevTab;

    /*  Updating the bit reversal index depending on the fft length  */
    pBitRevTab += bitRevFactor;
  }
}



/*    
 * @brief  In-place bit reversal function.   
 * @param[in, out] *pSrc        points to the in-place buffer of Q31 data type.   
 * @param[in]      fftLen       length of the FFT.   
 * @param[in]      bitRevFactor bit reversal modifier that supports different size FFTs with the same bit reversal table   
 * @param[in]      *pBitRevTab  points to bit reversal table.   
 * @return none.   
 */

void arm_bitreversal_q31(
  q31_t * pSrc,
  uint32_t fftLen,
  uint16_t bitRevFactor,
  uint16_t * pBitRevTable)
{
  uint32_t fftLenBy2, fftLenBy2p1, i, j;
  q31_t in;

  /*  Initializations      */
  j = 0u;
  fftLenBy2 = fftLen / 2u;
  fftLenBy2p1 = (fftLen / 2u) + 1u;

  /* Bit Reversal Implementation */
  for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u)
  {
    if(i < j)
    {
      /*  pSrc[i] <-> pSrc[j]; */
      in = pSrc[2u * i];
      pSrc[2u * i] = pSrc[2u * j];
      pSrc[2u * j] = in;

      /*  pSrc[i+1u] <-> pSrc[j+1u] */
      in = pSrc[(2u * i) + 1u];
      pSrc[(2u * i) + 1u] = pSrc[(2u * j) + 1u];
      pSrc[(2u * j) + 1u] = in;

      /*  pSrc[i+fftLenBy2p1] <-> pSrc[j+fftLenBy2p1] */
      in = pSrc[2u * (i + fftLenBy2p1)];
      pSrc[2u * (i + fftLenBy2p1)] = pSrc[2u * (j + fftLenBy2p1)];
      pSrc[2u * (j + fftLenBy2p1)] = in;

      /*  pSrc[i+fftLenBy2p1+1u] <-> pSrc[j+fftLenBy2p1+1u] */
      in = pSrc[(2u * (i + fftLenBy2p1)) + 1u];
      pSrc[(2u * (i + fftLenBy2p1)) + 1u] =
        pSrc[(2u * (j + fftLenBy2p1)) + 1u];
      pSrc[(2u * (j + fftLenBy2p1)) + 1u] = in;

    }

    /*  pSrc[i+1u] <-> pSrc[j+1u] */
    in = pSrc[2u * (i + 1u)];
    pSrc[2u * (i + 1u)] = pSrc[2u * (j + fftLenBy2)];
    pSrc[2u * (j + fftLenBy2)] = in;

    /*  pSrc[i+2u] <-> pSrc[j+2u] */
    in = pSrc[(2u * (i + 1u)) + 1u];
    pSrc[(2u * (i + 1u)) + 1u] = pSrc[(2u * (j + fftLenBy2)) + 1u];
    pSrc[(2u * (j + fftLenBy2)) + 1u] = in;

    /*  Reading the index for the bit reversal */
    j = *pBitRevTable;

    /*  Updating the bit reversal index depending on the fft length */
    pBitRevTable += bitRevFactor;
  }
}



/*    
   * @brief  In-place bit reversal function.   
   * @param[in, out] *pSrc        points to the in-place buffer of Q15 data type.   
   * @param[in]      fftLen       length of the FFT.   
   * @param[in]      bitRevFactor bit reversal modifier that supports different size FFTs with the same bit reversal table   
   * @param[in]      *pBitRevTab  points to bit reversal table.   
   * @return none.   
 */

void arm_bitreversal_q15(
  q15_t * pSrc16,
  uint32_t fftLen,
  uint16_t bitRevFactor,
  uint16_t * pBitRevTab)
{
  q31_t *pSrc = (q31_t *) pSrc16;
  q31_t in;
  uint32_t fftLenBy2, fftLenBy2p1;
  uint32_t i, j;

  /*  Initializations */
  j = 0u;
  fftLenBy2 = fftLen / 2u;
  fftLenBy2p1 = (fftLen / 2u) + 1u;

  /* Bit Reversal Implementation */
  for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u)
  {
    if(i < j)
    {
      /*  pSrc[i] <-> pSrc[j]; */
      /*  pSrc[i+1u] <-> pSrc[j+1u] */
      in = pSrc[i];
      pSrc[i] = pSrc[j];
      pSrc[j] = in;

      /*  pSrc[i + fftLenBy2p1] <-> pSrc[j + fftLenBy2p1];  */
      /*  pSrc[i + fftLenBy2p1+1u] <-> pSrc[j + fftLenBy2p1+1u] */
      in = pSrc[i + fftLenBy2p1];
      pSrc[i + fftLenBy2p1] = pSrc[j + fftLenBy2p1];
      pSrc[j + fftLenBy2p1] = in;
    }

    /*  pSrc[i+1u] <-> pSrc[j+fftLenBy2];         */
    /*  pSrc[i+2] <-> pSrc[j+fftLenBy2+1u]  */
    in = pSrc[i + 1u];
    pSrc[i + 1u] = pSrc[j + fftLenBy2];
    pSrc[j + fftLenBy2] = in;

    /*  Reading the index for the bit reversal */
    j = *pBitRevTab;

    /*  Updating the bit reversal index depending on the fft length  */
    pBitRevTab += bitRevFactor;
  }
}
