/* ----------------------------------------------------------------------    
* Copyright (C) 2010 ARM Limited. All rights reserved.    
*    
* $Date:        15. February 2012  
* $Revision: 	V1.1.0  
*    
* Project: 	    CMSIS DSP Library    
* Title:	    arm_cfft_radix4_f32.c    
*    
* Description:	Radix-4 Decimation in Frequency CFFT & CIFFT Floating point processing function    
*    
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
* 	 incorporated review comments and updated with latest CMSIS layer    
*    
* Version 0.0.3  2010/03/10     
*    Initial version    
* -------------------------------------------------------------------- */

#include "arm_math.h"

/**    
 * @ingroup groupTransforms    
 */

/**    
 * @defgroup Radix4_CFFT_CIFFT Radix-4 Complex FFT Functions    
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
 * Internally, the function utilize a radix-4 decimation in frequency(DIF) algorithm    
 * and the size of the FFT supported are of the lengths [16, 64, 256, 1024].   
 *     
 *    
 * \par Algorithm:    
 *    
 * <b>Complex Fast Fourier Transform:</b>    
 * \par     
 * Input real and imaginary data:    
 * <pre>    
 * x(n) = xa + j * ya    
 * x(n+N/4 ) = xb + j * yb    
 * x(n+N/2 ) = xc + j * yc    
 * x(n+3N 4) = xd + j * yd    
 * </pre>    
 * where N is length of FFT    
 * \par    
 * Output real and imaginary data:    
 * <pre>    
 * X(4r) = xa'+ j * ya'    
 * X(4r+1) = xb'+ j * yb'    
 * X(4r+2) = xc'+ j * yc'    
 * X(4r+3) = xd'+ j * yd'    
 * </pre>    
 * \par    
 * Twiddle factors for radix-4 FFT:    
 * <pre>    
 * Wn = co1 + j * (- si1)    
 * W2n = co2 + j * (- si2)    
 * W3n = co3 + j * (- si3)    
 * </pre>    
 *    
 * \par    
 * \image html CFFT.gif "Radix-4 Decimation-in Frequency Complex Fast Fourier Transform"    
 *    
 * \par    
 * Output from Radix-4 CFFT Results in Digit reversal order. Interchange middle two branches of every butterfly results in Bit reversed output.    
 * \par    
 * <b> Butterfly CFFT equations:</b>    
 * <pre>    
 * xa' = xa + xb + xc + xd    
 * ya' = ya + yb + yc + yd    
 * xc' = (xa+yb-xc-yd)* co1 + (ya-xb-yc+xd)* (si1)    
 * yc' = (ya-xb-yc+xd)* co1 - (xa+yb-xc-yd)* (si1)    
 * xb' = (xa-xb+xc-xd)* co2 + (ya-yb+yc-yd)* (si2)    
 * yb' = (ya-yb+yc-yd)* co2 - (xa-xb+xc-xd)* (si2)    
 * xd' = (xa-yb-xc+yd)* co3 + (ya+xb-yc-xd)* (si3)    
 * yd' = (ya+xb-yc-xd)* co3 - (xa-yb-xc+yd)* (si3)    
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
 * xa' = xa + xb + xc + xd    
 * ya' = ya + yb + yc + yd    
 * xc' = (xa-yb-xc+yd)* co1 - (ya+xb-yc-xd)* (si1)    
 * yc' = (ya+xb-yc-xd)* co1 + (xa-yb-xc+yd)* (si1)    
 * xb' = (xa-xb+xc-xd)* co2 - (ya-yb+yc-yd)* (si2)    
 * yb' = (ya-yb+yc-yd)* co2 + (xa-xb+xc-xd)* (si2)    
 * xd' = (xa+yb-xc-yd)* co3 - (ya-xb-yc+xd)* (si3)    
 * yd' = (ya-xb-yc+xd)* co3 + (xa+yb-xc-yd)* (si3)    
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
 *arm_cfft_radix4_instance_f32 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor, onebyfftLen};    
 *arm_cfft_radix4_instance_q31 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor};    
 *arm_cfft_radix4_instance_q15 S = {fftLen, ifftFlag, bitReverseFlag, pTwiddle, pBitRevTable, twidCoefModifier, bitRevFactor};    
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
 * @addtogroup Radix4_CFFT_CIFFT    
 * @{    
 */

/**    
 * @details    
 * @brief Processing function for the floating-point Radix-4 CFFT/CIFFT.   
 * @param[in]      *S    points to an instance of the floating-point Radix-4 CFFT/CIFFT structure.   
 * @param[in, out] *pSrc points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place.   
 * @return none.   
 */

void arm_cfft_radix4_f32(
  const arm_cfft_radix4_instance_f32 * S,
  float32_t * pSrc)
{

  if(S->ifftFlag == 1u)
  {
    /*  Complex IFFT radix-4  */
    arm_radix4_butterfly_inverse_f32(pSrc, S->fftLen, S->pTwiddle,
                                     S->twidCoefModifier, S->onebyfftLen);
  }
  else
  {
    /*  Complex FFT radix-4  */
    arm_radix4_butterfly_f32(pSrc, S->fftLen, S->pTwiddle,
                             S->twidCoefModifier);
  }

  if(S->bitReverseFlag == 1u)
  {
    /*  Bit Reversal */
    arm_bitreversal_f32(pSrc, S->fftLen, S->bitRevFactor, S->pBitRevTable);
  }

}


/**    
 * @} end of Radix4_CFFT_CIFFT group    
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

void arm_radix4_butterfly_f32(
  float32_t * pSrc,
  uint16_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier)
{

  float32_t co1, co2, co3, si1, si2, si3;
  uint32_t ia1, ia2, ia3;
  uint32_t i0, i1, i2, i3;
  uint32_t n1, n2, j, k;

#ifndef ARM_MATH_CM0

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float32_t xaIn, yaIn, xbIn, ybIn, xcIn, ycIn, xdIn, ydIn;
  float32_t Xaplusc, Xbplusd, Yaplusc, Ybplusd, Xaminusc, Xbminusd, Yaminusc,
    Ybminusd;
  float32_t Xb12C_out, Yb12C_out, Xc12C_out, Yc12C_out, Xd12C_out, Yd12C_out;
  float32_t Xb12_out, Yb12_out, Xc12_out, Yc12_out, Xd12_out, Yd12_out;
  float32_t *ptr1;

  /*  Initializations for the first stage */
  n2 = fftLen;
  n1 = n2;

  /* n2 = fftLen/4 */
  n2 >>= 2u;
  i0 = 0u;
  ia1 = 0u;

  j = n2;

  /*  Calculation of first stage */
  do
  {
    /*  index calculation for the input as, */
    /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;

    xaIn = pSrc[(2u * i0)];
    yaIn = pSrc[(2u * i0) + 1u];

    xcIn = pSrc[(2u * i2)];
    ycIn = pSrc[(2u * i2) + 1u];

    xbIn = pSrc[(2u * i1)];
    ybIn = pSrc[(2u * i1) + 1u];

    xdIn = pSrc[(2u * i3)];
    ydIn = pSrc[(2u * i3) + 1u];

    /* xa + xc */
    Xaplusc = xaIn + xcIn;
    /* xb + xd */
    Xbplusd = xbIn + xdIn;
    /* ya + yc */
    Yaplusc = yaIn + ycIn;
    /* yb + yd */
    Ybplusd = ybIn + ydIn;

    /*  index calculation for the coefficients */
    ia2 = ia1 + ia1;
    co2 = pCoef[ia2 * 2u];
    si2 = pCoef[(ia2 * 2u) + 1u];

    /* xa - xc */
    Xaminusc = xaIn - xcIn;
    /* xb - xd */
    Xbminusd = xbIn - xdIn;
    /* ya - yc */
    Yaminusc = yaIn - ycIn;
    /* yb + yd */
    Ybminusd = ybIn - ydIn;

    /* xa' = xa + xb + xc + xd */
    pSrc[(2u * i0)] = Xaplusc + Xbplusd;
    /* ya' = ya + yb + yc + yd */
    pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

    /* (xa - xc) + (yb - yd) */
    Xb12C_out = (Xaminusc + Ybminusd);
    /* (ya - yc) + (xb - xd) */
    Yb12C_out = (Yaminusc - Xbminusd);
    /* (xa + xc) - (xb + xd) */
    Xc12C_out = (Xaplusc - Xbplusd);
    /* (ya + yc) - (yb + yd) */
    Yc12C_out = (Yaplusc - Ybplusd);
    /* (xa - xc) - (yb - yd) */
    Xd12C_out = (Xaminusc - Ybminusd);
    /* (ya - yc) + (xb - xd) */
    Yd12C_out = (Xbminusd + Yaminusc);

    co1 = pCoef[ia1 * 2u];
    si1 = pCoef[(ia1 * 2u) + 1u];

    /*  index calculation for the coefficients */
    ia3 = ia2 + ia1;
    co3 = pCoef[ia3 * 2u];
    si3 = pCoef[(ia3 * 2u) + 1u];

    Xb12_out = Xb12C_out * co1;
    Yb12_out = Yb12C_out * co1;
    Xc12_out = Xc12C_out * co2;
    Yc12_out = Yc12C_out * co2;
    Xd12_out = Xd12C_out * co3;
    Yd12_out = Yd12C_out * co3;

    /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
    Xb12_out += Yb12C_out * si1;
    /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
    Yb12_out -= Xb12C_out * si1;
    /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
    Xc12_out += Yc12C_out * si2;
    /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
    Yc12_out -= Xc12C_out * si2;
    /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
    Xd12_out += Yd12C_out * si3;
    /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
    Yd12_out -= Xd12C_out * si3;


    /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
    pSrc[2u * i1] = Xc12_out;

    /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
    pSrc[(2u * i1) + 1u] = Yc12_out;

    /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
    pSrc[2u * i2] = Xb12_out;

    /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
    pSrc[(2u * i2) + 1u] = Yb12_out;

    /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
    pSrc[2u * i3] = Xd12_out;

    /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
    pSrc[(2u * i3) + 1u] = Yd12_out;

    /*  Twiddle coefficients index modifier */
    ia1 = ia1 + twidCoefModifier;

    /*  Updating input index */
    i0 = i0 + 1u;

  }
  while(--j);

  twidCoefModifier <<= 2u;

  /*  Calculation of second stage to excluding last stage */
  for (k = fftLen / 4; k > 4u; k >>= 2u)
  {
    /*  Initializations for the first stage */
    n1 = n2;
    n2 >>= 2u;
    ia1 = 0u;

    /*  Calculation of first stage */
    for (j = 0u; j <= (n2 - 1u); j++)
    {
      /*  index calculation for the coefficients */
      ia2 = ia1 + ia1;
      ia3 = ia2 + ia1;
      co1 = pCoef[ia1 * 2u];
      si1 = pCoef[(ia1 * 2u) + 1u];
      co2 = pCoef[ia2 * 2u];
      si2 = pCoef[(ia2 * 2u) + 1u];
      co3 = pCoef[ia3 * 2u];
      si3 = pCoef[(ia3 * 2u) + 1u];

      /*  Twiddle coefficients index modifier */
      ia1 = ia1 + twidCoefModifier;

      for (i0 = j; i0 < fftLen; i0 += n1)
      {
        /*  index calculation for the input as, */
        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        xaIn = pSrc[(2u * i0)];
        yaIn = pSrc[(2u * i0) + 1u];

        xbIn = pSrc[(2u * i1)];
        ybIn = pSrc[(2u * i1) + 1u];

        xcIn = pSrc[(2u * i2)];
        ycIn = pSrc[(2u * i2) + 1u];

        xdIn = pSrc[(2u * i3)];
        ydIn = pSrc[(2u * i3) + 1u];

        /* xa - xc */
        Xaminusc = xaIn - xcIn;
        /* (xb - xd) */
        Xbminusd = xbIn - xdIn;
        /* ya - yc */
        Yaminusc = yaIn - ycIn;
        /* (yb - yd) */
        Ybminusd = ybIn - ydIn;

        /* xa + xc */
        Xaplusc = xaIn + xcIn;
        /* xb + xd */
        Xbplusd = xbIn + xdIn;
        /* ya + yc */
        Yaplusc = yaIn + ycIn;
        /* yb + yd */
        Ybplusd = ybIn + ydIn;

        /* (xa - xc) + (yb - yd) */
        Xb12C_out = (Xaminusc + Ybminusd);
        /* (ya - yc) -  (xb - xd) */
        Yb12C_out = (Yaminusc - Xbminusd);
        /* xa + xc -(xb + xd) */
        Xc12C_out = (Xaplusc - Xbplusd);
        /* (ya + yc) - (yb + yd) */
        Yc12C_out = (Yaplusc - Ybplusd);
        /* (xa - xc) - (yb - yd) */
        Xd12C_out = (Xaminusc - Ybminusd);
        /* (ya - yc) +  (xb - xd) */
        Yd12C_out = (Xbminusd + Yaminusc);

        pSrc[(2u * i0)] = Xaplusc + Xbplusd;
        pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

        Xb12_out = Xb12C_out * co1;
        Yb12_out = Yb12C_out * co1;
        Xc12_out = Xc12C_out * co2;
        Yc12_out = Yc12C_out * co2;
        Xd12_out = Xd12C_out * co3;
        Yd12_out = Yd12C_out * co3;

        /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
        Xb12_out += Yb12C_out * si1;
        /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
        Yb12_out -= Xb12C_out * si1;
        /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
        Xc12_out += Yc12C_out * si2;
        /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
        Yc12_out -= Xc12C_out * si2;
        /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
        Xd12_out += Yd12C_out * si3;
        /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
        Yd12_out -= Xd12C_out * si3;

        /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
        pSrc[2u * i1] = Xc12_out;

        /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
        pSrc[(2u * i1) + 1u] = Yc12_out;

        /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
        pSrc[2u * i2] = Xb12_out;

        /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
        pSrc[(2u * i2) + 1u] = Yb12_out;

        /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
        pSrc[2u * i3] = Xd12_out;

        /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
        pSrc[(2u * i3) + 1u] = Yd12_out;

      }
    }
    twidCoefModifier <<= 2u;
  }

  j = fftLen >> 2;
  ptr1 = &pSrc[0];

  /*  Calculations of last stage */
  do
  {

    xaIn = ptr1[0];
    xcIn = ptr1[4];
    yaIn = ptr1[1];
    ycIn = ptr1[5];

    /* xa + xc */
    Xaplusc = xaIn + xcIn;

    xbIn = ptr1[2];

    /* xa - xc */
    Xaminusc = xaIn - xcIn;

    xdIn = ptr1[6];

    /* ya + yc */
    Yaplusc = yaIn + ycIn;

    ybIn = ptr1[3];

    /* ya - yc */
    Yaminusc = yaIn - ycIn;

    ydIn = ptr1[7];

    /* xb + xd */
    Xbplusd = xbIn + xdIn;

    /* yb + yd */
    Ybplusd = ybIn + ydIn;

    /* xa' = xa + xb + xc + xd */
    ptr1[0] = (Xaplusc + Xbplusd);

    /* (xb-xd) */
    Xbminusd = xbIn - xdIn;

    /* ya' = ya + yb + yc + yd */
    ptr1[1] = (Yaplusc + Ybplusd);

    /* (yb-yd) */
    Ybminusd = ybIn - ydIn;

    /* xc' = (xa-xb+xc-xd) */
    ptr1[2] = (Xaplusc - Xbplusd);
    /* yc' = (ya-yb+yc-yd) */
    ptr1[3] = (Yaplusc - Ybplusd);
    /* xb' = (xa+yb-xc-yd) */
    ptr1[4] = (Xaminusc + Ybminusd);
    /* yb' = (ya-xb-yc+xd) */
    ptr1[5] = (Yaminusc - Xbminusd);
    /* xd' = (xa-yb-xc+yd)) */
    ptr1[6] = (Xaminusc - Ybminusd);
    /* yd' = (ya+xb-yc-xd) */
    ptr1[7] = (Xbminusd + Yaminusc);

    /* increment pointer by 8 */
    ptr1 = ptr1 + 8u;

  } while(--j);

#else

  float32_t t1, t2, r1, r2, s1, s2;

  /* Run the below code for Cortex-M0 */

  /*  Initializations for the fft calculation */
  n2 = fftLen;
  n1 = n2;
  for (k = fftLen; k > 1u; k >>= 2u)
  {
    /*  Initializations for the fft calculation */
    n1 = n2;
    n2 >>= 2u;
    ia1 = 0u;

    /*  FFT Calculation */
    for (j = 0u; j <= (n2 - 1u); j++)
    {
      /*  index calculation for the coefficients */
      ia2 = ia1 + ia1;
      ia3 = ia2 + ia1;
      co1 = pCoef[ia1 * 2u];
      si1 = pCoef[(ia1 * 2u) + 1u];
      co2 = pCoef[ia2 * 2u];
      si2 = pCoef[(ia2 * 2u) + 1u];
      co3 = pCoef[ia3 * 2u];
      si3 = pCoef[(ia3 * 2u) + 1u];

      /*  Twiddle coefficients index modifier */
      ia1 = ia1 + twidCoefModifier;

      for (i0 = j; i0 < fftLen; i0 += n1)
      {
        /*  index calculation for the input as, */
        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* xa + xc */
        r1 = pSrc[(2u * i0)] + pSrc[(2u * i2)];

        /* xa - xc */
        r2 = pSrc[(2u * i0)] - pSrc[(2u * i2)];

        /* ya + yc */
        s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

        /* ya - yc */
        s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

        /* xb + xd */
        t1 = pSrc[2u * i1] + pSrc[2u * i3];

        /* xa' = xa + xb + xc + xd */
        pSrc[2u * i0] = r1 + t1;

        /* xa + xc -(xb + xd) */
        r1 = r1 - t1;

        /* yb + yd */
        t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

        /* ya' = ya + yb + yc + yd */
        pSrc[(2u * i0) + 1u] = s1 + t2;

        /* (ya + yc) - (yb + yd) */
        s1 = s1 - t2;

        /* (yb - yd) */
        t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

        /* (xb - xd) */
        t2 = pSrc[2u * i1] - pSrc[2u * i3];

        /* xc' = (xa-xb+xc-xd)co2 + (ya-yb+yc-yd)(si2) */
        pSrc[2u * i1] = (r1 * co2) + (s1 * si2);

        /* yc' = (ya-yb+yc-yd)co2 - (xa-xb+xc-xd)(si2) */
        pSrc[(2u * i1) + 1u] = (s1 * co2) - (r1 * si2);

        /* (xa - xc) + (yb - yd) */
        r1 = r2 + t1;

        /* (xa - xc) - (yb - yd) */
        r2 = r2 - t1;

        /* (ya - yc) -  (xb - xd) */
        s1 = s2 - t2;

        /* (ya - yc) +  (xb - xd) */
        s2 = s2 + t2;

        /* xb' = (xa+yb-xc-yd)co1 + (ya-xb-yc+xd)(si1) */
        pSrc[2u * i2] = (r1 * co1) + (s1 * si1);

        /* yb' = (ya-xb-yc+xd)co1 - (xa+yb-xc-yd)(si1) */
        pSrc[(2u * i2) + 1u] = (s1 * co1) - (r1 * si1);

        /* xd' = (xa-yb-xc+yd)co3 + (ya+xb-yc-xd)(si3) */
        pSrc[2u * i3] = (r2 * co3) + (s2 * si3);

        /* yd' = (ya+xb-yc-xd)co3 - (xa-yb-xc+yd)(si3) */
        pSrc[(2u * i3) + 1u] = (s2 * co3) - (r2 * si3);
      }
    }
    twidCoefModifier <<= 2u;
  }

#endif /* #ifndef ARM_MATH_CM0 */

}

/*    
 * @brief  Core function for the floating-point CIFFT butterfly process.   
 * @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.   
 * @param[in]      fftLen           length of the FFT.   
 * @param[in]      *pCoef           points to twiddle coefficient buffer.   
 * @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.   
 * @param[in]      onebyfftLen      value of 1/fftLen.   
 * @return none.   
 */

void arm_radix4_butterfly_inverse_f32(
  float32_t * pSrc,
  uint16_t fftLen,
  float32_t * pCoef,
  uint16_t twidCoefModifier,
  float32_t onebyfftLen)
{
  float32_t co1, co2, co3, si1, si2, si3;
  uint32_t ia1, ia2, ia3;
  uint32_t i0, i1, i2, i3;
  uint32_t n1, n2, j, k;

#ifndef ARM_MATH_CM0

  float32_t xaIn, yaIn, xbIn, ybIn, xcIn, ycIn, xdIn, ydIn;
  float32_t Xaplusc, Xbplusd, Yaplusc, Ybplusd, Xaminusc, Xbminusd, Yaminusc,
    Ybminusd;
  float32_t Xb12C_out, Yb12C_out, Xc12C_out, Yc12C_out, Xd12C_out, Yd12C_out;
  float32_t Xb12_out, Yb12_out, Xc12_out, Yc12_out, Xd12_out, Yd12_out;
  float32_t *ptr1;


  /*  Initializations for the first stage */
  n2 = fftLen;
  n1 = n2;

  /* n2 = fftLen/4 */
  n2 >>= 2u;
  i0 = 0u;
  ia1 = 0u;

  j = n2;

  /*  Calculation of first stage */
  do
  {
    /*  index calculation for the input as, */
    /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;

    /*  Butterfly implementation */
    xaIn = pSrc[(2u * i0)];
    yaIn = pSrc[(2u * i0) + 1u];

    xcIn = pSrc[(2u * i2)];
    ycIn = pSrc[(2u * i2) + 1u];

    xbIn = pSrc[(2u * i1)];
    ybIn = pSrc[(2u * i1) + 1u];

    xdIn = pSrc[(2u * i3)];
    ydIn = pSrc[(2u * i3) + 1u];

    /* xa + xc */
    Xaplusc = xaIn + xcIn;
    /* xb + xd */
    Xbplusd = xbIn + xdIn;
    /* ya + yc */
    Yaplusc = yaIn + ycIn;
    /* yb + yd */
    Ybplusd = ybIn + ydIn;

    /*  index calculation for the coefficients */
    ia2 = ia1 + ia1;
    co2 = pCoef[ia2 * 2u];
    si2 = pCoef[(ia2 * 2u) + 1u];

    /* xa - xc */
    Xaminusc = xaIn - xcIn;
    /* xb - xd */
    Xbminusd = xbIn - xdIn;
    /* ya - yc */
    Yaminusc = yaIn - ycIn;
    /* yb - yd */
    Ybminusd = ybIn - ydIn;

    /* xa' = xa + xb + xc + xd */
    pSrc[(2u * i0)] = Xaplusc + Xbplusd;

    /* ya' = ya + yb + yc + yd */
    pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

    /* (xa - xc) - (yb - yd) */
    Xb12C_out = (Xaminusc - Ybminusd);
    /* (ya - yc) + (xb - xd) */
    Yb12C_out = (Yaminusc + Xbminusd);
    /* (xa + xc) - (xb + xd) */
    Xc12C_out = (Xaplusc - Xbplusd);
    /* (ya + yc) - (yb + yd) */
    Yc12C_out = (Yaplusc - Ybplusd);
    /* (xa - xc) + (yb - yd) */
    Xd12C_out = (Xaminusc + Ybminusd);
    /* (ya - yc) - (xb - xd) */
    Yd12C_out = (Yaminusc - Xbminusd);

    co1 = pCoef[ia1 * 2u];
    si1 = pCoef[(ia1 * 2u) + 1u];

    /*  index calculation for the coefficients */
    ia3 = ia2 + ia1;
    co3 = pCoef[ia3 * 2u];
    si3 = pCoef[(ia3 * 2u) + 1u];

    Xb12_out = Xb12C_out * co1;
    Yb12_out = Yb12C_out * co1;
    Xc12_out = Xc12C_out * co2;
    Yc12_out = Yc12C_out * co2;
    Xd12_out = Xd12C_out * co3;
    Yd12_out = Yd12C_out * co3;

    /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
    Xb12_out -= Yb12C_out * si1;
    /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
    Yb12_out += Xb12C_out * si1;
    /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
    Xc12_out -= Yc12C_out * si2;
    /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
    Yc12_out += Xc12C_out * si2;
    /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
    Xd12_out -= Yd12C_out * si3;
    /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
    Yd12_out += Xd12C_out * si3;

    /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
    pSrc[2u * i1] = Xc12_out;

    /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
    pSrc[(2u * i1) + 1u] = Yc12_out;

    /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
    pSrc[2u * i2] = Xb12_out;

    /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
    pSrc[(2u * i2) + 1u] = Yb12_out;

    /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
    pSrc[2u * i3] = Xd12_out;

    /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
    pSrc[(2u * i3) + 1u] = Yd12_out;

    /*  Twiddle coefficients index modifier */
    ia1 = ia1 + twidCoefModifier;

    /*  Updating input index */
    i0 = i0 + 1u;

  } while(--j);

  twidCoefModifier <<= 2u;

  /*  Calculation of second stage to excluding last stage */
  for (k = fftLen / 4; k > 4u; k >>= 2u)
  {
    /*  Initializations for the first stage */
    n1 = n2;
    n2 >>= 2u;
    ia1 = 0u;

    /*  Calculation of first stage */
    for (j = 0u; j <= (n2 - 1u); j++)
    {
      /*  index calculation for the coefficients */
      ia2 = ia1 + ia1;
      ia3 = ia2 + ia1;
      co1 = pCoef[ia1 * 2u];
      si1 = pCoef[(ia1 * 2u) + 1u];
      co2 = pCoef[ia2 * 2u];
      si2 = pCoef[(ia2 * 2u) + 1u];
      co3 = pCoef[ia3 * 2u];
      si3 = pCoef[(ia3 * 2u) + 1u];

      /*  Twiddle coefficients index modifier */
      ia1 = ia1 + twidCoefModifier;

      for (i0 = j; i0 < fftLen; i0 += n1)
      {
        /*  index calculation for the input as, */
        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        xaIn = pSrc[(2u * i0)];
        yaIn = pSrc[(2u * i0) + 1u];

        xbIn = pSrc[(2u * i1)];
        ybIn = pSrc[(2u * i1) + 1u];

        xcIn = pSrc[(2u * i2)];
        ycIn = pSrc[(2u * i2) + 1u];

        xdIn = pSrc[(2u * i3)];
        ydIn = pSrc[(2u * i3) + 1u];

        /* xa - xc */
        Xaminusc = xaIn - xcIn;
        /* (xb - xd) */
        Xbminusd = xbIn - xdIn;
        /* ya - yc */
        Yaminusc = yaIn - ycIn;
        /* (yb - yd) */
        Ybminusd = ybIn - ydIn;

        /* xa + xc */
        Xaplusc = xaIn + xcIn;
        /* xb + xd */
        Xbplusd = xbIn + xdIn;
        /* ya + yc */
        Yaplusc = yaIn + ycIn;
        /* yb + yd */
        Ybplusd = ybIn + ydIn;

        /* (xa - xc) - (yb - yd) */
        Xb12C_out = (Xaminusc - Ybminusd);
        /* (ya - yc) +  (xb - xd) */
        Yb12C_out = (Yaminusc + Xbminusd);
        /* xa + xc -(xb + xd) */
        Xc12C_out = (Xaplusc - Xbplusd);
        /* (ya + yc) - (yb + yd) */
        Yc12C_out = (Yaplusc - Ybplusd);
        /* (xa - xc) + (yb - yd) */
        Xd12C_out = (Xaminusc + Ybminusd);
        /* (ya - yc) -  (xb - xd) */
        Yd12C_out = (Yaminusc - Xbminusd);

        pSrc[(2u * i0)] = Xaplusc + Xbplusd;
        pSrc[(2u * i0) + 1u] = Yaplusc + Ybplusd;

        Xb12_out = Xb12C_out * co1;
        Yb12_out = Yb12C_out * co1;
        Xc12_out = Xc12C_out * co2;
        Yc12_out = Yc12C_out * co2;
        Xd12_out = Xd12C_out * co3;
        Yd12_out = Yd12C_out * co3;

        /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
        Xb12_out -= Yb12C_out * si1;
        /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
        Yb12_out += Xb12C_out * si1;
        /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
        Xc12_out -= Yc12C_out * si2;
        /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
        Yc12_out += Xc12C_out * si2;
        /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
        Xd12_out -= Yd12C_out * si3;
        /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
        Yd12_out += Xd12C_out * si3;

        /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
        pSrc[2u * i1] = Xc12_out;

        /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
        pSrc[(2u * i1) + 1u] = Yc12_out;

        /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
        pSrc[2u * i2] = Xb12_out;

        /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
        pSrc[(2u * i2) + 1u] = Yb12_out;

        /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
        pSrc[2u * i3] = Xd12_out;

        /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
        pSrc[(2u * i3) + 1u] = Yd12_out;

      }
    }
    twidCoefModifier <<= 2u;
  }
  /*  Initializations of last stage */

  j = fftLen >> 2;
  ptr1 = &pSrc[0];

  /*  Calculations of last stage */
  do
  {

    xaIn = ptr1[0];
    xcIn = ptr1[4];
    yaIn = ptr1[1];
    ycIn = ptr1[5];

    /*  Butterfly implementation */
    /* xa + xc */
    Xaplusc = xaIn + xcIn;

    xbIn = ptr1[2];

    /* xa - xc */
    Xaminusc = xaIn - xcIn;

    xdIn = ptr1[6];

    /* ya + yc */
    Yaplusc = yaIn + ycIn;

    ybIn = ptr1[3];

    /* ya - yc */
    Yaminusc = yaIn - ycIn;

    ydIn = ptr1[7];

    /* xc + xd */
    Xbplusd = xbIn + xdIn;

    /* yb + yd */
    Ybplusd = ybIn + ydIn;

    /* xa' = xa + xb + xc + xd */
    ptr1[0] = (Xaplusc + Xbplusd) * onebyfftLen;

    /* (xb-xd) */
    Xbminusd = xbIn - xdIn;

    /* ya' = ya + yb + yc + yd */
    ptr1[1] = (Yaplusc + Ybplusd) * onebyfftLen;

    /* (yb-yd) */
    Ybminusd = ybIn - ydIn;

    /* xc' = (xa-xb+xc-xd) * onebyfftLen */
    ptr1[2] = (Xaplusc - Xbplusd) * onebyfftLen;

    /* yc' = (ya-yb+yc-yd) * onebyfftLen  */
    ptr1[3] = (Yaplusc - Ybplusd) * onebyfftLen;

    /* xb' = (xa-yb-xc+yd) * onebyfftLen */
    ptr1[4] = (Xaminusc - Ybminusd) * onebyfftLen;

    /* yb' = (ya+xb-yc-xd) * onebyfftLen */
    ptr1[5] = (Yaminusc + Xbminusd) * onebyfftLen;

    /* xd' = (xa-yb-xc+yd) * onebyfftLen */
    ptr1[6] = (Xaminusc + Ybminusd) * onebyfftLen;

    /* yd' = (ya-xb-yc+xd) * onebyfftLen */
    ptr1[7] = (Yaminusc - Xbminusd) * onebyfftLen;

    /* increment source pointer by 8 for next calculations */
    ptr1 = ptr1 + 8u;

  } while(--j);

#else

  float32_t t1, t2, r1, r2, s1, s2;

  /* Run the below code for Cortex-M0 */

  /*  Initializations for the first stage */
  n2 = fftLen;
  n1 = n2;

  /*  Calculation of first stage */
  for (k = fftLen; k > 4u; k >>= 2u)
  {
    /*  Initializations for the first stage */
    n1 = n2;
    n2 >>= 2u;
    ia1 = 0u;

    /*  Calculation of first stage */
    for (j = 0u; j <= (n2 - 1u); j++)
    {
      /*  index calculation for the coefficients */
      ia2 = ia1 + ia1;
      ia3 = ia2 + ia1;
      co1 = pCoef[ia1 * 2u];
      si1 = pCoef[(ia1 * 2u) + 1u];
      co2 = pCoef[ia2 * 2u];
      si2 = pCoef[(ia2 * 2u) + 1u];
      co3 = pCoef[ia3 * 2u];
      si3 = pCoef[(ia3 * 2u) + 1u];

      /*  Twiddle coefficients index modifier */
      ia1 = ia1 + twidCoefModifier;

      for (i0 = j; i0 < fftLen; i0 += n1)
      {
        /*  index calculation for the input as, */
        /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
        i1 = i0 + n2;
        i2 = i1 + n2;
        i3 = i2 + n2;

        /* xa + xc */
        r1 = pSrc[(2u * i0)] + pSrc[(2u * i2)];

        /* xa - xc */
        r2 = pSrc[(2u * i0)] - pSrc[(2u * i2)];

        /* ya + yc */
        s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

        /* ya - yc */
        s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

        /* xb + xd */
        t1 = pSrc[2u * i1] + pSrc[2u * i3];

        /* xa' = xa + xb + xc + xd */
        pSrc[2u * i0] = r1 + t1;

        /* xa + xc -(xb + xd) */
        r1 = r1 - t1;

        /* yb + yd */
        t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

        /* ya' = ya + yb + yc + yd */
        pSrc[(2u * i0) + 1u] = s1 + t2;

        /* (ya + yc) - (yb + yd) */
        s1 = s1 - t2;

        /* (yb - yd) */
        t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

        /* (xb - xd) */
        t2 = pSrc[2u * i1] - pSrc[2u * i3];

        /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
        pSrc[2u * i1] = (r1 * co2) - (s1 * si2);

        /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
        pSrc[(2u * i1) + 1u] = (s1 * co2) + (r1 * si2);

        /* (xa - xc) - (yb - yd) */
        r1 = r2 - t1;

        /* (xa - xc) + (yb - yd) */
        r2 = r2 + t1;

        /* (ya - yc) +  (xb - xd) */
        s1 = s2 + t2;

        /* (ya - yc) -  (xb - xd) */
        s2 = s2 - t2;

        /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
        pSrc[2u * i2] = (r1 * co1) - (s1 * si1);

        /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
        pSrc[(2u * i2) + 1u] = (s1 * co1) + (r1 * si1);

        /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
        pSrc[2u * i3] = (r2 * co3) - (s2 * si3);

        /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
        pSrc[(2u * i3) + 1u] = (s2 * co3) + (r2 * si3);
      }
    }
    twidCoefModifier <<= 2u;
  }
  /*  Initializations of last stage */
  n1 = n2;
  n2 >>= 2u;

  /*  Calculations of last stage */
  for (i0 = 0u; i0 <= (fftLen - n1); i0 += n1)
  {
    /*  index calculation for the input as, */
    /*  pSrc[i0 + 0], pSrc[i0 + fftLen/4], pSrc[i0 + fftLen/2], pSrc[i0 + 3fftLen/4] */
    i1 = i0 + n2;
    i2 = i1 + n2;
    i3 = i2 + n2;

    /*  Butterfly implementation */
    /* xa + xc */
    r1 = pSrc[2u * i0] + pSrc[2u * i2];

    /* xa - xc */
    r2 = pSrc[2u * i0] - pSrc[2u * i2];

    /* ya + yc */
    s1 = pSrc[(2u * i0) + 1u] + pSrc[(2u * i2) + 1u];

    /* ya - yc */
    s2 = pSrc[(2u * i0) + 1u] - pSrc[(2u * i2) + 1u];

    /* xc + xd */
    t1 = pSrc[2u * i1] + pSrc[2u * i3];

    /* xa' = xa + xb + xc + xd */
    pSrc[2u * i0] = (r1 + t1) * onebyfftLen;

    /* (xa + xb) - (xc + xd) */
    r1 = r1 - t1;

    /* yb + yd */
    t2 = pSrc[(2u * i1) + 1u] + pSrc[(2u * i3) + 1u];

    /* ya' = ya + yb + yc + yd */
    pSrc[(2u * i0) + 1u] = (s1 + t2) * onebyfftLen;

    /* (ya + yc) - (yb + yd) */
    s1 = s1 - t2;

    /* (yb-yd) */
    t1 = pSrc[(2u * i1) + 1u] - pSrc[(2u * i3) + 1u];

    /* (xb-xd) */
    t2 = pSrc[2u * i1] - pSrc[2u * i3];

    /* xc' = (xa-xb+xc-xd)co2 - (ya-yb+yc-yd)(si2) */
    pSrc[2u * i1] = r1 * onebyfftLen;

    /* yc' = (ya-yb+yc-yd)co2 + (xa-xb+xc-xd)(si2) */
    pSrc[(2u * i1) + 1u] = s1 * onebyfftLen;


    /* (xa - xc) - (yb-yd) */
    r1 = r2 - t1;

    /* (xa - xc) + (yb-yd) */
    r2 = r2 + t1;

    /* (ya - yc) + (xb-xd) */
    s1 = s2 + t2;

    /* (ya - yc) - (xb-xd) */
    s2 = s2 - t2;

    /* xb' = (xa+yb-xc-yd)co1 - (ya-xb-yc+xd)(si1) */
    pSrc[2u * i2] = r1 * onebyfftLen;

    /* yb' = (ya-xb-yc+xd)co1 + (xa+yb-xc-yd)(si1) */
    pSrc[(2u * i2) + 1u] = s1 * onebyfftLen;

    /* xd' = (xa-yb-xc+yd)co3 - (ya+xb-yc-xd)(si3) */
    pSrc[2u * i3] = r2 * onebyfftLen;

    /* yd' = (ya+xb-yc-xd)co3 + (xa-yb-xc+yd)(si3) */
    pSrc[(2u * i3) + 1u] = s2 * onebyfftLen;
  }

#endif /* #ifndef ARM_MATH_CM0 */

}
