#include "math.h"
#include "string.h"
#include "iirFilter.h"

///////////////////////////////////////////////////////////////////////////////
// Internal function prototypes

int btZpgcToZpgd(const TF_ZPG_t *pkZpgc, double Ts, TF_ZPG_t *pZpgd);

int btDifcToZpgd(const TF_DIF_t *pkDifc, double Ts, TF_ZPG_t *pZpgd);

int tPolydToFil(const TF_POLY_t *pkPolyd, FIL_T *pFilt);

int tZpgxToPolyx(const TF_ZPG_t *pkZpg, TF_POLY_t *pPoly);

///////////////////////////////////////////////////////////////////////////////
// external functions

int testFunction()
{
	printf("TEST\n");
	return 1;
}

int initFilter(const TF_DIF_t *pDifc, double Ts, FIL_T *pFilt)
{
	TF_POLY_t polyd;
	TF_ZPG_t zpgd;

	memset(pFilt, 0, sizeof(FIL_T));

	// perform bilinear transform with frequency pre warping
	btDifcToZpgd(pDifc, Ts, &zpgd);

	// calculate polynom
	tZpgxToPolyx(&zpgd, &polyd);

	// set the filter parameters
	tPolydToFil(&polyd, pFilt);

	return 1;
}

// run filter using inp, return output of the filter
float updateFilter(FIL_T *pFilt, float inp)
{
	float outp = 0;
	int idx; // index used for different things
	int i; // loop variable

	// Store the input to the input array
	idx = pFilt->inpCnt % MAX_LENGTH;
	pFilt->inpData[idx] = inp;

	// calculate numData * inpData
	for (i = 0; i < pFilt->numLength; i++)
	{
		// index of inp array
		idx = (pFilt->inpCnt + i - pFilt->numLength + 1) % MAX_LENGTH;
		outp += pFilt->numData[i] * pFilt->inpData[idx];
	}

	// calculate denData * outData
	for (i = 0; i < pFilt->denLength; i++)
	{
		// index of inp array
		idx = (pFilt->inpCnt + i - pFilt->denLength) % MAX_LENGTH;
		outp -= pFilt->denData[i] * pFilt->outData[idx];
	}

	// store the ouput data to the output array
	idx = pFilt->inpCnt % MAX_LENGTH;
	pFilt->outData[idx] = outp;

	pFilt->inpCnt += 1;

	return outp;
}

///////////////////////////////////////////////////////////////////////////////
// Internal functions

int tPolydToFil(const TF_POLY_t *pkPolyd, FIL_T *pFilt)
{
	double gain;
	int i;

	if (pkPolyd->Ts < 0)
	{
		return 0;
	}

	// initialize filter to 0
	memset(pFilt, 0, sizeof(FIL_T));

	gain = pkPolyd->denData[pkPolyd->denLength - 1];
	pFilt->Ts = pkPolyd->Ts;

	pFilt->denLength = pkPolyd->denLength - 1;
	pFilt->numLength = pkPolyd->numLength;

	for (i = 0; i < pkPolyd->numLength; i++)
	{
		pFilt->numData[i] = pkPolyd->numData[i] / gain;
	}

	for (i = 0; i < (pkPolyd->denLength - 1); i++)
	{
		pFilt->denData[i] = pkPolyd->denData[i] / gain;
	}
}

// bilinear transformation of poles and zeros
int btDifcToZpgd(const TF_DIF_t *pkDifc,
				 double Ts,
				 TF_ZPG_t *pZpgd)
{
	TF_ZPG_t zpgc;
	int totDiff;
	int i;

	zpgc.zerosLength = 0;
	zpgc.polesLength = 0;
	zpgc.gain = pkDifc->gain;
	zpgc.Ts = pkDifc->Ts;

	// Total number of differentiators / integerators
	// positive diff, negative integ, 0 for no int/diff
	totDiff = pkDifc->numDiff - pkDifc->numInt + pkDifc->highpassLength;

	while (zpgc.zerosLength < totDiff)
	{
		zpgc.zerosData[zpgc.zerosLength] = 0;
		zpgc.zerosLength++;
	}
	while (zpgc.polesLength < -totDiff)
	{
		zpgc.polesData[zpgc.polesLength] = 0;
		zpgc.polesLength++;
	}

	// The next step is to calculate the poles
	// This has to be done for the highpass and lowpass filters
	// As we are using bilinear transformation there will be frequency
	// warping which has to be accounted for
	for (i = 0; i < pkDifc->lowpassLength; i++)
	{
		// pre-warping:
		double freq = 2.0 / Ts * tan(pkDifc->lowpassData[i] * 2.0 * M_PI * Ts / 2.0);
		// calculate pole:
		zpgc.polesData[zpgc.polesLength] = -freq;
		// adjust gain such that lp has gain = 1 for low frequencies
		zpgc.gain *= freq;
		zpgc.polesLength++;
	}
	for (i = 0; i < pkDifc->highpassLength; i++)
	{
		// pre-warping
		double freq = 2.0 / Ts * tan(pkDifc->highpassData[i] * 2.0 * M_PI * Ts / 2.0);
		// calculate pole:
		zpgc.polesData[zpgc.polesLength] = -freq;
		// gain does not have to be adjusted
		zpgc.polesLength++;
	}

	return btZpgcToZpgd(&zpgc, Ts, pZpgd);
}

// bilinear transformation of poles and zeros
int btZpgcToZpgd(const TF_ZPG_t *pkZpgc,
				 double Ts,
				 TF_ZPG_t *pZpgd)
{
	int i;

	// init digital gain
	pZpgd->gain = pkZpgc->gain;

	// transform the poles
	pZpgd->polesLength = pkZpgc->polesLength;
	for (i = 0; i < pkZpgc->polesLength; i++)
	{
		pZpgd->polesData[i] = (2.0 / Ts + pkZpgc->polesData[i]) / (2.0 / Ts - pkZpgc->polesData[i]);
		pZpgd->gain /= (2.0 / Ts - pkZpgc->polesData[i]);
	}

	// transform the zeros
	pZpgd->zerosLength = pkZpgc->zerosLength;
	for (i = 0; i < pkZpgc->zerosLength; i++)
	{
		pZpgd->zerosData[i] = (2.0 / Ts + pkZpgc->zerosData[i]) / (2.0 / Ts + pkZpgc->zerosData[i]);
		pZpgd->gain *= (2.0 / Ts - pkZpgc->zerosData[i]);
	}

	// if we don't have the same number of poles as zeros we have to add
	// poles or zeros due to the bilinear transformation
	while (pZpgd->zerosLength < pZpgd->polesLength)
	{
		pZpgd->zerosData[pZpgd->zerosLength] = -1.0;
		pZpgd->zerosLength++;
	}
	while (pZpgd->zerosLength > pZpgd->polesLength)
	{
		pZpgd->polesData[pZpgd->polesLength] = -1.0;
		pZpgd->polesLength++;
	}

	pZpgd->Ts = Ts;

	return 1;
}

// calculate polynom from zero, pole, gain form
int tZpgxToPolyx(const TF_ZPG_t *pkZpg, TF_POLY_t *pPoly)
{
	int i, j; // counter variable
	double tmp0, tmp1, gain;

	// copy Ts
	pPoly->Ts = pkZpg->Ts;

	// calculate denominator polynom
	pPoly->denLength = 1;
	pPoly->denData[0] = 1.0;
	for (i = 0; i < pkZpg->polesLength; i++)
	{
		// init temporary variable
		tmp0 = 0.0;
		// increase the polynom by 1
		pPoly->denData[pPoly->denLength] = 0;
		pPoly->denLength++;
		for (j = 0; j < pPoly->denLength; j++)
		{
			tmp1 = pPoly->denData[j];
			pPoly->denData[j] = tmp1 * -pkZpg->polesData[i] + tmp0;
			tmp0 = tmp1;
		}
	}

	// calculate numerator polynom
	pPoly->numLength = 1;
	pPoly->numData[0] = pkZpg->gain;
	for (i = 0; i < pkZpg->zerosLength; i++)
	{
		tmp0 = 0.0;
		pPoly->numData[pPoly->numLength] = 0;
		pPoly->numLength++;
		for (j = 0; j < pPoly->numLength; j++)
		{
			tmp1 = pPoly->numData[j];
			pPoly->numData[j] = tmp1 * -pkZpg->zerosData[i] + tmp0;
			tmp0 = tmp1;
		}
	}
}
