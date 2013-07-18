#ifndef IIRFILTER__H__
#define IIRFILTER__H__

__BEGIN_DECLS

#define MAX_LENGTH 4

typedef struct FILTER_s
{
	float denData[MAX_LENGTH];
	float numData[MAX_LENGTH];
	int denLength;
	int numLength;
	float Ts;
	float inpData[MAX_LENGTH];
	float outData[MAX_LENGTH];
	unsigned int inpCnt;
} FIL_T;

typedef struct TF_ZPG_s
{
	int    zerosLength;
	double zerosData[MAX_LENGTH + 1];
	int    polesLength;
	double polesData[MAX_LENGTH + 1];
	double gain;
	double Ts;
} TF_ZPG_t;

typedef struct TF_POLY_s
{
	int    numLength;
	double numData[MAX_LENGTH];
	int    denLength;
	double denData[MAX_LENGTH];
	double Ts;
} TF_POLY_t;

typedef struct TF_DIF_s
{
	int    numInt;
	int    numDiff;
	int    lowpassLength;
	int    highpassLength;
	double lowpassData[MAX_LENGTH];
	int    highpassData[MAX_LENGTH];
	double gain;
	double Ts;
} TF_DIF_t;

__EXPORT int testFunction(void);

__EXPORT float updateFilter(FIL_T *pFilt, float inp);

__EXPORT int initFilter(const TF_DIF_t *pDifc, double Ts, FIL_T *pFilt);

__END_DECLS

#endif
