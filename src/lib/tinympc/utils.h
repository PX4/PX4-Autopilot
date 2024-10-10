#ifndef UTILS_H
# define UTILS_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include "types.h"
#include "constants.h"
#include "slap/slap.h"

//========================================
// Return length of an array
//========================================
#define T_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

//========================================
// Initialize memory with zeros
//========================================
#define T_INIT_ZEROS(data) (memset(data, 0, sizeof(data)))

//========================================
// Return a random noise from percentage
//========================================
#define T_NOISE(percent) (((2 * ((float)rand() / RAND_MAX)) - 1) / 100 * percent)

//========================================
// Maximum
//========================================
# ifndef T_MAX
#  define T_MAX(a, b) (((a) > (b)) ? (a) : (b))
# endif /* ifndef T_MAX */

//========================================
// Minimum
//========================================
# ifndef T_MIN
#  define T_MIN(a, b) (((a) < (b)) ? (a) : (b))
# endif /* ifndef T_MIN */

//========================================
// Absolute
//========================================
# ifndef T_ABS
#  define T_ABS(x) (((x) < 0) ? -(x) : (x))
# endif /* ifndef T_ABS */

//========================================
// Clamp the inputs to within min max value,
// will modify the provided array
//========================================
void tiny_Clamps(float* arr, const float* min, const float* max,
                 const int N);

void tiny_Clamp(float* arr, const float min, const float max, const int N);

void tiny_ClampMatrix(Matrix* mat, const Matrix min, const Matrix max);

void tiny_ShiftFill(Matrix* mats, const int length);

void tiny_ShiftFillWith(Matrix* mats, const float* x, const int length);


//========================================
// Raw matrix operators, ignore all metadata
//========================================
void SwapVectors(float **a, float **b);
void MatAdd(Matrix C, Matrix A, Matrix B, float alpha);
void MatCpy(Matrix des, Matrix src);
void MatScale(Matrix A, float alpha);
void MatMulAdd(Matrix C, Matrix A, Matrix B, float alpha, float beta);
void MatMulAdd2(Matrix D, Matrix C, Matrix A, Matrix B, float alpha, float beta);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef UTILS_H
