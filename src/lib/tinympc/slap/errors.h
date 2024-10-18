/**
 * @file errors.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Utilities for the slap error handling system
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup Utilities
 * @{
 */
#pragma once


enum slap_ErrorCode {
  SLAP_NO_ERROR = 0,
  SLAP_BAD_POINTER,
  SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS,
  SLAP_BAD_MATRIX_DATA_POINTER,
  SLAP_INVALID_DIMENSION,
  SLAP_INVALID_STRIDE,
  SLAP_MATRIX_NOT_SQUARE,
  SLAP_MATRIX_NOT_DENSE,
  SLAP_CHOLESKY_FAIL,
  SLAP_INVALID_MATRIX,
  SLAP_INDEX_OUT_OF_BOUNDS,
  SLAP_EMPTY_MATRIX,
};

const char* slap_ErrorString(enum slap_ErrorCode error_code);




// TODO (brian): Add compile option to turn this on/off
#include <stdio.h>

#define SLAP_COLOR_RED "\x1b[31m"
#define SLAP_COLOR_NORMAL "\x1b[0m"

enum slap_ErrorCode slap_PrintError(enum slap_ErrorCode error_code, const char* file, int line_number,
                                    const char* format, ...);

void slap_SetOutputFile(FILE* file);
FILE* slap_GetOutputFile(void);
int slap_AssertionsEnabled(void);

#ifdef NDEBUG

#define SLAP_ERROR(error_code, ...) error_code
#define SLAP_ASSERT(condition, error_code, return_value, ...) ((void)0)

#else

#define SLAP_ERROR(error_code, ...) \
  slap_PrintError(error_code, __FILE__, __LINE__, __VA_ARGS__)

#define SLAP_ASSERT(condition, error_code, return_value, ...) \
  if (!(condition)) {                                         \
    SLAP_ERROR(error_code, __VA_ARGS__);               \
    return return_value;                                      \
  }
#endif

#define SLAP_ASSERT_VALID(mat, return_value, ...) \
  SLAP_ASSERT(slap_IsValid(mat), SLAP_INVALID_MATRIX, return_value, __VA_ARGS__)

#define SLAP_ASSERT_DENSE(mat, return_value, ...) \
  SLAP_ASSERT(slap_IsDense(mat), SLAP_MATRIX_NOT_DENSE, return_value, __VA_ARGS__)

#define SLAP_ASSERT_SAME_SIZE(A, B, return_value, method_name)                          \
  SLAP_ASSERT(slap_NumRows(A) == slap_NumRows(B) && slap_NumCols(A) == slap_NumCols(B), \
              SLAP_INCOMPATIBLE_MATRIX_DIMENSIONS, (return_value),                      \
              "%s: matrices must be the same size. Got sizes (%d,%d) and (%d,%d)",      \
              method_name, slap_NumRows(A), slap_NumCols(A), slap_NumRows(B),           \
              slap_NumCols(B))
