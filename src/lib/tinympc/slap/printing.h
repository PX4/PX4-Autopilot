#pragma once

#ifndef PRECISION
#define PRECISION 3
#endif

#include "matrix.h"

/**
 * @brief Print the elements of a matrix to stdout
 *
 * Precision of the printing can be controlled by the global variable PRECISION.
 *
 * @param mat Matrix to be printed
 */
enum slap_ErrorCode slap_PrintMatrix(Matrix mat);

/**
 * @brief Print the entire matrix as a row vector
 *
 * Same result as calling PrintMatrix() after a call to MatrixFlattenToRow().
 *
 * @param mat Matrix to be printed
 * @return 0 if successful
 */
int slap_PrintRowVector(Matrix mat);
