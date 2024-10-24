#include "printing.h"

#include <stdio.h>

enum slap_ErrorCode slap_PrintMatrix(const Matrix mat) {
  // SLAP_ASSERT_VALID(mat, SLAP_INVALID_MATRIX, "PrintMatrix: invalid matrix");
  // for (int row = 0; row < slap_NumRows(mat); ++row) {
  //   for (int col = 0; col < slap_NumCols(mat); ++col) {
  //     printf("% 8.*g ", PRECISION, *slap_GetElementConst(mat, row, col));
  //   }
  //   printf("\n");
  // }
  return 0;
}

int slap_PrintRowVector(Matrix mat) {
  // printf("[ ");
  // for (int i = 0; i < slap_NumElements(mat); ++i) {
  //   printf("% 6.*g ", PRECISION, mat.data[i]);
  // }
  // printf("]\n");
  return 0;
}
