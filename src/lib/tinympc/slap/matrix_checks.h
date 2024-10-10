#pragma once
#include "matrix.h"

#define SLAP_CHECK_MATRIX(mat)                                      \
  {                                                                 \
    enum slap_ErrorCode _err_ = slap_CheckMatrix(mat);                \
    if (_err_ != SLAP_NO_ERROR)                                       \
      return SLAP_ERROR(_err_, "Matrix failed validity check"); \
  }

enum slap_ErrorCode slap_CheckMatrix(Matrix mat);


