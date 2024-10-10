#pragma once
#ifdef __cplusplus
extern "C" {
#endif

// NOTE: this is an odd fix to get gcov to run correctly on GitHub Actions:
// https://www.osadl.org/fileadmin/dam/interface/docbook/howtos/coverage.pdf
void __gcov_flush(void);

#include "matrix.h"
#include "new_matrix.h"
#include "copy_matrix.h"
#include "unary_ops.h"
#include "binary_ops.h"
#include "printing.h"
#include "vector_ops.h"
#include "vector_products.h"
#include "function_mapping.h"
#include "strided_matrix.h"
#include "matmul.h"
#include "cholesky.h"
#include "tri.h"
#include "qr.h"

#ifdef __cplusplus
}
#endif
