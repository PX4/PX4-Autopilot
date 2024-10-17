/**
 * @file matrix.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Matrix type and fundamental operations like indexing and size
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup Basics
 * @{
 */
#pragma once

#include <inttypes.h>
#include <stdbool.h>

#include "errors.h"

#ifdef SLAP_FLOAT
typedef SLAP_FLOAT sfloat;
#else
typedef float sfloat;
#endif

enum slap_MatrixType {
  slap_DENSE,
//  slap_TRANSPOSED,
  slap_TRIANGULAR_UPPER,
  slap_TRIANGULAR_LOWER,
  //  slap_DIAGONAL
};


/**
 * @brief Represents a matrix of float-precision data
 * @headerfile slap.h
 *
 * Simple wrapper around an arbitrary pointer to the underlying data.
 * The data is assumed to be stored in a contiguous block of memory.
 * The data is interpreted column-wise, such that `data[1]` is element `[1,0]` of the
 * matrix.
 */
typedef struct Matrix {
  uint16_t rows;      //!< number of rows
  uint16_t cols;      //!< number of columns
  uint16_t sy;        //!< column stride (distance between adjacent elements in the same row)
  bool is_transposed; //!< is transposed
  float* data;       //!< pointer to the start of the data
  enum slap_MatrixType mattype;  //!< type of matrix
} Matrix;

//*********************************************//
// Initialization
//*********************************************//

/**
 * @brief Wraps existing data in a Matrix class
 * @headerfile matrix.h
 *
 * This is the most common way to "create" a matrix. Typical usage will look something like
 * this:
 *
 * ```c
 * // Stack-allocated memory
 * float data_stack[24];
 * Matrix A = slap_MatrixFromArray(6, 4, data_stack);
 *
 * // Heap-allocated memory
 * float *data_heap = (float*)malloc(24 * sizeof(float));
 * Matrix B = slap_MatrixFromArray(6, 4, data_heap);
 * free(data_heap);
 * ```
 *
 * @param rows Number of rows in the matrix
 * @param cols Number of columns in the matrix
 * @param data Data for the matrix. Must not be NULL, and should have at least
 *             @a rows * @a cols elements.
 * @return A new matrix
 */
Matrix slap_MatrixFromArray(int rows, int cols, float* data);

/**
 * @brief Create a "Null" matrix
 *
 * Useful for default initialization of the matrix where the data it wraps hasn't been
 * specified or allocated yet. Can check if a matrix is in this state using slap_IsNull().
 *
 * # Example
 * ```c
 * Matrix A = slap_NullMatrix();
 * ```
 *
 * See also: slap_IsNull(), slap_SetNull()
 *
 * @return A default "Null" instance of a matrix.
 */
static inline Matrix slap_NullMatrix(void) {
  // NOTE: Can't use named initializer here because it's inlined
  // (so causes issues for C++)
  Matrix mat = {
      0, 0, 0, 0, NULL, slap_DENSE,
  };
  return mat;
}

/**
 * @brief Set a matrix to a "Null" instance
 *
 * See also: slap_NullMatrix(), slap_IsNull()
 * @param[in] mat Pointer to the matrix to set to null
 */
static inline void slap_SetNull(Matrix* mat) {
  mat->rows = 0;
  mat->cols = 0;
  mat->sy = 0;
  mat->is_transposed = 0;
  mat->data = NULL;
  mat->mattype = slap_DENSE;
}

//*********************************************//
// Boolean Checks
//*********************************************//

/**
 * @brief Check if matrix is transposed
 *
 * Matrices are transposed by setting a flag that flips the indexing operations.
 * You can have two `Matrix` instances that point to the same data, where one is
 * transposed and the other is not. You can use this method to check whether a matrix
 * is transposed.
 *
 * See also: slap_Transpose()
 *
 * @param[in] mat Matrix to check
 * @return true if transposed, false otherwise
 */
static inline bool slap_IsTransposed(Matrix mat) { return mat.is_transposed; }

/**
 * @brief Check if matrix is empty, i.e. if any dimension is 0
 * @param[in] mat Any Matrix
 * @return true if the matrix is empty
 */
static inline bool slap_IsEmpty(Matrix mat) { return mat.rows <= 0 || mat.cols <= 0; }

/**
 * @brief Check if both dimensions are the same
 *
 * Note this will still return true if both dimensions are 0.
 *
 * @param[in] mat Any matrix
 * @return true if matrix is square
 */
static inline bool slap_IsSquare(Matrix mat) { return mat.rows == mat.cols; }

/**
 * @brief Check if all elements are adjacent in memory
 *
 * True if column stride is equal to the number of rows.
 *
 * @param[in] mat Any matrix
 */
static inline bool slap_IsDense(Matrix mat) { return mat.sy == mat.rows; }

/**
 * @brief Check if a matrix is valid
 *
 * Check if the matrix contains data: if matrix is not empty and pointer isn't null.
 *
 * @param[in] mat
 * @return true if matrix is valid
 */
static inline bool slap_IsValid(Matrix mat) {
  return (mat.data != NULL);
}


/**
 * @brief Check if matrix is a "Null" instance of a matrix (i.e. uninitialized)
 *
 * The "Null" state of a matrix is an internally-defined state created by the
 * slap_NullMatrix() method.
 *
 * See also: slap_NullMatrix(), slap_SetNull()
 *
 * @param[in] mat
 * @return true if matrix is a "Null" matrix
 */
static inline bool slap_IsNull(Matrix mat) {
  return slap_IsEmpty(mat) && mat.data == NULL && mat.sy == 0;
}

//*********************************************//
// Getters
//*********************************************//

/**
 * @brief Return the raw pointer stored by the matrix
 * @param mat Any matrix
 */
static inline float *slap_GetData(Matrix mat) { return mat.data; }

/**
 * @brief Return the type of the matrix
 *
 * The type is mostly used internally, and changes how the data is interpreted.
 * It is used by methods to specify things like whether a matrix is upper or lower
 * triangular.
 *
 * @param mat Any matrix
 */
static inline enum slap_MatrixType slap_GetType(Matrix mat) { return mat.mattype; }

//*********************************************//
// Dimensions
//*********************************************//

/**
 * @brief Returns the smallest dimension
 * @param[in] mat Any matrix
 * @return Smaller of the number of rows and columns
 */
static inline uint16_t slap_MinDim(Matrix mat) {
  return mat.rows <= mat.cols ? mat.rows : mat.cols;
}

/**
 * @brief Get the number of rows
 * @param[in] mat Any matrix
 * @return Number of rows
 */
static inline int slap_NumRows(Matrix mat) {
  // NOTE: No need to worry about unsigned->signed conversion here since signed precision
  //       is much higher (all values are representable by an `int`)
  return slap_IsTransposed(mat) ? (int)mat.cols : (int)mat.rows;
}

/**
 * @brief Get the number of columns
 * @param[in] mat Any matrix
 * @return Number of columns
 */
static inline int slap_NumCols(Matrix mat) {
  return slap_IsTransposed(mat) ? (int)mat.rows : (int)mat.cols;
}

/**
 * @brief Get the number of elements in a matrix, i.e. `m * n`.
 *
 * @param mat Any matrix
 * @return Number of elements in the matrix
 */
static inline int slap_NumElements(const Matrix mat) { return mat.rows * mat.cols; }

/**
 * @brief Get the column-stride stride of the matrix
 *
 * This is the distance in memory between adjacent elements of the same row.
 * For a "Dense" slap matrix where all elements are contiguous in memory, the stride is
 * equal to the number of rows.
 *
 * See also: slap_IsDense()
 *
 * @param mat
 * @return
 */
static inline int slap_Stride(const Matrix mat) { return mat.sy; }

//*********************************************//
// Indexing
//*********************************************//
/**
 * @brief Get the linear index for a given row and column in the matrix
 * @headerfile slap.h
 *
 * Converts a cartesian index of row and column into a linear index for accessing
 * an element of the underlying data.
 *
 * Supports both strided and dense matrices.
 *
 * @param mat Matrix with nonzero size and initialized data
 * @param row Row index
 * @param col Column index
 * @return Linear index corresponding to `row` and `col`.
           Returns -1 for a bad input.
 */
static inline int slap_Cart2Index(const Matrix mat, int row, int col) {
  // clang-format off
  return (mat.is_transposed) ? col + (int)mat.sy * row
                             : row + (int)mat.sy * col;
  // clang-format on
}

/**
 * @brief Converts a linear index to a Cartesian index
 *
 * This is moderately expensive, since it relies on modulus and division operations.
 *
 * Passing null pointers to @a row and @a col results in undefined behavior, since this
 * method does not check that these are valid.
 *
 * @param[in] mat Any matrix
 * @param[in] k Linear index, from 0 to slap_NumElements()
 * @param[out] row Destination for row index
 * @param[out] col Destination for column index
 */
void slap_Linear2Cart(Matrix mat, int k, int* row, int* col);

/**
 * @brief Converts a linear index to the index into the underlying array
 *
 * If the matrix is Dense (stored contiguously in memory, see slap_IsDense()), the
 * output is the same as the input. This method is most helpful for strided matrices.
 *
 * Note that for strided arrays, this method is fairly expensive since it converts
 * the linear index into a Cartesian index, and then to the array index.
 *
 * @param mat A dense or strided matrix
 * @param k The linear index, ranging from 0 to slap_NumElements()
 * @return The index into mat.data corresponding the `k`th element of @a mat
 */
static inline int slap_Linear2Index(const Matrix mat, int k) {
  int index;
  if (slap_IsDense(mat)) {
    index = k;
  } else {
    int row;
    int col;
    slap_Linear2Cart(mat, k, &row, &col);
    index = slap_Cart2Index(mat, row, col);
  }
  return index;
}

/**
 * @brief Check if the row and column index is in the bounds of the matrix
 *
 * @param mat Matrix to be indexed
 * @param row Row index
 * @param col Column index
 * @return true if the row and column index is in bounds for the matrix
 */
static inline bool slap_CheckInbounds(Matrix mat, int row, int col) {
  return (row >= 0 && row < mat.rows) && (col >= 0 && col < mat.cols);
}

/**
 * @brief Get a pointer to matrix element given row, column indices
 *
 * Note that this method does NOT perform any bounds checking so can be used unsafely!
 * Passing an index that is out of bounds is undefined behavior.
 *
 * # Example
 * The following gets a pointer to the 1st element in the 2nd column, and then
 * modifies it.
 * ```c
 * float *x = slap_GetElement(A, 0, 1);
 * *x = 10;
 * ```
 *
 * To get the data directly, just de-reference the pointer at the call site:
 * The following reads the data in the 2nd element of the 1st column.
 * ```c
 * float y = *slap_GetElement(A, 1, 0);
 * ```
 *
 * @param mat Matrix of nonzero size
 * @param row Row index
 * @param col Column index
 * @return A pointer to the element of the matrix. NULL for invalid input.
 */
static inline float* slap_GetElement(Matrix mat, int row, int col) {
  return mat.data + slap_Cart2Index(mat, row, col);
}

/**
 * @brief Get a const pointer to matrix element given row, column indices
 *
 * # Example
 * The following gets a pointer to the 1st element in the 2nd column.
 * ```c
 * const float *x = slap_GetElementConst(A, 0, 1);
 * ```
 *
 * To get the data directly, just de-reference the pointer at the call site:
 * The following reads the data in the 2nd element of the 1st column.
 * ```c
 * float y = *slap_GetElementConst(A, 1, 0);
 * ```
 *
 * @param mat Matrix of nonzero size
 * @param row Row index
 * @param col Column index
 * @return A pointer to the element of the matrix. NULL for invalid input.
 */
static inline const float* slap_GetElementConst(const Matrix mat, int row, int col) {
  return mat.data + slap_Cart2Index(mat, row, col);
}

/**
 * @brief Set an matrix element to a given value
 *
 * This is a low-level function with no error checking. As such, it should
 * be used carefully, as it can easily result in undefined behavior.
 *
 * The user should make sure the indices are within bounds and that the
 * data pointer is valid.
 *
 * # Example
 * Set the 1st element of the 2nd column to 10.1;
 * ```c
 * slap_SetElement(A, 0, 1, 10.1);
 * ```
 *
 * @param mat Matrix with nonzero size and initialized data
 * @param row Row index
 * @param col Column index
 * @param val Value to which the element should be set
 */
static inline void slap_SetElement(Matrix mat, int row, int col, float val) {
  mat.data[slap_Cart2Index(mat, row, col)] = val;
}

//*********************************************//
// Transformations
//*********************************************//
/**
 * @brief Flatten a 2D matrix to a column vector
 *
 * Changes the row and column data so that the matrix is now a column vector. The
 * underlying data is unchanged.
 *
 * See also: slap_Reshape()
 *
 * **Header File:** `src/matrix.h`
 * @param mat Matrix to be flattened.
 * @return Flattened Matrix
 */
Matrix slap_Flatten(Matrix mat);

/**
 * @brief Transpose a 2D matrix
 *
 * This operation doesn't change the data, just it's interpretation.
 *
 * See also: slap_IsTransposed()
 *
 * **Header File:** `src/matrix.h`
 * @param mat The matrix to transpose
 * @return Transposed Matrix
 */
Matrix slap_Transpose(Matrix mat);

Matrix slap_UpperTri(Matrix mat);

Matrix slap_LowerTri(Matrix mat);

/**
 * @brief Set the dimensions of the matrix
 *
 * Note that this does not change the underlying data, only it's interpretation.
 *
 * See also: slap_Flatten(), slap_CreateSubMatrix()
 *
 * **Header File:** `src/matrix.h`
 * @param mat  Matrix
 * @param rows New number of rows
 * @param cols New number of columns
 * @return Resized Matrix
 */
Matrix slap_Reshape(Matrix mat, int rows, int cols);

/**@}*/
