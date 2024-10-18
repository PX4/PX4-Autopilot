/**
 * @file iterator.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Functions for efficient iteration
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup Advanced
 * @{
 */

#pragma once

#include "matrix.h"

/**
 * @brief A struct for conveniently and efficiently iterating over strided matrices
 *
 * This iterator provides the user to access the elements via either the linear
 * or cartesian indices.
 *
 * Note that this will always iterate linearly over the underlying memory, so is invariant
 * to whether or not the matrix is transposed.
 *
 * Note that if the matrix data is dense (there are no gaps in the memory), it is
 * most efficient to iterate directly over the elements of the underlying array.
 *
 * # Example
 * for (MatrixIterator it = slap_Iterator(mat); !slap_IsFinished(&it); slap_Step(&it)) {
 *  float value = mat.data[it.index];    // Use `index` to directly index the array
 *  int linear_index = it.k;              // `k` is the linear index into the array
 *  slap_SetElement(mat, it.i, it.j);     // `i` and `j` are the cartesian indices
 * }
 */
typedef struct MatrixIterator {
  uint32_t len;    // TODO (brian): safeguard against integer overflow
  uint16_t rows;
  uint16_t dx;     // index delta for movement in x
  uint16_t dy;     // index delta for movement in y
  uint16_t i;      // row index
  uint16_t j;      // column index
  uint16_t k;      // linear index
  uint16_t index;  // memory index
} MatrixIterator;

/**
 * @brief Create an iterator at the beginning of the matrix
 * @param mat A valid dense or strided matrix
 * @return A MatrixIterator for the matrix
 */
MatrixIterator slap_Iterator(Matrix mat);

/**
 * @brief Progress the iterator by one index. Updates the iterator-in-place.
 * @param iterator A valid iterator
 */
void slap_Step(MatrixIterator *iterator);

/**
 * @brief Checks if the iterator is past the end of the matrix
 * @param iterator A valid iterator
 * @return true if the iterator refers to a point past the end of the matrix
 */
static inline bool slap_IsFinished(const MatrixIterator *iterator) {
  return iterator->k >= iterator->len;
}
