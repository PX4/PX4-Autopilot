//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include "iterator.h"

MatrixIterator slap_Iterator(Matrix mat) {
  MatrixIterator iterator = {
      .len = mat.rows * mat.cols,
      .rows = mat.rows,
      .dx = 1,
      .dy = mat.sy - mat.rows + 1,
      .i = 0,
      .j = 0,
      .k = 0,
      .index = 0,
  };

  return iterator;
}

void slap_Step(MatrixIterator* iterator) {
  if (iterator->i < iterator->rows - 1) {
    ++iterator->i;
    iterator->index += iterator->dx;
  } else {
    iterator->i = 0;
    iterator->j += 1;
    iterator->index += iterator->dy;
  }
  ++iterator->k;
}
