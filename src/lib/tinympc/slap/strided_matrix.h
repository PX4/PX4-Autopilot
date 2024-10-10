//
// Created by Brian Jackson on 12/18/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include "matrix.h"

Matrix slap_CreateSubMatrix(Matrix mat, int top_left_row, int top_left_col, int new_rows,
                            int new_cols);
