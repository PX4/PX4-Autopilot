/**
 * @file Scalar.hpp
 *
 * Defines conversion of matrix to scalar.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "matrix.hpp"

namespace matrix
{

template<typename Type>
class Scalar : public Matrix<Type, 1, 1>
{
public:
    virtual ~Scalar() {};

    Scalar() : Matrix<Type, 1, 1>()
    {
    }

    Scalar(const Matrix<Type, 1, 1> & other)
    {
        (*this)(0,0) = other(0,0);
    }

    operator Type()
    {
        return (*this)(0,0);
    }

};

typedef Scalar<float> Scalarf;

}; // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
