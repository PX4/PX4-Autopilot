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

#include "math.hpp"

namespace matrix
{

template<typename Type>
class Scalar
{
public:
    virtual ~Scalar() {};

    Scalar() : _value()
    {
    }

    Scalar(const Matrix<Type, 1, 1> & other)
    {
        _value = other(0,0);
    }

    Scalar(float other)
    {
        _value = other;
    }

    operator Type()
    {
        return _value;
    }
private:
    Type _value;

};

typedef Scalar<float> Scalarf;

} // namespace matrix

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : */
