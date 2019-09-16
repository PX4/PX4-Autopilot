/**
 * @file Slice.hpp
 *
 * A simple matrix template library.
 *
 * @author Julian Kent < julian@auterion.com >
 */

#pragma once

#include "math.hpp"


namespace matrix {

template<typename Type, size_t M, size_t N>
class Matrix;

template<typename Type, size_t M>
class Vector;

template <typename Type, size_t P, size_t Q, size_t M, size_t N>
class Slice {
public:
    Slice(size_t x0, size_t y0, const Matrix<Type, M, N>* data) :
        _x0(x0),
        _y0(y0),
        _data(const_cast<Matrix<Type, M, N>*>(data)) {
        static_assert(P <= M, "Slice rows bigger than backing matrix");
        static_assert(Q <= N, "Slice cols bigger than backing matrix");
    }

    Type operator()(size_t i, size_t j) const
    {
        return (*_data)(_x0 + i, _y0 + j);
    }

    Type &operator()(size_t i, size_t j)
    {
        return (*_data)(_x0 + i, _y0 + j);
    }

    template<size_t MM, size_t NN>
    Slice<Type, P, Q, M, N>& operator=(const Slice<Type, P, Q, MM, NN>& in_matrix)
    {
        Slice<Type, P, Q, M, N>& self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                self(i, j) = in_matrix(i, j);
            }
        }
        return self;
    }

    Slice<Type, P, Q, M, N>& operator=(const Matrix<Type, P, Q>& in_matrix)
    {
        Slice<Type, P, Q, M, N>& self = *this;
        for (size_t i = 0; i < P; i++) {
            for (size_t j = 0; j < Q; j++) {
                self(i, j) = in_matrix(i, j);
            }
        }
        return self;
    }

    // allow assigning vectors to a slice that are in the axis
    Slice<Type, 1, Q, M, N>& operator=(const Vector<Type, Q>& in_vector)
    {
        Slice<Type, 1, Q, M, N>& self = *this;
        for (size_t j = 0; j < Q; j++) {
            self(0, j) = in_vector(j);
        }
        return self;
    }

private:
    size_t _x0, _y0;
    Matrix<Type,M,N>* _data;
};

}
