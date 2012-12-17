/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Matrix.h
 *
 * matrix code
 */

#pragma once


#include <inttypes.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "Vector.h"

namespace math
{

template<class T>
class Matrix {
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    // constructor
    Matrix(size_t rows, size_t cols) :
        _rows(rows),
        _cols(cols),
        _data((T*)malloc(getSize()))
    {
    }
    Matrix(size_t rows, size_t cols, const T * data) :
        _rows(rows),
        _cols(cols),
        _data((T*)malloc(getSize()))
    {
        memcpy(getData(),data,getSize());
    }
    // deconstructor
    virtual ~Matrix()
    {
        delete [] getData();
    }
    // copy constructor (deep)
    Matrix(const MatrixType & right) :
        _rows(right.getRows()),
        _cols(right.getCols()),
        _data((T*)malloc(getSize()))
    {
        memcpy(getData(),right.getData(),
                    right.getSize());
    }
    // assignment
    MatrixType & operator=(const MatrixType & right)
    {
        ASSERT(getRows()==right.getRows());
        ASSERT(getCols()==right.getCols());
        if (this != &right)
        {
            memcpy(getData(),right.getData(),
                        right.getSize());
        }
        return *this;
    }
    // element accessors
    T& operator()(size_t i, size_t j)
    {
        ASSERT(i<getRows());
        ASSERT(j<getCols());
        return getData()[i*getCols() + j];
    }
    T operator()(size_t i, size_t j) const
    {
        ASSERT(i<getRows());
        ASSERT(j<getCols());
        return getData()[i*getCols() + j];
    }
    // output
    void print() const
    {
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                printf("%8.4f\t",(double)(*this)(i,j));
            }
            printf("\n");
        }
    }
    // boolean ops
    bool operator==(const T & right) const
    {
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                if ((*this)(i,j) != right(i,j))
                    return false;
            }
        }
        return true;
    }
    // scalar ops
    T operator+(const T & right) const
    {
        MatrixType result(getRows(), getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) + right;
            }
        }
        return result;
    }
    T operator-(const T & right) const
    {
        MatrixType result(getRows(), getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) - right;
            }
        }
        return result;
    }
    T operator*(const T & right) const
    {
        MatrixType result(getRows(), right.getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) * right;
            }
        }
        return result;
    }
    T operator/(const T & right) const
    {
        MatrixType result(getRows(), right.getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) / right;
            }
        }
        return result;
    }
    // vector ops
    VectorType operator*(const VectorType & right) const
    {
        ASSERT(getCols()==right.getRows());
        MatrixType result(getRows(), right.getCols());
        result.set(0);
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) += (*this)(i,j) * right(j);
            }
        }
        return result;
    }
    // matrix ops
    MatrixType operator+(const MatrixType & right) const
    {
        ASSERT(getRows()==right.getRows());
        ASSERT(getCols()==right.getCols());
        MatrixType result(getRows(), getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) + right(i,j);
            }
        }
        return result;
    }
    MatrixType operator-(const MatrixType & right) const
    {
        ASSERT(getRows()==right.getRows());
        ASSERT(getCols()==right.getCols());
        MatrixType result(getRows(), getCols());
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<getCols(); j++)
            {
                result(i,j) = (*this)(i,j) - right(i,j);
            }
        }
        return result;
    }
    MatrixType operator*(const MatrixType & right) const
    {
        ASSERT(getCols()==right.getRows());
        MatrixType result(getRows(), right.getCols());
        result.set(0);
        for (size_t i=0; i<getRows(); i++)
        {
            for (size_t j=0; j<right.getCols(); j++)
            {
                for (size_t k=0; k<right.getRows(); k++)
                {
                    result(i,j) += (*this)(i,k) * right(k,j);
                }
            }
        }
        return result;
    }
    MatrixType operator/(const MatrixType & right) const
    {
        ASSERT(right.getRows()==right.getCols());
        ASSERT(getCols()==right.getCols());
        return (*this)*right.inverse();
    }
    // other functions
    MatrixType inverse() const
    {
        ASSERT(getRows()==getCols());
        size_t n = getRows();
        MatrixType L(n,n);
        L.set(0);
        MatrixType U(n,n);
        U.set(0);
        MatrixType result(n,n);
        result.set(0);
        //for (size_t i=0; i<n; i++) {
            //for (size_t j=0; j<n; j++) {
                //U(i,j) = (*this)(i,j);
                //for (size_t k=0; k<(i-1); k++) {
                    //U(i,j) -= L(i,j)*U(k,j);
                //}
            //}
            //for (size_t j=i+1; j<n; j++) {
                //L(j,i) = (*this)(j,i);
                //for (size_t k = 0; k<(i-1); k++) {
                    //L(j,i) -= L(j,k)*U(k,i);
                //}
                //L(j,i) /= U(i,i);
            //}
        //}
        return result;
    }
    void set(const T & val)
    {
        memset(getData(),val,getSize());
    }
    void set(const T * data)
    {
        memcpy(getData(),data,getSize());
    }
    size_t getRows() const { return _rows; }
    size_t getCols() const { return _cols; }
protected:
    size_t getSize() const { return sizeof(T)*getRows()*getCols(); }
    T * getData() const { return _data; }
    T * getData() { return _data; }
    void setData(T * data) { _data = data; }
private:
    size_t _rows;
    size_t _cols;
    T * _data;
};

typedef Matrix<float> MatrixFloat;

int matrixTest();
int matrixAddTest();
int matrixSubTest();
int matrixMultTest();
int matrixDivTest();

} // namespace math
