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
        result.setAll(0);
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
        result.setAll(0);
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
    MatrixType transpose() const
    {
        MatrixType result(getCols(),getRows());
        for(int i=0;i<getRows();i++) {
            for(int j=0;j<getCols();j++) {
                result(j,i) = (*this)(i,j);
            }
        }
        return result;
    }
    MatrixType inverse() const
    {
        ASSERT(getRows()==getCols());
        size_t N = getRows();
        MatrixType L = identity(N);
        const MatrixType & A = (*this);
        MatrixType U = A;

        //printf("A:\n"); A.print();

        // for all diagonal elements
        for (size_t n=0; n<N; n++) {
            // for all rows below diagonal
            for (size_t i=(n+1); i<N; i++) {
                ASSERT(fabs(U(n,n))>1e-8);
                L(i,n) = U(i,n)/U(n,n);
                // add i-th row and n-th row
                // multiplied by: -a(i,n)/a(n,n)
                for (size_t k=n; k<N; k++) {
                    U(i,k) -= L(i,n) * U(n,k);       
                    // TODO, know that nth col is zero
                    // can optimize here
                }
            }
        }

        //printf("L:\n"); L.print();
        //printf("U:\n"); U.print();

        // solve LY=I for Y by forward subst
        MatrixType Y = identity(N);
        // for all columns of Y
        for (size_t c=0; c<N; c++) {
            // for all rows of L
            for (size_t i=0; i<N; i++) {
                // for all columns of L
                for (size_t j=0; j<i; j++) {
                    // for all existing y
                    // subtract the component they 
                    // contribute to the solution
                    // TODO, is Y always lower
                    // triag? if so we can optim. 
                    Y(i,c) -= L(i,j)*Y(j,c); 
                }
                // divide by the factor 
                // on current
                // term to be solved
                // Y(i,c) /= L(i,i);
                // but L(i,i) = 1.0
            }
        }

        //printf("Y:\n"); Y.print();

        // solve Ux=y for x by back subst
        MatrixType X = Y;
        // for all columns of X
        for (size_t c=0; c<N; c++) {
            // for all rows of U
            for (size_t k=0; k<N; k++) {
                // have to go in reverse order
                size_t i = N-1-k; 
                // for all columns of U
                for (size_t j=i+1; j<N; j++) {
                    // for all existing x
                    // subtract the component they 
                    // contribute to the solution
                    X(i,c) -= U(i,j)*X(j,c); 
                }
                // divide by the factor 
                // on current
                // term to be solved
                X(i,c) /= U(i,i);
            }
        }
        //printf("X:\n"); X.print();
        return X;
    }
    void setAll(const T & val)
    {
        memset(getData(),val,getSize());
    }
    void set(const T * data)
    {
        memcpy(getData(),data,getSize());
    }
    size_t getRows() const { return _rows; }
    size_t getCols() const { return _cols; }
    static MatrixType identity(size_t size) {
        MatrixType result(size,size);
        result.setAll(0.0f);
        for (size_t i=0; i<size; i++) {
            result(i,i) = 1.0f; 
        }
        return result;
    }
    static MatrixType zero(size_t size) {
        MatrixType result(size,size);
        result.setAll(0.0f);
        return result;
    }
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
