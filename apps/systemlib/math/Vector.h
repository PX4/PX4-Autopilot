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
 * @file Vector.h
 *
 * math vector
 */

#pragma once

#include <inttypes.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace math
{

template<class T>
class Vector {
public:
    typedef Vector<T> VectorType;
    // constructor
    Vector(size_t rows) :
        _rows(rows),
        _data((T*)malloc(getSize()))
    {
    }
    Vector(size_t rows, const T * data) :
        _rows(rows),
        _data((T*)malloc(getSize()))
    {
        memcpy(getData(),data,getSize());
    }
    // deconstructor
    virtual ~Vector()
    {
        delete [] getData();
    }
    // copy constructor (deep)
    Vector(const VectorType & right) :
        _rows(right.getRows()),
        _data((T*)malloc(getSize()))
    {
        memcpy(getData(),right.getData(),
                    right.getSize());
    }
    // assignment
    VectorType & operator=(const VectorType & right)
    {
        ASSERT(getRows()==right.getRows());
        if (this != &right)
        {
            memcpy(getData(),right.getData(),
                        right.getSize());
        }
        return *this;
    }
    // element accessors
    T& operator()(size_t i)
    {
        ASSERT(i<getRows());
        return getData()[i];
    }
    T operator()(size_t i) const
    {
        ASSERT(i<getRows());
        return getData()[i];
    }
    // output
    void print() const
    {
        for (size_t i=0; i<getRows(); i++)
        {
            printf("%8.4f\t",(double)(*this)(i));
        }
        printf("\n");
    }
    // boolean ops
    bool operator==(const T & right) const
    {
        for (size_t i=0; i<getRows(); i++)
        {
            if ((*this)(i) != right(i))
                return false;
        }
        return true;
    }
    // scalar ops
    T operator+(const T & right) const
    {
        VectorType result(getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) + right;
        }
        return result;
    }
    T operator-(const T & right) const
    {
        VectorType result(getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) - right;
        }
        return result;
    }
    T operator*(const T & right) const
    {
        VectorType result(right.getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) * right;
        }
        return result;
    }
    T operator/(const T & right) const
    {
        VectorType result(right.getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) / right;
        }
        return result;
    }
    // vector ops
    VectorType operator+(const VectorType & right) const
    {
        ASSERT(getRows()==right.getRows());
        VectorType result(getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) + right(i);
        }
        return result;
    }
    VectorType operator-(const VectorType & right) const
    {
        ASSERT(getRows()==right.getRows());
        VectorType result(getRows());
        for (size_t i=0; i<getRows(); i++)
        {
            result(i) = (*this)(i) - right(i);
        }
        return result;
    }
    // other functions
    void setAll(const T & val)
    {
        memset(getData(),val,getSize());
    }
    void set(const T * data)
    {
        memcpy(getData(),data,getSize());
    }
    size_t getRows() const { return _rows; }
protected:
    size_t getSize() const { return sizeof(T)*getRows(); }
    T * getData() const { return _data; }
    T * getData() { return _data; }
    void setData(T * data) { _data = data; }
private:
    size_t _rows;
    T * _data;
};

typedef Vector<float> VectorFloat;

int __EXPORT vectorTest();
int __EXPORT vectorAddTest();
int __EXPORT vectorSubTest();

} // math
