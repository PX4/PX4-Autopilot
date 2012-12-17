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
 * @file Kalman.h
 *
 * kalman filter code
 */

#pragma once

#include "Vector.h"
#include "Matrix.h"

namespace math
{

template<class T>
class Kalman {
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    // constructor
    Kalman(size_t n) :
        _x(n),
        _P(n,n),
        _Q(n,n)
    {
    }
    // deconstructor
    virtual ~Kalman()
    {
    }
    void predict(const MatrixType & F)
    {
        getP() = F*getP()*F.transpose() + getQ();
    }
    void correct(const VectorType & z,
            const MatrixType & H, 
            const MatrixType & R) 
    {
        MatrixType S = H*getP()*H.transpose() + R;
        MatrixType K = getP()*H.transpose()*S.inverse();
        getP() -= K*H*getP();
        getX() +=  K*(z - H*getX());
    }
    VectorType getX() const { return _x; }
    MatrixType getP() const { return _P; }
    MatrixType getQ() const { return _Q; }
protected:
    VectorType getX() { return _x; }
    MatrixType getP() { return _P; }
    MatrixType getQ() { return _Q; }
private:
    Vector<T> _x;
    Matrix<T> _P;
    Matrix<T> _Q;
};

int kalmanTest();

} // namespace math
