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

#include "Vector.hpp"
#include "Matrix.hpp"

namespace math
{

template<class T>
class Kalman {
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    // constructor
    Kalman(size_t n) :
        _x(VectorType::zero(n)),
        _P(MatrixType::zero(n)),
        _Q(MatrixType::zero(n)),
        _F(MatrixType::zero(n))
    {
    }
    // deconstructor
    virtual ~Kalman()
    {
    }
    void predict(float dt)
    {
        setP(getF()*getP()*getF().transpose() + getQ());
    }
    void correct(const VectorType & z,
            const MatrixType & H, 
            const MatrixType & R) 
    {
        MatrixType S = H*getP()*H.transpose() + R;
        MatrixType K = getP()*H.transpose()*S.inverse();
        setP(getP() - K*H*getP());
        setX(getX() + K*(z - H*getX()));
    }
    const VectorType & getX() const { return _x; }
    const MatrixType & getP() const { return _P; }
    const MatrixType & getQ() const { return _Q; }
    const MatrixType & getF() const { return _F; }
protected:
    void setX(const VectorType & x) { _x = x; }
    void setP(const MatrixType & P) { _P = P; }
    void setQ(const MatrixType & Q) { _Q = Q; }
    void setF(const MatrixType & F) { _F = F; }
private:
    VectorType _x;
    MatrixType _P;
    MatrixType _Q;
    MatrixType _F;
};

template<class T>
class KalmanNav : public Kalman<T>
{
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    typedef Kalman<T> KalmanType;
    KalmanNav() :
        KalmanType(12),
        _hMag(3,12),
        _rMag(MatrixType::zero(3)),
        _hGps(6,12),
        _rGps(MatrixType::zero(6))
    {
        setP(MatrixType::identity(12)*0.001f);
        setQ(MatrixType::identity(12)*0.001f);
    }
    virtual ~KalmanNav() 
    {
    }
    void predict(float dt)
    {
        setF(MatrixType::identity(12));
        setX(KalmanType::getX() + 1.0f);
        KalmanType::predict(dt);
    }
    void correctMag(VectorType & zMag)
    {
        _hMag.setAll(1);
        setRMag(MatrixType::identity(3));
        correct(zMag,getHMag(),getRMag());
    }
    void correctGps(VectorType & zGps)
    {
        _hGps.setAll(1);
        setRGps(MatrixType::identity(6));
        correct(zGps,getHGps(),getRGps());
    }
    const MatrixType & getHMag() const { return _hMag; }
    const MatrixType & getRMag() const { return _rMag; }
    const MatrixType & getHGps() const { return _hGps; }
    const MatrixType & getRGps() const { return _rGps; }
protected:
    void setHMag(const MatrixType & hMag) { _hMag = hMag; }
    void setRMag(const MatrixType & rMag) { _rMag = rMag; }
    void setHGps(const MatrixType & hGps) { _hGps = hGps; }
    void setRGps(const MatrixType & rGps) { _rGps = rGps; }
private:
    MatrixType _hMag;
    MatrixType _rMag;
    MatrixType _hGps;
    MatrixType _rGps;
};

int kalmanTest();
int kalmanNavTest();

} // namespace math
