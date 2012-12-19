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
    Kalman(size_t n, size_t m) :
        _x(VectorType::zero(n)),
        _P(MatrixType::zero(n)),
        _F(MatrixType::zero(n)),
        _G(MatrixType::zero(n,m)),
        _Q(MatrixType::zero(n)),
        _V(MatrixType::zero(m))
    {
    }
    // deconstructor
    virtual ~Kalman()
    {
    }
    virtual void predict(float dt)
    {
        _P = _F*_P*_F.transpose() + _G*_V*_G.transpose() + _Q;
    }
    virtual void correct(const VectorType & z,
            const MatrixType & H, 
            const MatrixType & R) 
    {
        MatrixType S = H*_P*H.transpose() + R;
        MatrixType K = _P*H.transpose()*S.inverse();
        _P = _P - _P*K*H;
        _x = _x + K*(z - H*_x);
    }
    const VectorType & getX() const { return _x; }
    const MatrixType & getP() const { return _P; }
    void setX(const VectorType & x) { _x = x; }
    void setP(const MatrixType & P) { _P = P; }
protected:
    void setQ(const MatrixType & Q) { _Q = Q; }
    void setV(const MatrixType & V) { _V = V; }
    VectorType _x;
    MatrixType _P;
    MatrixType _F;
    MatrixType _G;
    MatrixType _Q;
    MatrixType _V;
};

template<class T>
class KalmanNav : public Kalman<T>
{
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    typedef Kalman<T> KalmanType;
    KalmanNav() :
        KalmanType(9,6),
        HMag(3,9),
        RMag(3,3),
        HGps(6,9),
        RGps(6,6),
        Dcm(3,3)
    {
        MatrixType I3 = MatrixType::identity(3);
        MatrixType I6 = MatrixType::identity(6);
        MatrixType I9 = MatrixType::identity(9);

        setP(I9*0.001f);
        setQ(I9*0.001f);
        setV(I6*0.001f);

        // RMag is constant
        RMag = I3*0.001f;
        RGps = I6*0.001f;

        // HGps is constant
        HGps(0,3) = 1.0f;
        HGps(1,4) = 1.0f;
        HGps(2,5) = 1.0f;
        HGps(3,6) = 1.0f;
        HGps(4,7) = 1.0f;
        HGps(5,8) = 1.0f;
    }
    virtual ~KalmanNav() 
    {
    }
    void predict(float dt, const VectorType & accelB, const VectorType & gyroB)
    {
        // constants
        static const float omega = 7.2921150e-5f; // earth rotation rate, rad/s
        static const float R = 6.371000e6f; // earth radius, m
        static const float RSq = 4.0589641e13f; // radius squared
        static const float g = 9.8f; // gravitational accel. m/s^2

        // state
        VectorType & x = this->_x;
        float & phi = x(0);
        float & theta = x(1);
        float & psi = x(2);
        float & vN = x(3);
        float & vE = x(4);
        float & vD = x(5);
        float & L = x(6);
        //float & l = x(7);
        //float & h = x(8);

        // trig
        float cosPhi = cosf(phi);
        float cosTheta = cosf(theta);
        float cosPsi = cosf(psi);
        float sinPhi = sinf(phi);
        float sinTheta = sinf(theta);
        float sinPsi = sinf(psi);
        float sinL = sinf(L);
        float cosL = cosf(L);
        float cosLSq = cosL*cosL;
        float tanL = tanf(L);

        // dcm update
        Dcm(0,0) = cosTheta*cosPsi;
        Dcm(0,1) = -cosPhi*sinPsi + sinPhi*sinTheta*cosPsi;
        Dcm(0,2) = sinPhi*sinPsi + cosPhi*sinTheta*cosPsi;
        Dcm(1,0) = cosTheta*sinPsi;
        Dcm(1,1) = cosPhi*cosPsi + sinPhi*sinTheta*sinPsi;
        Dcm(1,2) = -sinPhi*cosPsi + cosPhi*sinTheta*sinPsi;
        Dcm(2,0) = -sinTheta;
        Dcm(2,1) = sinPhi*cosTheta;
        Dcm(2,2) = cosPhi*cosTheta;

        // specific acceleration in nav frame
        VectorType accelN = Dcm*accelB;
        float fN = accelN(0);
        float fE = accelN(1);
        float fD = accelN(2) - g;

        // F Matrix
        MatrixType & F = this->_F;

        F(0,1) = -(omega*sinL + vE*tanL/R);
        F(0,2) = vN/R;
        F(0,4) = 1.0f/R;
        F(0,6) = -omega*sinL;
        F(0,8) = -vE/RSq;

        F(1,0) = omega*sinL + vE*tanL/R;
        F(1,2) = omega*cosL + vE/R;
        F(1,3) = -1.0f/R;
        F(1,8) = vN/RSq;
        
        F(2,0) = -vN/R;
        F(2,1) = -omega*cosL - vE/R;
        F(2,4) = -tanL/R;
        F(2,6) = -omega*cosL - vE/(R*cosLSq);
        F(2,8) = vE*tanL/RSq;

        F(3,1) = -fD;
        F(3,2) = fE;
        F(3,3) = vD/R;
        F(3,4) = -2*(omega*sinL + vE*tanL/R);
        F(3,5) = vN/R;
        F(3,6) = -vE*(2*omega*cosL + vE/(R*cosLSq));
        F(3,8) = (vE*vE*tanL - vN*vD)/RSq;

        F(4,0) = fD;
        F(4,2) = -fN;
        F(4,3) = 2*omega*sinL + vE*tanL/R;
        F(4,4) = (vN*tanL + vD)/R;
        F(4,5) = 2*omega*cosL + vE/R;
        F(4,6) = 2*omega*(vN*cosL - vD*sinL) + 
            vN*vE/(R*cosLSq);
        F(4,8) = -vE*(vN*tanL + vD)/RSq;

        F(5,0) = -fE;
        F(5,1) = fN;
        F(5,3) = -2*vN/R;
        F(5,4) = -2*(omega*cosL + vE/R);
        F(5,6) = 2*omega*vE*sinL;
        F(5,8) = (vN*vN + vE*vE)/RSq;

        F(6,3) = 1/R;
        F(6,8) = -vN/RSq;

        F(7,4) = 1/(R*cosL);
        F(7,6) = vE*tanL/(R*cosL);
        F(7,8) = -vE/(cosL*RSq);

        F(8,5) = -1;

        // G Matrix
        MatrixType & G = this->_G;

        G(0,0) = -Dcm(0,0); 
        G(0,1) = -Dcm(0,1); 
        G(0,2) = -Dcm(0,2); 
        G(1,0) = -Dcm(1,0); 
        G(1,1) = -Dcm(1,1); 
        G(1,2) = -Dcm(1,2); 
        G(2,0) = -Dcm(2,0); 
        G(2,1) = -Dcm(2,1); 
        G(2,2) = -Dcm(2,2); 

        G(3,3) = Dcm(0,0); 
        G(3,4) = Dcm(0,1); 
        G(3,5) = Dcm(0,2); 
        G(4,3) = Dcm(1,0); 
        G(4,4) = Dcm(1,1); 
        G(4,5) = Dcm(1,2); 
        G(5,3) = Dcm(2,0); 
        G(5,4) = Dcm(2,1); 
        G(5,5) = Dcm(2,2); 

        // update x
        // TODO: need to add non-linear state prediction functions

        // predict equations for kalman filter
        KalmanType::predict(dt);
    }
    void correctMag(VectorType & zMag)
    {
        // state
        VectorType & x = this->_x;
        float & phi = x(0);
        float & theta = x(1);
        float & psi = x(2);

        // trig
        float cosPhi = cosf(phi);
        float cosTheta = cosf(theta);
        float cosPsi = cosf(psi);
        float sinPhi = sinf(phi);
        float sinTheta = sinf(theta);
        float sinPsi = sinf(psi);

        float magFieldStrength = 0.5f;

        // choosing some typical magnetic field properties,
        //  TODO dip/dec depend on lat/ lon/ time
        static const float dip = 60.0f; // dip, inclination with level
        static const float dec = 0.0f; // declination, clockwise rotation from north
        float bN = magFieldStrength*cosf(dip)*cosf(dec);
        float bE = magFieldStrength*cosf(dip)*sinf(dec);
        float bD = magFieldStrength*sinf(dip);

        // HMag
        float tmp1 =
            cosPsi*cosTheta*bN +
            sinPsi*cosTheta*bE -
            sinTheta*bD;
        HMag(0,1) = -(
            cosPsi*sinTheta*bN +
            sinPsi*sinTheta*bE +
            cosTheta*bD
            );
        HMag(0,2) = -cosTheta*(sinPsi*bN - cosPsi*bE);
        HMag(1,0) = 
            (cosPhi*cosPsi*sinTheta + sinPhi*sinPsi)*bN + 
            (cosPhi*sinPsi*sinTheta - sinPhi*cosPsi)*bE +
            cosPhi*cosTheta*bD;
        HMag(1,1) = sinPhi*tmp1;
        HMag(1,2) = -(
            (sinPhi*sinPsi*sinTheta + cosPhi*cosPsi)*bN - 
            (sinPhi*cosPsi*sinTheta - cosPhi*sinPsi)*bE
            );
        HMag(2,0) = -(
            (sinPhi*cosPsi*sinTheta - cosPhi*sinPsi)*bN +
            (sinPhi*sinPsi*sinTheta + cosPhi*cosPsi)*bE +
            (sinPhi*cosTheta)*bD
            );
        HMag(2,1) = cosPhi*tmp1;
        HMag(2,2) = -(
            (cosPhi*sinPsi*sinTheta - sinPhi*cosTheta)*bN -
            (cosPhi*cosPsi*sinTheta + sinPhi*sinPsi)*bE
            );
        correct(zMag,HMag,RMag);
    }
    void correctGps(VectorType & zGps)
    {
        correct(zGps,HGps,RGps);
    }
protected:
    MatrixType HMag;
    MatrixType RMag;
    MatrixType HGps;
    MatrixType RGps;
    MatrixType Dcm;
};

int kalmanTest();
int kalmanNavTest();

} // namespace math
