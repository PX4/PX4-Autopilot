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
        _x(n),
        _P(n,n),
        _F(n,n),
        _Q(n,n)
    {
    }
    // deconstructor
    virtual ~Kalman()
    {
    }
    virtual void predict(float dt)
    {
        _P = _F*_P*_F.transpose() + _Q;
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
    VectorType _x;
    MatrixType _P;
    MatrixType _F;
    MatrixType _Q;
};

template<class T>
class KalmanNav : public Kalman<T>
{
public:
    typedef Matrix<T> MatrixType;
    typedef Vector<T> VectorType;
    typedef Kalman<T> KalmanType;
    KalmanNav() :
        KalmanType(9),
        G(9,6),
        V(6,6),
        HMag(3,9),
        RMag(3,3),
        HGps(6,9),
        RGps(6,6),
        Dcm(3,3)
    {
        MatrixType I3 = MatrixType::identity(3);
        MatrixType I6 = MatrixType::identity(6);
        MatrixType I9 = MatrixType::identity(9);

        VectorType & x = this->_x;
        x.setAll(1.0f);
        x.print();

        // initial state covariance matrix
        setP(I9*1.0f);

        // noise
        V = I6*1.0f;      // accel/ gyro
        RMag = I3*0.001f;   // magnetometer
        RGps = I6*0.001f;   // gps

        // Initialize F to identity
        this->_F = I9;

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

        // difference from Jacobian
        // multiplity by dt for all elements
        // add 1.0 to diagonal elements
        
        F(0,1) = (-(omega*sinL + vE*tanL/R))*dt;
        F(0,2) = (vN/R)*dt;
        F(0,4) = (1.0f/R)*dt;
        F(0,6) = (-omega*sinL)*dt;
        F(0,8) = (-vE/RSq)*dt;

        F(1,0) = (omega*sinL + vE*tanL/R)*dt;
        F(1,2) = (omega*cosL + vE/R)*dt;
        F(1,3) = (-1.0f/R)*dt;
        F(1,8) = (vN/RSq)*dt;
        
        F(2,0) = (-vN/R)*dt;
        F(2,1) = (-omega*cosL - vE/R)*dt;
        F(2,4) = (-tanL/R)*dt;
        F(2,6) = (-omega*cosL - vE/(R*cosLSq))*dt;
        F(2,8) = (vE*tanL/RSq)*dt;

        F(3,1) = (-fD)*dt;
        F(3,2) = (fE)*dt;
        F(3,3) = 1.0f + (vD/R)*dt; // on diagonal
        F(3,4) = (-2*(omega*sinL + vE*tanL/R))*dt;
        F(3,5) = (vN/R)*dt;
        F(3,6) = (-vE*(2*omega*cosL + vE/(R*cosLSq)))*dt;
        F(3,8) = ((vE*vE*tanL - vN*vD)/RSq)*dt;

        F(4,0) = (fD)*dt;
        F(4,2) = (-fN)*dt;
        F(4,3) = (2*omega*sinL + vE*tanL/R)*dt;
        F(4,4) = 1.0f + ((vN*tanL + vD)/R)*dt; // on diagonal
        F(4,5) = (2*omega*cosL + vE/R)*dt;
        F(4,6) = (2*omega*(vN*cosL - vD*sinL) + 
            vN*vE/(R*cosLSq))*dt;
        F(4,8) = (-vE*(vN*tanL + vD)/RSq)*dt;

        F(5,0) = (-fE)*dt;
        F(5,1) = (fN)*dt;
        F(5,3) = (-2*vN/R)*dt;
        F(5,4) = (-2*(omega*cosL + vE/R))*dt;
        F(5,6) = (2*omega*vE*sinL)*dt;
        F(5,8) = ((vN*vN + vE*vE)/RSq)*dt;

        F(6,3) = (1/R)*dt;
        F(6,8) = (-vN/RSq)*dt;

        F(7,4) = (1/(R*cosL))*dt;
        F(7,6) = (vE*tanL/(R*cosL))*dt;
        F(7,8) = (-vE/(cosL*RSq))*dt;

        F(8,5) = (-1)*dt;

        // G Matrix
        G(0,0) = -Dcm(0,0)*dt; 
        G(0,1) = -Dcm(0,1)*dt; 
        G(0,2) = -Dcm(0,2)*dt; 
        G(1,0) = -Dcm(1,0)*dt; 
        G(1,1) = -Dcm(1,1)*dt; 
        G(1,2) = -Dcm(1,2)*dt; 
        G(2,0) = -Dcm(2,0)*dt; 
        G(2,1) = -Dcm(2,1)*dt; 
        G(2,2) = -Dcm(2,2)*dt; 

        G(3,3) = Dcm(0,0)*dt; 
        G(3,4) = Dcm(0,1)*dt; 
        G(3,5) = Dcm(0,2)*dt; 
        G(4,3) = Dcm(1,0)*dt; 
        G(4,4) = Dcm(1,1)*dt; 
        G(4,5) = Dcm(1,2)*dt; 
        G(5,3) = Dcm(2,0)*dt; 
        G(5,4) = Dcm(2,1)*dt; 
        G(5,5) = Dcm(2,2)*dt; 

        MatrixType & Q = this->_Q;
        Q = G*V*G.transpose();

        // update x
        // TODO: should use non-linear state prediction functions
        // instead of using linearization
        VectorType u(6);
        u(0) = gyroB(0);
        u(1) = gyroB(1);
        u(2) = gyroB(2);
        u(3) = accelB(0);
        u(4) = accelB(1);
        u(5) = accelB(2);
        //x = F*x + G*u;

        // predict equations for kalman filter
        //KalmanType::predict(dt);
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

        // choosing some typical magnetic field properties,
        //  TODO dip/dec depend on lat/ lon/ time
        static const float magFieldStrength = 0.5f;
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
    MatrixType G;
    MatrixType V;
    MatrixType HMag;
    MatrixType RMag;
    MatrixType HGps;
    MatrixType RGps;
    MatrixType Dcm;
};

int kalmanTest();
int kalmanNavTest();

} // namespace math
