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
 * @file KalmanNav.hpp
 *
 * kalman filter navigation code
 */

#pragma once

#include <nuttx/config.h>

#include <systemlib/math/Vector.hpp>
#include <systemlib/math/Matrix.hpp>
#include <systemlib/math/Kalman.hpp>
#include <systemlib/control/blocks.hpp>
#include <systemlib/control/block/UOrbSubscription.hpp>
#include <systemlib/control/block/UOrbPublication.hpp>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_combined.h>

#include <drivers/drv_hrt.h>
#include <poll.h>
#include <unistd.h>

class KalmanNav : public control::SuperBlock
{
public:
    KalmanNav(SuperBlock * parent, const char * name) :
        SuperBlock(parent,name),
        _kalman(9),
        G(9,6),
        V(6,6),
        HMag(3,9),
        RMag(3,3),
        HGps(6,9),
        RGps(6,6),
        Dcm(3,3),
        _sensors(&getSubscriptions(), ORB_ID(sensor_combined),20),
        _pos(&getPublications(), ORB_ID(vehicle_global_position)),
        _att(&getPublications(), ORB_ID(vehicle_attitude)),
        _timeStamp(hrt_absolute_time()),
        _gpsTimeStamp(hrt_absolute_time()),
        _magTimeStamp(hrt_absolute_time()),
        _outTimeStamp(hrt_absolute_time()),
        _navFrames(0)
    {
        using namespace math;
        setDt(1.0f /200.0f);

        Matrix I3 = Matrix::identity(3);
        Matrix I6 = Matrix::identity(6);
        Matrix I9 = Matrix::identity(9);

        // initial state covariance matrix
        _kalman.setP(I9*1.0f);

        // noise
        V(0,0) = 0.01f;    // gyro x, rad/s
        V(1,1) = 0.01f;    // gyro y
        V(2,2) = 0.01f;    // gyro z
        V(3,3) = 0.01f;    // accel x, m/s^2
        V(4,4) = 0.01f;    // accel y
        V(5,5) = 0.01f;    // accel z

        // magnetometer noise
        RMag = I3*0.01f;   // gauss

        // gps noise
        RGps(0,0) = 0.1f;        // vn, m/s
        RGps(1,1) = 0.1f;        // ve
        RGps(2,2) = 0.1f;        // vd
        RGps(3,3) = 0.000001f;   // L, rad
        RGps(4,4) = 0.000001f;   // l, rad
        RGps(5,5) = 10.0f;       // h, m

        // Initialize F to identity
        _kalman.getF() = I9;

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
    void update()
    {
        // get new information from subscriptions
        updateSubscriptions();
        _navFrames += 1;

        uint64_t newTimeStamp = hrt_absolute_time();
        if (newTimeStamp - _gpsTimeStamp > 1e6/2) // 2 Hz
        {
            correctGps();
            _gpsTimeStamp = newTimeStamp;
        }
        if (newTimeStamp - _magTimeStamp > 1e6/5) // 5 Hz
        {
            correctMag();
        }
        predict();

        _timeStamp = hrt_absolute_time();
        float dtActual = (_timeStamp - newTimeStamp)/1.0e6f;

        if (newTimeStamp - _outTimeStamp > 1e6) // 1 Hz
        {
            printf("nav: sched %4d Hz, actual %4d Hz\n",
                    (int)(1/getDt()),_navFrames);
            _kalman.getX().print();
            _outTimeStamp = newTimeStamp;
            _navFrames = 0;
        }

        // sleep for approximately the right amount of time
        float timeSleep = getDt() - dtActual;
        if (timeSleep > 0.0f) {
            usleep((double)(1e6f*timeSleep));
        } 
        else if (timeSleep < -0.001f) {
            printf("kalman_demo: missed deadline by %8.4f sec\n", (double)(-timeSleep));
        }
    }
    void predict()
    {
        using namespace math;
        using namespace math;
        Vector gyroB(3);
        Vector accelB(3);
        for (int i=0;i<3;i++)
        {
            gyroB(i)  = _sensors.gyro_rad_s[i];
            accelB(i) = _sensors.accelerometer_m_s2[i];
        }

        // constants
        static const float omega = 7.2921150e-5f; // earth rotation rate, rad/s
        static const float R = 6.371000e6f; // earth radius, m
        static const float RSq = 4.0589641e13f; // radius squared
        static const float g = 9.8f; // gravitational accel. m/s^2

        // state
        float dt = getDt();
        Vector & x = _kalman.getX();
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
        Vector accelN = Dcm*accelB;
        float fN = accelN(0);
        float fE = accelN(1);
        float fD = accelN(2) - g;

        // F Matrix
        Matrix & F = _kalman.getF();

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

        Matrix & Q = _kalman.getQ();
        Q = G*V*G.transpose();

        // update x
        // TODO: should use non-linear state prediction functions
        // instead of using linearization
        Vector u(6);
        u(0) = gyroB(0);
        u(1) = gyroB(1);
        u(2) = gyroB(2);
        u(3) = accelB(0);
        u(4) = accelB(1);
        u(5) = accelB(2);
        x = F*x + G*u;

        // predict equations for kalman filter
        _kalman.predict(dt);
    }
    void correctMag()
    {
        using namespace math;
        Vector zMag(3);
        for (int i=0;i<3;i++) {
            zMag(i) = _sensors.magnetometer_raw[i];
        }
        // state
        Vector & x = _kalman.getX();
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
        _kalman.correct(zMag,HMag,RMag);
    }
    void correctGps()
    {
        using namespace math;
        Vector zGps(6);
        zGps(0) = _pos.vx; // vn
        zGps(1) = _pos.vy; // ve
        zGps(2) = _pos.vy; // vd
        zGps(3) = _pos.lat; // L
        zGps(4) = _pos.lon; // l
        zGps(5) = _pos.alt; // h
        _kalman.correct(zGps,HGps,RGps);
    }
protected:
    math::Kalman _kalman;
    math::Matrix G;
    math::Matrix V;
    math::Matrix HMag;
    math::Matrix RMag;
    math::Matrix HGps;
    math::Matrix RGps;
    math::Matrix Dcm;
    control::UOrbSubscription<sensor_combined_s> _sensors;
    control::UOrbPublication<vehicle_global_position_s> _pos;
    control::UOrbPublication<vehicle_attitude_s> _att;
    uint64_t _timeStamp;
    uint64_t _gpsTimeStamp;
    uint64_t _magTimeStamp;
    uint64_t _outTimeStamp;
    uint16_t _navFrames;
};
