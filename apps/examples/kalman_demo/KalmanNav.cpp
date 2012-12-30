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
 * @file KalmanNav.cpp
 *
 * kalman filter navigation code
 */

#include "KalmanNav.hpp"

// constants
static const float omega = 7.2921150e-5f; // earth rotation rate, rad/s
static const float R = 6.371000e6f; // earth radius, m
static const float RSq = 4.0589641e13f; // radius squared
static const float g = 9.8f; // gravitational accel. m/s^2

KalmanNav::KalmanNav(SuperBlock * parent, const char * name) :
    SuperBlock(parent,name),
    _kalman(9),
    G(9,6),
    V(6,6),
    HAtt(6,9),
    RAtt(6,6),
    HGps(6,9),
    RGps(6,6),
    C_nb(),
    q(),
    _sensors(&getSubscriptions(), ORB_ID(sensor_combined),20),
    _gps(&getSubscriptions(), ORB_ID(vehicle_gps_position),20),
    _pos(&getPublications(), ORB_ID(vehicle_global_position)),
    _att(&getPublications(), ORB_ID(vehicle_attitude)),
    _pubTimeStamp(hrt_absolute_time()),
    _slowTimeStamp(hrt_absolute_time()),
    _gpsTimeStamp(hrt_absolute_time()),
    _attTimeStamp(hrt_absolute_time()),
    _outTimeStamp(hrt_absolute_time()),
    _navFrames(0),
    fN(0), fE(0), fD(0),
    x(_kalman.getX()),
    phi(x(0)), theta(x(1)), psi(x(2)),
    vN(x(3)), vE(x(4)), vD(x(5)),
    L(x(6)), l(x(7)), h(x(8)),
    a(q(0)), b(q(1)), c(q(2)), d(q(3)),
    _vGyro(this,"V_GYRO"),
    _vAccel(this,"V_ACCEL"),
    _rMag(this,"R_MAG"),
    _rGpsV(this,"R_GPS_V"),
    _rGpsGeo(this,"R_GPS_GEO"),
    _rGpsAlt(this,"R_GPS_ALT"),
    _rAccel(this,"R_ACCEL")
{
    using namespace math;
    setDt(1.0f /200.0f);

    // initial state covariance matrix
    _kalman.setP(Matrix::identity(9)*1.0f);

    // update subscriptions
    while(1)
    {
        updateSubscriptions();
        if (_gps.fix_type>2) break;
        printf("[kalman_demo] waiting for gps lock\n");
        usleep(1000000);
    }

    // initial state
    phi = 0.0f;
    theta = 0.0f;
    psi = 0.0f;
    vN = _gps.vel_n;
    vE = _gps.vel_e;
    vD = _gps.vel_d;
    L = _gps.lat/1.0e7f;
    l = _gps.lon/1.0e7f;
    h = _gps.alt/1.0e3f; 

    // initialize quaternions
    q = Quaternion(EulerAngles(phi,theta,psi)); 

    updateParams();

    // Initialize F to identity
    _kalman.getF() = Matrix::identity(9);

    // HGps is constant
    HGps(0,3) = 1.0f;
    HGps(1,4) = 1.0f;
    HGps(2,5) = 1.0f;
    HGps(3,6) = 1.0f;
    HGps(4,7) = 1.0f;
    HGps(5,8) = 1.0f;
}

void KalmanNav::update()
{
    using namespace math;

    uint64_t newTimeStamp = hrt_absolute_time();

    // get new information from subscriptions
    updateSubscriptions();
    _navFrames += 1;

    // fast prediciton step
    predictFast(); // 200 Hz

    // slow prediction step
    if (newTimeStamp - _slowTimeStamp > 1e6/200) // 200 Hz
    {
        _slowTimeStamp = newTimeStamp;
        predictSlow();
    }

    // gps correction step
    if (newTimeStamp - _gpsTimeStamp > 1e6/10) // 10 Hz
    {
        _gpsTimeStamp = newTimeStamp;
        correctGps();
    }

    // attitude correction step
    if (newTimeStamp - _attTimeStamp > 1e6/1) // 1 Hz
    {
        _attTimeStamp = newTimeStamp;
        correctAtt();
    }

    // publication
    if (newTimeStamp - _pubTimeStamp > 1e6/50) // 50 Hz
    {
        _pubTimeStamp = newTimeStamp;
        updatePublications();
    }

    // output
    if (newTimeStamp - _outTimeStamp > 1e6) // 1 Hz
    {
        _outTimeStamp = newTimeStamp;
        uint16_t schedRate = 1.0f/getDt();
        if (fabsf(float(_navFrames)/schedRate - 1.0f) > 0.01f)
        {
            printf("WARNING: nav: sched %4d Hz, actual %4d Hz\n",
                    schedRate, _navFrames);
        }
        updateParams();
        //_kalman.getX().print();
        _navFrames = 0;
    }

    // monitor speed
    float calcTime = (hrt_absolute_time() - newTimeStamp)/1.0e6f;

    // sleep for approximately the right amount of time
    float timeSleep = 0.95f*(getDt() - calcTime);
    if (timeSleep > 0.0f) {
        usleep((double)(1e6f*timeSleep));
    } 
    else if (timeSleep < -0.001f) {
        printf("kalman_demo: missed deadline by %8.4f sec\n", (double)(-timeSleep));
    }
}

void KalmanNav::updatePublications()
{
    using namespace math;

    // position publication
    _pos.timestamp = _pubTimeStamp;
    _pos.time_gps_usec = _gps.timestamp;
    _pos.valid = true;
    _pos.lat = L*1.0e7f;
    _pos.lon = l*1.0e7f;
    _pos.alt = h;
    _pos.relative_alt = h; // TODO, make relative
    _pos.vx = vN;
    _pos.vy = vE;
    _pos.vz = vD;
    _pos.hdg = psi;

    // attitude publication
    _att.timestamp = _pubTimeStamp;
    _att.roll = phi;
    _att.pitch = theta;
    _att.yaw = psi;
    _att.rollspeed = _sensors.gyro_rad_s[0];
    _att.pitchspeed = _sensors.gyro_rad_s[1];
    _att.yawspeed = _sensors.gyro_rad_s[2];
    // TODO, add gyro offsets to filter
    _att.rate_offsets[0] = 0.0f;
    _att.rate_offsets[1] = 0.0f;
    _att.rate_offsets[2] = 0.0f;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++)
        _att.R[i][j] = C_nb(i,j);
    for (int i=0;i<4;i++) _att.q[i] = q(i);
    _att.R_valid = true;
    _att.q_valid = true;
    _att.counter = _navFrames;

    // update publications
    SuperBlock::updatePublications();
}

void KalmanNav::predictFast()
{
    using namespace math;
    Vector3 w;
    float dt = getDt(); // TODO, could make this actual dt, 
    // instead of scheduled dt

    for (int i=0;i<3;i++) w(i)  = _sensors.gyro_rad_s[i];

    // attitude
    q = q + q.derivative(w)*getDt();
    
    // renormalize quaternion if needed
    if (fabsf(q.norm()-1.0f) > 1e-4f)
    {
        q = q.unit();
    }

    // C_nb update
    C_nb = Dcm(q);

    // euler update
    EulerAngles euler(C_nb);
    phi = euler.getPhi();
    theta = euler.getTheta();
    psi = euler.getPsi();

   // specific acceleration in nav frame
    Vector accelB(3,_sensors.accelerometer_m_s2);
    Vector accelN = C_nb*accelB;
    fN = accelN(0);
    fE = accelN(1);
    fD = accelN(2);

    // trig
    float sinL = sinf(L);
    float cosL = cosf(L);

    // position update
    // neglects angular deflections in local gravity
    // see Titerton pg. 70
    float LDot = vN/(R+h);
    float lDot = vE/(cosL*(R+h));
    float vNDot = fN - vE*(2*omega +
        lDot)*sinL +
        vD*LDot;
    float vDDot = fD - vE*(2*omega + lDot)*cosL -
        vN*LDot + g;
    float vEDot = fE + vN*(2*omega + lDot)*sinL + 
        vDDot*(2*omega*cosL);

    // rectangular integration
    vN += vNDot*dt;
    vE += vEDot*dt;
    vD += vDDot*dt;
    L  += LDot*dt;
    l  += lDot*dt;
    h  += -vD*dt;
}

void KalmanNav::predictSlow()
{
    using namespace math;
    // state
    float dt = getDt();

    // trig
    float sinL = sinf(L);
    float cosL = cosf(L);
    float cosLSq = cosL*cosL;
    float tanL = tanf(L);

    // F Matrix
    // Titterton pg. 291
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
    // Titterton pg. 291
    G(0,0) = -C_nb(0,0)*dt; 
    G(0,1) = -C_nb(0,1)*dt; 
    G(0,2) = -C_nb(0,2)*dt; 
    G(1,0) = -C_nb(1,0)*dt; 
    G(1,1) = -C_nb(1,1)*dt; 
    G(1,2) = -C_nb(1,2)*dt; 
    G(2,0) = -C_nb(2,0)*dt; 
    G(2,1) = -C_nb(2,1)*dt; 
    G(2,2) = -C_nb(2,2)*dt; 

    G(3,3) = C_nb(0,0)*dt; 
    G(3,4) = C_nb(0,1)*dt; 
    G(3,5) = C_nb(0,2)*dt; 
    G(4,3) = C_nb(1,0)*dt; 
    G(4,4) = C_nb(1,1)*dt; 
    G(4,5) = C_nb(1,2)*dt; 
    G(5,3) = C_nb(2,0)*dt; 
    G(5,4) = C_nb(2,1)*dt; 
    G(5,5) = C_nb(2,2)*dt; 

    Matrix & Q = _kalman.getQ();
    Q = G*V*G.transpose();

    // predict equations for kalman filter
    _kalman.predict(dt);
}

void KalmanNav::correctAtt()
{
    using namespace math;

    // trig
    float cosPhi = cosf(phi);
    float cosTheta = cosf(theta);
    float cosPsi = cosf(psi);
    float sinPhi = sinf(phi);
    float sinTheta = sinf(theta);
    float sinPsi = sinf(psi);

    // mag measurement
    Vector zMag(3);
    for (int i=0;i<3;i++) {
        zMag(i) = _sensors.magnetometer_ga[i];
    }
    zMag = zMag.unit();

    // mag predicted measurement
    // choosing some typical magnetic field properties,
    //  TODO dip/dec depend on lat/ lon/ time
    static const float dip = 60.0f; // dip, inclination with level
    static const float dec = 0.0f; // declination, clockwise rotation from north
    float bN = cosf(dip)*cosf(dec);
    float bE = cosf(dip)*sinf(dec);
    float bD = sinf(dip);
    Vector bNav(3);
    bNav(0) = bN;
    bNav(1) = bE;
    bNav(2) = bD;
    Vector zMagHat = (C_nb.transpose()*bNav).unit();

    // accel measurement
    Vector zAccel(3);
    for (int i=0;i<3;i++) {
        zAccel(i) = _sensors.accelerometer_m_s2[i];
    }
    zAccel = zAccel.unit();

    // accel predicted measurement
    Vector zAccelHat = (C_nb.transpose()*Vector3(0,0,-1)).unit();

    // combined measurement
    Vector zAtt(6);
    Vector zAttHat(6);
    for (int i=0;i<3;i++) {
        zAtt(i) = zMag(i);
        zAtt(i+3) = zAccel(i);
        zAttHat(i) = zMagHat(i);
        zAttHat(i+3) = zAccelHat(i);
    }

    // HMag , HAtt (0-2,:)
    float tmp1 =
        cosPsi*cosTheta*bN +
        sinPsi*cosTheta*bE -
        sinTheta*bD;
    HAtt(0,1) = -(
        cosPsi*sinTheta*bN +
        sinPsi*sinTheta*bE +
        cosTheta*bD
        );
    HAtt(0,2) = -cosTheta*(sinPsi*bN - cosPsi*bE);
    HAtt(1,0) = 
        (cosPhi*cosPsi*sinTheta + sinPhi*sinPsi)*bN + 
        (cosPhi*sinPsi*sinTheta - sinPhi*cosPsi)*bE +
        cosPhi*cosTheta*bD;
    HAtt(1,1) = sinPhi*tmp1;
    HAtt(1,2) = -(
        (sinPhi*sinPsi*sinTheta + cosPhi*cosPsi)*bN - 
        (sinPhi*cosPsi*sinTheta - cosPhi*sinPsi)*bE
        );
    HAtt(2,0) = -(
        (sinPhi*cosPsi*sinTheta - cosPhi*sinPsi)*bN +
        (sinPhi*sinPsi*sinTheta + cosPhi*cosPsi)*bE +
        (sinPhi*cosTheta)*bD
        );
    HAtt(2,1) = cosPhi*tmp1;
    HAtt(2,2) = -(
        (cosPhi*sinPsi*sinTheta - sinPhi*cosTheta)*bN -
        (cosPhi*cosPsi*sinTheta + sinPhi*sinPsi)*bE
        );

    // HAccel , HAtt (3-5,:)
    HAtt(3,1) = cosTheta;
    HAtt(4,0) = -cosPhi*cosTheta;
    HAtt(4,1) = sinPhi*sinTheta;
    HAtt(5,0) = sinPhi*cosTheta;
    HAtt(5,1) = cosPhi*sinTheta;

    // Kalman correction
    Vector y = zAtt - zAttHat; // residual
    Matrix S(6,6); // residual covariance
    _kalman.correct(y,HAtt,RAtt,S);

    // fault in attitude
    float beta = y.dot(S.inverse()*y);
    if (beta > 10.0f) {
        printf("fault in attitude: beta = %8.4f\n", (double)beta);
        printf("y:\n"); y.print();
        printf("zAtt:\n"); zAtt.print();
        printf("zAttHat:\n"); zAttHat.print();
    }

    // update quaternions from euler 
    // angle correction
    q = Quaternion(EulerAngles(phi,theta,psi));
}

void KalmanNav::correctGps()
{
    using namespace math;
    Vector zGps(6);
    // force state to gps values for testing
    vN = _gps.vel_n;
    vE = _gps.vel_e;
    vD = _gps.vel_d;
    L = _gps.lat/1.0e7f;
    l = _gps.lon/1.0e7f;
    h = _gps.alt/1.0e3f;
    // set measurement vector
    zGps(0) = _gps.vel_n; // vn
    zGps(1) = _gps.vel_e; // ve
    zGps(2) = _gps.vel_d; // vd
    zGps(3) = _gps.lat/1.0e7f; // L
    zGps(4) = _gps.lon/1.0e7f; // l
    zGps(5) = _gps.alt/1.0e3f; // h
    Vector zGpsHat = HGps*_kalman.getX();
    Vector y = zGps - zGpsHat; // residual
    Matrix S(6,6); // residual covariance
    _kalman.correct(y,HGps,RGps,S);

    // fault detetcion
    float beta = y.dot(S.inverse()*y);
    if (beta > 100.0f) {
        printf("fault in gps: beta = %8.4f\n", (double)beta);
        printf("y:\n"); y.print();
        printf("zGps:\n"); zGps.print();
        printf("zGpsHat:\n"); zGpsHat.print();

    }
}

void KalmanNav::updateParams()
{
    using namespace math;
    using namespace control;
    SuperBlock::updateParams();

    // gyro noise
    V(0,0) = _vGyro.get();    // gyro x, rad/s
    V(1,1) = _vGyro.get();    // gyro y
    V(2,2) = _vGyro.get();    // gyro z

    // accel noise
    V(3,3) = _vAccel.get();    // accel x, m/s^2
    V(4,4) = _vAccel.get();    // accel y
    V(5,5) = _vAccel.get();    // accel z

    // magnetometer noise
    RAtt(0,0) = _rMag.get(); // normalized direction
    RAtt(1,1) = _rMag.get();
    RAtt(2,2) = _rMag.get();

    // accelerometer noise
    RAtt(3,3) = _rAccel.get(); // normalized direction
    RAtt(4,4) = _rAccel.get();
    RAtt(5,5) = _rAccel.get();

    // gps noise
    RGps(0,0) = _rGpsV.get(); // vn, m/s
    RGps(1,1) = _rGpsV.get(); // ve
    RGps(2,2) = _rGpsV.get(); // vd
    RGps(3,3) = _rGpsGeo.get(); // L, rad
    RGps(4,4) = _rGpsGeo.get(); // l, rad
    RGps(5,5) = _rGpsAlt.get(); // h, m
}
