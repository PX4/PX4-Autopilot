#include "estimator_21states.h"

#include <string.h>

AttPosEKF::AttPosEKF() :
    fusionModeGPS(0),
    covSkipCount(0),
    EAS2TAS(1.0f),
    statesInitialised(false),
    fuseVelData(false),
    fusePosData(false),
    fuseHgtData(false),
    fuseMagData(false),
    fuseVtasData(false),
    onGround(true),
    staticMode(true),
    useAirspeed(true),
    useCompass(true),
    numericalProtection(true),
    storeIndex(0),
    magDeclination(0.0f)
{
    InitialiseParameters();
}

AttPosEKF::~AttPosEKF()
{
}

void AttPosEKF::UpdateStrapdownEquationsNED()
{
    Vector3f delVelNav;
    float q00;
    float q11;
    float q22;
    float q33;
    float q01;
    float q02;
    float q03;
    float q12;
    float q13;
    float q23;
    Mat3f Tbn;
    Mat3f Tnb;
    float rotationMag;
    float qUpdated[4];
    float quatMag;
    double deltaQuat[4];
    const Vector3f gravityNED = {0.0,0.0,GRAVITY_MSS};

// Remove sensor bias errors
    correctedDelAng.x = dAngIMU.x - states[10];
    correctedDelAng.y = dAngIMU.y - states[11];
    correctedDelAng.z = dAngIMU.z - states[12];
    dVelIMU.x = dVelIMU.x;
    dVelIMU.y = dVelIMU.y;
    dVelIMU.z = dVelIMU.z;

// Save current measurements
    Vector3f  prevDelAng = correctedDelAng;

// Apply corrections for earths rotation rate and coning errors
// * and + operators have been overloaded
    correctedDelAng   = correctedDelAng - Tnb*earthRateNED*dtIMU + 8.333333333333333e-2f*(prevDelAng % correctedDelAng);
// Convert the rotation vector to its equivalent quaternion
    rotationMag = correctedDelAng.length();
    if (rotationMag < 1e-12f)
    {
        deltaQuat[0] = 1.0;
        deltaQuat[1] = 0.0;
        deltaQuat[2] = 0.0;
        deltaQuat[3] = 0.0;
    }
    else
    {
        deltaQuat[0] = cos(0.5f*rotationMag);
        double rotScaler = (sin(0.5f*rotationMag))/rotationMag;
        deltaQuat[1] = correctedDelAng.x*rotScaler;
        deltaQuat[2] = correctedDelAng.y*rotScaler;
        deltaQuat[3] = correctedDelAng.z*rotScaler;
    }

// Update the quaternions by rotating from the previous attitude through
// the delta angle rotation quaternion
    qUpdated[0] = states[0]*deltaQuat[0] - states[1]*deltaQuat[1] - states[2]*deltaQuat[2] - states[3]*deltaQuat[3];
    qUpdated[1] = states[0]*deltaQuat[1] + states[1]*deltaQuat[0] + states[2]*deltaQuat[3] - states[3]*deltaQuat[2];
    qUpdated[2] = states[0]*deltaQuat[2] + states[2]*deltaQuat[0] + states[3]*deltaQuat[1] - states[1]*deltaQuat[3];
    qUpdated[3] = states[0]*deltaQuat[3] + states[3]*deltaQuat[0] + states[1]*deltaQuat[2] - states[2]*deltaQuat[1];

// Normalise the quaternions and update the quaternion states
    quatMag = sqrtf(sq(qUpdated[0]) + sq(qUpdated[1]) + sq(qUpdated[2]) + sq(qUpdated[3]));
    if (quatMag > 1e-16f)
    {
        float quatMagInv = 1.0f/quatMag;
        states[0] = quatMagInv*qUpdated[0];
        states[1] = quatMagInv*qUpdated[1];
        states[2] = quatMagInv*qUpdated[2];
        states[3] = quatMagInv*qUpdated[3];
    }

// Calculate the body to nav cosine matrix
    q00 = sq(states[0]);
    q11 = sq(states[1]);
    q22 = sq(states[2]);
    q33 = sq(states[3]);
    q01 =  states[0]*states[1];
    q02 =  states[0]*states[2];
    q03 =  states[0]*states[3];
    q12 =  states[1]*states[2];
    q13 =  states[1]*states[3];
    q23 =  states[2]*states[3];

    Tbn.x.x = q00 + q11 - q22 - q33;
    Tbn.y.y = q00 - q11 + q22 - q33;
    Tbn.z.z = q00 - q11 - q22 + q33;
    Tbn.x.y = 2*(q12 - q03);
    Tbn.x.z = 2*(q13 + q02);
    Tbn.y.x = 2*(q12 + q03);
    Tbn.y.z = 2*(q23 - q01);
    Tbn.z.x = 2*(q13 - q02);
    Tbn.z.y = 2*(q23 + q01);

    Tnb = Tbn.transpose();

// transform body delta velocities to delta velocities in the nav frame
// * and + operators have been overloaded
    //delVelNav = Tbn*dVelIMU + gravityNED*dtIMU;
    delVelNav.x = Tbn.x.x*dVelIMU.x + Tbn.x.y*dVelIMU.y + Tbn.x.z*dVelIMU.z + gravityNED.x*dtIMU;
    delVelNav.y = Tbn.y.x*dVelIMU.x + Tbn.y.y*dVelIMU.y + Tbn.y.z*dVelIMU.z + gravityNED.y*dtIMU;
    delVelNav.z = Tbn.z.x*dVelIMU.x + Tbn.z.y*dVelIMU.y + Tbn.z.z*dVelIMU.z + gravityNED.z*dtIMU;

// calculate the magnitude of the nav acceleration (required for GPS
// variance estimation)
    accNavMag = delVelNav.length()/dtIMU;

// If calculating position save previous velocity
    float lastVelocity[3];
    lastVelocity[0] = states[4];
    lastVelocity[1] = states[5];
    lastVelocity[2] = states[6];

// Sum delta velocities to get velocity
    states[4] = states[4] + delVelNav.x;
    states[5] = states[5] + delVelNav.y;
    states[6] = states[6] + delVelNav.z;

// If calculating postions, do a trapezoidal integration for position
    states[7] = states[7] + 0.5f*(states[4] + lastVelocity[0])*dtIMU;
    states[8] = states[8] + 0.5f*(states[5] + lastVelocity[1])*dtIMU;
    states[9] = states[9] + 0.5f*(states[6] + lastVelocity[2])*dtIMU;

    // Constrain states (to protect against filter divergence)
    //ConstrainStates();
}

void AttPosEKF::CovariancePrediction(float dt)
{
    // scalars
    float daxCov;
    float dayCov;
    float dazCov;
    float dvxCov;
    float dvyCov;
    float dvzCov;
    float dvx;
    float dvy;
    float dvz;
    float dax;
    float day;
    float daz;
    float q0;
    float q1;
    float q2;
    float q3;
    float dax_b;
    float day_b;
    float daz_b;

    // arrays
    float processNoise[21];
    float SF[14];
    float SG[8];
    float SQ[11];
    float SPP[13] = {0};
    float nextP[21][21];

    // calculate covariance prediction process noise
    windVelSigma = dt*0.1f;
    dAngBiasSigma = dt*5.0e-7f;
    magEarthSigma = dt*3.0e-4f;
    magBodySigma  = dt*3.0e-4f;
    for (uint8_t i= 0; i<=9; i++) processNoise[i]  = 1.0e-9f;
    for (uint8_t i=10; i<=12; i++) processNoise[i] = dAngBiasSigma;
    if (onGround) processNoise[12] = dAngBiasSigma * yawVarScale;
    for (uint8_t i=13; i<=14; i++) processNoise[i] = windVelSigma;
    for (uint8_t i=15; i<=17; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=18; i<=20; i++) processNoise[i] = magBodySigma;
    for (uint8_t i= 0; i<=20; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = summedDelVel.x;
    dvy = summedDelVel.y;
    dvz = summedDelVel.z;
    dax = summedDelAng.x;
    day = summedDelAng.y;
    daz = summedDelAng.z;
    q0 = states[0];
    q1 = states[1];
    q2 = states[2];
    q3 = states[3];
    dax_b = states[10];
    day_b = states[11];
    daz_b = states[12];
    daxCov = sq(dt*1.4544411e-2f);
    dayCov = sq(dt*1.4544411e-2f);
    dazCov = sq(dt*1.4544411e-2f);
    if (onGround) dazCov = dazCov * sq(yawVarScale);
    dvxCov = sq(dt*0.5f);
    dvyCov = sq(dt*0.5f);
    dvzCov = sq(dt*0.5f);

    // Predicted covariance calculation
    SF[0] = 2*dvx*q1 + 2*dvy*q2 + 2*dvz*q3;
    SF[1] = 2*dvx*q3 + 2*dvy*q0 - 2*dvz*q1;
    SF[2] = 2*dvx*q0 - 2*dvy*q3 + 2*dvz*q2;
    SF[3] = day/2 - day_b/2;
    SF[4] = daz/2 - daz_b/2;
    SF[5] = dax/2 - dax_b/2;
    SF[6] = dax_b/2 - dax/2;
    SF[7] = daz_b/2 - daz/2;
    SF[8] = day_b/2 - day/2;
    SF[9] = q1/2;
    SF[10] = q2/2;
    SF[11] = q3/2;
    SF[12] = 2*dvz*q0;
    SF[13] = 2*dvy*q1;

    SG[0] = q0/2;
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);
    SG[5] = 2*q2*q3;
    SG[6] = 2*q1*q3;
    SG[7] = 2*q1*q2;

    SQ[0] = dvzCov*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyCov*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxCov*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
    SQ[1] = dvzCov*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxCov*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyCov*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
    SQ[2] = dvzCov*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyCov*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxCov*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
    SQ[3] = (dayCov*q1*SG[0])/2 - (dazCov*q1*SG[0])/2 - (daxCov*q2*q3)/4;
    SQ[4] = (dazCov*q2*SG[0])/2 - (daxCov*q2*SG[0])/2 - (dayCov*q1*q3)/4;
    SQ[5] = (daxCov*q3*SG[0])/2 - (dayCov*q3*SG[0])/2 - (dazCov*q1*q2)/4;
    SQ[6] = (daxCov*q1*q2)/4 - (dazCov*q3*SG[0])/2 - (dayCov*q1*q2)/4;
    SQ[7] = (dazCov*q1*q3)/4 - (daxCov*q1*q3)/4 - (dayCov*q2*SG[0])/2;
    SQ[8] = (dayCov*q2*q3)/4 - (daxCov*q1*SG[0])/2 - (dazCov*q2*q3)/4;
    SQ[9] = sq(SG[0]);
    SQ[10] = sq(q1);

    SPP[0] = SF[12] + SF[13] - 2*dvx*q2;
    SPP[1] = 2*dvx*q0 - 2*dvy*q3 + 2*dvz*q2;
    SPP[2] = 2*dvx*q3 + 2*dvy*q0 - 2*dvz*q1;
    SPP[3] = SF[11];
    SPP[4] = SF[10];
    SPP[5] = SF[9];
    SPP[6] = SF[7];
    SPP[7] = SF[8];

    nextP[0][0] = P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3] + (daxCov*SQ[10])/4 + SF[6]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]) + SPP[7]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]) + SPP[6]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]) + SPP[5]*(P[0][10] + P[1][10]*SF[6] + P[2][10]*SPP[7] + P[3][10]*SPP[6] + P[10][10]*SPP[5] + P[11][10]*SPP[4] + P[12][10]*SPP[3]) + SPP[4]*(P[0][11] + P[1][11]*SF[6] + P[2][11]*SPP[7] + P[3][11]*SPP[6] + P[10][11]*SPP[5] + P[11][11]*SPP[4] + P[12][11]*SPP[3]) + SPP[3]*(P[0][12] + P[1][12]*SF[6] + P[2][12]*SPP[7] + P[3][12]*SPP[6] + P[10][12]*SPP[5] + P[11][12]*SPP[4] + P[12][12]*SPP[3]) + (dayCov*sq(q2))/4 + (dazCov*sq(q3))/4;
    nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3] + SF[5]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) + SF[4]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]) + SPP[7]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]) + SPP[3]*(P[0][11] + P[1][11]*SF[6] + P[2][11]*SPP[7] + P[3][11]*SPP[6] + P[10][11]*SPP[5] + P[11][11]*SPP[4] + P[12][11]*SPP[3]) - SPP[4]*(P[0][12] + P[1][12]*SF[6] + P[2][12]*SPP[7] + P[3][12]*SPP[6] + P[10][12]*SPP[5] + P[11][12]*SPP[4] + P[12][12]*SPP[3]) - (q0*(P[0][10] + P[1][10]*SF[6] + P[2][10]*SPP[7] + P[3][10]*SPP[6] + P[10][10]*SPP[5] + P[11][10]*SPP[4] + P[12][10]*SPP[3]))/2;
    nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3] + SF[3]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) + SF[5]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]) + SPP[6]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]) - SPP[3]*(P[0][10] + P[1][10]*SF[6] + P[2][10]*SPP[7] + P[3][10]*SPP[6] + P[10][10]*SPP[5] + P[11][10]*SPP[4] + P[12][10]*SPP[3]) + SPP[5]*(P[0][12] + P[1][12]*SF[6] + P[2][12]*SPP[7] + P[3][12]*SPP[6] + P[10][12]*SPP[5] + P[11][12]*SPP[4] + P[12][12]*SPP[3]) - (q0*(P[0][11] + P[1][11]*SF[6] + P[2][11]*SPP[7] + P[3][11]*SPP[6] + P[10][11]*SPP[5] + P[11][11]*SPP[4] + P[12][11]*SPP[3]))/2;
    nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3] + SF[4]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) + SF[3]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]) + SF[6]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]) + SPP[4]*(P[0][10] + P[1][10]*SF[6] + P[2][10]*SPP[7] + P[3][10]*SPP[6] + P[10][10]*SPP[5] + P[11][10]*SPP[4] + P[12][10]*SPP[3]) - SPP[5]*(P[0][11] + P[1][11]*SF[6] + P[2][11]*SPP[7] + P[3][11]*SPP[6] + P[10][11]*SPP[5] + P[11][11]*SPP[4] + P[12][11]*SPP[3]) - (q0*(P[0][12] + P[1][12]*SF[6] + P[2][12]*SPP[7] + P[3][12]*SPP[6] + P[10][12]*SPP[5] + P[11][12]*SPP[4] + P[12][12]*SPP[3]))/2;
    nextP[0][4] = P[0][4] + P[1][4]*SF[6] + P[2][4]*SPP[7] + P[3][4]*SPP[6] + P[10][4]*SPP[5] + P[11][4]*SPP[4] + P[12][4]*SPP[3] + SF[2]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) + SF[0]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]) + SPP[0]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]) - SPP[2]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]);
    nextP[0][5] = P[0][5] + P[1][5]*SF[6] + P[2][5]*SPP[7] + P[3][5]*SPP[6] + P[10][5]*SPP[5] + P[11][5]*SPP[4] + P[12][5]*SPP[3] + SF[1]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) + SF[0]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]) + SF[2]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]) - SPP[0]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]);
    nextP[0][6] = P[0][6] + P[1][6]*SF[6] + P[2][6]*SPP[7] + P[3][6]*SPP[6] + P[10][6]*SPP[5] + P[11][6]*SPP[4] + P[12][6]*SPP[3] + SF[1]*(P[0][1] + P[1][1]*SF[6] + P[2][1]*SPP[7] + P[3][1]*SPP[6] + P[10][1]*SPP[5] + P[11][1]*SPP[4] + P[12][1]*SPP[3]) + SF[0]*(P[0][3] + P[1][3]*SF[6] + P[2][3]*SPP[7] + P[3][3]*SPP[6] + P[10][3]*SPP[5] + P[11][3]*SPP[4] + P[12][3]*SPP[3]) + SPP[0]*(P[0][0] + P[1][0]*SF[6] + P[2][0]*SPP[7] + P[3][0]*SPP[6] + P[10][0]*SPP[5] + P[11][0]*SPP[4] + P[12][0]*SPP[3]) - SPP[1]*(P[0][2] + P[1][2]*SF[6] + P[2][2]*SPP[7] + P[3][2]*SPP[6] + P[10][2]*SPP[5] + P[11][2]*SPP[4] + P[12][2]*SPP[3]);
    nextP[0][7] = P[0][7] + P[1][7]*SF[6] + P[2][7]*SPP[7] + P[3][7]*SPP[6] + P[10][7]*SPP[5] + P[11][7]*SPP[4] + P[12][7]*SPP[3] + dt*(P[0][4] + P[1][4]*SF[6] + P[2][4]*SPP[7] + P[3][4]*SPP[6] + P[10][4]*SPP[5] + P[11][4]*SPP[4] + P[12][4]*SPP[3]);
    nextP[0][8] = P[0][8] + P[1][8]*SF[6] + P[2][8]*SPP[7] + P[3][8]*SPP[6] + P[10][8]*SPP[5] + P[11][8]*SPP[4] + P[12][8]*SPP[3] + dt*(P[0][5] + P[1][5]*SF[6] + P[2][5]*SPP[7] + P[3][5]*SPP[6] + P[10][5]*SPP[5] + P[11][5]*SPP[4] + P[12][5]*SPP[3]);
    nextP[0][9] = P[0][9] + P[1][9]*SF[6] + P[2][9]*SPP[7] + P[3][9]*SPP[6] + P[10][9]*SPP[5] + P[11][9]*SPP[4] + P[12][9]*SPP[3] + dt*(P[0][6] + P[1][6]*SF[6] + P[2][6]*SPP[7] + P[3][6]*SPP[6] + P[10][6]*SPP[5] + P[11][6]*SPP[4] + P[12][6]*SPP[3]);
    nextP[0][10] = P[0][10] + P[1][10]*SF[6] + P[2][10]*SPP[7] + P[3][10]*SPP[6] + P[10][10]*SPP[5] + P[11][10]*SPP[4] + P[12][10]*SPP[3];
    nextP[0][11] = P[0][11] + P[1][11]*SF[6] + P[2][11]*SPP[7] + P[3][11]*SPP[6] + P[10][11]*SPP[5] + P[11][11]*SPP[4] + P[12][11]*SPP[3];
    nextP[0][12] = P[0][12] + P[1][12]*SF[6] + P[2][12]*SPP[7] + P[3][12]*SPP[6] + P[10][12]*SPP[5] + P[11][12]*SPP[4] + P[12][12]*SPP[3];
    nextP[0][13] = P[0][13] + P[1][13]*SF[6] + P[2][13]*SPP[7] + P[3][13]*SPP[6] + P[10][13]*SPP[5] + P[11][13]*SPP[4] + P[12][13]*SPP[3];
    nextP[0][14] = P[0][14] + P[1][14]*SF[6] + P[2][14]*SPP[7] + P[3][14]*SPP[6] + P[10][14]*SPP[5] + P[11][14]*SPP[4] + P[12][14]*SPP[3];
    nextP[0][15] = P[0][15] + P[1][15]*SF[6] + P[2][15]*SPP[7] + P[3][15]*SPP[6] + P[10][15]*SPP[5] + P[11][15]*SPP[4] + P[12][15]*SPP[3];
    nextP[0][16] = P[0][16] + P[1][16]*SF[6] + P[2][16]*SPP[7] + P[3][16]*SPP[6] + P[10][16]*SPP[5] + P[11][16]*SPP[4] + P[12][16]*SPP[3];
    nextP[0][17] = P[0][17] + P[1][17]*SF[6] + P[2][17]*SPP[7] + P[3][17]*SPP[6] + P[10][17]*SPP[5] + P[11][17]*SPP[4] + P[12][17]*SPP[3];
    nextP[0][18] = P[0][18] + P[1][18]*SF[6] + P[2][18]*SPP[7] + P[3][18]*SPP[6] + P[10][18]*SPP[5] + P[11][18]*SPP[4] + P[12][18]*SPP[3];
    nextP[0][19] = P[0][19] + P[1][19]*SF[6] + P[2][19]*SPP[7] + P[3][19]*SPP[6] + P[10][19]*SPP[5] + P[11][19]*SPP[4] + P[12][19]*SPP[3];
    nextP[0][20] = P[0][20] + P[1][20]*SF[6] + P[2][20]*SPP[7] + P[3][20]*SPP[6] + P[10][20]*SPP[5] + P[11][20]*SPP[4] + P[12][20]*SPP[3];
    nextP[1][0] = P[1][0] + SQ[8] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2 + SF[6]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2) + SPP[7]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2) + SPP[6]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2) + SPP[5]*(P[1][10] + P[0][10]*SF[5] + P[2][10]*SF[4] + P[3][10]*SPP[7] + P[11][10]*SPP[3] - P[12][10]*SPP[4] - (P[10][10]*q0)/2) + SPP[4]*(P[1][11] + P[0][11]*SF[5] + P[2][11]*SF[4] + P[3][11]*SPP[7] + P[11][11]*SPP[3] - P[12][11]*SPP[4] - (P[10][11]*q0)/2) + SPP[3]*(P[1][12] + P[0][12]*SF[5] + P[2][12]*SF[4] + P[3][12]*SPP[7] + P[11][12]*SPP[3] - P[12][12]*SPP[4] - (P[10][12]*q0)/2);
    nextP[1][1] = P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] + daxCov*SQ[9] - (P[10][1]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) + SF[4]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2) + SPP[7]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2) + SPP[3]*(P[1][11] + P[0][11]*SF[5] + P[2][11]*SF[4] + P[3][11]*SPP[7] + P[11][11]*SPP[3] - P[12][11]*SPP[4] - (P[10][11]*q0)/2) - SPP[4]*(P[1][12] + P[0][12]*SF[5] + P[2][12]*SF[4] + P[3][12]*SPP[7] + P[11][12]*SPP[3] - P[12][12]*SPP[4] - (P[10][12]*q0)/2) + (dayCov*sq(q3))/4 + (dazCov*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[5] + P[2][10]*SF[4] + P[3][10]*SPP[7] + P[11][10]*SPP[3] - P[12][10]*SPP[4] - (P[10][10]*q0)/2))/2;
    nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2 + SF[3]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) + SF[5]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2) + SPP[6]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2) - SPP[3]*(P[1][10] + P[0][10]*SF[5] + P[2][10]*SF[4] + P[3][10]*SPP[7] + P[11][10]*SPP[3] - P[12][10]*SPP[4] - (P[10][10]*q0)/2) + SPP[5]*(P[1][12] + P[0][12]*SF[5] + P[2][12]*SF[4] + P[3][12]*SPP[7] + P[11][12]*SPP[3] - P[12][12]*SPP[4] - (P[10][12]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[5] + P[2][11]*SF[4] + P[3][11]*SPP[7] + P[11][11]*SPP[3] - P[12][11]*SPP[4] - (P[10][11]*q0)/2))/2;
    nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) + SF[3]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2) + SF[6]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2) + SPP[4]*(P[1][10] + P[0][10]*SF[5] + P[2][10]*SF[4] + P[3][10]*SPP[7] + P[11][10]*SPP[3] - P[12][10]*SPP[4] - (P[10][10]*q0)/2) - SPP[5]*(P[1][11] + P[0][11]*SF[5] + P[2][11]*SF[4] + P[3][11]*SPP[7] + P[11][11]*SPP[3] - P[12][11]*SPP[4] - (P[10][11]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[5] + P[2][12]*SF[4] + P[3][12]*SPP[7] + P[11][12]*SPP[3] - P[12][12]*SPP[4] - (P[10][12]*q0)/2))/2;
    nextP[1][4] = P[1][4] + P[0][4]*SF[5] + P[2][4]*SF[4] + P[3][4]*SPP[7] + P[11][4]*SPP[3] - P[12][4]*SPP[4] - (P[10][4]*q0)/2 + SF[2]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) + SF[0]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2) - SPP[2]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2);
    nextP[1][5] = P[1][5] + P[0][5]*SF[5] + P[2][5]*SF[4] + P[3][5]*SPP[7] + P[11][5]*SPP[3] - P[12][5]*SPP[4] - (P[10][5]*q0)/2 + SF[1]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) + SF[0]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2) + SF[2]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2);
    nextP[1][6] = P[1][6] + P[0][6]*SF[5] + P[2][6]*SF[4] + P[3][6]*SPP[7] + P[11][6]*SPP[3] - P[12][6]*SPP[4] - (P[10][6]*q0)/2 + SF[1]*(P[1][1] + P[0][1]*SF[5] + P[2][1]*SF[4] + P[3][1]*SPP[7] + P[11][1]*SPP[3] - P[12][1]*SPP[4] - (P[10][1]*q0)/2) + SF[0]*(P[1][3] + P[0][3]*SF[5] + P[2][3]*SF[4] + P[3][3]*SPP[7] + P[11][3]*SPP[3] - P[12][3]*SPP[4] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[5] + P[2][0]*SF[4] + P[3][0]*SPP[7] + P[11][0]*SPP[3] - P[12][0]*SPP[4] - (P[10][0]*q0)/2) - SPP[1]*(P[1][2] + P[0][2]*SF[5] + P[2][2]*SF[4] + P[3][2]*SPP[7] + P[11][2]*SPP[3] - P[12][2]*SPP[4] - (P[10][2]*q0)/2);
    nextP[1][7] = P[1][7] + P[0][7]*SF[5] + P[2][7]*SF[4] + P[3][7]*SPP[7] + P[11][7]*SPP[3] - P[12][7]*SPP[4] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[5] + P[2][4]*SF[4] + P[3][4]*SPP[7] + P[11][4]*SPP[3] - P[12][4]*SPP[4] - (P[10][4]*q0)/2);
    nextP[1][8] = P[1][8] + P[0][8]*SF[5] + P[2][8]*SF[4] + P[3][8]*SPP[7] + P[11][8]*SPP[3] - P[12][8]*SPP[4] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[5] + P[2][5]*SF[4] + P[3][5]*SPP[7] + P[11][5]*SPP[3] - P[12][5]*SPP[4] - (P[10][5]*q0)/2);
    nextP[1][9] = P[1][9] + P[0][9]*SF[5] + P[2][9]*SF[4] + P[3][9]*SPP[7] + P[11][9]*SPP[3] - P[12][9]*SPP[4] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[5] + P[2][6]*SF[4] + P[3][6]*SPP[7] + P[11][6]*SPP[3] - P[12][6]*SPP[4] - (P[10][6]*q0)/2);
    nextP[1][10] = P[1][10] + P[0][10]*SF[5] + P[2][10]*SF[4] + P[3][10]*SPP[7] + P[11][10]*SPP[3] - P[12][10]*SPP[4] - (P[10][10]*q0)/2;
    nextP[1][11] = P[1][11] + P[0][11]*SF[5] + P[2][11]*SF[4] + P[3][11]*SPP[7] + P[11][11]*SPP[3] - P[12][11]*SPP[4] - (P[10][11]*q0)/2;
    nextP[1][12] = P[1][12] + P[0][12]*SF[5] + P[2][12]*SF[4] + P[3][12]*SPP[7] + P[11][12]*SPP[3] - P[12][12]*SPP[4] - (P[10][12]*q0)/2;
    nextP[1][13] = P[1][13] + P[0][13]*SF[5] + P[2][13]*SF[4] + P[3][13]*SPP[7] + P[11][13]*SPP[3] - P[12][13]*SPP[4] - (P[10][13]*q0)/2;
    nextP[1][14] = P[1][14] + P[0][14]*SF[5] + P[2][14]*SF[4] + P[3][14]*SPP[7] + P[11][14]*SPP[3] - P[12][14]*SPP[4] - (P[10][14]*q0)/2;
    nextP[1][15] = P[1][15] + P[0][15]*SF[5] + P[2][15]*SF[4] + P[3][15]*SPP[7] + P[11][15]*SPP[3] - P[12][15]*SPP[4] - (P[10][15]*q0)/2;
    nextP[1][16] = P[1][16] + P[0][16]*SF[5] + P[2][16]*SF[4] + P[3][16]*SPP[7] + P[11][16]*SPP[3] - P[12][16]*SPP[4] - (P[10][16]*q0)/2;
    nextP[1][17] = P[1][17] + P[0][17]*SF[5] + P[2][17]*SF[4] + P[3][17]*SPP[7] + P[11][17]*SPP[3] - P[12][17]*SPP[4] - (P[10][17]*q0)/2;
    nextP[1][18] = P[1][18] + P[0][18]*SF[5] + P[2][18]*SF[4] + P[3][18]*SPP[7] + P[11][18]*SPP[3] - P[12][18]*SPP[4] - (P[10][18]*q0)/2;
    nextP[1][19] = P[1][19] + P[0][19]*SF[5] + P[2][19]*SF[4] + P[3][19]*SPP[7] + P[11][19]*SPP[3] - P[12][19]*SPP[4] - (P[10][19]*q0)/2;
    nextP[1][20] = P[1][20] + P[0][20]*SF[5] + P[2][20]*SF[4] + P[3][20]*SPP[7] + P[11][20]*SPP[3] - P[12][20]*SPP[4] - (P[10][20]*q0)/2;
    nextP[2][0] = P[2][0] + SQ[7] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2 + SF[6]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2) + SPP[7]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2) + SPP[6]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2) + SPP[5]*(P[2][10] + P[0][10]*SF[3] + P[3][10]*SF[5] + P[1][10]*SPP[6] - P[10][10]*SPP[3] + P[12][10]*SPP[5] - (P[11][10]*q0)/2) + SPP[4]*(P[2][11] + P[0][11]*SF[3] + P[3][11]*SF[5] + P[1][11]*SPP[6] - P[10][11]*SPP[3] + P[12][11]*SPP[5] - (P[11][11]*q0)/2) + SPP[3]*(P[2][12] + P[0][12]*SF[3] + P[3][12]*SF[5] + P[1][12]*SPP[6] - P[10][12]*SPP[3] + P[12][12]*SPP[5] - (P[11][12]*q0)/2);
    nextP[2][1] = P[2][1] + SQ[5] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) + SF[4]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2) + SPP[7]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2) + SPP[3]*(P[2][11] + P[0][11]*SF[3] + P[3][11]*SF[5] + P[1][11]*SPP[6] - P[10][11]*SPP[3] + P[12][11]*SPP[5] - (P[11][11]*q0)/2) - SPP[4]*(P[2][12] + P[0][12]*SF[3] + P[3][12]*SF[5] + P[1][12]*SPP[6] - P[10][12]*SPP[3] + P[12][12]*SPP[5] - (P[11][12]*q0)/2) - (q0*(P[2][10] + P[0][10]*SF[3] + P[3][10]*SF[5] + P[1][10]*SPP[6] - P[10][10]*SPP[3] + P[12][10]*SPP[5] - (P[11][10]*q0)/2))/2;
    nextP[2][2] = P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] + dayCov*SQ[9] + (dazCov*SQ[10])/4 - (P[11][2]*q0)/2 + SF[3]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) + SF[5]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2) + SPP[6]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2) - SPP[3]*(P[2][10] + P[0][10]*SF[3] + P[3][10]*SF[5] + P[1][10]*SPP[6] - P[10][10]*SPP[3] + P[12][10]*SPP[5] - (P[11][10]*q0)/2) + SPP[5]*(P[2][12] + P[0][12]*SF[3] + P[3][12]*SF[5] + P[1][12]*SPP[6] - P[10][12]*SPP[3] + P[12][12]*SPP[5] - (P[11][12]*q0)/2) + (daxCov*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[3] + P[3][11]*SF[5] + P[1][11]*SPP[6] - P[10][11]*SPP[3] + P[12][11]*SPP[5] - (P[11][11]*q0)/2))/2;
    nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) + SF[3]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2) + SF[6]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2) + SPP[4]*(P[2][10] + P[0][10]*SF[3] + P[3][10]*SF[5] + P[1][10]*SPP[6] - P[10][10]*SPP[3] + P[12][10]*SPP[5] - (P[11][10]*q0)/2) - SPP[5]*(P[2][11] + P[0][11]*SF[3] + P[3][11]*SF[5] + P[1][11]*SPP[6] - P[10][11]*SPP[3] + P[12][11]*SPP[5] - (P[11][11]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[3] + P[3][12]*SF[5] + P[1][12]*SPP[6] - P[10][12]*SPP[3] + P[12][12]*SPP[5] - (P[11][12]*q0)/2))/2;
    nextP[2][4] = P[2][4] + P[0][4]*SF[3] + P[3][4]*SF[5] + P[1][4]*SPP[6] - P[10][4]*SPP[3] + P[12][4]*SPP[5] - (P[11][4]*q0)/2 + SF[2]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) + SF[0]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2) - SPP[2]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2);
    nextP[2][5] = P[2][5] + P[0][5]*SF[3] + P[3][5]*SF[5] + P[1][5]*SPP[6] - P[10][5]*SPP[3] + P[12][5]*SPP[5] - (P[11][5]*q0)/2 + SF[1]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) + SF[0]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2) + SF[2]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2);
    nextP[2][6] = P[2][6] + P[0][6]*SF[3] + P[3][6]*SF[5] + P[1][6]*SPP[6] - P[10][6]*SPP[3] + P[12][6]*SPP[5] - (P[11][6]*q0)/2 + SF[1]*(P[2][1] + P[0][1]*SF[3] + P[3][1]*SF[5] + P[1][1]*SPP[6] - P[10][1]*SPP[3] + P[12][1]*SPP[5] - (P[11][1]*q0)/2) + SF[0]*(P[2][3] + P[0][3]*SF[3] + P[3][3]*SF[5] + P[1][3]*SPP[6] - P[10][3]*SPP[3] + P[12][3]*SPP[5] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[3] + P[3][0]*SF[5] + P[1][0]*SPP[6] - P[10][0]*SPP[3] + P[12][0]*SPP[5] - (P[11][0]*q0)/2) - SPP[1]*(P[2][2] + P[0][2]*SF[3] + P[3][2]*SF[5] + P[1][2]*SPP[6] - P[10][2]*SPP[3] + P[12][2]*SPP[5] - (P[11][2]*q0)/2);
    nextP[2][7] = P[2][7] + P[0][7]*SF[3] + P[3][7]*SF[5] + P[1][7]*SPP[6] - P[10][7]*SPP[3] + P[12][7]*SPP[5] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[3] + P[3][4]*SF[5] + P[1][4]*SPP[6] - P[10][4]*SPP[3] + P[12][4]*SPP[5] - (P[11][4]*q0)/2);
    nextP[2][8] = P[2][8] + P[0][8]*SF[3] + P[3][8]*SF[5] + P[1][8]*SPP[6] - P[10][8]*SPP[3] + P[12][8]*SPP[5] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[3] + P[3][5]*SF[5] + P[1][5]*SPP[6] - P[10][5]*SPP[3] + P[12][5]*SPP[5] - (P[11][5]*q0)/2);
    nextP[2][9] = P[2][9] + P[0][9]*SF[3] + P[3][9]*SF[5] + P[1][9]*SPP[6] - P[10][9]*SPP[3] + P[12][9]*SPP[5] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[3] + P[3][6]*SF[5] + P[1][6]*SPP[6] - P[10][6]*SPP[3] + P[12][6]*SPP[5] - (P[11][6]*q0)/2);
    nextP[2][10] = P[2][10] + P[0][10]*SF[3] + P[3][10]*SF[5] + P[1][10]*SPP[6] - P[10][10]*SPP[3] + P[12][10]*SPP[5] - (P[11][10]*q0)/2;
    nextP[2][11] = P[2][11] + P[0][11]*SF[3] + P[3][11]*SF[5] + P[1][11]*SPP[6] - P[10][11]*SPP[3] + P[12][11]*SPP[5] - (P[11][11]*q0)/2;
    nextP[2][12] = P[2][12] + P[0][12]*SF[3] + P[3][12]*SF[5] + P[1][12]*SPP[6] - P[10][12]*SPP[3] + P[12][12]*SPP[5] - (P[11][12]*q0)/2;
    nextP[2][13] = P[2][13] + P[0][13]*SF[3] + P[3][13]*SF[5] + P[1][13]*SPP[6] - P[10][13]*SPP[3] + P[12][13]*SPP[5] - (P[11][13]*q0)/2;
    nextP[2][14] = P[2][14] + P[0][14]*SF[3] + P[3][14]*SF[5] + P[1][14]*SPP[6] - P[10][14]*SPP[3] + P[12][14]*SPP[5] - (P[11][14]*q0)/2;
    nextP[2][15] = P[2][15] + P[0][15]*SF[3] + P[3][15]*SF[5] + P[1][15]*SPP[6] - P[10][15]*SPP[3] + P[12][15]*SPP[5] - (P[11][15]*q0)/2;
    nextP[2][16] = P[2][16] + P[0][16]*SF[3] + P[3][16]*SF[5] + P[1][16]*SPP[6] - P[10][16]*SPP[3] + P[12][16]*SPP[5] - (P[11][16]*q0)/2;
    nextP[2][17] = P[2][17] + P[0][17]*SF[3] + P[3][17]*SF[5] + P[1][17]*SPP[6] - P[10][17]*SPP[3] + P[12][17]*SPP[5] - (P[11][17]*q0)/2;
    nextP[2][18] = P[2][18] + P[0][18]*SF[3] + P[3][18]*SF[5] + P[1][18]*SPP[6] - P[10][18]*SPP[3] + P[12][18]*SPP[5] - (P[11][18]*q0)/2;
    nextP[2][19] = P[2][19] + P[0][19]*SF[3] + P[3][19]*SF[5] + P[1][19]*SPP[6] - P[10][19]*SPP[3] + P[12][19]*SPP[5] - (P[11][19]*q0)/2;
    nextP[2][20] = P[2][20] + P[0][20]*SF[3] + P[3][20]*SF[5] + P[1][20]*SPP[6] - P[10][20]*SPP[3] + P[12][20]*SPP[5] - (P[11][20]*q0)/2;
    nextP[3][0] = P[3][0] + SQ[6] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2 + SF[6]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2) + SPP[7]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2) + SPP[6]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2) + SPP[5]*(P[3][10] + P[0][10]*SF[4] + P[1][10]*SF[3] + P[2][10]*SF[6] + P[10][10]*SPP[4] - P[11][10]*SPP[5] - (P[12][10]*q0)/2) + SPP[4]*(P[3][11] + P[0][11]*SF[4] + P[1][11]*SF[3] + P[2][11]*SF[6] + P[10][11]*SPP[4] - P[11][11]*SPP[5] - (P[12][11]*q0)/2) + SPP[3]*(P[3][12] + P[0][12]*SF[4] + P[1][12]*SF[3] + P[2][12]*SF[6] + P[10][12]*SPP[4] - P[11][12]*SPP[5] - (P[12][12]*q0)/2);
    nextP[3][1] = P[3][1] + SQ[4] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) + SF[4]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2) + SPP[7]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2) + SPP[3]*(P[3][11] + P[0][11]*SF[4] + P[1][11]*SF[3] + P[2][11]*SF[6] + P[10][11]*SPP[4] - P[11][11]*SPP[5] - (P[12][11]*q0)/2) - SPP[4]*(P[3][12] + P[0][12]*SF[4] + P[1][12]*SF[3] + P[2][12]*SF[6] + P[10][12]*SPP[4] - P[11][12]*SPP[5] - (P[12][12]*q0)/2) - (q0*(P[3][10] + P[0][10]*SF[4] + P[1][10]*SF[3] + P[2][10]*SF[6] + P[10][10]*SPP[4] - P[11][10]*SPP[5] - (P[12][10]*q0)/2))/2;
    nextP[3][2] = P[3][2] + SQ[3] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2 + SF[3]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) + SF[5]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2) + SPP[6]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2) - SPP[3]*(P[3][10] + P[0][10]*SF[4] + P[1][10]*SF[3] + P[2][10]*SF[6] + P[10][10]*SPP[4] - P[11][10]*SPP[5] - (P[12][10]*q0)/2) + SPP[5]*(P[3][12] + P[0][12]*SF[4] + P[1][12]*SF[3] + P[2][12]*SF[6] + P[10][12]*SPP[4] - P[11][12]*SPP[5] - (P[12][12]*q0)/2) - (q0*(P[3][11] + P[0][11]*SF[4] + P[1][11]*SF[3] + P[2][11]*SF[6] + P[10][11]*SPP[4] - P[11][11]*SPP[5] - (P[12][11]*q0)/2))/2;
    nextP[3][3] = P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] + (dayCov*SQ[10])/4 + dazCov*SQ[9] - (P[12][3]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) + SF[3]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2) + SF[6]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2) + SPP[4]*(P[3][10] + P[0][10]*SF[4] + P[1][10]*SF[3] + P[2][10]*SF[6] + P[10][10]*SPP[4] - P[11][10]*SPP[5] - (P[12][10]*q0)/2) - SPP[5]*(P[3][11] + P[0][11]*SF[4] + P[1][11]*SF[3] + P[2][11]*SF[6] + P[10][11]*SPP[4] - P[11][11]*SPP[5] - (P[12][11]*q0)/2) + (daxCov*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[4] + P[1][12]*SF[3] + P[2][12]*SF[6] + P[10][12]*SPP[4] - P[11][12]*SPP[5] - (P[12][12]*q0)/2))/2;
    nextP[3][4] = P[3][4] + P[0][4]*SF[4] + P[1][4]*SF[3] + P[2][4]*SF[6] + P[10][4]*SPP[4] - P[11][4]*SPP[5] - (P[12][4]*q0)/2 + SF[2]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) + SF[0]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2) - SPP[2]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2);
    nextP[3][5] = P[3][5] + P[0][5]*SF[4] + P[1][5]*SF[3] + P[2][5]*SF[6] + P[10][5]*SPP[4] - P[11][5]*SPP[5] - (P[12][5]*q0)/2 + SF[1]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) + SF[0]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2) + SF[2]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2);
    nextP[3][6] = P[3][6] + P[0][6]*SF[4] + P[1][6]*SF[3] + P[2][6]*SF[6] + P[10][6]*SPP[4] - P[11][6]*SPP[5] - (P[12][6]*q0)/2 + SF[1]*(P[3][1] + P[0][1]*SF[4] + P[1][1]*SF[3] + P[2][1]*SF[6] + P[10][1]*SPP[4] - P[11][1]*SPP[5] - (P[12][1]*q0)/2) + SF[0]*(P[3][3] + P[0][3]*SF[4] + P[1][3]*SF[3] + P[2][3]*SF[6] + P[10][3]*SPP[4] - P[11][3]*SPP[5] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[4] + P[1][0]*SF[3] + P[2][0]*SF[6] + P[10][0]*SPP[4] - P[11][0]*SPP[5] - (P[12][0]*q0)/2) - SPP[1]*(P[3][2] + P[0][2]*SF[4] + P[1][2]*SF[3] + P[2][2]*SF[6] + P[10][2]*SPP[4] - P[11][2]*SPP[5] - (P[12][2]*q0)/2);
    nextP[3][7] = P[3][7] + P[0][7]*SF[4] + P[1][7]*SF[3] + P[2][7]*SF[6] + P[10][7]*SPP[4] - P[11][7]*SPP[5] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[4] + P[1][4]*SF[3] + P[2][4]*SF[6] + P[10][4]*SPP[4] - P[11][4]*SPP[5] - (P[12][4]*q0)/2);
    nextP[3][8] = P[3][8] + P[0][8]*SF[4] + P[1][8]*SF[3] + P[2][8]*SF[6] + P[10][8]*SPP[4] - P[11][8]*SPP[5] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[4] + P[1][5]*SF[3] + P[2][5]*SF[6] + P[10][5]*SPP[4] - P[11][5]*SPP[5] - (P[12][5]*q0)/2);
    nextP[3][9] = P[3][9] + P[0][9]*SF[4] + P[1][9]*SF[3] + P[2][9]*SF[6] + P[10][9]*SPP[4] - P[11][9]*SPP[5] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[4] + P[1][6]*SF[3] + P[2][6]*SF[6] + P[10][6]*SPP[4] - P[11][6]*SPP[5] - (P[12][6]*q0)/2);
    nextP[3][10] = P[3][10] + P[0][10]*SF[4] + P[1][10]*SF[3] + P[2][10]*SF[6] + P[10][10]*SPP[4] - P[11][10]*SPP[5] - (P[12][10]*q0)/2;
    nextP[3][11] = P[3][11] + P[0][11]*SF[4] + P[1][11]*SF[3] + P[2][11]*SF[6] + P[10][11]*SPP[4] - P[11][11]*SPP[5] - (P[12][11]*q0)/2;
    nextP[3][12] = P[3][12] + P[0][12]*SF[4] + P[1][12]*SF[3] + P[2][12]*SF[6] + P[10][12]*SPP[4] - P[11][12]*SPP[5] - (P[12][12]*q0)/2;
    nextP[3][13] = P[3][13] + P[0][13]*SF[4] + P[1][13]*SF[3] + P[2][13]*SF[6] + P[10][13]*SPP[4] - P[11][13]*SPP[5] - (P[12][13]*q0)/2;
    nextP[3][14] = P[3][14] + P[0][14]*SF[4] + P[1][14]*SF[3] + P[2][14]*SF[6] + P[10][14]*SPP[4] - P[11][14]*SPP[5] - (P[12][14]*q0)/2;
    nextP[3][15] = P[3][15] + P[0][15]*SF[4] + P[1][15]*SF[3] + P[2][15]*SF[6] + P[10][15]*SPP[4] - P[11][15]*SPP[5] - (P[12][15]*q0)/2;
    nextP[3][16] = P[3][16] + P[0][16]*SF[4] + P[1][16]*SF[3] + P[2][16]*SF[6] + P[10][16]*SPP[4] - P[11][16]*SPP[5] - (P[12][16]*q0)/2;
    nextP[3][17] = P[3][17] + P[0][17]*SF[4] + P[1][17]*SF[3] + P[2][17]*SF[6] + P[10][17]*SPP[4] - P[11][17]*SPP[5] - (P[12][17]*q0)/2;
    nextP[3][18] = P[3][18] + P[0][18]*SF[4] + P[1][18]*SF[3] + P[2][18]*SF[6] + P[10][18]*SPP[4] - P[11][18]*SPP[5] - (P[12][18]*q0)/2;
    nextP[3][19] = P[3][19] + P[0][19]*SF[4] + P[1][19]*SF[3] + P[2][19]*SF[6] + P[10][19]*SPP[4] - P[11][19]*SPP[5] - (P[12][19]*q0)/2;
    nextP[3][20] = P[3][20] + P[0][20]*SF[4] + P[1][20]*SF[3] + P[2][20]*SF[6] + P[10][20]*SPP[4] - P[11][20]*SPP[5] - (P[12][20]*q0)/2;
    nextP[4][0] = P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2] + SF[6]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]) + SPP[7]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]) + SPP[6]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) + SPP[5]*(P[4][10] + P[0][10]*SF[2] + P[1][10]*SF[0] + P[2][10]*SPP[0] - P[3][10]*SPP[2]) + SPP[4]*(P[4][11] + P[0][11]*SF[2] + P[1][11]*SF[0] + P[2][11]*SPP[0] - P[3][11]*SPP[2]) + SPP[3]*(P[4][12] + P[0][12]*SF[2] + P[1][12]*SF[0] + P[2][12]*SPP[0] - P[3][12]*SPP[2]);
    nextP[4][1] = P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2] + SF[5]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) + SF[4]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]) + SPP[7]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) + SPP[3]*(P[4][11] + P[0][11]*SF[2] + P[1][11]*SF[0] + P[2][11]*SPP[0] - P[3][11]*SPP[2]) - SPP[4]*(P[4][12] + P[0][12]*SF[2] + P[1][12]*SF[0] + P[2][12]*SPP[0] - P[3][12]*SPP[2]) - (q0*(P[4][10] + P[0][10]*SF[2] + P[1][10]*SF[0] + P[2][10]*SPP[0] - P[3][10]*SPP[2]))/2;
    nextP[4][2] = P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2] + SF[3]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) + SF[5]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) + SPP[6]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]) - SPP[3]*(P[4][10] + P[0][10]*SF[2] + P[1][10]*SF[0] + P[2][10]*SPP[0] - P[3][10]*SPP[2]) + SPP[5]*(P[4][12] + P[0][12]*SF[2] + P[1][12]*SF[0] + P[2][12]*SPP[0] - P[3][12]*SPP[2]) - (q0*(P[4][11] + P[0][11]*SF[2] + P[1][11]*SF[0] + P[2][11]*SPP[0] - P[3][11]*SPP[2]))/2;
    nextP[4][3] = P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2] + SF[4]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) + SF[3]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]) + SF[6]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]) + SPP[4]*(P[4][10] + P[0][10]*SF[2] + P[1][10]*SF[0] + P[2][10]*SPP[0] - P[3][10]*SPP[2]) - SPP[5]*(P[4][11] + P[0][11]*SF[2] + P[1][11]*SF[0] + P[2][11]*SPP[0] - P[3][11]*SPP[2]) - (q0*(P[4][12] + P[0][12]*SF[2] + P[1][12]*SF[0] + P[2][12]*SPP[0] - P[3][12]*SPP[2]))/2;
    nextP[4][4] = P[4][4] + P[0][4]*SF[2] + P[1][4]*SF[0] + P[2][4]*SPP[0] - P[3][4]*SPP[2] + dvyCov*sq(SG[7] - 2*q0*q3) + dvzCov*sq(SG[6] + 2*q0*q2) + SF[2]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) + SF[0]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]) + SPP[0]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]) - SPP[2]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) + dvxCov*sq(SG[1] + SG[2] - SG[3] - SG[4]);
    nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[2] + P[1][5]*SF[0] + P[2][5]*SPP[0] - P[3][5]*SPP[2] + SF[1]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) + SF[0]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]) + SF[2]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) - SPP[0]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]);
    nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[2] + P[1][6]*SF[0] + P[2][6]*SPP[0] - P[3][6]*SPP[2] + SF[1]*(P[4][1] + P[0][1]*SF[2] + P[1][1]*SF[0] + P[2][1]*SPP[0] - P[3][1]*SPP[2]) + SF[0]*(P[4][3] + P[0][3]*SF[2] + P[1][3]*SF[0] + P[2][3]*SPP[0] - P[3][3]*SPP[2]) + SPP[0]*(P[4][0] + P[0][0]*SF[2] + P[1][0]*SF[0] + P[2][0]*SPP[0] - P[3][0]*SPP[2]) - SPP[1]*(P[4][2] + P[0][2]*SF[2] + P[1][2]*SF[0] + P[2][2]*SPP[0] - P[3][2]*SPP[2]);
    nextP[4][7] = P[4][7] + P[0][7]*SF[2] + P[1][7]*SF[0] + P[2][7]*SPP[0] - P[3][7]*SPP[2] + dt*(P[4][4] + P[0][4]*SF[2] + P[1][4]*SF[0] + P[2][4]*SPP[0] - P[3][4]*SPP[2]);
    nextP[4][8] = P[4][8] + P[0][8]*SF[2] + P[1][8]*SF[0] + P[2][8]*SPP[0] - P[3][8]*SPP[2] + dt*(P[4][5] + P[0][5]*SF[2] + P[1][5]*SF[0] + P[2][5]*SPP[0] - P[3][5]*SPP[2]);
    nextP[4][9] = P[4][9] + P[0][9]*SF[2] + P[1][9]*SF[0] + P[2][9]*SPP[0] - P[3][9]*SPP[2] + dt*(P[4][6] + P[0][6]*SF[2] + P[1][6]*SF[0] + P[2][6]*SPP[0] - P[3][6]*SPP[2]);
    nextP[4][10] = P[4][10] + P[0][10]*SF[2] + P[1][10]*SF[0] + P[2][10]*SPP[0] - P[3][10]*SPP[2];
    nextP[4][11] = P[4][11] + P[0][11]*SF[2] + P[1][11]*SF[0] + P[2][11]*SPP[0] - P[3][11]*SPP[2];
    nextP[4][12] = P[4][12] + P[0][12]*SF[2] + P[1][12]*SF[0] + P[2][12]*SPP[0] - P[3][12]*SPP[2];
    nextP[4][13] = P[4][13] + P[0][13]*SF[2] + P[1][13]*SF[0] + P[2][13]*SPP[0] - P[3][13]*SPP[2];
    nextP[4][14] = P[4][14] + P[0][14]*SF[2] + P[1][14]*SF[0] + P[2][14]*SPP[0] - P[3][14]*SPP[2];
    nextP[4][15] = P[4][15] + P[0][15]*SF[2] + P[1][15]*SF[0] + P[2][15]*SPP[0] - P[3][15]*SPP[2];
    nextP[4][16] = P[4][16] + P[0][16]*SF[2] + P[1][16]*SF[0] + P[2][16]*SPP[0] - P[3][16]*SPP[2];
    nextP[4][17] = P[4][17] + P[0][17]*SF[2] + P[1][17]*SF[0] + P[2][17]*SPP[0] - P[3][17]*SPP[2];
    nextP[4][18] = P[4][18] + P[0][18]*SF[2] + P[1][18]*SF[0] + P[2][18]*SPP[0] - P[3][18]*SPP[2];
    nextP[4][19] = P[4][19] + P[0][19]*SF[2] + P[1][19]*SF[0] + P[2][19]*SPP[0] - P[3][19]*SPP[2];
    nextP[4][20] = P[4][20] + P[0][20]*SF[2] + P[1][20]*SF[0] + P[2][20]*SPP[0] - P[3][20]*SPP[2];
    nextP[5][0] = P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0] + SF[6]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) + SPP[7]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]) + SPP[6]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]) + SPP[5]*(P[5][10] + P[0][10]*SF[1] + P[2][10]*SF[0] + P[3][10]*SF[2] - P[1][10]*SPP[0]) + SPP[4]*(P[5][11] + P[0][11]*SF[1] + P[2][11]*SF[0] + P[3][11]*SF[2] - P[1][11]*SPP[0]) + SPP[3]*(P[5][12] + P[0][12]*SF[1] + P[2][12]*SF[0] + P[3][12]*SF[2] - P[1][12]*SPP[0]);
    nextP[5][1] = P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0] + SF[5]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) + SF[4]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]) + SPP[7]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]) + SPP[3]*(P[5][11] + P[0][11]*SF[1] + P[2][11]*SF[0] + P[3][11]*SF[2] - P[1][11]*SPP[0]) - SPP[4]*(P[5][12] + P[0][12]*SF[1] + P[2][12]*SF[0] + P[3][12]*SF[2] - P[1][12]*SPP[0]) - (q0*(P[5][10] + P[0][10]*SF[1] + P[2][10]*SF[0] + P[3][10]*SF[2] - P[1][10]*SPP[0]))/2;
    nextP[5][2] = P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0] + SF[3]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) + SF[5]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]) + SPP[6]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) - SPP[3]*(P[5][10] + P[0][10]*SF[1] + P[2][10]*SF[0] + P[3][10]*SF[2] - P[1][10]*SPP[0]) + SPP[5]*(P[5][12] + P[0][12]*SF[1] + P[2][12]*SF[0] + P[3][12]*SF[2] - P[1][12]*SPP[0]) - (q0*(P[5][11] + P[0][11]*SF[1] + P[2][11]*SF[0] + P[3][11]*SF[2] - P[1][11]*SPP[0]))/2;
    nextP[5][3] = P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0] + SF[4]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) + SF[3]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) + SF[6]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]) + SPP[4]*(P[5][10] + P[0][10]*SF[1] + P[2][10]*SF[0] + P[3][10]*SF[2] - P[1][10]*SPP[0]) - SPP[5]*(P[5][11] + P[0][11]*SF[1] + P[2][11]*SF[0] + P[3][11]*SF[2] - P[1][11]*SPP[0]) - (q0*(P[5][12] + P[0][12]*SF[1] + P[2][12]*SF[0] + P[3][12]*SF[2] - P[1][12]*SPP[0]))/2;
    nextP[5][4] = P[5][4] + SQ[2] + P[0][4]*SF[1] + P[2][4]*SF[0] + P[3][4]*SF[2] - P[1][4]*SPP[0] + SF[2]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) + SF[0]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) + SPP[0]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]) - SPP[2]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]);
    nextP[5][5] = P[5][5] + P[0][5]*SF[1] + P[2][5]*SF[0] + P[3][5]*SF[2] - P[1][5]*SPP[0] + dvxCov*sq(SG[7] + 2*q0*q3) + dvzCov*sq(SG[5] - 2*q0*q1) + SF[1]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) + SF[0]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]) + SF[2]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]) - SPP[0]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) + dvyCov*sq(SG[1] - SG[2] + SG[3] - SG[4]);
    nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[1] + P[2][6]*SF[0] + P[3][6]*SF[2] - P[1][6]*SPP[0] + SF[1]*(P[5][1] + P[0][1]*SF[1] + P[2][1]*SF[0] + P[3][1]*SF[2] - P[1][1]*SPP[0]) + SF[0]*(P[5][3] + P[0][3]*SF[1] + P[2][3]*SF[0] + P[3][3]*SF[2] - P[1][3]*SPP[0]) + SPP[0]*(P[5][0] + P[0][0]*SF[1] + P[2][0]*SF[0] + P[3][0]*SF[2] - P[1][0]*SPP[0]) - SPP[1]*(P[5][2] + P[0][2]*SF[1] + P[2][2]*SF[0] + P[3][2]*SF[2] - P[1][2]*SPP[0]);
    nextP[5][7] = P[5][7] + P[0][7]*SF[1] + P[2][7]*SF[0] + P[3][7]*SF[2] - P[1][7]*SPP[0] + dt*(P[5][4] + P[0][4]*SF[1] + P[2][4]*SF[0] + P[3][4]*SF[2] - P[1][4]*SPP[0]);
    nextP[5][8] = P[5][8] + P[0][8]*SF[1] + P[2][8]*SF[0] + P[3][8]*SF[2] - P[1][8]*SPP[0] + dt*(P[5][5] + P[0][5]*SF[1] + P[2][5]*SF[0] + P[3][5]*SF[2] - P[1][5]*SPP[0]);
    nextP[5][9] = P[5][9] + P[0][9]*SF[1] + P[2][9]*SF[0] + P[3][9]*SF[2] - P[1][9]*SPP[0] + dt*(P[5][6] + P[0][6]*SF[1] + P[2][6]*SF[0] + P[3][6]*SF[2] - P[1][6]*SPP[0]);
    nextP[5][10] = P[5][10] + P[0][10]*SF[1] + P[2][10]*SF[0] + P[3][10]*SF[2] - P[1][10]*SPP[0];
    nextP[5][11] = P[5][11] + P[0][11]*SF[1] + P[2][11]*SF[0] + P[3][11]*SF[2] - P[1][11]*SPP[0];
    nextP[5][12] = P[5][12] + P[0][12]*SF[1] + P[2][12]*SF[0] + P[3][12]*SF[2] - P[1][12]*SPP[0];
    nextP[5][13] = P[5][13] + P[0][13]*SF[1] + P[2][13]*SF[0] + P[3][13]*SF[2] - P[1][13]*SPP[0];
    nextP[5][14] = P[5][14] + P[0][14]*SF[1] + P[2][14]*SF[0] + P[3][14]*SF[2] - P[1][14]*SPP[0];
    nextP[5][15] = P[5][15] + P[0][15]*SF[1] + P[2][15]*SF[0] + P[3][15]*SF[2] - P[1][15]*SPP[0];
    nextP[5][16] = P[5][16] + P[0][16]*SF[1] + P[2][16]*SF[0] + P[3][16]*SF[2] - P[1][16]*SPP[0];
    nextP[5][17] = P[5][17] + P[0][17]*SF[1] + P[2][17]*SF[0] + P[3][17]*SF[2] - P[1][17]*SPP[0];
    nextP[5][18] = P[5][18] + P[0][18]*SF[1] + P[2][18]*SF[0] + P[3][18]*SF[2] - P[1][18]*SPP[0];
    nextP[5][19] = P[5][19] + P[0][19]*SF[1] + P[2][19]*SF[0] + P[3][19]*SF[2] - P[1][19]*SPP[0];
    nextP[5][20] = P[5][20] + P[0][20]*SF[1] + P[2][20]*SF[0] + P[3][20]*SF[2] - P[1][20]*SPP[0];
    nextP[6][0] = P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1] + SF[6]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]) + SPP[7]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) + SPP[6]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]) + SPP[5]*(P[6][10] + P[1][10]*SF[1] + P[3][10]*SF[0] + P[0][10]*SPP[0] - P[2][10]*SPP[1]) + SPP[4]*(P[6][11] + P[1][11]*SF[1] + P[3][11]*SF[0] + P[0][11]*SPP[0] - P[2][11]*SPP[1]) + SPP[3]*(P[6][12] + P[1][12]*SF[1] + P[3][12]*SF[0] + P[0][12]*SPP[0] - P[2][12]*SPP[1]);
    nextP[6][1] = P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1] + SF[5]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) + SF[4]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) + SPP[7]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]) + SPP[3]*(P[6][11] + P[1][11]*SF[1] + P[3][11]*SF[0] + P[0][11]*SPP[0] - P[2][11]*SPP[1]) - SPP[4]*(P[6][12] + P[1][12]*SF[1] + P[3][12]*SF[0] + P[0][12]*SPP[0] - P[2][12]*SPP[1]) - (q0*(P[6][10] + P[1][10]*SF[1] + P[3][10]*SF[0] + P[0][10]*SPP[0] - P[2][10]*SPP[1]))/2;
    nextP[6][2] = P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1] + SF[3]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) + SF[5]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]) + SPP[6]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]) - SPP[3]*(P[6][10] + P[1][10]*SF[1] + P[3][10]*SF[0] + P[0][10]*SPP[0] - P[2][10]*SPP[1]) + SPP[5]*(P[6][12] + P[1][12]*SF[1] + P[3][12]*SF[0] + P[0][12]*SPP[0] - P[2][12]*SPP[1]) - (q0*(P[6][11] + P[1][11]*SF[1] + P[3][11]*SF[0] + P[0][11]*SPP[0] - P[2][11]*SPP[1]))/2;
    nextP[6][3] = P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1] + SF[4]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) + SF[3]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]) + SF[6]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) + SPP[4]*(P[6][10] + P[1][10]*SF[1] + P[3][10]*SF[0] + P[0][10]*SPP[0] - P[2][10]*SPP[1]) - SPP[5]*(P[6][11] + P[1][11]*SF[1] + P[3][11]*SF[0] + P[0][11]*SPP[0] - P[2][11]*SPP[1]) - (q0*(P[6][12] + P[1][12]*SF[1] + P[3][12]*SF[0] + P[0][12]*SPP[0] - P[2][12]*SPP[1]))/2;
    nextP[6][4] = P[6][4] + SQ[1] + P[1][4]*SF[1] + P[3][4]*SF[0] + P[0][4]*SPP[0] - P[2][4]*SPP[1] + SF[2]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) + SF[0]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]) + SPP[0]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) - SPP[2]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]);
    nextP[6][5] = P[6][5] + SQ[0] + P[1][5]*SF[1] + P[3][5]*SF[0] + P[0][5]*SPP[0] - P[2][5]*SPP[1] + SF[1]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) + SF[0]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) + SF[2]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]) - SPP[0]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]);
    nextP[6][6] = P[6][6] + P[1][6]*SF[1] + P[3][6]*SF[0] + P[0][6]*SPP[0] - P[2][6]*SPP[1] + dvxCov*sq(SG[6] - 2*q0*q2) + dvyCov*sq(SG[5] + 2*q0*q1) + SF[1]*(P[6][1] + P[1][1]*SF[1] + P[3][1]*SF[0] + P[0][1]*SPP[0] - P[2][1]*SPP[1]) + SF[0]*(P[6][3] + P[1][3]*SF[1] + P[3][3]*SF[0] + P[0][3]*SPP[0] - P[2][3]*SPP[1]) + SPP[0]*(P[6][0] + P[1][0]*SF[1] + P[3][0]*SF[0] + P[0][0]*SPP[0] - P[2][0]*SPP[1]) - SPP[1]*(P[6][2] + P[1][2]*SF[1] + P[3][2]*SF[0] + P[0][2]*SPP[0] - P[2][2]*SPP[1]) + dvzCov*sq(SG[1] - SG[2] - SG[3] + SG[4]);
    nextP[6][7] = P[6][7] + P[1][7]*SF[1] + P[3][7]*SF[0] + P[0][7]*SPP[0] - P[2][7]*SPP[1] + dt*(P[6][4] + P[1][4]*SF[1] + P[3][4]*SF[0] + P[0][4]*SPP[0] - P[2][4]*SPP[1]);
    nextP[6][8] = P[6][8] + P[1][8]*SF[1] + P[3][8]*SF[0] + P[0][8]*SPP[0] - P[2][8]*SPP[1] + dt*(P[6][5] + P[1][5]*SF[1] + P[3][5]*SF[0] + P[0][5]*SPP[0] - P[2][5]*SPP[1]);
    nextP[6][9] = P[6][9] + P[1][9]*SF[1] + P[3][9]*SF[0] + P[0][9]*SPP[0] - P[2][9]*SPP[1] + dt*(P[6][6] + P[1][6]*SF[1] + P[3][6]*SF[0] + P[0][6]*SPP[0] - P[2][6]*SPP[1]);
    nextP[6][10] = P[6][10] + P[1][10]*SF[1] + P[3][10]*SF[0] + P[0][10]*SPP[0] - P[2][10]*SPP[1];
    nextP[6][11] = P[6][11] + P[1][11]*SF[1] + P[3][11]*SF[0] + P[0][11]*SPP[0] - P[2][11]*SPP[1];
    nextP[6][12] = P[6][12] + P[1][12]*SF[1] + P[3][12]*SF[0] + P[0][12]*SPP[0] - P[2][12]*SPP[1];
    nextP[6][13] = P[6][13] + P[1][13]*SF[1] + P[3][13]*SF[0] + P[0][13]*SPP[0] - P[2][13]*SPP[1];
    nextP[6][14] = P[6][14] + P[1][14]*SF[1] + P[3][14]*SF[0] + P[0][14]*SPP[0] - P[2][14]*SPP[1];
    nextP[6][15] = P[6][15] + P[1][15]*SF[1] + P[3][15]*SF[0] + P[0][15]*SPP[0] - P[2][15]*SPP[1];
    nextP[6][16] = P[6][16] + P[1][16]*SF[1] + P[3][16]*SF[0] + P[0][16]*SPP[0] - P[2][16]*SPP[1];
    nextP[6][17] = P[6][17] + P[1][17]*SF[1] + P[3][17]*SF[0] + P[0][17]*SPP[0] - P[2][17]*SPP[1];
    nextP[6][18] = P[6][18] + P[1][18]*SF[1] + P[3][18]*SF[0] + P[0][18]*SPP[0] - P[2][18]*SPP[1];
    nextP[6][19] = P[6][19] + P[1][19]*SF[1] + P[3][19]*SF[0] + P[0][19]*SPP[0] - P[2][19]*SPP[1];
    nextP[6][20] = P[6][20] + P[1][20]*SF[1] + P[3][20]*SF[0] + P[0][20]*SPP[0] - P[2][20]*SPP[1];
    nextP[7][0] = P[7][0] + P[4][0]*dt + SF[6]*(P[7][1] + P[4][1]*dt) + SPP[7]*(P[7][2] + P[4][2]*dt) + SPP[6]*(P[7][3] + P[4][3]*dt) + SPP[5]*(P[7][10] + P[4][10]*dt) + SPP[4]*(P[7][11] + P[4][11]*dt) + SPP[3]*(P[7][12] + P[4][12]*dt);
    nextP[7][1] = P[7][1] + P[4][1]*dt + SF[5]*(P[7][0] + P[4][0]*dt) + SF[4]*(P[7][2] + P[4][2]*dt) + SPP[7]*(P[7][3] + P[4][3]*dt) + SPP[3]*(P[7][11] + P[4][11]*dt) - SPP[4]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][10] + P[4][10]*dt))/2;
    nextP[7][2] = P[7][2] + P[4][2]*dt + SF[3]*(P[7][0] + P[4][0]*dt) + SF[5]*(P[7][3] + P[4][3]*dt) + SPP[6]*(P[7][1] + P[4][1]*dt) - SPP[3]*(P[7][10] + P[4][10]*dt) + SPP[5]*(P[7][12] + P[4][12]*dt) - (q0*(P[7][11] + P[4][11]*dt))/2;
    nextP[7][3] = P[7][3] + P[4][3]*dt + SF[4]*(P[7][0] + P[4][0]*dt) + SF[3]*(P[7][1] + P[4][1]*dt) + SF[6]*(P[7][2] + P[4][2]*dt) + SPP[4]*(P[7][10] + P[4][10]*dt) - SPP[5]*(P[7][11] + P[4][11]*dt) - (q0*(P[7][12] + P[4][12]*dt))/2;
    nextP[7][4] = P[7][4] + P[4][4]*dt + SF[0]*(P[7][1] + P[4][1]*dt) + SF[2]*(P[7][0] + P[4][0]*dt) + SPP[0]*(P[7][2] + P[4][2]*dt) - SPP[2]*(P[7][3] + P[4][3]*dt);
    nextP[7][5] = P[7][5] + P[4][5]*dt + SF[1]*(P[7][0] + P[4][0]*dt) + SF[0]*(P[7][2] + P[4][2]*dt) + SF[2]*(P[7][3] + P[4][3]*dt) - SPP[0]*(P[7][1] + P[4][1]*dt);
    nextP[7][6] = P[7][6] + P[4][6]*dt + SF[1]*(P[7][1] + P[4][1]*dt) + SF[0]*(P[7][3] + P[4][3]*dt) + SPP[0]*(P[7][0] + P[4][0]*dt) - SPP[1]*(P[7][2] + P[4][2]*dt);
    nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
    nextP[7][10] = P[7][10] + P[4][10]*dt;
    nextP[7][11] = P[7][11] + P[4][11]*dt;
    nextP[7][12] = P[7][12] + P[4][12]*dt;
    nextP[7][13] = P[7][13] + P[4][13]*dt;
    nextP[7][14] = P[7][14] + P[4][14]*dt;
    nextP[7][15] = P[7][15] + P[4][15]*dt;
    nextP[7][16] = P[7][16] + P[4][16]*dt;
    nextP[7][17] = P[7][17] + P[4][17]*dt;
    nextP[7][18] = P[7][18] + P[4][18]*dt;
    nextP[7][19] = P[7][19] + P[4][19]*dt;
    nextP[7][20] = P[7][20] + P[4][20]*dt;
    nextP[8][0] = P[8][0] + P[5][0]*dt + SF[6]*(P[8][1] + P[5][1]*dt) + SPP[7]*(P[8][2] + P[5][2]*dt) + SPP[6]*(P[8][3] + P[5][3]*dt) + SPP[5]*(P[8][10] + P[5][10]*dt) + SPP[4]*(P[8][11] + P[5][11]*dt) + SPP[3]*(P[8][12] + P[5][12]*dt);
    nextP[8][1] = P[8][1] + P[5][1]*dt + SF[5]*(P[8][0] + P[5][0]*dt) + SF[4]*(P[8][2] + P[5][2]*dt) + SPP[7]*(P[8][3] + P[5][3]*dt) + SPP[3]*(P[8][11] + P[5][11]*dt) - SPP[4]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][10] + P[5][10]*dt))/2;
    nextP[8][2] = P[8][2] + P[5][2]*dt + SF[3]*(P[8][0] + P[5][0]*dt) + SF[5]*(P[8][3] + P[5][3]*dt) + SPP[6]*(P[8][1] + P[5][1]*dt) - SPP[3]*(P[8][10] + P[5][10]*dt) + SPP[5]*(P[8][12] + P[5][12]*dt) - (q0*(P[8][11] + P[5][11]*dt))/2;
    nextP[8][3] = P[8][3] + P[5][3]*dt + SF[4]*(P[8][0] + P[5][0]*dt) + SF[3]*(P[8][1] + P[5][1]*dt) + SF[6]*(P[8][2] + P[5][2]*dt) + SPP[4]*(P[8][10] + P[5][10]*dt) - SPP[5]*(P[8][11] + P[5][11]*dt) - (q0*(P[8][12] + P[5][12]*dt))/2;
    nextP[8][4] = P[8][4] + P[5][4]*dt + SF[0]*(P[8][1] + P[5][1]*dt) + SF[2]*(P[8][0] + P[5][0]*dt) + SPP[0]*(P[8][2] + P[5][2]*dt) - SPP[2]*(P[8][3] + P[5][3]*dt);
    nextP[8][5] = P[8][5] + P[5][5]*dt + SF[1]*(P[8][0] + P[5][0]*dt) + SF[0]*(P[8][2] + P[5][2]*dt) + SF[2]*(P[8][3] + P[5][3]*dt) - SPP[0]*(P[8][1] + P[5][1]*dt);
    nextP[8][6] = P[8][6] + P[5][6]*dt + SF[1]*(P[8][1] + P[5][1]*dt) + SF[0]*(P[8][3] + P[5][3]*dt) + SPP[0]*(P[8][0] + P[5][0]*dt) - SPP[1]*(P[8][2] + P[5][2]*dt);
    nextP[8][7] = P[8][7] + P[5][7]*dt + dt*(P[8][4] + P[5][4]*dt);
    nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
    nextP[8][10] = P[8][10] + P[5][10]*dt;
    nextP[8][11] = P[8][11] + P[5][11]*dt;
    nextP[8][12] = P[8][12] + P[5][12]*dt;
    nextP[8][13] = P[8][13] + P[5][13]*dt;
    nextP[8][14] = P[8][14] + P[5][14]*dt;
    nextP[8][15] = P[8][15] + P[5][15]*dt;
    nextP[8][16] = P[8][16] + P[5][16]*dt;
    nextP[8][17] = P[8][17] + P[5][17]*dt;
    nextP[8][18] = P[8][18] + P[5][18]*dt;
    nextP[8][19] = P[8][19] + P[5][19]*dt;
    nextP[8][20] = P[8][20] + P[5][20]*dt;
    nextP[9][0] = P[9][0] + P[6][0]*dt + SF[6]*(P[9][1] + P[6][1]*dt) + SPP[7]*(P[9][2] + P[6][2]*dt) + SPP[6]*(P[9][3] + P[6][3]*dt) + SPP[5]*(P[9][10] + P[6][10]*dt) + SPP[4]*(P[9][11] + P[6][11]*dt) + SPP[3]*(P[9][12] + P[6][12]*dt);
    nextP[9][1] = P[9][1] + P[6][1]*dt + SF[5]*(P[9][0] + P[6][0]*dt) + SF[4]*(P[9][2] + P[6][2]*dt) + SPP[7]*(P[9][3] + P[6][3]*dt) + SPP[3]*(P[9][11] + P[6][11]*dt) - SPP[4]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][10] + P[6][10]*dt))/2;
    nextP[9][2] = P[9][2] + P[6][2]*dt + SF[3]*(P[9][0] + P[6][0]*dt) + SF[5]*(P[9][3] + P[6][3]*dt) + SPP[6]*(P[9][1] + P[6][1]*dt) - SPP[3]*(P[9][10] + P[6][10]*dt) + SPP[5]*(P[9][12] + P[6][12]*dt) - (q0*(P[9][11] + P[6][11]*dt))/2;
    nextP[9][3] = P[9][3] + P[6][3]*dt + SF[4]*(P[9][0] + P[6][0]*dt) + SF[3]*(P[9][1] + P[6][1]*dt) + SF[6]*(P[9][2] + P[6][2]*dt) + SPP[4]*(P[9][10] + P[6][10]*dt) - SPP[5]*(P[9][11] + P[6][11]*dt) - (q0*(P[9][12] + P[6][12]*dt))/2;
    nextP[9][4] = P[9][4] + P[6][4]*dt + SF[0]*(P[9][1] + P[6][1]*dt) + SF[2]*(P[9][0] + P[6][0]*dt) + SPP[0]*(P[9][2] + P[6][2]*dt) - SPP[2]*(P[9][3] + P[6][3]*dt);
    nextP[9][5] = P[9][5] + P[6][5]*dt + SF[1]*(P[9][0] + P[6][0]*dt) + SF[0]*(P[9][2] + P[6][2]*dt) + SF[2]*(P[9][3] + P[6][3]*dt) - SPP[0]*(P[9][1] + P[6][1]*dt);
    nextP[9][6] = P[9][6] + P[6][6]*dt + SF[1]*(P[9][1] + P[6][1]*dt) + SF[0]*(P[9][3] + P[6][3]*dt) + SPP[0]*(P[9][0] + P[6][0]*dt) - SPP[1]*(P[9][2] + P[6][2]*dt);
    nextP[9][7] = P[9][7] + P[6][7]*dt + dt*(P[9][4] + P[6][4]*dt);
    nextP[9][8] = P[9][8] + P[6][8]*dt + dt*(P[9][5] + P[6][5]*dt);
    nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
    nextP[9][10] = P[9][10] + P[6][10]*dt;
    nextP[9][11] = P[9][11] + P[6][11]*dt;
    nextP[9][12] = P[9][12] + P[6][12]*dt;
    nextP[9][13] = P[9][13] + P[6][13]*dt;
    nextP[9][14] = P[9][14] + P[6][14]*dt;
    nextP[9][15] = P[9][15] + P[6][15]*dt;
    nextP[9][16] = P[9][16] + P[6][16]*dt;
    nextP[9][17] = P[9][17] + P[6][17]*dt;
    nextP[9][18] = P[9][18] + P[6][18]*dt;
    nextP[9][19] = P[9][19] + P[6][19]*dt;
    nextP[9][20] = P[9][20] + P[6][20]*dt;
    nextP[10][0] = P[10][0] + P[10][1]*SF[6] + P[10][2]*SPP[7] + P[10][3]*SPP[6] + P[10][10]*SPP[5] + P[10][11]*SPP[4] + P[10][12]*SPP[3];
    nextP[10][1] = P[10][1] + P[10][0]*SF[5] + P[10][2]*SF[4] + P[10][3]*SPP[7] + P[10][11]*SPP[3] - P[10][12]*SPP[4] - (P[10][10]*q0)/2;
    nextP[10][2] = P[10][2] + P[10][0]*SF[3] + P[10][3]*SF[5] + P[10][1]*SPP[6] - P[10][10]*SPP[3] + P[10][12]*SPP[5] - (P[10][11]*q0)/2;
    nextP[10][3] = P[10][3] + P[10][0]*SF[4] + P[10][1]*SF[3] + P[10][2]*SF[6] + P[10][10]*SPP[4] - P[10][11]*SPP[5] - (P[10][12]*q0)/2;
    nextP[10][4] = P[10][4] + P[10][1]*SF[0] + P[10][0]*SF[2] + P[10][2]*SPP[0] - P[10][3]*SPP[2];
    nextP[10][5] = P[10][5] + P[10][0]*SF[1] + P[10][2]*SF[0] + P[10][3]*SF[2] - P[10][1]*SPP[0];
    nextP[10][6] = P[10][6] + P[10][1]*SF[1] + P[10][3]*SF[0] + P[10][0]*SPP[0] - P[10][2]*SPP[1];
    nextP[10][7] = P[10][7] + P[10][4]*dt;
    nextP[10][8] = P[10][8] + P[10][5]*dt;
    nextP[10][9] = P[10][9] + P[10][6]*dt;
    nextP[10][10] = P[10][10];
    nextP[10][11] = P[10][11];
    nextP[10][12] = P[10][12];
    nextP[10][13] = P[10][13];
    nextP[10][14] = P[10][14];
    nextP[10][15] = P[10][15];
    nextP[10][16] = P[10][16];
    nextP[10][17] = P[10][17];
    nextP[10][18] = P[10][18];
    nextP[10][19] = P[10][19];
    nextP[10][20] = P[10][20];
    nextP[11][0] = P[11][0] + P[11][1]*SF[6] + P[11][2]*SPP[7] + P[11][3]*SPP[6] + P[11][10]*SPP[5] + P[11][11]*SPP[4] + P[11][12]*SPP[3];
    nextP[11][1] = P[11][1] + P[11][0]*SF[5] + P[11][2]*SF[4] + P[11][3]*SPP[7] + P[11][11]*SPP[3] - P[11][12]*SPP[4] - (P[11][10]*q0)/2;
    nextP[11][2] = P[11][2] + P[11][0]*SF[3] + P[11][3]*SF[5] + P[11][1]*SPP[6] - P[11][10]*SPP[3] + P[11][12]*SPP[5] - (P[11][11]*q0)/2;
    nextP[11][3] = P[11][3] + P[11][0]*SF[4] + P[11][1]*SF[3] + P[11][2]*SF[6] + P[11][10]*SPP[4] - P[11][11]*SPP[5] - (P[11][12]*q0)/2;
    nextP[11][4] = P[11][4] + P[11][1]*SF[0] + P[11][0]*SF[2] + P[11][2]*SPP[0] - P[11][3]*SPP[2];
    nextP[11][5] = P[11][5] + P[11][0]*SF[1] + P[11][2]*SF[0] + P[11][3]*SF[2] - P[11][1]*SPP[0];
    nextP[11][6] = P[11][6] + P[11][1]*SF[1] + P[11][3]*SF[0] + P[11][0]*SPP[0] - P[11][2]*SPP[1];
    nextP[11][7] = P[11][7] + P[11][4]*dt;
    nextP[11][8] = P[11][8] + P[11][5]*dt;
    nextP[11][9] = P[11][9] + P[11][6]*dt;
    nextP[11][10] = P[11][10];
    nextP[11][11] = P[11][11];
    nextP[11][12] = P[11][12];
    nextP[11][13] = P[11][13];
    nextP[11][14] = P[11][14];
    nextP[11][15] = P[11][15];
    nextP[11][16] = P[11][16];
    nextP[11][17] = P[11][17];
    nextP[11][18] = P[11][18];
    nextP[11][19] = P[11][19];
    nextP[11][20] = P[11][20];
    nextP[12][0] = P[12][0] + P[12][1]*SF[6] + P[12][2]*SPP[7] + P[12][3]*SPP[6] + P[12][10]*SPP[5] + P[12][11]*SPP[4] + P[12][12]*SPP[3];
    nextP[12][1] = P[12][1] + P[12][0]*SF[5] + P[12][2]*SF[4] + P[12][3]*SPP[7] + P[12][11]*SPP[3] - P[12][12]*SPP[4] - (P[12][10]*q0)/2;
    nextP[12][2] = P[12][2] + P[12][0]*SF[3] + P[12][3]*SF[5] + P[12][1]*SPP[6] - P[12][10]*SPP[3] + P[12][12]*SPP[5] - (P[12][11]*q0)/2;
    nextP[12][3] = P[12][3] + P[12][0]*SF[4] + P[12][1]*SF[3] + P[12][2]*SF[6] + P[12][10]*SPP[4] - P[12][11]*SPP[5] - (P[12][12]*q0)/2;
    nextP[12][4] = P[12][4] + P[12][1]*SF[0] + P[12][0]*SF[2] + P[12][2]*SPP[0] - P[12][3]*SPP[2];
    nextP[12][5] = P[12][5] + P[12][0]*SF[1] + P[12][2]*SF[0] + P[12][3]*SF[2] - P[12][1]*SPP[0];
    nextP[12][6] = P[12][6] + P[12][1]*SF[1] + P[12][3]*SF[0] + P[12][0]*SPP[0] - P[12][2]*SPP[1];
    nextP[12][7] = P[12][7] + P[12][4]*dt;
    nextP[12][8] = P[12][8] + P[12][5]*dt;
    nextP[12][9] = P[12][9] + P[12][6]*dt;
    nextP[12][10] = P[12][10];
    nextP[12][11] = P[12][11];
    nextP[12][12] = P[12][12];
    nextP[12][13] = P[12][13];
    nextP[12][14] = P[12][14];
    nextP[12][15] = P[12][15];
    nextP[12][16] = P[12][16];
    nextP[12][17] = P[12][17];
    nextP[12][18] = P[12][18];
    nextP[12][19] = P[12][19];
    nextP[12][20] = P[12][20];
    nextP[13][0] = P[13][0] + P[13][1]*SF[6] + P[13][2]*SPP[7] + P[13][3]*SPP[6] + P[13][10]*SPP[5] + P[13][11]*SPP[4] + P[13][12]*SPP[3];
    nextP[13][1] = P[13][1] + P[13][0]*SF[5] + P[13][2]*SF[4] + P[13][3]*SPP[7] + P[13][11]*SPP[3] - P[13][12]*SPP[4] - (P[13][10]*q0)/2;
    nextP[13][2] = P[13][2] + P[13][0]*SF[3] + P[13][3]*SF[5] + P[13][1]*SPP[6] - P[13][10]*SPP[3] + P[13][12]*SPP[5] - (P[13][11]*q0)/2;
    nextP[13][3] = P[13][3] + P[13][0]*SF[4] + P[13][1]*SF[3] + P[13][2]*SF[6] + P[13][10]*SPP[4] - P[13][11]*SPP[5] - (P[13][12]*q0)/2;
    nextP[13][4] = P[13][4] + P[13][1]*SF[0] + P[13][0]*SF[2] + P[13][2]*SPP[0] - P[13][3]*SPP[2];
    nextP[13][5] = P[13][5] + P[13][0]*SF[1] + P[13][2]*SF[0] + P[13][3]*SF[2] - P[13][1]*SPP[0];
    nextP[13][6] = P[13][6] + P[13][1]*SF[1] + P[13][3]*SF[0] + P[13][0]*SPP[0] - P[13][2]*SPP[1];
    nextP[13][7] = P[13][7] + P[13][4]*dt;
    nextP[13][8] = P[13][8] + P[13][5]*dt;
    nextP[13][9] = P[13][9] + P[13][6]*dt;
    nextP[13][10] = P[13][10];
    nextP[13][11] = P[13][11];
    nextP[13][12] = P[13][12];
    nextP[13][13] = P[13][13];
    nextP[13][14] = P[13][14];
    nextP[13][15] = P[13][15];
    nextP[13][16] = P[13][16];
    nextP[13][17] = P[13][17];
    nextP[13][18] = P[13][18];
    nextP[13][19] = P[13][19];
    nextP[13][20] = P[13][20];
    nextP[14][0] = P[14][0] + P[14][1]*SF[6] + P[14][2]*SPP[7] + P[14][3]*SPP[6] + P[14][10]*SPP[5] + P[14][11]*SPP[4] + P[14][12]*SPP[3];
    nextP[14][1] = P[14][1] + P[14][0]*SF[5] + P[14][2]*SF[4] + P[14][3]*SPP[7] + P[14][11]*SPP[3] - P[14][12]*SPP[4] - (P[14][10]*q0)/2;
    nextP[14][2] = P[14][2] + P[14][0]*SF[3] + P[14][3]*SF[5] + P[14][1]*SPP[6] - P[14][10]*SPP[3] + P[14][12]*SPP[5] - (P[14][11]*q0)/2;
    nextP[14][3] = P[14][3] + P[14][0]*SF[4] + P[14][1]*SF[3] + P[14][2]*SF[6] + P[14][10]*SPP[4] - P[14][11]*SPP[5] - (P[14][12]*q0)/2;
    nextP[14][4] = P[14][4] + P[14][1]*SF[0] + P[14][0]*SF[2] + P[14][2]*SPP[0] - P[14][3]*SPP[2];
    nextP[14][5] = P[14][5] + P[14][0]*SF[1] + P[14][2]*SF[0] + P[14][3]*SF[2] - P[14][1]*SPP[0];
    nextP[14][6] = P[14][6] + P[14][1]*SF[1] + P[14][3]*SF[0] + P[14][0]*SPP[0] - P[14][2]*SPP[1];
    nextP[14][7] = P[14][7] + P[14][4]*dt;
    nextP[14][8] = P[14][8] + P[14][5]*dt;
    nextP[14][9] = P[14][9] + P[14][6]*dt;
    nextP[14][10] = P[14][10];
    nextP[14][11] = P[14][11];
    nextP[14][12] = P[14][12];
    nextP[14][13] = P[14][13];
    nextP[14][14] = P[14][14];
    nextP[14][15] = P[14][15];
    nextP[14][16] = P[14][16];
    nextP[14][17] = P[14][17];
    nextP[14][18] = P[14][18];
    nextP[14][19] = P[14][19];
    nextP[14][20] = P[14][20];
    nextP[15][0] = P[15][0] + P[15][1]*SF[6] + P[15][2]*SPP[7] + P[15][3]*SPP[6] + P[15][10]*SPP[5] + P[15][11]*SPP[4] + P[15][12]*SPP[3];
    nextP[15][1] = P[15][1] + P[15][0]*SF[5] + P[15][2]*SF[4] + P[15][3]*SPP[7] + P[15][11]*SPP[3] - P[15][12]*SPP[4] - (P[15][10]*q0)/2;
    nextP[15][2] = P[15][2] + P[15][0]*SF[3] + P[15][3]*SF[5] + P[15][1]*SPP[6] - P[15][10]*SPP[3] + P[15][12]*SPP[5] - (P[15][11]*q0)/2;
    nextP[15][3] = P[15][3] + P[15][0]*SF[4] + P[15][1]*SF[3] + P[15][2]*SF[6] + P[15][10]*SPP[4] - P[15][11]*SPP[5] - (P[15][12]*q0)/2;
    nextP[15][4] = P[15][4] + P[15][1]*SF[0] + P[15][0]*SF[2] + P[15][2]*SPP[0] - P[15][3]*SPP[2];
    nextP[15][5] = P[15][5] + P[15][0]*SF[1] + P[15][2]*SF[0] + P[15][3]*SF[2] - P[15][1]*SPP[0];
    nextP[15][6] = P[15][6] + P[15][1]*SF[1] + P[15][3]*SF[0] + P[15][0]*SPP[0] - P[15][2]*SPP[1];
    nextP[15][7] = P[15][7] + P[15][4]*dt;
    nextP[15][8] = P[15][8] + P[15][5]*dt;
    nextP[15][9] = P[15][9] + P[15][6]*dt;
    nextP[15][10] = P[15][10];
    nextP[15][11] = P[15][11];
    nextP[15][12] = P[15][12];
    nextP[15][13] = P[15][13];
    nextP[15][14] = P[15][14];
    nextP[15][15] = P[15][15];
    nextP[15][16] = P[15][16];
    nextP[15][17] = P[15][17];
    nextP[15][18] = P[15][18];
    nextP[15][19] = P[15][19];
    nextP[15][20] = P[15][20];
    nextP[16][0] = P[16][0] + P[16][1]*SF[6] + P[16][2]*SPP[7] + P[16][3]*SPP[6] + P[16][10]*SPP[5] + P[16][11]*SPP[4] + P[16][12]*SPP[3];
    nextP[16][1] = P[16][1] + P[16][0]*SF[5] + P[16][2]*SF[4] + P[16][3]*SPP[7] + P[16][11]*SPP[3] - P[16][12]*SPP[4] - (P[16][10]*q0)/2;
    nextP[16][2] = P[16][2] + P[16][0]*SF[3] + P[16][3]*SF[5] + P[16][1]*SPP[6] - P[16][10]*SPP[3] + P[16][12]*SPP[5] - (P[16][11]*q0)/2;
    nextP[16][3] = P[16][3] + P[16][0]*SF[4] + P[16][1]*SF[3] + P[16][2]*SF[6] + P[16][10]*SPP[4] - P[16][11]*SPP[5] - (P[16][12]*q0)/2;
    nextP[16][4] = P[16][4] + P[16][1]*SF[0] + P[16][0]*SF[2] + P[16][2]*SPP[0] - P[16][3]*SPP[2];
    nextP[16][5] = P[16][5] + P[16][0]*SF[1] + P[16][2]*SF[0] + P[16][3]*SF[2] - P[16][1]*SPP[0];
    nextP[16][6] = P[16][6] + P[16][1]*SF[1] + P[16][3]*SF[0] + P[16][0]*SPP[0] - P[16][2]*SPP[1];
    nextP[16][7] = P[16][7] + P[16][4]*dt;
    nextP[16][8] = P[16][8] + P[16][5]*dt;
    nextP[16][9] = P[16][9] + P[16][6]*dt;
    nextP[16][10] = P[16][10];
    nextP[16][11] = P[16][11];
    nextP[16][12] = P[16][12];
    nextP[16][13] = P[16][13];
    nextP[16][14] = P[16][14];
    nextP[16][15] = P[16][15];
    nextP[16][16] = P[16][16];
    nextP[16][17] = P[16][17];
    nextP[16][18] = P[16][18];
    nextP[16][19] = P[16][19];
    nextP[16][20] = P[16][20];
    nextP[17][0] = P[17][0] + P[17][1]*SF[6] + P[17][2]*SPP[7] + P[17][3]*SPP[6] + P[17][10]*SPP[5] + P[17][11]*SPP[4] + P[17][12]*SPP[3];
    nextP[17][1] = P[17][1] + P[17][0]*SF[5] + P[17][2]*SF[4] + P[17][3]*SPP[7] + P[17][11]*SPP[3] - P[17][12]*SPP[4] - (P[17][10]*q0)/2;
    nextP[17][2] = P[17][2] + P[17][0]*SF[3] + P[17][3]*SF[5] + P[17][1]*SPP[6] - P[17][10]*SPP[3] + P[17][12]*SPP[5] - (P[17][11]*q0)/2;
    nextP[17][3] = P[17][3] + P[17][0]*SF[4] + P[17][1]*SF[3] + P[17][2]*SF[6] + P[17][10]*SPP[4] - P[17][11]*SPP[5] - (P[17][12]*q0)/2;
    nextP[17][4] = P[17][4] + P[17][1]*SF[0] + P[17][0]*SF[2] + P[17][2]*SPP[0] - P[17][3]*SPP[2];
    nextP[17][5] = P[17][5] + P[17][0]*SF[1] + P[17][2]*SF[0] + P[17][3]*SF[2] - P[17][1]*SPP[0];
    nextP[17][6] = P[17][6] + P[17][1]*SF[1] + P[17][3]*SF[0] + P[17][0]*SPP[0] - P[17][2]*SPP[1];
    nextP[17][7] = P[17][7] + P[17][4]*dt;
    nextP[17][8] = P[17][8] + P[17][5]*dt;
    nextP[17][9] = P[17][9] + P[17][6]*dt;
    nextP[17][10] = P[17][10];
    nextP[17][11] = P[17][11];
    nextP[17][12] = P[17][12];
    nextP[17][13] = P[17][13];
    nextP[17][14] = P[17][14];
    nextP[17][15] = P[17][15];
    nextP[17][16] = P[17][16];
    nextP[17][17] = P[17][17];
    nextP[17][18] = P[17][18];
    nextP[17][19] = P[17][19];
    nextP[17][20] = P[17][20];
    nextP[18][0] = P[18][0] + P[18][1]*SF[6] + P[18][2]*SPP[7] + P[18][3]*SPP[6] + P[18][10]*SPP[5] + P[18][11]*SPP[4] + P[18][12]*SPP[3];
    nextP[18][1] = P[18][1] + P[18][0]*SF[5] + P[18][2]*SF[4] + P[18][3]*SPP[7] + P[18][11]*SPP[3] - P[18][12]*SPP[4] - (P[18][10]*q0)/2;
    nextP[18][2] = P[18][2] + P[18][0]*SF[3] + P[18][3]*SF[5] + P[18][1]*SPP[6] - P[18][10]*SPP[3] + P[18][12]*SPP[5] - (P[18][11]*q0)/2;
    nextP[18][3] = P[18][3] + P[18][0]*SF[4] + P[18][1]*SF[3] + P[18][2]*SF[6] + P[18][10]*SPP[4] - P[18][11]*SPP[5] - (P[18][12]*q0)/2;
    nextP[18][4] = P[18][4] + P[18][1]*SF[0] + P[18][0]*SF[2] + P[18][2]*SPP[0] - P[18][3]*SPP[2];
    nextP[18][5] = P[18][5] + P[18][0]*SF[1] + P[18][2]*SF[0] + P[18][3]*SF[2] - P[18][1]*SPP[0];
    nextP[18][6] = P[18][6] + P[18][1]*SF[1] + P[18][3]*SF[0] + P[18][0]*SPP[0] - P[18][2]*SPP[1];
    nextP[18][7] = P[18][7] + P[18][4]*dt;
    nextP[18][8] = P[18][8] + P[18][5]*dt;
    nextP[18][9] = P[18][9] + P[18][6]*dt;
    nextP[18][10] = P[18][10];
    nextP[18][11] = P[18][11];
    nextP[18][12] = P[18][12];
    nextP[18][13] = P[18][13];
    nextP[18][14] = P[18][14];
    nextP[18][15] = P[18][15];
    nextP[18][16] = P[18][16];
    nextP[18][17] = P[18][17];
    nextP[18][18] = P[18][18];
    nextP[18][19] = P[18][19];
    nextP[18][20] = P[18][20];
    nextP[19][0] = P[19][0] + P[19][1]*SF[6] + P[19][2]*SPP[7] + P[19][3]*SPP[6] + P[19][10]*SPP[5] + P[19][11]*SPP[4] + P[19][12]*SPP[3];
    nextP[19][1] = P[19][1] + P[19][0]*SF[5] + P[19][2]*SF[4] + P[19][3]*SPP[7] + P[19][11]*SPP[3] - P[19][12]*SPP[4] - (P[19][10]*q0)/2;
    nextP[19][2] = P[19][2] + P[19][0]*SF[3] + P[19][3]*SF[5] + P[19][1]*SPP[6] - P[19][10]*SPP[3] + P[19][12]*SPP[5] - (P[19][11]*q0)/2;
    nextP[19][3] = P[19][3] + P[19][0]*SF[4] + P[19][1]*SF[3] + P[19][2]*SF[6] + P[19][10]*SPP[4] - P[19][11]*SPP[5] - (P[19][12]*q0)/2;
    nextP[19][4] = P[19][4] + P[19][1]*SF[0] + P[19][0]*SF[2] + P[19][2]*SPP[0] - P[19][3]*SPP[2];
    nextP[19][5] = P[19][5] + P[19][0]*SF[1] + P[19][2]*SF[0] + P[19][3]*SF[2] - P[19][1]*SPP[0];
    nextP[19][6] = P[19][6] + P[19][1]*SF[1] + P[19][3]*SF[0] + P[19][0]*SPP[0] - P[19][2]*SPP[1];
    nextP[19][7] = P[19][7] + P[19][4]*dt;
    nextP[19][8] = P[19][8] + P[19][5]*dt;
    nextP[19][9] = P[19][9] + P[19][6]*dt;
    nextP[19][10] = P[19][10];
    nextP[19][11] = P[19][11];
    nextP[19][12] = P[19][12];
    nextP[19][13] = P[19][13];
    nextP[19][14] = P[19][14];
    nextP[19][15] = P[19][15];
    nextP[19][16] = P[19][16];
    nextP[19][17] = P[19][17];
    nextP[19][18] = P[19][18];
    nextP[19][19] = P[19][19];
    nextP[19][20] = P[19][20];
    nextP[20][0] = P[20][0] + P[20][1]*SF[6] + P[20][2]*SPP[7] + P[20][3]*SPP[6] + P[20][10]*SPP[5] + P[20][11]*SPP[4] + P[20][12]*SPP[3];
    nextP[20][1] = P[20][1] + P[20][0]*SF[5] + P[20][2]*SF[4] + P[20][3]*SPP[7] + P[20][11]*SPP[3] - P[20][12]*SPP[4] - (P[20][10]*q0)/2;
    nextP[20][2] = P[20][2] + P[20][0]*SF[3] + P[20][3]*SF[5] + P[20][1]*SPP[6] - P[20][10]*SPP[3] + P[20][12]*SPP[5] - (P[20][11]*q0)/2;
    nextP[20][3] = P[20][3] + P[20][0]*SF[4] + P[20][1]*SF[3] + P[20][2]*SF[6] + P[20][10]*SPP[4] - P[20][11]*SPP[5] - (P[20][12]*q0)/2;
    nextP[20][4] = P[20][4] + P[20][1]*SF[0] + P[20][0]*SF[2] + P[20][2]*SPP[0] - P[20][3]*SPP[2];
    nextP[20][5] = P[20][5] + P[20][0]*SF[1] + P[20][2]*SF[0] + P[20][3]*SF[2] - P[20][1]*SPP[0];
    nextP[20][6] = P[20][6] + P[20][1]*SF[1] + P[20][3]*SF[0] + P[20][0]*SPP[0] - P[20][2]*SPP[1];
    nextP[20][7] = P[20][7] + P[20][4]*dt;
    nextP[20][8] = P[20][8] + P[20][5]*dt;
    nextP[20][9] = P[20][9] + P[20][6]*dt;
    nextP[20][10] = P[20][10];
    nextP[20][11] = P[20][11];
    nextP[20][12] = P[20][12];
    nextP[20][13] = P[20][13];
    nextP[20][14] = P[20][14];
    nextP[20][15] = P[20][15];
    nextP[20][16] = P[20][16];
    nextP[20][17] = P[20][17];
    nextP[20][18] = P[20][18];
    nextP[20][19] = P[20][19];
    nextP[20][20] = P[20][20];

    for (unsigned i = 0; i < n_states; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // If on ground or no magnetometer fitted, inhibit magnetometer bias updates by
    // setting the coresponding covariance terms to zero.
    if (onGround || !useCompass)
    {
        zeroRows(nextP,15,20);
        zeroCols(nextP,15,20);
    }

    // If on ground or not using airspeed sensing, inhibit wind velocity
    // covariance growth.
    if (onGround || !useAirspeed)
    {
        zeroRows(nextP,13,14);
        zeroCols(nextP,13,14);
    }

    // If the total position variance exceds 1E6 (1000m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    if ((P[7][7] + P[8][8]) > 1E6f)
    {
        for (uint8_t i=7; i<=8; i++)
        {
            for (unsigned j = 0; j < n_states; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    if (onGround || staticMode) {
        // copy the portion of the variances we want to
        // propagate
        for (unsigned i = 0; i < 14; i++) {
            P[i][i] = nextP[i][i];

            // force symmetry for observable states
            // force zero for non-observable states
            for (unsigned i = 1; i < n_states; i++)
            {
                for (uint8_t j = 0; j < i; j++)
                {
                    if ((i > 12) || (j > 12)) {
                        P[i][j] = 0.0f;
                    } else {
                        P[i][j] = 0.5f * (nextP[i][j] + nextP[j][i]);
                    }
                    P[j][i] = P[i][j];
                }
            }
        }

    } else {

        // Copy covariance
        for (unsigned i = 0; i < n_states; i++) {
            P[i][i] = nextP[i][i];
        }

        // force symmetry for observable states
        for (unsigned i = 1; i < n_states; i++)
        {
            for (uint8_t j = 0; j < i; j++)
            {
                P[i][j] = 0.5f * (nextP[i][j] + nextP[j][i]);
                P[j][i] = P[i][j];
            }
        }

    }

    ConstrainVariances();
}

void AttPosEKF::FuseVelposNED()
{

// declare variables used by fault isolation logic
    uint32_t gpsRetryTime = 30000; // time in msec before GPS fusion will be retried following innovation consistency failure
    uint32_t gpsRetryTimeNoTAS = 5000; // retry time if no TAS measurement available
    uint32_t hgtRetryTime = 5000; // height measurement retry time
    uint32_t horizRetryTime;

// declare variables used to check measurement errors
    float velInnov[3] = {0.0f,0.0f,0.0f};
    float posInnov[2] = {0.0f,0.0f};
    float hgtInnov = 0.0f;

// declare variables used to control access to arrays
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;
    uint8_t indexLimit;

// declare variables used by state and covariance update calculations
    float velErr;
    float posErr;
    float R_OBS[6];
    float observation[6];
    float SK;
    float quatMag;

// Perform sequential fusion of GPS measurements. This assumes that the
// errors in the different velocity and position components are
// uncorrelated which is not true, however in the absence of covariance
// data from the GPS receiver it is the only assumption we can make
// so we might as well take advantage of the computational efficiencies
// associated with sequential fusion
    if (fuseVelData || fusePosData || fuseHgtData)
    {
        // set the GPS data timeout depending on whether airspeed data is present
        if (useAirspeed) horizRetryTime = gpsRetryTime;
        else horizRetryTime = gpsRetryTimeNoTAS;

        // Form the observation vector
        for (uint8_t i=0; i<=2; i++) observation[i] = velNED[i];
        for (uint8_t i=3; i<=4; i++) observation[i] = posNE[i-3];
        observation[5] = -(hgtMea);

        // Estimate the GPS Velocity, GPS horiz position and height measurement variances.
        velErr = 0.2f*accNavMag; // additional error in GPS velocities caused by manoeuvring
        posErr = 0.2f*accNavMag; // additional error in GPS position caused by manoeuvring
        R_OBS[0] = 0.04f + sq(velErr);
        R_OBS[1] = R_OBS[0];
        R_OBS[2] = 0.08f + sq(velErr);
        R_OBS[3] = R_OBS[2];
        R_OBS[4] = 4.0f  + sq(posErr);
        R_OBS[5] = 4.0f;

        // Set innovation variances to zero default
        for (uint8_t i = 0; i<=5; i++)
        {
            varInnovVelPos[i] = 0.0f;
        }
        // calculate innovations and check GPS data validity using an innovation consistency check
        if (fuseVelData)
        {
            // test velocity measurements
            uint8_t imax = 2;
            if (fusionModeGPS == 1) imax = 1;
            for (uint8_t i = 0; i<=imax; i++)
            {
                velInnov[i] = statesAtVelTime[i+4] - velNED[i];
                stateIndex = 4 + i;
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS[i];
            }
            // apply a 5-sigma threshold
            current_ekf_state.velHealth = (sq(velInnov[0]) + sq(velInnov[1]) + sq(velInnov[2])) < 25.0f * (varInnovVelPos[0] + varInnovVelPos[1] + varInnovVelPos[2]);
            current_ekf_state.velTimeout = (millis() - current_ekf_state.velFailTime) > horizRetryTime;
            if (current_ekf_state.velHealth || current_ekf_state.velTimeout)
            {
                current_ekf_state.velHealth = true;
                current_ekf_state.velFailTime = millis();
            }
            else
            {
                current_ekf_state.velHealth = false;
            }
        }
        if (fusePosData)
        {
            // test horizontal position measurements
            posInnov[0] = statesAtPosTime[7] - posNE[0];
            posInnov[1] = statesAtPosTime[8] - posNE[1];
            varInnovVelPos[3] = P[7][7] + R_OBS[3];
            varInnovVelPos[4] = P[8][8] + R_OBS[4];
            // apply a 10-sigma threshold
            current_ekf_state.posHealth = (sq(posInnov[0]) + sq(posInnov[1])) < 100.0f*(varInnovVelPos[3] + varInnovVelPos[4]);
            current_ekf_state.posTimeout = (millis() - current_ekf_state.posFailTime) > horizRetryTime;
            if (current_ekf_state.posHealth || current_ekf_state.posTimeout)
            {
                current_ekf_state.posHealth = true;
                current_ekf_state.posFailTime = millis();
            }
            else
            {
                current_ekf_state.posHealth = false;
            }
        }
        // test height measurements
        if (fuseHgtData)
        {
            hgtInnov = statesAtHgtTime[9] + hgtMea;
            varInnovVelPos[5] = P[9][9] + R_OBS[5];
            // apply a 10-sigma threshold
            current_ekf_state.hgtHealth = sq(hgtInnov) < 100.0f*varInnovVelPos[5];
            current_ekf_state.hgtTimeout = (millis() - current_ekf_state.hgtFailTime) > hgtRetryTime;
            if (current_ekf_state.hgtHealth || current_ekf_state.hgtTimeout)
            {
                current_ekf_state.hgtHealth = true;
                current_ekf_state.hgtFailTime = millis();
            }
            else
            {
                current_ekf_state.hgtHealth = false;
            }
        }
        // Set range for sequential fusion of velocity and position measurements depending
        // on which data is available and its health
        if (fuseVelData && fusionModeGPS == 0 && current_ekf_state.velHealth)
        {
            fuseData[0] = true;
            fuseData[1] = true;
            fuseData[2] = true;
        }
        if (fuseVelData && fusionModeGPS == 1 && current_ekf_state.velHealth)
        {
            fuseData[0] = true;
            fuseData[1] = true;
        }
        if (fusePosData && fusionModeGPS <= 2 && current_ekf_state.posHealth)
        {
            fuseData[3] = true;
            fuseData[4] = true;
        }
        if (fuseHgtData && current_ekf_state.hgtHealth)
        {
            fuseData[5] = true;
        }
        // Limit range of states modified when on ground
        if(!onGround)
        {
            indexLimit = 20;
        }
        else
        {
            indexLimit = 12;
        }
        // Fuse measurements sequentially
        for (obsIndex=0; obsIndex<=5; obsIndex++)
        {
            if (fuseData[obsIndex])
            {
                stateIndex = 4 + obsIndex;
                // Calculate the measurement innovation, using states from a
                // different time coordinate if fusing height data
                if (obsIndex >= 0 && obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = statesAtVelTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 3 || obsIndex == 4)
                {
                    innovVelPos[obsIndex] = statesAtPosTime[stateIndex] - observation[obsIndex];
                }
                else if (obsIndex == 5)
                {
                    innovVelPos[obsIndex] = statesAtHgtTime[stateIndex] - observation[obsIndex];
                }
                // Calculate the Kalman Gain
                // Calculate innovation variances - also used for data logging
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0/varInnovVelPos[obsIndex];
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }
                // Calculate state corrections and re-normalise the quaternions
                for (uint8_t i = 0; i<=indexLimit; i++)
                {
                    states[i] = states[i] - Kfusion[i] * innovVelPos[obsIndex];
                }
                quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
                if (quatMag > 1e-12f) // divide by  0 protection
                {
                    for (uint8_t i = 0; i<=3; i++)
                    {
                        states[i] = states[i] / quatMag;
                    }
                }
                // Update the covariance - take advantage of direct observation of a
                // single state at index = stateIndex to reduce computations
                // Optimised implementation of standard equation P = (I - K*H)*P;
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                for (uint8_t i= 0; i<=indexLimit; i++)
                {
                    for (uint8_t j= 0; j<=indexLimit; j++)
                    {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();

    //printf("velh: %s, posh: %s, hgth: %s\n", ((velHealth) ? "OK" : "FAIL"), ((posHealth) ? "OK" : "FAIL"), ((hgtHealth) ? "OK" : "FAIL"));
}

void AttPosEKF::FuseMagnetometer()
{
    uint8_t obsIndex;
    uint8_t indexLimit;
    float DCM[3][3] =
    {
        {1.0f,0.0f,0.0f} ,
        {0.0f,1.0f,0.0f} ,
        {0.0f,0.0f,1.0f}
    };
    float MagPred[3] = {0.0f,0.0f,0.0f};
    float SK_MX[6];
    float SK_MY[5];
    float SK_MZ[6];
    float SH_MAG[9] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

// Perform sequential fusion of Magnetometer measurements.
// This assumes that the errors in the different components are
// uncorrelated which is not true, however in the absence of covariance
// data fit is the only assumption we can make
// so we might as well take advantage of the computational efficiencies
// associated with sequential fusion
    if (useCompass && (fuseMagData || obsIndex == 1 || obsIndex == 2))
    {
        // Limit range of states modified when on ground
        if(!onGround)
        {
            indexLimit = 20;
        }
        else
        {
            indexLimit = 12;
        }

        static float q0 = 0.0f;
        static float q1 = 0.0f;
        static float q2 = 0.0f;
        static float q3 = 1.0f;
        static float magN = 0.4f;
        static float magE = 0.0f;
        static float magD = 0.3f;

        static float R_MAG = 0.0025f;

        float H_MAG[21] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

        // Sequential fusion of XYZ components to spread processing load across
        // three prediction time steps.

        // Calculate observation jacobians and Kalman gains
        if (fuseMagData)
        {
            static float magXbias = 0.0f;
            static float magYbias = 0.0f;
            static float magZbias = 0.0f;

            // Copy required states to local variable names
            q0       = statesAtMagMeasTime[0];
            q1       = statesAtMagMeasTime[1];
            q2       = statesAtMagMeasTime[2];
            q3       = statesAtMagMeasTime[3];
            magN     = statesAtMagMeasTime[15];
            magE     = statesAtMagMeasTime[16];
            magD     = statesAtMagMeasTime[17];
            magXbias = statesAtMagMeasTime[18];
            magYbias = statesAtMagMeasTime[19];
            magZbias = statesAtMagMeasTime[20];

            // rotate predicted earth components into body axes and calculate
            // predicted measurments
            DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
            DCM[0][1] = 2*(q1*q2 + q0*q3);
            DCM[0][2] = 2*(q1*q3-q0*q2);
            DCM[1][0] = 2*(q1*q2 - q0*q3);
            DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
            DCM[1][2] = 2*(q2*q3 + q0*q1);
            DCM[2][0] = 2*(q1*q3 + q0*q2);
            DCM[2][1] = 2*(q2*q3 - q0*q1);
            DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
            MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
            MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
            MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

            // scale magnetometer observation error with total angular rate
            R_MAG = 0.0025f + sq(0.05f*dAngIMU.length()/dtIMU);

            // Calculate observation jacobians
            SH_MAG[0] = 2*magD*q3 + 2*magE*q2 + 2*magN*q1;
            SH_MAG[1] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
            SH_MAG[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
            SH_MAG[3] = sq(q3);
            SH_MAG[4] = sq(q2);
            SH_MAG[5] = sq(q1);
            SH_MAG[6] = sq(q0);
            SH_MAG[7] = 2*magN*q0;
            SH_MAG[8] = 2*magE*q3;

            for (uint8_t i=0; i<=20; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            H_MAG[1] = SH_MAG[0];
            H_MAG[2] = 2*magE*q1 - 2*magD*q0 - 2*magN*q2;
            H_MAG[3] = SH_MAG[2];
            H_MAG[15] = SH_MAG[5] - SH_MAG[4] - SH_MAG[3] + SH_MAG[6];
            H_MAG[16] = 2*q0*q3 + 2*q1*q2;
            H_MAG[17] = 2*q1*q3 - 2*q0*q2;
            H_MAG[18] = 1.0f;

            // Calculate Kalman gain
            SK_MX[0] = 1/(P[18][18] + R_MAG + P[1][18]*SH_MAG[0] + P[3][18]*SH_MAG[2] - P[15][18]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) - (2*magD*q0 - 2*magE*q1 + 2*magN*q2)*(P[18][2] + P[1][2]*SH_MAG[0] + P[3][2]*SH_MAG[2] - P[15][2]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][2]*(2*q0*q3 + 2*q1*q2) - P[17][2]*(2*q0*q2 - 2*q1*q3) - P[2][2]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[18][0] + P[1][0]*SH_MAG[0] + P[3][0]*SH_MAG[2] - P[15][0]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][0]*(2*q0*q3 + 2*q1*q2) - P[17][0]*(2*q0*q2 - 2*q1*q3) - P[2][0]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[18][1] + P[1][1]*SH_MAG[0] + P[3][1]*SH_MAG[2] - P[15][1]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][1]*(2*q0*q3 + 2*q1*q2) - P[17][1]*(2*q0*q2 - 2*q1*q3) - P[2][1]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[2]*(P[18][3] + P[1][3]*SH_MAG[0] + P[3][3]*SH_MAG[2] - P[15][3]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][3]*(2*q0*q3 + 2*q1*q2) - P[17][3]*(2*q0*q2 - 2*q1*q3) - P[2][3]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6])*(P[18][15] + P[1][15]*SH_MAG[0] + P[3][15]*SH_MAG[2] - P[15][15]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][15]*(2*q0*q3 + 2*q1*q2) - P[17][15]*(2*q0*q2 - 2*q1*q3) - P[2][15]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][15]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[16][18]*(2*q0*q3 + 2*q1*q2) - P[17][18]*(2*q0*q2 - 2*q1*q3) - P[2][18]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + (2*q0*q3 + 2*q1*q2)*(P[18][16] + P[1][16]*SH_MAG[0] + P[3][16]*SH_MAG[2] - P[15][16]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][16]*(2*q0*q3 + 2*q1*q2) - P[17][16]*(2*q0*q2 - 2*q1*q3) - P[2][16]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q2 - 2*q1*q3)*(P[18][17] + P[1][17]*SH_MAG[0] + P[3][17]*SH_MAG[2] - P[15][17]*(SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6]) + P[16][17]*(2*q0*q3 + 2*q1*q2) - P[17][17]*(2*q0*q2 - 2*q1*q3) - P[2][17]*(2*magD*q0 - 2*magE*q1 + 2*magN*q2) + P[0][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[0][18]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            SK_MX[1] = SH_MAG[3] + SH_MAG[4] - SH_MAG[5] - SH_MAG[6];
            SK_MX[2] = 2*magD*q0 - 2*magE*q1 + 2*magN*q2;
            SK_MX[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MX[4] = 2*q0*q2 - 2*q1*q3;
            SK_MX[5] = 2*q0*q3 + 2*q1*q2;
            Kfusion[0] = SK_MX[0]*(P[0][18] + P[0][1]*SH_MAG[0] + P[0][3]*SH_MAG[2] + P[0][0]*SK_MX[3] - P[0][2]*SK_MX[2] - P[0][15]*SK_MX[1] + P[0][16]*SK_MX[5] - P[0][17]*SK_MX[4]);
            Kfusion[1] = SK_MX[0]*(P[1][18] + P[1][1]*SH_MAG[0] + P[1][3]*SH_MAG[2] + P[1][0]*SK_MX[3] - P[1][2]*SK_MX[2] - P[1][15]*SK_MX[1] + P[1][16]*SK_MX[5] - P[1][17]*SK_MX[4]);
            Kfusion[2] = SK_MX[0]*(P[2][18] + P[2][1]*SH_MAG[0] + P[2][3]*SH_MAG[2] + P[2][0]*SK_MX[3] - P[2][2]*SK_MX[2] - P[2][15]*SK_MX[1] + P[2][16]*SK_MX[5] - P[2][17]*SK_MX[4]);
            Kfusion[3] = SK_MX[0]*(P[3][18] + P[3][1]*SH_MAG[0] + P[3][3]*SH_MAG[2] + P[3][0]*SK_MX[3] - P[3][2]*SK_MX[2] - P[3][15]*SK_MX[1] + P[3][16]*SK_MX[5] - P[3][17]*SK_MX[4]);
            Kfusion[4] = SK_MX[0]*(P[4][18] + P[4][1]*SH_MAG[0] + P[4][3]*SH_MAG[2] + P[4][0]*SK_MX[3] - P[4][2]*SK_MX[2] - P[4][15]*SK_MX[1] + P[4][16]*SK_MX[5] - P[4][17]*SK_MX[4]);
            Kfusion[5] = SK_MX[0]*(P[5][18] + P[5][1]*SH_MAG[0] + P[5][3]*SH_MAG[2] + P[5][0]*SK_MX[3] - P[5][2]*SK_MX[2] - P[5][15]*SK_MX[1] + P[5][16]*SK_MX[5] - P[5][17]*SK_MX[4]);
            Kfusion[6] = SK_MX[0]*(P[6][18] + P[6][1]*SH_MAG[0] + P[6][3]*SH_MAG[2] + P[6][0]*SK_MX[3] - P[6][2]*SK_MX[2] - P[6][15]*SK_MX[1] + P[6][16]*SK_MX[5] - P[6][17]*SK_MX[4]);
            Kfusion[7] = SK_MX[0]*(P[7][18] + P[7][1]*SH_MAG[0] + P[7][3]*SH_MAG[2] + P[7][0]*SK_MX[3] - P[7][2]*SK_MX[2] - P[7][15]*SK_MX[1] + P[7][16]*SK_MX[5] - P[7][17]*SK_MX[4]);
            Kfusion[8] = SK_MX[0]*(P[8][18] + P[8][1]*SH_MAG[0] + P[8][3]*SH_MAG[2] + P[8][0]*SK_MX[3] - P[8][2]*SK_MX[2] - P[8][15]*SK_MX[1] + P[8][16]*SK_MX[5] - P[8][17]*SK_MX[4]);
            Kfusion[9] = SK_MX[0]*(P[9][18] + P[9][1]*SH_MAG[0] + P[9][3]*SH_MAG[2] + P[9][0]*SK_MX[3] - P[9][2]*SK_MX[2] - P[9][15]*SK_MX[1] + P[9][16]*SK_MX[5] - P[9][17]*SK_MX[4]);
            Kfusion[10] = SK_MX[0]*(P[10][18] + P[10][1]*SH_MAG[0] + P[10][3]*SH_MAG[2] + P[10][0]*SK_MX[3] - P[10][2]*SK_MX[2] - P[10][15]*SK_MX[1] + P[10][16]*SK_MX[5] - P[10][17]*SK_MX[4]);
            Kfusion[11] = SK_MX[0]*(P[11][18] + P[11][1]*SH_MAG[0] + P[11][3]*SH_MAG[2] + P[11][0]*SK_MX[3] - P[11][2]*SK_MX[2] - P[11][15]*SK_MX[1] + P[11][16]*SK_MX[5] - P[11][17]*SK_MX[4]);
            Kfusion[12] = SK_MX[0]*(P[12][18] + P[12][1]*SH_MAG[0] + P[12][3]*SH_MAG[2] + P[12][0]*SK_MX[3] - P[12][2]*SK_MX[2] - P[12][15]*SK_MX[1] + P[12][16]*SK_MX[5] - P[12][17]*SK_MX[4]);
            Kfusion[13] = SK_MX[0]*(P[13][18] + P[13][1]*SH_MAG[0] + P[13][3]*SH_MAG[2] + P[13][0]*SK_MX[3] - P[13][2]*SK_MX[2] - P[13][15]*SK_MX[1] + P[13][16]*SK_MX[5] - P[13][17]*SK_MX[4]);
            Kfusion[14] = SK_MX[0]*(P[14][18] + P[14][1]*SH_MAG[0] + P[14][3]*SH_MAG[2] + P[14][0]*SK_MX[3] - P[14][2]*SK_MX[2] - P[14][15]*SK_MX[1] + P[14][16]*SK_MX[5] - P[14][17]*SK_MX[4]);
            Kfusion[15] = SK_MX[0]*(P[15][18] + P[15][1]*SH_MAG[0] + P[15][3]*SH_MAG[2] + P[15][0]*SK_MX[3] - P[15][2]*SK_MX[2] - P[15][15]*SK_MX[1] + P[15][16]*SK_MX[5] - P[15][17]*SK_MX[4]);
            Kfusion[16] = SK_MX[0]*(P[16][18] + P[16][1]*SH_MAG[0] + P[16][3]*SH_MAG[2] + P[16][0]*SK_MX[3] - P[16][2]*SK_MX[2] - P[16][15]*SK_MX[1] + P[16][16]*SK_MX[5] - P[16][17]*SK_MX[4]);
            Kfusion[17] = SK_MX[0]*(P[17][18] + P[17][1]*SH_MAG[0] + P[17][3]*SH_MAG[2] + P[17][0]*SK_MX[3] - P[17][2]*SK_MX[2] - P[17][15]*SK_MX[1] + P[17][16]*SK_MX[5] - P[17][17]*SK_MX[4]);
            Kfusion[18] = SK_MX[0]*(P[18][18] + P[18][1]*SH_MAG[0] + P[18][3]*SH_MAG[2] + P[18][0]*SK_MX[3] - P[18][2]*SK_MX[2] - P[18][15]*SK_MX[1] + P[18][16]*SK_MX[5] - P[18][17]*SK_MX[4]);
            Kfusion[19] = SK_MX[0]*(P[19][18] + P[19][1]*SH_MAG[0] + P[19][3]*SH_MAG[2] + P[19][0]*SK_MX[3] - P[19][2]*SK_MX[2] - P[19][15]*SK_MX[1] + P[19][16]*SK_MX[5] - P[19][17]*SK_MX[4]);
            Kfusion[20] = SK_MX[0]*(P[20][18] + P[20][1]*SH_MAG[0] + P[20][3]*SH_MAG[2] + P[20][0]*SK_MX[3] - P[20][2]*SK_MX[2] - P[20][15]*SK_MX[1] + P[20][16]*SK_MX[5] - P[20][17]*SK_MX[4]);
            varInnovMag[0] = 1.0f/SK_MX[0];
            innovMag[0] = MagPred[0] - magData.x;

            // reset the observation index to 0 (we start by fusing the X
            // measurement)
            obsIndex = 0;
        }
        else if (obsIndex == 1) // we are now fusing the Y measurement
        {
            // Calculate observation jacobians
            for (unsigned int i=0; i<n_states; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[2];
            H_MAG[1] = SH_MAG[1];
            H_MAG[2] = SH_MAG[0];
            H_MAG[3] = 2*magD*q2 - SH_MAG[8] - SH_MAG[7];
            H_MAG[15] = 2*q1*q2 - 2*q0*q3;
            H_MAG[16] = SH_MAG[4] - SH_MAG[3] - SH_MAG[5] + SH_MAG[6];
            H_MAG[17] = 2*q0*q1 + 2*q2*q3;
            H_MAG[19] = 1;

            // Calculate Kalman gain
            SK_MY[0] = 1/(P[19][19] + R_MAG + P[0][19]*SH_MAG[2] + P[1][19]*SH_MAG[1] + P[2][19]*SH_MAG[0] - P[16][19]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - (2*q0*q3 - 2*q1*q2)*(P[19][15] + P[0][15]*SH_MAG[2] + P[1][15]*SH_MAG[1] + P[2][15]*SH_MAG[0] - P[16][15]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][15]*(2*q0*q3 - 2*q1*q2) + P[17][15]*(2*q0*q1 + 2*q2*q3) - P[3][15]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (2*q0*q1 + 2*q2*q3)*(P[19][17] + P[0][17]*SH_MAG[2] + P[1][17]*SH_MAG[1] + P[2][17]*SH_MAG[0] - P[16][17]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][17]*(2*q0*q3 - 2*q1*q2) + P[17][17]*(2*q0*q1 + 2*q2*q3) - P[3][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[19][3] + P[0][3]*SH_MAG[2] + P[1][3]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[16][3]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][3]*(2*q0*q3 - 2*q1*q2) + P[17][3]*(2*q0*q1 + 2*q2*q3) - P[3][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[15][19]*(2*q0*q3 - 2*q1*q2) + P[17][19]*(2*q0*q1 + 2*q2*q3) + SH_MAG[2]*(P[19][0] + P[0][0]*SH_MAG[2] + P[1][0]*SH_MAG[1] + P[2][0]*SH_MAG[0] - P[16][0]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][0]*(2*q0*q3 - 2*q1*q2) + P[17][0]*(2*q0*q1 + 2*q2*q3) - P[3][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[19][1] + P[0][1]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[2][1]*SH_MAG[0] - P[16][1]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][1]*(2*q0*q3 - 2*q1*q2) + P[17][1]*(2*q0*q1 + 2*q2*q3) - P[3][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[19][2] + P[0][2]*SH_MAG[2] + P[1][2]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[16][2]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][2]*(2*q0*q3 - 2*q1*q2) + P[17][2]*(2*q0*q1 + 2*q2*q3) - P[3][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6])*(P[19][16] + P[0][16]*SH_MAG[2] + P[1][16]*SH_MAG[1] + P[2][16]*SH_MAG[0] - P[16][16]*(SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6]) - P[15][16]*(2*q0*q3 - 2*q1*q2) + P[17][16]*(2*q0*q1 + 2*q2*q3) - P[3][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - P[3][19]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            SK_MY[1] = SH_MAG[3] - SH_MAG[4] + SH_MAG[5] - SH_MAG[6];
            SK_MY[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MY[3] = 2*q0*q3 - 2*q1*q2;
            SK_MY[4] = 2*q0*q1 + 2*q2*q3;
            Kfusion[0] = SK_MY[0]*(P[0][19] + P[0][0]*SH_MAG[2] + P[0][1]*SH_MAG[1] + P[0][2]*SH_MAG[0] - P[0][3]*SK_MY[2] - P[0][16]*SK_MY[1] - P[0][15]*SK_MY[3] + P[0][17]*SK_MY[4]);
            Kfusion[1] = SK_MY[0]*(P[1][19] + P[1][0]*SH_MAG[2] + P[1][1]*SH_MAG[1] + P[1][2]*SH_MAG[0] - P[1][3]*SK_MY[2] - P[1][16]*SK_MY[1] - P[1][15]*SK_MY[3] + P[1][17]*SK_MY[4]);
            Kfusion[2] = SK_MY[0]*(P[2][19] + P[2][0]*SH_MAG[2] + P[2][1]*SH_MAG[1] + P[2][2]*SH_MAG[0] - P[2][3]*SK_MY[2] - P[2][16]*SK_MY[1] - P[2][15]*SK_MY[3] + P[2][17]*SK_MY[4]);
            Kfusion[3] = SK_MY[0]*(P[3][19] + P[3][0]*SH_MAG[2] + P[3][1]*SH_MAG[1] + P[3][2]*SH_MAG[0] - P[3][3]*SK_MY[2] - P[3][16]*SK_MY[1] - P[3][15]*SK_MY[3] + P[3][17]*SK_MY[4]);
            Kfusion[4] = SK_MY[0]*(P[4][19] + P[4][0]*SH_MAG[2] + P[4][1]*SH_MAG[1] + P[4][2]*SH_MAG[0] - P[4][3]*SK_MY[2] - P[4][16]*SK_MY[1] - P[4][15]*SK_MY[3] + P[4][17]*SK_MY[4]);
            Kfusion[5] = SK_MY[0]*(P[5][19] + P[5][0]*SH_MAG[2] + P[5][1]*SH_MAG[1] + P[5][2]*SH_MAG[0] - P[5][3]*SK_MY[2] - P[5][16]*SK_MY[1] - P[5][15]*SK_MY[3] + P[5][17]*SK_MY[4]);
            Kfusion[6] = SK_MY[0]*(P[6][19] + P[6][0]*SH_MAG[2] + P[6][1]*SH_MAG[1] + P[6][2]*SH_MAG[0] - P[6][3]*SK_MY[2] - P[6][16]*SK_MY[1] - P[6][15]*SK_MY[3] + P[6][17]*SK_MY[4]);
            Kfusion[7] = SK_MY[0]*(P[7][19] + P[7][0]*SH_MAG[2] + P[7][1]*SH_MAG[1] + P[7][2]*SH_MAG[0] - P[7][3]*SK_MY[2] - P[7][16]*SK_MY[1] - P[7][15]*SK_MY[3] + P[7][17]*SK_MY[4]);
            Kfusion[8] = SK_MY[0]*(P[8][19] + P[8][0]*SH_MAG[2] + P[8][1]*SH_MAG[1] + P[8][2]*SH_MAG[0] - P[8][3]*SK_MY[2] - P[8][16]*SK_MY[1] - P[8][15]*SK_MY[3] + P[8][17]*SK_MY[4]);
            Kfusion[9] = SK_MY[0]*(P[9][19] + P[9][0]*SH_MAG[2] + P[9][1]*SH_MAG[1] + P[9][2]*SH_MAG[0] - P[9][3]*SK_MY[2] - P[9][16]*SK_MY[1] - P[9][15]*SK_MY[3] + P[9][17]*SK_MY[4]);
            Kfusion[10] = SK_MY[0]*(P[10][19] + P[10][0]*SH_MAG[2] + P[10][1]*SH_MAG[1] + P[10][2]*SH_MAG[0] - P[10][3]*SK_MY[2] - P[10][16]*SK_MY[1] - P[10][15]*SK_MY[3] + P[10][17]*SK_MY[4]);
            Kfusion[11] = SK_MY[0]*(P[11][19] + P[11][0]*SH_MAG[2] + P[11][1]*SH_MAG[1] + P[11][2]*SH_MAG[0] - P[11][3]*SK_MY[2] - P[11][16]*SK_MY[1] - P[11][15]*SK_MY[3] + P[11][17]*SK_MY[4]);
            Kfusion[12] = SK_MY[0]*(P[12][19] + P[12][0]*SH_MAG[2] + P[12][1]*SH_MAG[1] + P[12][2]*SH_MAG[0] - P[12][3]*SK_MY[2] - P[12][16]*SK_MY[1] - P[12][15]*SK_MY[3] + P[12][17]*SK_MY[4]);
            Kfusion[13] = SK_MY[0]*(P[13][19] + P[13][0]*SH_MAG[2] + P[13][1]*SH_MAG[1] + P[13][2]*SH_MAG[0] - P[13][3]*SK_MY[2] - P[13][16]*SK_MY[1] - P[13][15]*SK_MY[3] + P[13][17]*SK_MY[4]);
            Kfusion[14] = SK_MY[0]*(P[14][19] + P[14][0]*SH_MAG[2] + P[14][1]*SH_MAG[1] + P[14][2]*SH_MAG[0] - P[14][3]*SK_MY[2] - P[14][16]*SK_MY[1] - P[14][15]*SK_MY[3] + P[14][17]*SK_MY[4]);
            Kfusion[15] = SK_MY[0]*(P[15][19] + P[15][0]*SH_MAG[2] + P[15][1]*SH_MAG[1] + P[15][2]*SH_MAG[0] - P[15][3]*SK_MY[2] - P[15][16]*SK_MY[1] - P[15][15]*SK_MY[3] + P[15][17]*SK_MY[4]);
            Kfusion[16] = SK_MY[0]*(P[16][19] + P[16][0]*SH_MAG[2] + P[16][1]*SH_MAG[1] + P[16][2]*SH_MAG[0] - P[16][3]*SK_MY[2] - P[16][16]*SK_MY[1] - P[16][15]*SK_MY[3] + P[16][17]*SK_MY[4]);
            Kfusion[17] = SK_MY[0]*(P[17][19] + P[17][0]*SH_MAG[2] + P[17][1]*SH_MAG[1] + P[17][2]*SH_MAG[0] - P[17][3]*SK_MY[2] - P[17][16]*SK_MY[1] - P[17][15]*SK_MY[3] + P[17][17]*SK_MY[4]);
            Kfusion[18] = SK_MY[0]*(P[18][19] + P[18][0]*SH_MAG[2] + P[18][1]*SH_MAG[1] + P[18][2]*SH_MAG[0] - P[18][3]*SK_MY[2] - P[18][16]*SK_MY[1] - P[18][15]*SK_MY[3] + P[18][17]*SK_MY[4]);
            Kfusion[19] = SK_MY[0]*(P[19][19] + P[19][0]*SH_MAG[2] + P[19][1]*SH_MAG[1] + P[19][2]*SH_MAG[0] - P[19][3]*SK_MY[2] - P[19][16]*SK_MY[1] - P[19][15]*SK_MY[3] + P[19][17]*SK_MY[4]);
            Kfusion[20] = SK_MY[0]*(P[20][19] + P[20][0]*SH_MAG[2] + P[20][1]*SH_MAG[1] + P[20][2]*SH_MAG[0] - P[20][3]*SK_MY[2] - P[20][16]*SK_MY[1] - P[20][15]*SK_MY[3] + P[20][17]*SK_MY[4]);
            varInnovMag[1] = 1.0f/SK_MY[0];
            innovMag[1] = MagPred[1] - magData.y;
        }
        else if (obsIndex == 2) // we are now fusing the Z measurement
        {
            // Calculate observation jacobians
            for (uint8_t i=0; i<=20; i++) H_MAG[i] = 0;
            H_MAG[0] = SH_MAG[1];
            H_MAG[1] = 2*magN*q3 - 2*magE*q0 - 2*magD*q1;
            H_MAG[2] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            H_MAG[3] = SH_MAG[0];
            H_MAG[15] = 2*q0*q2 + 2*q1*q3;
            H_MAG[16] = 2*q2*q3 - 2*q0*q1;
            H_MAG[17] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            H_MAG[20] = 1;

            // Calculate Kalman gain
            SK_MZ[0] = 1/(P[20][20] + R_MAG + P[0][20]*SH_MAG[1] + P[3][20]*SH_MAG[0] + P[17][20]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) - (2*magD*q1 + 2*magE*q0 - 2*magN*q3)*(P[20][1] + P[0][1]*SH_MAG[1] + P[3][1]*SH_MAG[0] + P[17][1]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][1]*(2*q0*q2 + 2*q1*q3) - P[16][1]*(2*q0*q1 - 2*q2*q3) - P[1][1]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][1]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[7] + SH_MAG[8] - 2*magD*q2)*(P[20][2] + P[0][2]*SH_MAG[1] + P[3][2]*SH_MAG[0] + P[17][2]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][2]*(2*q0*q2 + 2*q1*q3) - P[16][2]*(2*q0*q1 - 2*q2*q3) - P[1][2]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][2]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[1]*(P[20][0] + P[0][0]*SH_MAG[1] + P[3][0]*SH_MAG[0] + P[17][0]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][0]*(2*q0*q2 + 2*q1*q3) - P[16][0]*(2*q0*q1 - 2*q2*q3) - P[1][0]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][0]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + SH_MAG[0]*(P[20][3] + P[0][3]*SH_MAG[1] + P[3][3]*SH_MAG[0] + P[17][3]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][3]*(2*q0*q2 + 2*q1*q3) - P[16][3]*(2*q0*q1 - 2*q2*q3) - P[1][3]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][3]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + (SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6])*(P[20][17] + P[0][17]*SH_MAG[1] + P[3][17]*SH_MAG[0] + P[17][17]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][17]*(2*q0*q2 + 2*q1*q3) - P[16][17]*(2*q0*q1 - 2*q2*q3) - P[1][17]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][17]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[15][20]*(2*q0*q2 + 2*q1*q3) - P[16][20]*(2*q0*q1 - 2*q2*q3) - P[1][20]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + (2*q0*q2 + 2*q1*q3)*(P[20][15] + P[0][15]*SH_MAG[1] + P[3][15]*SH_MAG[0] + P[17][15]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][15]*(2*q0*q2 + 2*q1*q3) - P[16][15]*(2*q0*q1 - 2*q2*q3) - P[1][15]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][15]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) - (2*q0*q1 - 2*q2*q3)*(P[20][16] + P[0][16]*SH_MAG[1] + P[3][16]*SH_MAG[0] + P[17][16]*(SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6]) + P[15][16]*(2*q0*q2 + 2*q1*q3) - P[16][16]*(2*q0*q1 - 2*q2*q3) - P[1][16]*(2*magD*q1 + 2*magE*q0 - 2*magN*q3) + P[2][16]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2)) + P[2][20]*(SH_MAG[7] + SH_MAG[8] - 2*magD*q2));
            SK_MZ[1] = SH_MAG[3] - SH_MAG[4] - SH_MAG[5] + SH_MAG[6];
            SK_MZ[2] = 2*magD*q1 + 2*magE*q0 - 2*magN*q3;
            SK_MZ[3] = SH_MAG[7] + SH_MAG[8] - 2*magD*q2;
            SK_MZ[4] = 2*q0*q1 - 2*q2*q3;
            SK_MZ[5] = 2*q0*q2 + 2*q1*q3;
            Kfusion[0] = SK_MZ[0]*(P[0][20] + P[0][0]*SH_MAG[1] + P[0][3]*SH_MAG[0] - P[0][1]*SK_MZ[2] + P[0][2]*SK_MZ[3] + P[0][17]*SK_MZ[1] + P[0][15]*SK_MZ[5] - P[0][16]*SK_MZ[4]);
            Kfusion[1] = SK_MZ[0]*(P[1][20] + P[1][0]*SH_MAG[1] + P[1][3]*SH_MAG[0] - P[1][1]*SK_MZ[2] + P[1][2]*SK_MZ[3] + P[1][17]*SK_MZ[1] + P[1][15]*SK_MZ[5] - P[1][16]*SK_MZ[4]);
            Kfusion[2] = SK_MZ[0]*(P[2][20] + P[2][0]*SH_MAG[1] + P[2][3]*SH_MAG[0] - P[2][1]*SK_MZ[2] + P[2][2]*SK_MZ[3] + P[2][17]*SK_MZ[1] + P[2][15]*SK_MZ[5] - P[2][16]*SK_MZ[4]);
            Kfusion[3] = SK_MZ[0]*(P[3][20] + P[3][0]*SH_MAG[1] + P[3][3]*SH_MAG[0] - P[3][1]*SK_MZ[2] + P[3][2]*SK_MZ[3] + P[3][17]*SK_MZ[1] + P[3][15]*SK_MZ[5] - P[3][16]*SK_MZ[4]);
            Kfusion[4] = SK_MZ[0]*(P[4][20] + P[4][0]*SH_MAG[1] + P[4][3]*SH_MAG[0] - P[4][1]*SK_MZ[2] + P[4][2]*SK_MZ[3] + P[4][17]*SK_MZ[1] + P[4][15]*SK_MZ[5] - P[4][16]*SK_MZ[4]);
            Kfusion[5] = SK_MZ[0]*(P[5][20] + P[5][0]*SH_MAG[1] + P[5][3]*SH_MAG[0] - P[5][1]*SK_MZ[2] + P[5][2]*SK_MZ[3] + P[5][17]*SK_MZ[1] + P[5][15]*SK_MZ[5] - P[5][16]*SK_MZ[4]);
            Kfusion[6] = SK_MZ[0]*(P[6][20] + P[6][0]*SH_MAG[1] + P[6][3]*SH_MAG[0] - P[6][1]*SK_MZ[2] + P[6][2]*SK_MZ[3] + P[6][17]*SK_MZ[1] + P[6][15]*SK_MZ[5] - P[6][16]*SK_MZ[4]);
            Kfusion[7] = SK_MZ[0]*(P[7][20] + P[7][0]*SH_MAG[1] + P[7][3]*SH_MAG[0] - P[7][1]*SK_MZ[2] + P[7][2]*SK_MZ[3] + P[7][17]*SK_MZ[1] + P[7][15]*SK_MZ[5] - P[7][16]*SK_MZ[4]);
            Kfusion[8] = SK_MZ[0]*(P[8][20] + P[8][0]*SH_MAG[1] + P[8][3]*SH_MAG[0] - P[8][1]*SK_MZ[2] + P[8][2]*SK_MZ[3] + P[8][17]*SK_MZ[1] + P[8][15]*SK_MZ[5] - P[8][16]*SK_MZ[4]);
            Kfusion[9] = SK_MZ[0]*(P[9][20] + P[9][0]*SH_MAG[1] + P[9][3]*SH_MAG[0] - P[9][1]*SK_MZ[2] + P[9][2]*SK_MZ[3] + P[9][17]*SK_MZ[1] + P[9][15]*SK_MZ[5] - P[9][16]*SK_MZ[4]);
            Kfusion[10] = SK_MZ[0]*(P[10][20] + P[10][0]*SH_MAG[1] + P[10][3]*SH_MAG[0] - P[10][1]*SK_MZ[2] + P[10][2]*SK_MZ[3] + P[10][17]*SK_MZ[1] + P[10][15]*SK_MZ[5] - P[10][16]*SK_MZ[4]);
            Kfusion[11] = SK_MZ[0]*(P[11][20] + P[11][0]*SH_MAG[1] + P[11][3]*SH_MAG[0] - P[11][1]*SK_MZ[2] + P[11][2]*SK_MZ[3] + P[11][17]*SK_MZ[1] + P[11][15]*SK_MZ[5] - P[11][16]*SK_MZ[4]);
            Kfusion[12] = SK_MZ[0]*(P[12][20] + P[12][0]*SH_MAG[1] + P[12][3]*SH_MAG[0] - P[12][1]*SK_MZ[2] + P[12][2]*SK_MZ[3] + P[12][17]*SK_MZ[1] + P[12][15]*SK_MZ[5] - P[12][16]*SK_MZ[4]);
            Kfusion[13] = SK_MZ[0]*(P[13][20] + P[13][0]*SH_MAG[1] + P[13][3]*SH_MAG[0] - P[13][1]*SK_MZ[2] + P[13][2]*SK_MZ[3] + P[13][17]*SK_MZ[1] + P[13][15]*SK_MZ[5] - P[13][16]*SK_MZ[4]);
            Kfusion[14] = SK_MZ[0]*(P[14][20] + P[14][0]*SH_MAG[1] + P[14][3]*SH_MAG[0] - P[14][1]*SK_MZ[2] + P[14][2]*SK_MZ[3] + P[14][17]*SK_MZ[1] + P[14][15]*SK_MZ[5] - P[14][16]*SK_MZ[4]);
            Kfusion[15] = SK_MZ[0]*(P[15][20] + P[15][0]*SH_MAG[1] + P[15][3]*SH_MAG[0] - P[15][1]*SK_MZ[2] + P[15][2]*SK_MZ[3] + P[15][17]*SK_MZ[1] + P[15][15]*SK_MZ[5] - P[15][16]*SK_MZ[4]);
            Kfusion[16] = SK_MZ[0]*(P[16][20] + P[16][0]*SH_MAG[1] + P[16][3]*SH_MAG[0] - P[16][1]*SK_MZ[2] + P[16][2]*SK_MZ[3] + P[16][17]*SK_MZ[1] + P[16][15]*SK_MZ[5] - P[16][16]*SK_MZ[4]);
            Kfusion[17] = SK_MZ[0]*(P[17][20] + P[17][0]*SH_MAG[1] + P[17][3]*SH_MAG[0] - P[17][1]*SK_MZ[2] + P[17][2]*SK_MZ[3] + P[17][17]*SK_MZ[1] + P[17][15]*SK_MZ[5] - P[17][16]*SK_MZ[4]);
            Kfusion[18] = SK_MZ[0]*(P[18][20] + P[18][0]*SH_MAG[1] + P[18][3]*SH_MAG[0] - P[18][1]*SK_MZ[2] + P[18][2]*SK_MZ[3] + P[18][17]*SK_MZ[1] + P[18][15]*SK_MZ[5] - P[18][16]*SK_MZ[4]);
            Kfusion[19] = SK_MZ[0]*(P[19][20] + P[19][0]*SH_MAG[1] + P[19][3]*SH_MAG[0] - P[19][1]*SK_MZ[2] + P[19][2]*SK_MZ[3] + P[19][17]*SK_MZ[1] + P[19][15]*SK_MZ[5] - P[19][16]*SK_MZ[4]);
            Kfusion[20] = SK_MZ[0]*(P[20][20] + P[20][0]*SH_MAG[1] + P[20][3]*SH_MAG[0] - P[20][1]*SK_MZ[2] + P[20][2]*SK_MZ[3] + P[20][17]*SK_MZ[1] + P[20][15]*SK_MZ[5] - P[20][16]*SK_MZ[4]);
            varInnovMag[2] = 1.0f/SK_MZ[0];
            innovMag[2] = MagPred[2] - magData.z;

        }

        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovMag[obsIndex]*innovMag[obsIndex]/varInnovMag[obsIndex]) < 25.0)
        {
            // correct the state vector
            for (uint8_t j= 0; j<=indexLimit; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovMag[obsIndex];
            }
            // normalise the quaternion states
            float quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=3; j++)
                {
                    KH[i][j] = Kfusion[i] * H_MAG[j];
                }
                for (uint8_t j = 4; j<=17; j++) KH[i][j] = 0.0f;
                if (!onGround)
                {
                    for (uint8_t j = 15; j<=20; j++)
                    {
                        KH[i][j] = Kfusion[i] * H_MAG[j];
                    }
                }
                else
                {
                    for (uint8_t j = 15; j<=20; j++)
                    {
                        KH[i][j] = 0.0f;
                    }
                }
            }
            for (uint8_t i = 0; i<=indexLimit; i++)
            {
                for (uint8_t j = 0; j<=indexLimit; j++)
                {
                    KHP[i][j] = 0.0f;
                    for (uint8_t k = 0; k<=3; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    if (!onGround)
                    {
                        for (uint8_t k = 15; k<=20; k++)
                        {
                            KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                        }
                    }
                }
            }
        }
        for (uint8_t i = 0; i<=indexLimit; i++)
        {
            for (uint8_t j = 0; j<=indexLimit; j++)
            {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }
    }
    obsIndex = obsIndex + 1;

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::FuseAirspeed()
{
    float vn;
    float ve;
    float vd;
    float vwn;
    float vwe;
    const float R_TAS = 2.0f;
    float SH_TAS[3];
    float Kfusion[21];
    float VtasPred;

    // Copy required states to local variable names
    vn = statesAtVtasMeasTime[4];
    ve = statesAtVtasMeasTime[5];
    vd = statesAtVtasMeasTime[6];
    vwn = statesAtVtasMeasTime[13];
    vwe = statesAtVtasMeasTime[14];

    // Need to check that it is flying before fusing airspeed data
    // Calculate the predicted airspeed
    VtasPred = sqrtf((ve - vwe)*(ve - vwe) + (vn - vwn)*(vn - vwn) + vd*vd);
    // Perform fusion of True Airspeed measurement
    if (useAirspeed && fuseVtasData && (VtasPred > 1.0f) && (VtasMeas > 8.0f))
    {
        // Calculate observation jacobians
        SH_TAS[0] = 1/(sqrt(sq(ve - vwe) + sq(vn - vwn) + sq(vd)));
        SH_TAS[1] = (SH_TAS[0]*(2.0f*ve - 2*vwe))/2.0f;
        SH_TAS[2] = (SH_TAS[0]*(2.0f*vn - 2*vwn))/2.0f;
        
        float H_TAS[21];
        for (uint8_t i=0; i<=20; i++) H_TAS[i] = 0.0f;
        H_TAS[4] = SH_TAS[2];
        H_TAS[5] = SH_TAS[1];
        H_TAS[6] = vd*SH_TAS[0];
        H_TAS[13] = -SH_TAS[2];
        H_TAS[14] = -SH_TAS[1];

        // Calculate Kalman gains
        float SK_TAS = 1.0f/(R_TAS + SH_TAS[2]*(P[4][4]*SH_TAS[2] + P[5][4]*SH_TAS[1] - P[13][4]*SH_TAS[2] - P[14][4]*SH_TAS[1] + P[6][4]*vd*SH_TAS[0]) + SH_TAS[1]*(P[4][5]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[13][5]*SH_TAS[2] - P[14][5]*SH_TAS[1] + P[6][5]*vd*SH_TAS[0]) - SH_TAS[2]*(P[4][13]*SH_TAS[2] + P[5][13]*SH_TAS[1] - P[13][13]*SH_TAS[2] - P[14][13]*SH_TAS[1] + P[6][13]*vd*SH_TAS[0]) - SH_TAS[1]*(P[4][14]*SH_TAS[2] + P[5][14]*SH_TAS[1] - P[13][14]*SH_TAS[2] - P[14][14]*SH_TAS[1] + P[6][14]*vd*SH_TAS[0]) + vd*SH_TAS[0]*(P[4][6]*SH_TAS[2] + P[5][6]*SH_TAS[1] - P[13][6]*SH_TAS[2] - P[14][6]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]));
        Kfusion[0] = SK_TAS*(P[0][4]*SH_TAS[2] - P[0][13]*SH_TAS[2] + P[0][5]*SH_TAS[1] - P[0][14]*SH_TAS[1] + P[0][6]*vd*SH_TAS[0]);
        Kfusion[1] = SK_TAS*(P[1][4]*SH_TAS[2] - P[1][13]*SH_TAS[2] + P[1][5]*SH_TAS[1] - P[1][14]*SH_TAS[1] + P[1][6]*vd*SH_TAS[0]);
        Kfusion[2] = SK_TAS*(P[2][4]*SH_TAS[2] - P[2][13]*SH_TAS[2] + P[2][5]*SH_TAS[1] - P[2][14]*SH_TAS[1] + P[2][6]*vd*SH_TAS[0]);
        Kfusion[3] = SK_TAS*(P[3][4]*SH_TAS[2] - P[3][13]*SH_TAS[2] + P[3][5]*SH_TAS[1] - P[3][14]*SH_TAS[1] + P[3][6]*vd*SH_TAS[0]);
        Kfusion[4] = SK_TAS*(P[4][4]*SH_TAS[2] - P[4][13]*SH_TAS[2] + P[4][5]*SH_TAS[1] - P[4][14]*SH_TAS[1] + P[4][6]*vd*SH_TAS[0]);
        Kfusion[5] = SK_TAS*(P[5][4]*SH_TAS[2] - P[5][13]*SH_TAS[2] + P[5][5]*SH_TAS[1] - P[5][14]*SH_TAS[1] + P[5][6]*vd*SH_TAS[0]);
        Kfusion[6] = SK_TAS*(P[6][4]*SH_TAS[2] - P[6][13]*SH_TAS[2] + P[6][5]*SH_TAS[1] - P[6][14]*SH_TAS[1] + P[6][6]*vd*SH_TAS[0]);
        Kfusion[7] = SK_TAS*(P[7][4]*SH_TAS[2] - P[7][13]*SH_TAS[2] + P[7][5]*SH_TAS[1] - P[7][14]*SH_TAS[1] + P[7][6]*vd*SH_TAS[0]);
        Kfusion[8] = SK_TAS*(P[8][4]*SH_TAS[2] - P[8][13]*SH_TAS[2] + P[8][5]*SH_TAS[1] - P[8][14]*SH_TAS[1] + P[8][6]*vd*SH_TAS[0]);
        Kfusion[9] = SK_TAS*(P[9][4]*SH_TAS[2] - P[9][13]*SH_TAS[2] + P[9][5]*SH_TAS[1] - P[9][14]*SH_TAS[1] + P[9][6]*vd*SH_TAS[0]);
        Kfusion[10] = SK_TAS*(P[10][4]*SH_TAS[2] - P[10][13]*SH_TAS[2] + P[10][5]*SH_TAS[1] - P[10][14]*SH_TAS[1] + P[10][6]*vd*SH_TAS[0]);
        Kfusion[11] = SK_TAS*(P[11][4]*SH_TAS[2] - P[11][13]*SH_TAS[2] + P[11][5]*SH_TAS[1] - P[11][14]*SH_TAS[1] + P[11][6]*vd*SH_TAS[0]);
        Kfusion[12] = SK_TAS*(P[12][4]*SH_TAS[2] - P[12][13]*SH_TAS[2] + P[12][5]*SH_TAS[1] - P[12][14]*SH_TAS[1] + P[12][6]*vd*SH_TAS[0]);
        Kfusion[13] = SK_TAS*(P[13][4]*SH_TAS[2] - P[13][13]*SH_TAS[2] + P[13][5]*SH_TAS[1] - P[13][14]*SH_TAS[1] + P[13][6]*vd*SH_TAS[0]);
        Kfusion[14] = SK_TAS*(P[14][4]*SH_TAS[2] - P[14][13]*SH_TAS[2] + P[14][5]*SH_TAS[1] - P[14][14]*SH_TAS[1] + P[14][6]*vd*SH_TAS[0]);
        Kfusion[15] = SK_TAS*(P[15][4]*SH_TAS[2] - P[15][13]*SH_TAS[2] + P[15][5]*SH_TAS[1] - P[15][14]*SH_TAS[1] + P[15][6]*vd*SH_TAS[0]);
        Kfusion[16] = SK_TAS*(P[16][4]*SH_TAS[2] - P[16][13]*SH_TAS[2] + P[16][5]*SH_TAS[1] - P[16][14]*SH_TAS[1] + P[16][6]*vd*SH_TAS[0]);
        Kfusion[17] = SK_TAS*(P[17][4]*SH_TAS[2] - P[17][13]*SH_TAS[2] + P[17][5]*SH_TAS[1] - P[17][14]*SH_TAS[1] + P[17][6]*vd*SH_TAS[0]);
        Kfusion[18] = SK_TAS*(P[18][4]*SH_TAS[2] - P[18][13]*SH_TAS[2] + P[18][5]*SH_TAS[1] - P[18][14]*SH_TAS[1] + P[18][6]*vd*SH_TAS[0]);
        Kfusion[19] = SK_TAS*(P[19][4]*SH_TAS[2] - P[19][13]*SH_TAS[2] + P[19][5]*SH_TAS[1] - P[19][14]*SH_TAS[1] + P[19][6]*vd*SH_TAS[0]);
        Kfusion[20] = SK_TAS*(P[20][4]*SH_TAS[2] - P[20][13]*SH_TAS[2] + P[20][5]*SH_TAS[1] - P[20][14]*SH_TAS[1] + P[20][6]*vd*SH_TAS[0]);
        varInnovVtas = 1.0f/SK_TAS;

        // Calculate the measurement innovation
        innovVtas = VtasPred - VtasMeas;
        // Check the innovation for consistency and don't fuse if > 5Sigma
        if ((innovVtas*innovVtas*SK_TAS) < 25.0)
        {
            // correct the state vector
            for (uint8_t j=0; j<=20; j++)
            {
                states[j] = states[j] - Kfusion[j] * innovVtas;
            }
            // normalise the quaternion states
            float quatMag = sqrt(states[0]*states[0] + states[1]*states[1] + states[2]*states[2] + states[3]*states[3]);
            if (quatMag > 1e-12f)
            {
                for (uint8_t j= 0; j<=3; j++)
                {
                    float quatMagInv = 1.0f/quatMag;
                    states[j] = states[j] * quatMagInv;
                }
            }
            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in H to reduce the
            // number of operations
            for (uint8_t i = 0; i<=20; i++)
            {
                for (uint8_t j = 0; j<=3; j++) KH[i][j] = 0.0;
                for (uint8_t j = 4; j<=6; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 7; j<=12; j++) KH[i][j] = 0.0;
                for (uint8_t j = 13; j<=14; j++)
                {
                    KH[i][j] = Kfusion[i] * H_TAS[j];
                }
                for (uint8_t j = 15; j<=20; j++) KH[i][j] = 0.0;
            }
            for (uint8_t i = 0; i<=20; i++)
            {
                for (uint8_t j = 0; j<=20; j++)
                {
                    KHP[i][j] = 0.0;
                    for (uint8_t k = 4; k<=6; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                    for (uint8_t k = 13; k<=14; k++)
                    {
                        KHP[i][j] = KHP[i][j] + KH[i][k] * P[k][j];
                    }
                }
            }
            for (uint8_t i = 0; i<=20; i++)
            {
                for (uint8_t j = 0; j<=20; j++)
                {
                    P[i][j] = P[i][j] - KHP[i][j];
                }
            }
        }
    }

    ForceSymmetry();
    ConstrainVariances();
}

void AttPosEKF::zeroRows(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (row=first; row<=last; row++)
    {
        for (col=0; col<n_states; col++)
        {
            covMat[row][col] = 0.0;
        }
    }
}

void AttPosEKF::zeroCols(float (&covMat)[n_states][n_states], uint8_t first, uint8_t last)
{
    uint8_t row;
    uint8_t col;
    for (col=first; col<=last; col++)
    {
        for (row=0; row < n_states; row++)
        {
            covMat[row][col] = 0.0;
        }
    }
}

float AttPosEKF::sq(float valIn)
{
    return valIn*valIn;
}

// Store states in a history array along with time stamp
void AttPosEKF::StoreStates(uint64_t timestamp_ms)
{
    for (unsigned i=0; i<n_states; i++)
        storedStates[i][storeIndex] = states[i];
    statetimeStamp[storeIndex] = timestamp_ms;
    storeIndex++;
    if (storeIndex == data_buffer_size)
        storeIndex = 0;
}

void AttPosEKF::ResetStoredStates()
{
    // reset all stored states
    memset(&storedStates[0][0], 0, sizeof(storedStates));
    memset(&statetimeStamp[0], 0, sizeof(statetimeStamp));

    // reset store index to first
    storeIndex = 0;

    // overwrite all existing states
    for (unsigned i = 0; i < n_states; i++) {
        storedStates[i][storeIndex] = states[i];
    }

    statetimeStamp[storeIndex] = millis();

    // increment to next storage index
    storeIndex++;
}

// Output the state vector stored at the time that best matches that specified by msec
int AttPosEKF::RecallStates(float statesForFusion[n_states], uint64_t msec)
{
    int ret = 0;

    // int64_t bestTimeDelta = 200;
    // unsigned bestStoreIndex = 0;
    // for (unsigned storeIndex = 0; storeIndex < data_buffer_size; storeIndex++)
    // {
    //     // The time delta can also end up as negative number,
    //     // since we might compare future to past or past to future
    //     // therefore cast to int64.
    //     int64_t timeDelta = (int64_t)msec - (int64_t)statetimeStamp[storeIndex];
    //     if (timeDelta < 0) {
    //         timeDelta = -timeDelta;
    //     }

    //     if (timeDelta < bestTimeDelta)
    //     {
    //         bestStoreIndex = storeIndex;
    //         bestTimeDelta = timeDelta;
    //     }
    // }
    // if (bestTimeDelta < 200) // only output stored state if < 200 msec retrieval error
    // {
    //     for (uint8_t i=0; i < n_states; i++) {
    //         if (isfinite(storedStates[i][bestStoreIndex])) {
    //             statesForFusion[i] = storedStates[i][bestStoreIndex];
    //         } else if (isfinite(states[i])) {
    //             statesForFusion[i] = states[i];
    //         } else {
    //             // There is not much we can do here, except reporting the error we just
    //             // found.
    //             ret++;
    //         }
    // }
    // else // otherwise output current state
    // {
        for (uint8_t i=0; i < n_states; i++) {
            if (isfinite(states[i])) {
                statesForFusion[i] = states[i];
            } else {
                ret++;
            }
        }
    // }

    return ret;
}

void AttPosEKF::quat2Tnb(Mat3f &Tnb, const float (&quat)[4])
{
    // Calculate the nav to body cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    Tnb.x.x = q00 + q11 - q22 - q33;
    Tnb.y.y = q00 - q11 + q22 - q33;
    Tnb.z.z = q00 - q11 - q22 + q33;
    Tnb.y.x = 2*(q12 - q03);
    Tnb.z.x = 2*(q13 + q02);
    Tnb.x.y = 2*(q12 + q03);
    Tnb.z.y = 2*(q23 - q01);
    Tnb.x.z = 2*(q13 - q02);
    Tnb.y.z = 2*(q23 + q01);
}

void AttPosEKF::quat2Tbn(Mat3f &Tbn, const float (&quat)[4])
{
    // Calculate the body to nav cosine matrix
    float q00 = sq(quat[0]);
    float q11 = sq(quat[1]);
    float q22 = sq(quat[2]);
    float q33 = sq(quat[3]);
    float q01 =  quat[0]*quat[1];
    float q02 =  quat[0]*quat[2];
    float q03 =  quat[0]*quat[3];
    float q12 =  quat[1]*quat[2];
    float q13 =  quat[1]*quat[3];
    float q23 =  quat[2]*quat[3];

    Tbn.x.x = q00 + q11 - q22 - q33;
    Tbn.y.y = q00 - q11 + q22 - q33;
    Tbn.z.z = q00 - q11 - q22 + q33;
    Tbn.x.y = 2*(q12 - q03);
    Tbn.x.z = 2*(q13 + q02);
    Tbn.y.x = 2*(q12 + q03);
    Tbn.y.z = 2*(q23 - q01);
    Tbn.z.x = 2*(q13 - q02);
    Tbn.z.y = 2*(q23 + q01);
}

void AttPosEKF::eul2quat(float (&quat)[4], const float (&eul)[3])
{
    float u1 = cos(0.5f*eul[0]);
    float u2 = cos(0.5f*eul[1]);
    float u3 = cos(0.5f*eul[2]);
    float u4 = sin(0.5f*eul[0]);
    float u5 = sin(0.5f*eul[1]);
    float u6 = sin(0.5f*eul[2]);
    quat[0] = u1*u2*u3+u4*u5*u6;
    quat[1] = u4*u2*u3-u1*u5*u6;
    quat[2] = u1*u5*u3+u4*u2*u6;
    quat[3] = u1*u2*u6-u4*u5*u3;
}

void AttPosEKF::quat2eul(float (&y)[3], const float (&u)[4])
{
    y[0] = atan2f((2.0f*(u[2]*u[3]+u[0]*u[1])) , (u[0]*u[0]-u[1]*u[1]-u[2]*u[2]+u[3]*u[3]));
    y[1] = -asinf(2.0f*(u[1]*u[3]-u[0]*u[2]));
    y[2] = atan2f((2.0f*(u[1]*u[2]+u[0]*u[3])) , (u[0]*u[0]+u[1]*u[1]-u[2]*u[2]-u[3]*u[3]));
}

void AttPosEKF::calcvelNED(float (&velNED)[3], float gpsCourse, float gpsGndSpd, float gpsVelD)
{
    velNED[0] = gpsGndSpd*cosf(gpsCourse);
    velNED[1] = gpsGndSpd*sinf(gpsCourse);
    velNED[2] = gpsVelD;
}

void AttPosEKF::calcposNED(float (&posNED)[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef)
{
    posNED[0] = earthRadius * (lat - latRef);
    posNED[1] = earthRadius * cos(latRef) * (lon - lonRef);
    posNED[2] = -(hgt - hgtRef);
}

void AttPosEKF::calcLLH(float (&posNED)[3], float lat, float lon, float hgt, float latRef, float lonRef, float hgtRef)
{
    lat = latRef + posNED[0] * earthRadiusInv;
    lon = lonRef + posNED[1] * earthRadiusInv / cos(latRef);
    hgt = hgtRef - posNED[2];
}

void AttPosEKF::OnGroundCheck()
{
    onGround = (((sq(velNED[0]) + sq(velNED[1]) + sq(velNED[2])) < 4.0f) && (VtasMeas < 8.0f));
    if (staticMode) {
        staticMode = !(GPSstatus > GPS_FIX_2D);
    }
}

void AttPosEKF::calcEarthRateNED(Vector3f &omega, float latitude)
{
    //Define Earth rotation vector in the NED navigation frame
    omega.x  = earthRate*cosf(latitude);
    omega.y  = 0.0f;
    omega.z  = -earthRate*sinf(latitude);
}

void AttPosEKF::CovarianceInit()
{
    // Calculate the initial covariance matrix P
    P[0][0]   = 0.25f * sq(1.0f*deg2rad);
    P[1][1]   = 0.25f * sq(1.0f*deg2rad);
    P[2][2]   = 0.25f * sq(1.0f*deg2rad);
    P[3][3]   = 0.25f * sq(10.0f*deg2rad);
    P[4][4]   = sq(0.7);
    P[5][5]   = P[4][4];
    P[6][6]   = sq(0.7);
    P[7][7]   = sq(15.0);
    P[8][8]   = P[7][7];
    P[9][9]   = sq(5.0);
    P[10][10] = sq(0.1*deg2rad*dtIMU);
    P[11][11] = P[10][10];
    P[12][12] = P[10][10];
    P[13][13] = sq(8.0f);
    P[14][4]  = P[13][13];
    P[15][15] = sq(0.02f);
    P[16][16] = P[15][15];
    P[17][17] = P[15][15];
    P[18][18] = sq(0.02f);
    P[19][19] = P[18][18];
    P[20][20] = P[18][18];
}

float AttPosEKF::ConstrainFloat(float val, float min, float max)
{
    return (val > max) ? max : ((val < min) ? min : val);
}

void AttPosEKF::ConstrainVariances()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13-14: Wind Vector  - m/sec (North,East)
    // 15-17: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 18-20: Body Magnetic Field Vector - gauss (X,Y,Z)

    // Constrain quaternion variances
    for (unsigned i = 0; i < 4; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Constrain velocitie variances
    for (unsigned i = 4; i < 7; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Constrain position variances
    for (unsigned i = 7; i < 10; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e6f);
    }

    // Angle bias variances
    for (unsigned i = 10; i < 13; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, sq(0.175f * dtIMU));
    }

    // Wind velocity variances
    for (unsigned i = 13; i < 15; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0e3f);
    }

    // Earth magnetic field variances
    for (unsigned i = 15; i < 18; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

    // Body magnetic field variances
    for (unsigned i = 18; i < 21; i++) {
        P[i][i] = ConstrainFloat(P[i][i], 0.0f, 1.0f);
    }

}

void AttPosEKF::ConstrainStates()
{
    if (!numericalProtection) {
        return;
    }

    // State vector:
    // 0-3: quaternions (q0, q1, q2, q3)
    // 4-6: Velocity - m/sec (North, East, Down)
    // 7-9: Position - m (North, East, Down)
    // 10-12: Delta Angle bias - rad (X,Y,Z)
    // 13-14: Wind Vector  - m/sec (North,East)
    // 15-17: Earth Magnetic Field Vector - gauss (North, East, Down)
    // 18-20: Body Magnetic Field Vector - gauss (X,Y,Z)


    // Constrain quaternion
    for (unsigned i = 0; i < 4; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Constrain velocities to what GPS can do for us
    for (unsigned i = 4; i < 7; i++) {
        states[i] = ConstrainFloat(states[i], -5.0e2f, 5.0e2f);
    }

    // Constrain position to a reasonable vehicle range (in meters)
    for (unsigned i = 7; i < 9; i++) {
        states[i] = ConstrainFloat(states[i], -1.0e6f, 1.0e6f);
    }

    // Constrain altitude
    states[9] = ConstrainFloat(states[9], -4.0e4f, 1.0e4f);

    // Angle bias limit - set to 8 degrees / sec
    for (unsigned i = 10; i < 13; i++) {
        states[i] = ConstrainFloat(states[i], -0.12f * dtIMU, 0.12f * dtIMU);
    }

    // Wind velocity limits - assume 120 m/s max velocity
    for (unsigned i = 13; i < 15; i++) {
        states[i] = ConstrainFloat(states[i], -120.0f, 120.0f);
    }

    // Earth magnetic field limits (in Gauss)
    for (unsigned i = 15; i < 18; i++) {
        states[i] = ConstrainFloat(states[i], -1.0f, 1.0f);
    }

    // Body magnetic field variances (in Gauss).
    // the max offset should be in this range.
    for (unsigned i = 18; i < 21; i++) {
        states[i] = ConstrainFloat(states[i], -0.5f, 0.5f);
    }

}

void AttPosEKF::ForceSymmetry()
{
    if (!numericalProtection) {
        return;
    }

    // Force symmetry on the covariance matrix to prevent ill-conditioning
    // of the matrix which would cause the filter to blow-up
    for (unsigned i = 1; i < n_states; i++)
    {
        for (uint8_t j = 0; j < i; j++)
        {
            P[i][j] = 0.5f * (P[i][j] + P[j][i]);
            P[j][i] = P[i][j];
        }
    }
}

bool AttPosEKF::FilterHealthy()
{
    if (!statesInitialised) {
        return false;
    }

    // XXX Check state vector for NaNs and ill-conditioning

    // Check if any of the major inputs timed out
    if (current_ekf_state.posTimeout || current_ekf_state.velTimeout || current_ekf_state.hgtTimeout) {
        return false;
    }

    // Nothing fired, return ok.
    return true;
}

/**
 * Reset the filter position states
 *
 * This resets the position to the last GPS measurement
 * or to zero in case of static position.
 */
void AttPosEKF::ResetPosition(void)
{
    if (staticMode) {
        states[7] = 0;
        states[8] = 0;
    } else if (GPSstatus >= GPS_FIX_3D) {

        // reset the states from the GPS measurements
        states[7] = posNE[0];
        states[8] = posNE[1];
    }
}

/**
 * Reset the height state.
 *
 * This resets the height state with the last altitude measurements
 */
void AttPosEKF::ResetHeight(void)
{
    // write to the state vector
    states[9]   = -hgtMea;
}

/**
 * Reset the velocity state.
 */
void AttPosEKF::ResetVelocity(void)
{
    if (staticMode) {
        states[4] = 0.0f;
        states[5] = 0.0f;
        states[6] = 0.0f;
    } else if (GPSstatus >= GPS_FIX_3D) {

        states[4]  = velNED[0]; // north velocity from last reading
        states[5]  = velNED[1]; // east velocity from last reading
        states[6]  = velNED[2]; // down velocity from last reading
    }
}


void AttPosEKF::FillErrorReport(struct ekf_status_report *err)
{
    for (int i = 0; i < n_states; i++)
    {
        err->states[i] = states[i];
    }

    err->velHealth = current_ekf_state.velHealth;
    err->posHealth = current_ekf_state.posHealth;
    err->hgtHealth = current_ekf_state.hgtHealth;
    err->velTimeout = current_ekf_state.velTimeout;
    err->posTimeout = current_ekf_state.posTimeout;
    err->hgtTimeout = current_ekf_state.hgtTimeout;
}

bool AttPosEKF::StatesNaN(struct ekf_status_report *err_report) {
    bool err = false;

    // check all states and covariance matrices
    for (unsigned i = 0; i < n_states; i++) {
        for (unsigned j = 0; j < n_states; j++) {
            if (!isfinite(KH[i][j])) {

                err_report->covarianceNaN = true;
                err = true;
            } //  intermediate result used for covariance updates
            if (!isfinite(KHP[i][j])) {

                err_report->covarianceNaN = true;
                err = true;
            } // intermediate result used for covariance updates
            if (!isfinite(P[i][j])) {

                err_report->covarianceNaN = true;
                err = true;
            } // covariance matrix
        }

        if (!isfinite(Kfusion[i])) {

            err_report->kalmanGainsNaN = true;
            err = true;
        } // Kalman gains

        if (!isfinite(states[i])) {

            err_report->statesNaN = true;
            err = true;
        } // state matrix
    }

    if (err) {
        FillErrorReport(err_report);
    }

    return err;

}

/**
 * Check the filter inputs and bound its operational state
 *
 * This check will reset the filter states if required
 * due to a failure of consistency or timeout checks.
 * it should be run after the measurement data has been
 * updated, but before any of the fusion steps are
 * executed.
 */
int AttPosEKF::CheckAndBound()
{

    // Store the old filter state
    bool currStaticMode = staticMode;

    // Reset the filter if the states went NaN
    if (StatesNaN(&last_ekf_error)) {

        InitializeDynamic(velNED, magDeclination);

        return 1;
    }

    // Reset the filter if the IMU data is too old
    if (dtIMU > 0.2f) {

        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        // that's all we can do here, return
        return 2;
    }

    // Check if we're on ground - this also sets static mode.
    OnGroundCheck();

    // Check if we switched between states
    if (currStaticMode != staticMode) {
        ResetVelocity();
        ResetPosition();
        ResetHeight();
        ResetStoredStates();

        return 3;
    }

    return 0;
}

void AttPosEKF::AttitudeInit(float ax, float ay, float az, float mx, float my, float mz, float declination, float *initQuat)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);
    /* true heading is the mag heading minus declination */
    initialHdg += declination;

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    initQuat[0] = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    initQuat[1] = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    initQuat[2] = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    initQuat[3] = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
}

void AttPosEKF::InitializeDynamic(float (&initvelNED)[3], float declination)
{

    // Clear the init flag
    statesInitialised = false;

    magDeclination = declination;

    ZeroVariables();

    // Calculate initial filter quaternion states from raw measurements
    float initQuat[4];
    Vector3f initMagXYZ;
    initMagXYZ   = magData - magBias;
    AttitudeInit(accel.x, accel.y, accel.z, initMagXYZ.x, initMagXYZ.y, initMagXYZ.z, magDeclination, initQuat);

    // Calculate initial Tbn matrix and rotate Mag measurements into NED
    // to set initial NED magnetic field states
    Mat3f DCM;
    quat2Tbn(DCM, initQuat);
    Vector3f initMagNED;
    initMagXYZ   = magData - magBias;
    initMagNED.x = DCM.x.x*initMagXYZ.x + DCM.x.y*initMagXYZ.y + DCM.x.z*initMagXYZ.z;
    initMagNED.y = DCM.y.x*initMagXYZ.x + DCM.y.y*initMagXYZ.y + DCM.y.z*initMagXYZ.z;
    initMagNED.z = DCM.z.x*initMagXYZ.x + DCM.z.y*initMagXYZ.y + DCM.z.z*initMagXYZ.z;



    // write to state vector
    for (uint8_t j=0; j<=3; j++) states[j] = initQuat[j]; // quaternions
    for (uint8_t j=0; j<=2; j++) states[j+4] = initvelNED[j]; // velocities
    for (uint8_t j=0; j<=7; j++) states[j+7] = 0.0f; // positiions, dAngBias, windVel
    states[15] = initMagNED.x; // Magnetic Field North
    states[16] = initMagNED.y; // Magnetic Field East
    states[17] = initMagNED.z; // Magnetic Field Down
    states[18] = magBias.x; // Magnetic Field Bias X
    states[19] = magBias.y; // Magnetic Field Bias Y
    states[20] = magBias.z; // Magnetic Field Bias Z

    statesInitialised = true;

    // initialise the covariance matrix
    CovarianceInit();

    //Define Earth rotation vector in the NED navigation frame
    calcEarthRateNED(earthRateNED, latRef);

    //Initialise summed variables used by covariance prediction
    summedDelAng.x = 0.0f;
    summedDelAng.y = 0.0f;
    summedDelAng.z = 0.0f;
    summedDelVel.x = 0.0f;
    summedDelVel.y = 0.0f;
    summedDelVel.z = 0.0f;
}

void AttPosEKF::InitialiseFilter(float (&initvelNED)[3], double referenceLat, double referenceLon, float referenceHgt, float declination)
{
    //store initial lat,long and height
    latRef = referenceLat;
    lonRef = referenceLon;
    hgtRef = referenceHgt;

    memset(&last_ekf_error, 0, sizeof(last_ekf_error));

    InitializeDynamic(initvelNED, declination);
}

void AttPosEKF::ZeroVariables()
{
    // Do the data structure init
    for (unsigned i = 0; i < n_states; i++) {
        for (unsigned j = 0; j < n_states; j++) {
            KH[i][j] = 0.0f; //  intermediate result used for covariance updates
            KHP[i][j] = 0.0f; // intermediate result used for covariance updates
            P[i][j] = 0.0f; // covariance matrix
        }

        Kfusion[i] = 0.0f; // Kalman gains
        states[i] = 0.0f; // state matrix
    }

    for (unsigned i = 0; i < data_buffer_size; i++) {

        for (unsigned j = 0; j < n_states; j++) {
            storedStates[j][i] = 0.0f;
        }

        statetimeStamp[i] = 0;
    }

    memset(&current_ekf_state, 0, sizeof(current_ekf_state));
}

void AttPosEKF::GetFilterState(struct ekf_status_report *state)
{
    memcpy(state, &current_ekf_state, sizeof(state));
}

void AttPosEKF::GetLastErrorState(struct ekf_status_report *last_error)
{
    memcpy(last_error, &last_ekf_error, sizeof(last_error));
}
